/****
 * Controller.cpp
 *
 * Copyright 2021 mikee47 <mike@sillyhouse.net>
 * 
 * This file is part of the HardwareSPI Library
 *
 * This library is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, version 3 or later.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this library.
 * If not, see <https://www.gnu.org/licenses/>.
 *
 * @author: August 2021 - mikee47 <mike@sillyhouse.net>
 *
 ****/

#include <HSPI/Controller.h>
#include <HSPI/Device.h>
#include <driver/spi_master.h>
#include <soc/spi_periph.h>
#include <esp_intr_alloc.h>
#include <Platform/Timers.h>
#include <debug_progmem.h>

namespace HSPI
{
constexpr size_t hardwareBufferSize{SPI_MAX_DMA_LEN};

#ifdef HSPI_ENABLE_STATS
volatile Controller::Stats Controller::stats;
#endif

const SpiPins defaultPins[]{
	{.sck = SPI_IOMUX_PIN_NUM_CLK, .miso = SPI_IOMUX_PIN_NUM_MISO, .mosi = SPI_IOMUX_PIN_NUM_MOSI},
	{.sck = SPI2_IOMUX_PIN_NUM_CLK, .miso = SPI2_IOMUX_PIN_NUM_MISO, .mosi = SPI2_IOMUX_PIN_NUM_MOSI},
#ifdef SPI3_IOMUX_PIN_NUM_CLK
	{.sck = SPI3_IOMUX_PIN_NUM_CLK, .miso = SPI3_IOMUX_PIN_NUM_MISO, .mosi = SPI3_IOMUX_PIN_NUM_MOSI},
#else
	{.sck = SPI_PIN_DEFAULT, .miso = SPI_PIN_DEFAULT, .mosi = SPI_PIN_DEFAULT},
#endif
};

struct EspTransaction {
	spi_transaction_ext_t ext;
};

ControllerBase::ControllerBase()
{
	esp_trans = std::make_unique<EspTransaction>();
	dmaBuffer.reset(new uint32_t[hardwareBufferSize / sizeof(uint32_t)]);
}

ControllerBase::~ControllerBase()
{
}

uint8_t ControllerBase::getHost() const
{
	return unsigned(static_cast<const Controller*>(this)->getBusId()) - 1;
}

bool Controller::begin()
{
	if(busId < SpiBus::MIN || busId > SpiBus::MAX) {
		debug_e("[SPI] Invalid bus");
		return false;
	}

	assignDefaultPins(defaultPins[unsigned(busId) - 1]);

	auto getPinValue = [](uint8_t pin) -> int { return (pin == SPI_PIN_NONE) ? -1 : pin; };
	spi_bus_config_t buscfg = {
		.mosi_io_num = getPinValue(pins.mosi),
		.miso_io_num = getPinValue(pins.miso),
		.sclk_io_num = getPinValue(pins.sck),
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0, // Use default
		.flags = 0,
		.intr_flags = ESP_INTR_FLAG_LOWMED, // ESP_INTR_FLAG_IRAM,
	};

	auto err = spi_bus_initialize(spi_host_device_t(unsigned(busId) - 1), &buscfg, SPI_DMA_CH_AUTO);
	if(err != ESP_OK) {
		return false;
	}

	flags.initialised = true;
	return true;
}

void Controller::end()
{
	if(!flags.initialised) {
		return;
	}

	flags.initialised = false;

	// Check all devices have been released
	assert(deviceCount == 0);
}

IoModes Controller::getSupportedIoModes(const Device& dev) const
{
	// Hardware supports all modes
	return dev.getSupportedIoModes();
}

void IRAM_ATTR Controller::pre_transfer_callback(spi_transaction_t* t)
{
	auto self = static_cast<Controller*>(t->user);
	auto req = self->trans.request;
	req->device->transferStarting(*req);
}

void IRAM_ATTR Controller::post_transfer_callback(spi_transaction_t* t)
{
	auto self = static_cast<Controller*>(t->user);
	self->transactionDone();
}

bool Controller::startDevice(Device& dev, PinSet pinSet, uint8_t chipSelect, uint32_t clockSpeed)
{
	if(!flags.initialised) {
		debug_e("SPI Controller not initialised");
		return false;
	}

	if(dev.pinSet != PinSet::none) {
		debug_e("SPI device already started on bus %u, CS #%u", unsigned(busId), dev.chipSelect);
		return false;
	}

	if(pinSet != PinSet::normal) {
		debug_e("[SPI] PinSet not supported");
		return false;
	}

	spi_device_interface_config_t devcfg{
		.mode = uint8_t(dev.getClockMode()),
		.clock_speed_hz = int(clockSpeed),
		.spics_io_num = chipSelect,
		.flags = 0,
		.queue_size = 1,
		.pre_cb = pre_transfer_callback,
		.post_cb = post_transfer_callback,
	};
	auto ioMode = dev.getIoMode();
	if(ioMode == IoMode::SPI) {
		devcfg.flags |= SPI_DEVICE_HALFDUPLEX;
	} else if(ioMode == IoMode::SPI3WIRE) {
		devcfg.flags |= SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE;
	}

	auto err = spi_bus_add_device(spi_host_device_t(unsigned(busId) - 1), &devcfg, &dev.config.handle);
	if(err != ESP_OK) {
		return false;
	}

	++deviceCount;
	dev.pinSet = pinSet;
	dev.chipSelect = chipSelect;
	dev.speed = clockSpeed; // IDF doesn't report back actual clock speed

	debug_i("[SPI] Bus %u, CS #%u acquired", unsigned(busId), chipSelect);
	return true;
}

void Controller::stopDevice(Device& dev)
{
	switch(dev.pinSet) {
	case PinSet::normal:
		assert(deviceCount > 0);
		--deviceCount;
		break;

	case PinSet::none:
		return;

	case PinSet::overlap:
	default:
		assert(false);
		return;
	}

	auto cs = dev.chipSelect;

	auto err = spi_bus_remove_device(spi_device_handle_t(dev.config.handle));
	if(err == ESP_OK) {
		debug_i("[SPI] Bus %u, CS #%u released", unsigned(busId), cs);
	} else {
		debug_e("[SPI] Problem releasing bus %u, CS #%u", unsigned(busId), cs);
	}

	dev.pinSet = PinSet::none;
	dev.chipSelect = 255;
}

void Controller::configChanged(Device& dev)
{
}

void Controller::updateConfig(Device& dev)
{
}

/*
 * With the ESP32 we have both regular FIFO operation and the alternative DMA operation. In both cases
 * a transaction is set up as usual, command, address, etc. with the only difference with the data
 * transfer. It's not only faster but there's no interrupt overhead and the processor doesn't need
 * to do any memory copies. The ESP32 can handle a single transfer of up to 4092 bytes.
 * If a request is larger than that we'll need to repeat it.
 *
 * We must use the ESP32 driver to allow this stack to co-exist with native IDF components such as SPI ethernet.
 * New requests cannot be started from interrupt context, so a task is queued to do this.
 * We probably only need 2 slots in the queue to handle this (one in flight, one being prepared).
 * 
 * Note: Polling mode is not suitable since our interrupt callback handler isn't invoked until `spi_device_polling_end`
 * is called.
 * 
 */
void Controller::execute(Request& req)
{
	if(!flags.initialised || req.device == nullptr || req.device->pinSet == PinSet::none) {
		debug_e("[SPI] Device not initialised");
		return;
	}

	req.next = nullptr;
	req.busy = true;

	// Packet transfer already in progress?
	/*
	   Note: Interrupt needs to be disabled whilst updating the queue.
	   This call does a bit more than that unfortunately.
	   Calling portDISABLE_INTERRUPTS() doesn't do the job.
	 */
	spi_device_acquire_bus(req.device->config.handle, portMAX_DELAY);
	if(trans.busy) {
		// Tack new packet onto end of chain
		auto pkt = trans.request;
		while(pkt->next) {
			pkt = pkt->next;
		}
		pkt->next = &req;
	} else {
		// Not currently running, so do this one now
		trans.request = &req;
		startRequest();
	}
	spi_device_release_bus(req.device->config.handle);

	if(!req.async) {
		// Block and poll
		wait(req);
	}
}

void Controller::wait(Request& request)
{
	if(request.busy) {
#ifdef HSPI_ENABLE_STATS
		CpuCycleTimer timer;
#endif
		do {
		} while(request.busy);
#ifdef HSPI_ENABLE_STATS
		stats.waitCycles += timer.elapsedTicks();
#endif
	}
}

/*
 * Start transfer of a new request (trans.request)
 * May be called from interrupt context at completion of previous request
 */
void IRAM_ATTR Controller::startRequest()
{
	auto& req = *trans.request;
	auto& dev = *req.device;

	if(selectDeviceCallback) {
		selectDeviceCallback(dev.chipSelect, true);
	}
	dev.transferStarting(req);

	trans.addr = req.addr;
	trans.outOffset = 0;
	trans.inOffset = 0;
	trans.inlen = 0;
	trans.ioMode = dev.ioMode;
	trans.bitOrder = dev.bitOrder;
	trans.busy = true;

	auto& t = esp_trans->ext;

	t.base.user = this;
	t.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;

	// TODO: Driver won't let us directly change DUPLEX mode on a per-transaction basis
	// If necessary we can hack this using HAL calls
	switch(trans.ioMode) {
	case IoMode::SPI:
	case IoMode::SPIHD:
	case IoMode::SPI3WIRE:
		break;
	case IoMode::SDI:
	case IoMode::DIO:
		t.base.flags |= SPI_TRANS_MODE_DIO | SPI_TRANS_MODE_DIOQIO_ADDR;
		break;
	case IoMode::DUAL:
		t.base.flags |= SPI_TRANS_MODE_DIO;
		break;
	case IoMode::SQI:
	case IoMode::QIO:
		t.base.flags |= SPI_TRANS_MODE_QIO | SPI_TRANS_MODE_DIOQIO_ADDR;
		break;
	case IoMode::QUAD:
		t.base.flags |= SPI_TRANS_MODE_QIO;
		break;
	default:
		assert(false);
	}

	// Setup command bits
	t.command_bits = req.cmdLen;
	t.base.cmd = req.cmd;

	// Address bits
	t.address_bits = req.addrLen;
	t.base.addr = req.addr;

	// Setup dummy bits
	t.dummy_bits = req.dummyLen;

	nextTransaction();
}

void IRAM_ATTR Controller::nextTransaction()
{
	auto& req = *trans.request;
	auto& dev = *req.device;

	auto& t = esp_trans->ext;

	// If there's too much data to fit in a single transaction, trim it down
	auto sizeAlign = [&](size_t len) {
		if(len <= hardwareBufferSize || req.sizeAlign <= 1) {
			return len;
		}
		return hardwareBufferSize - len % req.sizeAlign;
	};

	// Setup outgoing data (MOSI)
	unsigned outlen = req.out.length - trans.outOffset;
	if(outlen != 0) {
		if(req.out.isPointer) {
			outlen = sizeAlign(outlen);
			auto outptr = req.out.ptr8 + trans.outOffset;
			if(esp_ptr_dma_capable(outptr) && IS_ALIGNED(outptr)) {
				t.base.tx_buffer = outptr;
			} else {
				memcpy(dmaBuffer.get(), outptr, outlen);
				t.base.tx_buffer = dmaBuffer.get();
			}
		} else {
			dmaBuffer[0] = req.out.data32;
			t.base.tx_buffer = dmaBuffer.get();
		}
		t.base.length = outlen * 8;
		trans.outOffset += outlen;
	} else {
		t.base.tx_buffer = nullptr;
		t.base.length = 0;
	}

	// Setup incoming data (MISO)
	unsigned inlen = req.in.length - trans.inOffset;
	if(inlen != 0) {
		if(req.in.isPointer) {
			inlen = sizeAlign(inlen);
			auto inptr = req.in.ptr8 + trans.inOffset;
			if(esp_ptr_dma_capable(inptr) && IS_ALIGNED(inptr)) {
				t.base.rx_buffer = inptr;
			} else {
				t.base.rx_buffer = dmaBuffer.get();
			}
		} else {
			t.base.rx_buffer = dmaBuffer.get();
		}
		trans.inlen = inlen;
		t.base.rxlength = inlen * 8;
		t.base.length = std::max(t.base.length, t.base.rxlength);
	} else {
		t.base.rx_buffer = nullptr;
		t.base.rxlength = 0;
	}

	// Setup address
	t.base.addr = trans.addr;
	trans.addr += std::max(outlen, inlen);

#ifdef HSPI_ENABLE_STATS
	++stats.transCount;
#endif

	// Execute now
	spi_device_queue_trans_from_isr(dev.config.handle, &t.base);
}

/*
 * Read incoming data, if there is any, and start next transaction.
 * Called from interrupt context at completion of transaction.
 */
void IRAM_ATTR Controller::transactionDone()
{
	auto& req = *trans.request;
	auto& dev = *req.device;

	if(selectDeviceCallback) {
		selectDeviceCallback(dev.chipSelect, false);
	}

	// Read incoming data
	if(trans.inlen != 0) {
		if(esp_trans->ext.base.rx_buffer == dmaBuffer.get()) {
			if(req.in.isPointer) {
				memcpy(req.in.ptr8 + trans.inOffset, dmaBuffer.get(), trans.inlen);
			} else {
				req.in.data32 = dmaBuffer[0];
			}
		}
		trans.inOffset += trans.inlen;
		trans.inlen = 0;
	}

	// Packet complete?
	if(trans.inOffset < req.in.length || trans.outOffset < req.out.length) {
		// Nope, continue
		nextTransaction();
		return;
	}

	trans.busy = false;
	req.busy = false;
#ifdef HSPI_ENABLE_STATS
	++stats.requestCount;
#endif

	if(dev.transferComplete(req)) {
		// Note next packet in chain and de-queue this one
		trans.request = req.next;
		req.next = nullptr;
	} else {
		trans.request = reQueueRequest(req.next, &req);
		req.busy = true;
	}

	// Feed the hardware
	if(trans.request != nullptr) {
		startRequest();
	}
}

bool Controller::loopback(bool enable)
{
	if(!flags.initialised) {
		return false;
	}

	auto& sig = spi_periph_signal[unsigned(busId) - 1];
	gpio_matrix_in(enable ? pins.mosi : pins.miso, sig.spiq_in, false);
	return true;
}

} // namespace HSPI
