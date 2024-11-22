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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <HSPI/Controller.h>
#include <HSPI/Device.h>
#include <driver/spi_master.h>
#include <soc/spi_periph.h>
#include <esp_intr_alloc.h>
#include <Platform/Timers.h>
#include <debug_progmem.h>
#include <esp_private/spi_common_internal.h>
#include <soc/gpio_periph.h>
#include <hal/gpio_ll.h>
#include <hal/spi_ll.h>
#include <esp_clk_tree.h>

#pragma GCC diagnostic pop

namespace HSPI
{
constexpr size_t hardwareBufferSize{SPI_MAX_DMA_LEN};

#ifdef HSPI_ENABLE_STATS
volatile Controller::Stats Controller::stats;
#endif

const SpiPins defaultPins[]{
	{
		.sck = SPI_IOMUX_PIN_NUM_CLK,
		.miso = SPI_IOMUX_PIN_NUM_MISO,
		.mosi = SPI_IOMUX_PIN_NUM_MOSI,
		.io2 = SPI_IOMUX_PIN_NUM_WP,
		.io3 = SPI_IOMUX_PIN_NUM_HD,
	},
	{
		.sck = SPI2_IOMUX_PIN_NUM_CLK,
		.miso = SPI2_IOMUX_PIN_NUM_MISO,
		.mosi = SPI2_IOMUX_PIN_NUM_MOSI,
		.io2 = SPI2_IOMUX_PIN_NUM_WP,
		.io3 = SPI2_IOMUX_PIN_NUM_HD,
	},
#ifdef SPI3_IOMUX_PIN_NUM_CLK
	{
		.sck = SPI3_IOMUX_PIN_NUM_CLK,
		.miso = SPI3_IOMUX_PIN_NUM_MISO,
		.mosi = SPI3_IOMUX_PIN_NUM_MOSI,
		.io2 = SPI3_IOMUX_PIN_NUM_WP,
		.io3 = SPI3_IOMUX_PIN_NUM_HD,
	},
#else
	{
		.sck = SPI_PIN_DEFAULT,
		.miso = SPI_PIN_DEFAULT,
		.mosi = SPI_PIN_DEFAULT,
		.io2 = SPI_PIN_DEFAULT,
		.io3 = SPI_PIN_DEFAULT,
	},
#endif
};

#if SOC_NON_CACHEABLE_OFFSET
#define ADDR_DMA_2_CPU(addr) ((typeof(addr))((uint32_t)(addr) + SOC_NON_CACHEABLE_OFFSET))
#define ADDR_CPU_2_DMA(addr) ((typeof(addr))((uint32_t)(addr)-SOC_NON_CACHEABLE_OFFSET))
#else
#define ADDR_DMA_2_CPU(addr) (addr)
#define ADDR_CPU_2_DMA(addr) (addr)
#endif

void dma_desc_setup_link(spi_dma_desc_t* dmadesc, void* data, uint32_t len, bool is_rx)
{
	dmadesc = ADDR_DMA_2_CPU(dmadesc);
	unsigned n = 0;
	auto dataptr = static_cast<uint8_t*>(data);
	while(len) {
		auto& desc = dmadesc[n];
		auto dmachunklen = std::min(len, uint32_t(DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED));
		if(is_rx) {
			// Receive needs DMA length rounded to next 32-bit boundary
			desc.dw0.size = ALIGNUP4(dmachunklen);
			desc.dw0.length = ALIGNUP4(dmachunklen);
		} else {
			desc.dw0.size = dmachunklen;
			desc.dw0.length = dmachunklen;
		}
		desc.buffer = dataptr;
		desc.dw0.suc_eof = 0;
		desc.dw0.owner = 1;
		desc.next = ADDR_CPU_2_DMA(&dmadesc[n + 1]);
		len -= dmachunklen;
		dataptr += dmachunklen;
		n++;
	}
	auto& desc = dmadesc[n - 1];
	desc.dw0.suc_eof = 1; // Mark last DMA desc as end of stream.
	desc.next = nullptr;
}

ControllerBase::ControllerBase()
{
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

	// bool spi_chan_claimed = spicommon_periph_claim(host_id, "spi master");
	// SPI_CHECK(spi_chan_claimed, "host_id already in use", ESP_ERR_INVALID_STATE);

	auto host_id = spi_host_device_t(getHost());

	assignDefaultPins(defaultPins[host_id]);

	auto getPinValue = [](uint8_t pin) -> int { return (pin == SPI_PIN_NONE) ? -1 : pin; };
	spi_bus_config_t buscfg = {
		.mosi_io_num = getPinValue(pins.mosi),
		.miso_io_num = getPinValue(pins.miso),
		.sclk_io_num = getPinValue(pins.sck),
		.quadwp_io_num = getPinValue(pins.io2),
		.quadhd_io_num = getPinValue(pins.io3),
		.intr_flags = ESP_INTR_FLAG_LOWMED, // ESP_INTR_FLAG_IRAM,
	};

	debug_i("[HSPI] host %u, mosi %u, miso %u, sclk %u, io2 %u, io3 %u", host_id, buscfg.mosi_io_num,
			buscfg.miso_io_num, buscfg.sclk_io_num, buscfg.quadwp_io_num, buscfg.quadhd_io_num);

	auto err = spi_bus_initialize(spi_host_device_t(unsigned(busId) - 1), &buscfg, SPI_DMA_CH_AUTO);
	if(err != ESP_OK) {
		return false;
	}

	// interrupts are not allowed on SPI1 bus
	if(host_id != SPI1_HOST) {
		err = esp_intr_alloc(spi_periph_signal[host_id].irq, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED,
							 intr_handler_t(isr), this, &intr_handle);
		debug_i("[HSPI] intr_alloc %u -> %u", spi_periph_signal[host_id].irq, err);
	}

	const spi_bus_attr_t* bus_attr = spi_bus_get_attr(host_id);

	spi_dev_t* hw = SPI_LL_GET_HW(host_id);

	spi_ll_enable_clock(host_id, true);

#if SPI_LL_MOSI_FREE_LEVEL
	// Change default data line level to low which same as esp32
	spi_ll_set_mosi_free_level(hw, 0);
#endif
	spi_ll_master_init(hw);
	spi_dma_ll_rx_enable_burst_data(hw, bus_attr->rx_dma_chan, 1);
	spi_dma_ll_tx_enable_burst_data(hw, bus_attr->tx_dma_chan, 1);
	spi_dma_ll_rx_enable_burst_desc(hw, bus_attr->rx_dma_chan, 1);
	spi_dma_ll_tx_enable_burst_desc(hw, bus_attr->tx_dma_chan, 1);

	spi_ll_enable_int(hw);
	spi_ll_set_int_stat(hw);
	spi_ll_set_mosi_delay(hw, 0, 0);
	spi_ll_apply_config(hw);

	flags.initialised = true;
	return true;
}

void Controller::end()
{
	if(!flags.initialised) {
		return;
	}

	esp_intr_free(intr_handle);
	intr_handle = nullptr;

	auto host_id = spi_host_device_t(getHost());
	spi_bus_free(host_id);

	flags.initialised = false;

	// Check all devices have been released
	assert(deviceCount == 0);
}

IoModes Controller::getSupportedIoModes(const Device& dev) const
{
	// Hardware supports all modes
	return dev.getSupportedIoModes();
}

void IRAM_ATTR Controller::isr(Controller* spi)
{
	spi->transactionDone();
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

	auto host_id = spi_host_device_t(getHost());
	auto bus_attr = spi_bus_get_attr(host_id);

	if(!bus_attr) {
		debug_e("[HSPI] No bus_attr!");
		return false;
	}

	auto& cfg = dev.config;
	cfg.cs_id = 255;
	unsigned num_cs = SOC_SPI_PERIPH_CS_NUM(host_id);
	for(unsigned cs = 0; cs < num_cs; ++cs) {
		if(chipSelectsInUse[cs]) {
			continue;
		}
		cfg.cs_id = cs;
		chipSelectsInUse[cs] = 1;
		break;
	}
	if(cfg.cs_id == 255) {
		debug_e("[HSPI] No free CS");
		return false;
	}

	setClockSpeed(dev, clockSpeed);

	/*
	 * TODO: if we have to change the apb clock among transactions,
	 * re-calculate this each time the apb clock lock is locked.
	 */

	// Set CS pin, CS options
	bool use_gpio = !(bus_attr->flags & SPICOMMON_BUSFLAG_IOMUX_PINS);

	debug_i("[HSPI] host_id %u, cs_pin %u, cs_id %u, use_gpio %u", host_id, chipSelect, cfg.cs_id, use_gpio);
	spicommon_cs_initialize(host_id, chipSelect, cfg.cs_id, use_gpio);

	//
	++deviceCount;
	dev.pinSet = pinSet;
	dev.chipSelect = chipSelect;
	dev.speed = clockSpeed; // IDF doesn't report back actual clock speed

	debug_i("[HSPI] Bus %u, CS #%u acquired", unsigned(busId), chipSelect);

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

	auto& cfg = dev.config;
	chipSelectsInUse[cfg.cs_id] = 0;
	dev.pinSet = PinSet::none;
	dev.chipSelect = 255;
}

void Controller::configChanged(Device& dev)
{
}

void Controller::updateConfig(Device& dev)
{
}

uint32_t Controller::setClockSpeed(Device& dev, uint32_t freq)
{
	auto host_id = spi_host_device_t(getHost());
	auto bus_attr = spi_bus_get_attr(host_id);

	uint32_t clock_source_hz{0};
	esp_clk_tree_src_get_freq_hz(soc_module_clk_t(SPI_CLK_SRC_DEFAULT), ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX,
								 &clock_source_hz);

	// Calculate timing configuration
	auto ioMode = dev.getIoMode();
	spi_hal_timing_param_t timing_param{
		.clk_src_hz = clock_source_hz,
		.half_duplex = !(ioMode == IoMode::SPI || ioMode == IoMode::SPI3WIRE),
		.no_compensate = 0,
		.expected_freq = freq,
		.duty_cycle = 128,
		.input_delay_ns = 0,
		.use_gpio = !(bus_attr->flags & SPICOMMON_BUSFLAG_IOMUX_PINS),
	};
	auto& cfg = dev.config;
	int real_freq;
	auto err = spi_hal_cal_clock_conf(&timing_param, &real_freq, &cfg.timing);
	if(err) {
		debug_e("[HSPI] Unsupported clock speed %u", dev.speed);
	} else {
		debug_i("[HSPI] Requested clock %u, got %d", freq, real_freq);
		cfg.timing.clock_source = SPI_CLK_SRC_DEFAULT;
		dev.speed = real_freq;
	}

	return dev.speed;
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

	// debug_i("[HSPI] Req CMD 0x%x (%u), ADDR 0x%x (%u), DOUT %u, DIN %u", req.cmd, req.cmdLen, req.addr, req.addrLen,
	// 		req.out.length, req.in.length);

	req.next = nullptr;
	req.busy = true;

	// Packet transfer already in progress?
	esp_intr_disable(intr_handle);
	if(trans.busy) {
		debug_i("[HSPI] Queue transaction, cur %p, new %p", trans.request, &req);
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

	if(req.async) {
		esp_intr_enable(intr_handle);
	} else {
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
		spi_dev_t* hw = SPI_LL_GET_HW(getHost());
		do {
			if(spi_ll_usr_is_done(hw)) {
				transactionDone();
			}
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

	// TODO: Driver won't let us directly change DUPLEX mode on a per-transaction basis
	// If necessary we can hack this using HAL calls
	spi_line_mode_t line_mode;
	switch(trans.ioMode) {
	case IoMode::SPI:
	case IoMode::SPIHD:
	case IoMode::SPI3WIRE:
		line_mode = {1, 1, 1};
		break;
	case IoMode::SDI:
	case IoMode::DIO:
		line_mode = {2, 2, 2};
		break;
	case IoMode::DUAL:
		line_mode = {1, 1, 2};
		break;
	case IoMode::SQI:
	case IoMode::QIO:
		line_mode = {4, 4, 4};
		break;
	case IoMode::QUAD:
		line_mode = {1, 1, 4};
		break;
	default:
		assert(false);
	}

	auto host_id = spi_host_device_t(getHost());
	spi_dev_t* hw = SPI_LL_GET_HW(host_id);

	auto& cfg = dev.config;
	bool selected_device_changed = true; // TODO
	bool half_duplex = !(trans.ioMode == IoMode::SPI || trans.ioMode == IoMode::SPI3WIRE);
	bool lsb_first = (trans.bitOrder != MSBFIRST);
	if(selected_device_changed) {
		// Configure clock settings
#if SOC_SPI_AS_CS_SUPPORTED
		spi_ll_master_set_cksel(hw, cfg.cs_id, 0);
#endif
		spi_ll_master_set_pos_cs(hw, cfg.cs_id, 0);
		spi_ll_master_set_clock_by_reg(hw, &cfg.timing.clock_reg);
		// Configure bit order
		spi_ll_set_rx_lsbfirst(hw, lsb_first);
		spi_ll_set_tx_lsbfirst(hw, lsb_first);
		spi_ll_master_set_mode(hw, unsigned(dev.clockMode));
		// Configure misc stuff
		spi_ll_set_half_duplex(hw, half_duplex);
		spi_ll_set_sio_mode(hw, trans.ioMode == IoMode::SPI3WIRE);
		// Configure CS pin and timing
		spi_ll_master_set_cs_setup(hw, 0);
		spi_ll_master_set_cs_hold(hw, 1); // Required in most cases
		spi_ll_master_select_cs(hw, cfg.cs_id);
		// Clock
		spi_ll_set_clk_source(hw, cfg.timing.clock_source);
	}

	// void spi_hal_setup_trans(spi_hal_context_t *hal, const spi_hal_dev_config_t *dev, const spi_hal_trans_config_t *trans)
	// clear int bit
	spi_ll_clear_int_stat(hw);
	// We should be done with the transmission.
	HAL_ASSERT(spi_ll_get_running_cmd(hw) == 0);
	// set transaction line mode
	spi_ll_master_set_line_mode(hw, line_mode);

	int extra_dummy = 0;
	// when no_dummy is not set and in half-duplex mode, sets the dummy bit if RX phase exist
	bool no_compensate = false; // IDF has this as device parameter
	if(req.in.length && !no_compensate && half_duplex) {
		extra_dummy = cfg.timing.timing_dummy;
	}

	// SPI iface needs to be configured for a delay in some cases.
	// configure dummy bits
	spi_ll_set_dummy(hw, extra_dummy + req.dummyLen);

	uint32_t miso_delay_num = 0;
	uint32_t miso_delay_mode = 0;
	if(cfg.timing.timing_miso_delay < 0) {
		// If the data comes too late, delay half a SPI clock to improve reading
		switch(dev.clockMode) {
		case ClockMode::mode0:
			miso_delay_mode = 2;
			break;
		case ClockMode::mode1:
			miso_delay_mode = 1;
			break;
		case ClockMode::mode2:
			miso_delay_mode = 1;
			break;
		case ClockMode::mode3:
			miso_delay_mode = 2;
			break;
		}
		miso_delay_num = 0;
	} else {
		//if the data is so fast that dummy_bit is used, delay some apb clocks to meet the timing
		miso_delay_num = extra_dummy ? cfg.timing.timing_miso_delay : 0;
		miso_delay_mode = 0;
	}
	spi_ll_set_miso_delay(hw, miso_delay_mode, miso_delay_num);

	//Configure bit sizes, load addr and command
	// int cmdlen = trans->cmd_bits;
	// int addrlen = trans->addr_bits;
	// if(!dev->half_duplex && dev->cs_setup != 0) {
	// 	/* The command and address phase is not compatible with cs_ena_pretrans
	//      * in full duplex mode.
	//      */
	// 	cmdlen = 0;
	// 	addrlen = 0;
	// }

	spi_ll_set_addr_bitlen(hw, req.addrLen);

	spi_ll_set_command_bitlen(hw, req.cmdLen);
	spi_ll_set_command(hw, req.cmd, req.cmdLen, lsb_first);

	// Configure keep active CS
	spi_ll_master_keep_cs(hw, 0);

	//
	nextTransaction();
}

void IRAM_ATTR Controller::nextTransaction()
{
	auto& req = *trans.request;
	auto& dev = *req.device;

	// If there's too much data to fit in a single transaction, trim it down
	auto sizeAlign = [&](size_t len) {
		if(len <= hardwareBufferSize) {
			return len;
		}
		if(req.sizeAlign <= 1) {
			return hardwareBufferSize;
		}
		return hardwareBufferSize - len % req.sizeAlign;
	};

	// Setup outgoing data (MOSI)
	uint32_t tx_bitlen{0};
	unsigned outlen = req.out.length - trans.outOffset;
	if(outlen != 0) {
		if(req.out.isPointer) {
			outlen = sizeAlign(outlen);
			auto outptr = req.out.ptr8 + trans.outOffset;
			if(esp_ptr_dma_capable(outptr) && IS_ALIGNED(outptr)) {
				trans.tx_buffer = outptr;
			} else {
				memcpy(dmaBuffer.get(), outptr, outlen);
				trans.tx_buffer = dmaBuffer.get();
			}
		} else {
			dmaBuffer[0] = req.out.data32;
			trans.tx_buffer = dmaBuffer.get();
		}
		tx_bitlen = outlen * 8;
		trans.outOffset += outlen;
	} else {
		trans.tx_buffer = nullptr;
		tx_bitlen = 0;
	}

	// Setup incoming data (MISO)
	uint32_t rx_bitlen{0};
	unsigned inlen = req.in.length - trans.inOffset;
	if(inlen != 0) {
		if(req.in.isPointer) {
			inlen = sizeAlign(inlen);
			auto inptr = req.in.ptr8 + trans.inOffset;
			if(esp_ptr_dma_capable(inptr) && IS_ALIGNED(inptr)) {
				trans.rx_buffer = inptr;
			} else {
				trans.rx_buffer = dmaBuffer.get();
			}
		} else {
			trans.rx_buffer = dmaBuffer.get();
		}
		trans.inlen = inlen;
		rx_bitlen = inlen * 8;
		// t.base.length = std::max(t.base.length, t.base.rxlength);
	} else {
		trans.rx_buffer = nullptr;
		trans.inlen = 0;
		rx_bitlen = 0;
	}

	bool half_duplex = !(trans.ioMode == IoMode::SPI || trans.ioMode == IoMode::SPI3WIRE);

	auto host_id = spi_host_device_t(getHost());
	spi_dev_t* hw = SPI_LL_GET_HW(host_id);

	spi_ll_set_mosi_bitlen(hw, tx_bitlen);
	if(half_duplex) {
		spi_ll_set_miso_bitlen(hw, std::max(tx_bitlen, rx_bitlen));
	} else {
		spi_ll_set_miso_bitlen(hw, rx_bitlen);
	}

	// Setup address
	bool lsb_first = (trans.bitOrder != MSBFIRST);
	spi_ll_set_address(hw, trans.addr, req.addrLen, lsb_first);
	trans.addr += std::max(outlen, inlen);

#ifdef HSPI_ENABLE_STATS
	++stats.transCount;
#endif

	// Fill DMA descriptors
	const spi_bus_attr_t* bus_attr = spi_bus_get_attr(host_id);

#if CONFIG_IDF_TARGET_ESP32
	if(trans.rx_buffer || trans.tx_buffer) {
		// mark channel as active, so that the DMA will not be reset by the slave
		// This workaround is only for esp32, where tx_dma_chan and rx_dma_chan are always same
		spicommon_dmaworkaround_transfer_active(bus_attr->tx_dma_chan);
	}
#endif

	if(trans.rx_buffer) {
		// debug_i("[HSPI] RX DMA %u", inlen);
		dma_desc_setup_link(bus_attr->dmadesc_rx, trans.rx_buffer, inlen, true);
		spi_dma_ll_rx_reset(hw, bus_attr->rx_dma_chan);
		spi_ll_dma_rx_fifo_reset(hw);
		spi_ll_infifo_full_clr(hw);
		spi_ll_dma_rx_enable(hw, 1);
		spi_dma_ll_rx_start(hw, bus_attr->rx_dma_chan, (lldesc_t*)bus_attr->dmadesc_rx);
	} else {
#if CONFIG_IDF_TARGET_ESP32
		// DMA temporary workaround: let RX DMA work somehow to avoid the issue in ESP32 v0/v1 silicon
		if(!half_duplex) {
			spi_ll_dma_rx_enable(hw, 1);
			spi_dma_ll_rx_start(hw, bus_attr->rx_dma_chan, 0);
		}
#endif
	}

	if(trans.tx_buffer) {
		// debug_i("[HSPI] TX DMA %u", outlen);
		dma_desc_setup_link(bus_attr->dmadesc_tx, trans.tx_buffer, outlen, false);
		spi_dma_ll_tx_reset(hw, bus_attr->tx_dma_chan);
		spi_ll_dma_tx_fifo_reset(hw);
		spi_ll_outfifo_empty_clr(hw);
		spi_ll_dma_tx_enable(hw, 1);
		spi_dma_ll_tx_start(hw, bus_attr->tx_dma_chan, (lldesc_t*)bus_attr->dmadesc_tx);
	}

	// in ESP32 these registers should be configured after the DMA is set
	if((!half_duplex && trans.rx_buffer) || trans.tx_buffer) {
		// debug_i("[HSPI] ENABLE MOSI");
		spi_ll_enable_mosi(hw, 1);
	} else {
		// debug_i("[HSPI] DISABLE MOSI");
		spi_ll_enable_mosi(hw, 0);
	}
	// debug_i("[HSPI] rx_buffer %p", trans.rx_buffer);
	spi_ll_enable_miso(hw, trans.rx_buffer ? 1 : 0);

	//
	dev.transferStarting(req);

	// Kick off transfer
	spi_ll_apply_config(hw);
	spi_ll_user_start(hw);
}

/*
 * Read incoming data, if there is any, and start next transaction.
 * Called from interrupt context at completion of transaction.
 */
void IRAM_ATTR Controller::transactionDone()
{
	auto host_id = spi_host_device_t(getHost());
	spi_dev_t* hw = SPI_LL_GET_HW(host_id);
	assert(spi_ll_usr_is_done(hw));
	spi_ll_clear_int_stat(hw);

	if(trans.request == nullptr) {
		return;
	}

#if CONFIG_IDF_TARGET_ESP32
	// This workaround is only for esp32, where tx_dma_chan and rx_dma_chan are always same
	const spi_bus_attr_t* bus_attr = spi_bus_get_attr(host_id);
	spicommon_dmaworkaround_idle(bus_attr->tx_dma_chan);
#endif

	auto& req = *trans.request;
	auto& dev = *req.device;

	if(selectDeviceCallback) {
		selectDeviceCallback(dev.chipSelect, false);
	}

	// Read incoming data
	if(trans.inlen != 0) {
		if(trans.rx_buffer == dmaBuffer.get()) {
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
