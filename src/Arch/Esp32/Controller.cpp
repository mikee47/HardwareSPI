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
#ifdef HSPI_ENABLE_STATS
volatile Controller::Stats Controller::stats;
#endif

/*
 * Aligned data can be transferred using linked DMA transfers.
 * The length of this list is determined by this value.
 * See `dma_desc_setup_link`.
 */
constexpr size_t maxDmaTransferSize{0x10000};

/*
 * Mis-aligned data is handled by copying to an internal buffer.
 */
constexpr size_t hardwareBufferSize{SPI_MAX_DMA_LEN};

const HSPI::SpiPins defaultPins[]{
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

uint8_t __forceinline IRAM_ATTR ControllerBase::getHost() const
{
	return unsigned(static_cast<const Controller*>(this)->getBusId()) - 1;
}

bool Controller::begin()
{
	if(busId < SpiBus::MIN || busId > SpiBus::MAX) {
		debug_e("[SPI] Invalid bus");
		return false;
	}

	auto host_id = spi_host_device_t(getHost());

	assignDefaultPins(defaultPins[host_id]);

	auto getPinValue = [](uint8_t pin) -> int { return (pin == SPI_PIN_NONE) ? -1 : pin; };
	spi_bus_config_t buscfg = {
		.mosi_io_num = getPinValue(pins.mosi),
		.miso_io_num = getPinValue(pins.miso),
		.sclk_io_num = getPinValue(pins.sck),
		.quadwp_io_num = getPinValue(pins.io2),
		.quadhd_io_num = getPinValue(pins.io3),
		.max_transfer_sz = maxDmaTransferSize,
		.flags = SPICOMMON_BUSFLAG_MASTER,
		.intr_flags = ESP_INTR_FLAG_IRAM,
	};

	debug_i("[HSPI] host %u, mosi %u, miso %u, sclk %u, io2 %u, io3 %u", host_id, buscfg.mosi_io_num,
			buscfg.miso_io_num, buscfg.sclk_io_num, buscfg.quadwp_io_num, buscfg.quadhd_io_num);

	auto err = spi_bus_initialize(host_id, &buscfg, SPI_DMA_CH_AUTO);
	if(err != ESP_OK) {
		return false;
	}

	// interrupts are not allowed on SPI1 bus
	if(host_id != SPI1_HOST) {
		// Use priority level >= LOWMED (1-3) so `queueFromISR` works OK
		err = esp_intr_alloc(spi_periph_signal[host_id].irq,
							 ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_INTRDISABLED,
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

	// Set CS pin, CS options
	bool use_gpio = !(bus_attr->flags & SPICOMMON_BUSFLAG_IOMUX_PINS);

	debug_i("[HSPI] host_id %u, cs_pin %u, cs_id %u, use_gpio %u", host_id, chipSelect, cfg.cs_id, use_gpio);
	spicommon_cs_initialize(host_id, chipSelect, cfg.cs_id, use_gpio);

	//
	++deviceCount;
	dev.pinSet = pinSet;
	dev.chipSelect = chipSelect;

	setClockSpeed(dev, clockSpeed);

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
	dev.config.changed = true;
}

void Controller::updateConfig(Device&)
{
}

uint32_t Controller::setClockSpeed(Device& dev, uint32_t freq)
{
	if(dev.config.requestedSpeed == freq) {
		return dev.speed;
	}

	auto host_id = spi_host_device_t(getHost());
	auto bus_attr = spi_bus_get_attr(host_id);

	uint32_t clock_source_hz{0};
	esp_clk_tree_src_get_freq_hz(soc_module_clk_t(SPI_CLK_SRC_DEFAULT), ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX,
								 &clock_source_hz);

	// Calculate timing configuration
	auto ioMode = dev.getIoMode();
	spi_hal_timing_param_t timing_param{
		.clk_src_hz = clock_source_hz,
		.half_duplex = !isDuplex(ioMode),
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
		cfg.changed = true;
	}

	return dev.speed;
}

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
	if(interruptsEnabled) {
		esp_intr_disable(intr_handle);
		interruptsEnabled = false;
	}
	if(trans.busy) {
		debug_d("[HSPI] Queue transaction, cur %p, new %p", trans.request, &req);
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
		interruptsEnabled = true;
	} else {
		// Block and poll
		wait(req);
	}
}

bool IRAM_ATTR Controller::queueFromISR(Request& req)
{
	if(!flags.initialised || req.device == nullptr || req.device->pinSet == PinSet::none) {
		return false;
	}

	if(req.busy || !req.async) {
		return false;
	}

	req.next = nullptr;
	req.busy = true;

	if(interruptsEnabled) {
		esp_intr_disable(intr_handle);
	}
	// Packet transfer already in progress?
	if(trans.busy) {
		// Tack new packet onto end of chain
		auto pkt = trans.request;
		while(pkt->next) {
			pkt = pkt->next;
		}
		pkt->next = &req;
		if(interruptsEnabled) {
			esp_intr_enable(intr_handle);
		}
		return true;
	}

	// Start this request now
	trans.request = &req;
	startRequest();
	esp_intr_enable(intr_handle);
	interruptsEnabled = true;
	return true;
}

void Controller::wait(Request& request)
{
	if(request.busy) {
#ifdef HSPI_ENABLE_STATS
		CpuCycleTimer timer;
#endif
		spi_dev_t* hw = SPI_LL_GET_HW(getHost());
		do {
			if(!interruptsEnabled && spi_ll_usr_is_done(hw)) {
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
	auto& cfg = dev.config;

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

	bool activeDeviceChanged = (&dev != activeDevice);
	activeDevice = &dev;
	bool configChanged = activeDeviceChanged || cfg.changed;
	cfg.changed = false;

	spi_dev_t* hw = SPI_LL_GET_HW(getHost());

	// We should be done with the transmission
	spi_ll_clear_int_stat(hw);
	assert(spi_ll_get_running_cmd(hw) == 0);

	bool half_duplex = !isDuplex(trans.ioMode);
	if(configChanged) {
		// Configure clock settings
#if SOC_SPI_AS_CS_SUPPORTED
		spi_ll_master_set_cksel(hw, cfg.cs_id, 0);
#endif
		spi_ll_master_set_pos_cs(hw, cfg.cs_id, 0);
		spi_ll_master_set_clock_by_reg(hw, &cfg.timing.clock_reg);
		// Configure bit order
		spi_ll_set_rx_lsbfirst(hw, trans.bitOrder != MSBFIRST);
		spi_ll_set_tx_lsbfirst(hw, trans.bitOrder != MSBFIRST);
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
		// Configure keep active CS
		spi_ll_master_keep_cs(hw, 0);

		// set transaction line mode
		spi_line_mode_t line_mode{};
		switch(trans.ioMode) {
		case IoMode::SPI:
		case IoMode::SPIHD:
		case IoMode::SPI3WIRE:
			line_mode = {1, 1, 1};
			break;
		case IoMode::DUAL:
			line_mode = {1, 1, 2};
			break;
		case IoMode::DIO:
			line_mode = {1, 2, 2};
			break;
		case IoMode::SDI:
			line_mode = {2, 2, 2};
			break;
		case IoMode::QUAD:
			line_mode = {1, 1, 4};
			break;
		case IoMode::QIO:
			line_mode = {1, 4, 4};
			break;
		case IoMode::SQI:
			line_mode = {4, 4, 4};
			break;
		}
		spi_ll_master_set_line_mode(hw, line_mode);
	}

	// SPI iface needs to be configured for a delay in some cases
	int extra_dummy = 0;
	// when no_dummy is not set and in half-duplex mode, sets the dummy bit if RX phase exist
	bool no_compensate = false; // IDF has this as device parameter
	if(req.in.length && !no_compensate && half_duplex) {
		extra_dummy = cfg.timing.timing_dummy;
	}
	spi_ll_set_dummy(hw, extra_dummy + req.dummyLen);

#if 0
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
#endif

	spi_ll_set_addr_bitlen(hw, req.addrLen);

	spi_ll_set_command_bitlen(hw, req.cmdLen);
	spi_ll_set_command(hw, req.cmd, req.cmdLen, trans.bitOrder != MSBFIRST);

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
			auto outptr = req.out.ptr8 + trans.outOffset;
			if(esp_ptr_dma_capable(outptr) && IS_ALIGNED(outptr)) {
				outlen = std::min(outlen, maxDmaTransferSize);
				trans.tx_buffer = outptr;
			} else {
				outlen = sizeAlign(outlen);
				memcpy(dmaBuffer.get(), outptr, outlen);
				trans.tx_buffer = dmaBuffer.get();
			}
		} else {
			trans.tx_buffer = &req.out.data32;
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
			auto inptr = req.in.ptr8 + trans.inOffset;
			if(esp_ptr_dma_capable(inptr) && IS_ALIGNED(inptr)) {
				inlen = std::min(inlen, maxDmaTransferSize);
				trans.rx_buffer = inptr;
			} else {
				inlen = sizeAlign(inlen);
				trans.rx_buffer = dmaBuffer.get();
			}
		} else {
			trans.rx_buffer = &req.in.data32;
		}
		trans.inlen = inlen;
		rx_bitlen = inlen * 8;
		// t.base.length = std::max(t.base.length, t.base.rxlength);
	} else {
		trans.rx_buffer = nullptr;
		trans.inlen = 0;
		rx_bitlen = 0;
	}

	bool half_duplex = !isDuplex(trans.ioMode);

	auto host_id = spi_host_device_t(getHost());
	spi_dev_t* hw = SPI_LL_GET_HW(host_id);

	spi_ll_set_mosi_bitlen(hw, tx_bitlen);
	if(half_duplex) {
		spi_ll_set_miso_bitlen(hw, std::max(tx_bitlen, rx_bitlen));
	} else {
		spi_ll_set_miso_bitlen(hw, rx_bitlen);
	}

	// Setup address
	spi_ll_set_address(hw, trans.addr, req.addrLen, trans.bitOrder != MSBFIRST);
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
