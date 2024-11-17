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
#include <esp_private/spi_common_internal.h>
#include <soc/gpio_periph.h>
#include <hal/gpio_ll.h>

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

struct EspTransaction {
	spi_transaction_ext_t ext;
};

#if 0
static esp_err_t alloc_dma_chan(spi_host_device_t host_id, gdma_channel_handle_t& tx_channel,
								gdma_channel_handle_t& rx_channel)
{
#if SOC_GDMA_SUPPORTED
	assert(is_valid_host(host_id));
	assert(dma_chan == SPI_DMA_CH_AUTO);

	gdma_channel_alloc_config_t tx_alloc_config = {
		.flags.reserve_sibling = 1,
		.direction = GDMA_CHANNEL_DIRECTION_TX,
	};
	ESP_RETURN_ON_ERROR(SPI_GDMA_NEW_CHANNEL(&tx_alloc_config, &tx_channel), SPI_TAG, "alloc gdma tx failed");

	gdma_channel_alloc_config_t rx_alloc_config = {
		.direction = GDMA_CHANNEL_DIRECTION_RX,
		.sibling_chan = ctx->tx_channel,
	};
	ESP_RETURN_ON_ERROR(SPI_GDMA_NEW_CHANNEL(&rx_alloc_config, &rx_channel), SPI_TAG, "alloc gdma rx failed");

	if(host_id == SPI2_HOST) {
		gdma_connect(ctx->rx_channel, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_SPI, 2));
		gdma_connect(ctx->tx_channel, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_SPI, 2));
	}
#if(SOC_SPI_PERIPH_NUM >= 3)
	else if(host_id == SPI3_HOST) {
		gdma_connect(ctx->rx_channel, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_SPI, 3));
		gdma_connect(ctx->tx_channel, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_SPI, 3));
	}
#endif
	gdma_get_channel_id(ctx->tx_channel, (int*)out_actual_tx_dma_chan);
	gdma_get_channel_id(ctx->rx_channel, (int*)out_actual_rx_dma_chan);

#else // SOC_GDMA_SUPPORTED

	bool success = false;
	uint32_t actual_dma_chan;
#if CONFIG_IDF_TARGET_ESP32
	for(unsigned i = 1; i < SOC_SPI_DMA_CHAN_NUM + 1; i++) {
		success = claim_dma_chan(i, &actual_dma_chan);
		if(success) {
			break;
		}
	}
#elif CONFIG_IDF_TARGET_ESP32S2
	// On ESP32S2, each SPI controller has its own DMA channel
	success = claim_dma_chan(host_id, &actual_dma_chan);
#endif //#if CONFIG_IDF_TARGET_XXX

	//On ESP32 and ESP32S2, actual_tx_dma_chan and actual_rx_dma_chan are always same
	*out_actual_tx_dma_chan = actual_dma_chan;
	*out_actual_rx_dma_chan = actual_dma_chan;

	if(!success) {
		debug_e("[HSPI] no available dma channel");
		return ESP_ERR_NOT_FOUND;
	}

	connect_spi_and_dma(host_id, *out_actual_tx_dma_chan);

#endif

	return ESP_OK;
}

#endif

static bool check_iomux_pins(spi_host_device_t host, const spi_bus_config_t* bus_config)
{
	auto& sig = spi_periph_signal[host];
	if(bus_config->sclk_io_num >= 0 && bus_config->sclk_io_num != sig.spiclk_iomux_pin) {
		return false;
	}
	if(bus_config->quadwp_io_num >= 0 && bus_config->quadwp_io_num != sig.spiwp_iomux_pin) {
		return false;
	}
	if(bus_config->quadhd_io_num >= 0 && bus_config->quadhd_io_num != sig.spihd_iomux_pin) {
		return false;
	}
	if(bus_config->mosi_io_num >= 0 && bus_config->mosi_io_num != sig.spid_iomux_pin) {
		return false;
	}
	if(bus_config->miso_io_num >= 0 && bus_config->miso_io_num != sig.spiq_iomux_pin) {
		return false;
	}
	return true;
}

esp_err_t spicommon_bus_initialize_io(spi_host_device_t host, const spi_bus_config_t* bus_config)
{
	// Check if the selected pins correspond to the iomux pins of the peripheral
	bool use_iomux = check_iomux_pins(host, bus_config);

	if(use_iomux) {
		if(bus_config->mosi_io_num >= 0) {
			gpio_iomux_in(bus_config->mosi_io_num, spi_periph_signal[host].spid_in);
			gpio_iomux_out(bus_config->mosi_io_num, spi_periph_signal[host].func, false);
		}
		if(bus_config->miso_io_num >= 0) {
			gpio_iomux_in(bus_config->miso_io_num, spi_periph_signal[host].spiq_in);
			gpio_iomux_out(bus_config->miso_io_num, spi_periph_signal[host].func, false);
		}
		if(bus_config->quadwp_io_num >= 0) {
			gpio_iomux_in(bus_config->quadwp_io_num, spi_periph_signal[host].spiwp_in);
			gpio_iomux_out(bus_config->quadwp_io_num, spi_periph_signal[host].func, false);
		}
		if(bus_config->quadhd_io_num >= 0) {
			gpio_iomux_in(bus_config->quadhd_io_num, spi_periph_signal[host].spihd_in);
			gpio_iomux_out(bus_config->quadhd_io_num, spi_periph_signal[host].func, false);
		}
		if(bus_config->sclk_io_num >= 0) {
			gpio_iomux_in(bus_config->sclk_io_num, spi_periph_signal[host].spiclk_in);
			gpio_iomux_out(bus_config->sclk_io_num, spi_periph_signal[host].func, false);
		}
		return ESP_OK;
	}

	// Use GPIO matrix
	if(bus_config->mosi_io_num >= 0) {
		gpio_set_direction((gpio_num_t)bus_config->mosi_io_num, GPIO_MODE_INPUT_OUTPUT);
		esp_rom_gpio_connect_out_signal(bus_config->mosi_io_num, spi_periph_signal[host].spid_out, false, false);
		esp_rom_gpio_connect_in_signal(bus_config->mosi_io_num, spi_periph_signal[host].spid_in, false);
#if CONFIG_IDF_TARGET_ESP32S2
		PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[bus_config->mosi_io_num]);
#endif
		gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[bus_config->mosi_io_num], PIN_FUNC_GPIO);
	}
	if(bus_config->miso_io_num >= 0) {
		gpio_set_direction((gpio_num_t)bus_config->miso_io_num, GPIO_MODE_INPUT);
		esp_rom_gpio_connect_in_signal(bus_config->miso_io_num, spi_periph_signal[host].spiq_in, false);
#if CONFIG_IDF_TARGET_ESP32S2
		PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[bus_config->miso_io_num]);
#endif
		gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[bus_config->miso_io_num], PIN_FUNC_GPIO);
	}
	if(bus_config->quadwp_io_num >= 0) {
		gpio_set_direction((gpio_num_t)bus_config->quadwp_io_num, GPIO_MODE_INPUT_OUTPUT);
		esp_rom_gpio_connect_out_signal(bus_config->quadwp_io_num, spi_periph_signal[host].spiwp_out, false, false);
		esp_rom_gpio_connect_in_signal(bus_config->quadwp_io_num, spi_periph_signal[host].spiwp_in, false);
#if CONFIG_IDF_TARGET_ESP32S2
		PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[bus_config->quadwp_io_num]);
#endif
		gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[bus_config->quadwp_io_num], PIN_FUNC_GPIO);
	}
	if(bus_config->quadhd_io_num >= 0) {
		gpio_set_direction((gpio_num_t)bus_config->quadhd_io_num, GPIO_MODE_INPUT_OUTPUT);
		esp_rom_gpio_connect_out_signal(bus_config->quadhd_io_num, spi_periph_signal[host].spihd_out, false, false);
		esp_rom_gpio_connect_in_signal(bus_config->quadhd_io_num, spi_periph_signal[host].spihd_in, false);
#if CONFIG_IDF_TARGET_ESP32S2
		PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[bus_config->quadhd_io_num]);
#endif
		gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[bus_config->quadhd_io_num], PIN_FUNC_GPIO);
	}
	if(bus_config->sclk_io_num >= 0) {
		gpio_set_direction((gpio_num_t)bus_config->sclk_io_num, GPIO_MODE_INPUT_OUTPUT);
		esp_rom_gpio_connect_out_signal(bus_config->sclk_io_num, spi_periph_signal[host].spiclk_out, false, false);
		esp_rom_gpio_connect_in_signal(bus_config->sclk_io_num, spi_periph_signal[host].spiclk_in, false);
#if CONFIG_IDF_TARGET_ESP32S2
		PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[bus_config->sclk_io_num]);
#endif
		gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[bus_config->sclk_io_num], PIN_FUNC_GPIO);
	}

	return ESP_OK;
}

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
		.max_transfer_sz = 0, // Use default
		.flags = 0,
		.intr_flags = ESP_INTR_FLAG_LOWMED, // ESP_INTR_FLAG_IRAM,
	};

	uint32_t tx_channel;
	uint32_t rx_channel;
	auto err = spicommon_dma_chan_alloc(host_id, SPI_DMA_CH_AUTO, &tx_channel, &rx_channel);
	// 	auto err = spi_bus_initialize(spi_host_device_t(unsigned(busId) - 1), &buscfg, SPI_DMA_CH_AUTO);
	if(err != ESP_OK) {
		debug_e("[HSPI] DMA allocation failed");
		return false;
	}

	spicommon_bus_initialize_io(host_id);

	// interrupts are not allowed on SPI1 bus
	if(host_id != SPI1_HOST) {
		err = esp_intr_alloc(spi_periph_signal[host_id].irq, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED, spi_intr,
							 host, &host->intr);
		if(err != ESP_OK) {
			goto cleanup;
		}
	}

	spi_dev_t* hw = SPI_LL_GET_HW(host_id);

	//assign the SPI, RX DMA and TX DMA peripheral registers beginning address
	spi_hal_config_t hal_config{
		// On ESP32-S2 and earlier chips, DMA registers are part of SPI registers. Pass the registers of SPI peripheral to control it.
		.dma_enabled = bus_attr->dma_enabled, .dmadesc_tx = bus_attr->dmadesc_tx,   .dmadesc_rx = bus_attr->dmadesc_rx,
		.tx_dma_chan = bus_attr->tx_dma_chan, .rx_dma_chan = bus_attr->rx_dma_chan, .dmadesc_n = bus_attr->dma_desc_num,
	};
	spi_ll_enable_clock(host_id, true);

#if SPI_LL_MOSI_FREE_LEVEL
	// Change default data line level to low which same as esp32
	spi_ll_set_mosi_free_level(hw, 0);
#endif
	spi_ll_master_init(hw);
	spi_dma_ll_rx_enable_burst_data(hw, rx_dma_chan, 1);
	spi_dma_ll_tx_enable_burst_data(hw, tx_dma_chan, 1);
	spi_dma_ll_rx_enable_burst_desc(hw, rx_dma_chan, 1);
	spi_dma_ll_tx_enable_burst_desc(hw, tx_dma_chan, 1);

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

	flags.initialised = false;

	// Check all devices have been released
	assert(deviceCount == 0);
}

IoModes Controller::getSupportedIoModes(const Device& dev) const
{
	// Hardware supports all modes
	return dev.getSupportedIoModes();
}

void IRAM_ATTR isr(Controller* spi)
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

	spi_device_interface_config_t devcfg{
		.mode = uint8_t(dev.getClockMode()),
		.clock_speed_hz = int(clockSpeed),
		.spics_io_num = (chipSelect == SPI_PIN_NONE) ? GPIO_NUM_NC : chipSelect,
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

	// auto err = spi_bus_add_device(spi_host_device_t(unsigned(busId) - 1), &devcfg, &dev.config.handle);
	// if(err != ESP_OK) {
	// 	return false;
	// }

	spi_device_t* dev = NULL;
	esp_err_t err = ESP_OK;

	uint32_t clock_source_hz = 0;
	esp_clk_tree_src_get_freq_hz(SPI_CLK_SRC_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX, &clock_source_hz);
	// SPI_CHECK((dev_config->clock_speed_hz > 0) && (dev_config->clock_speed_hz <= clock_source_hz), "invalid sclk speed", ESP_ERR_INVALID_ARG);

	int freecs = 0; // spi_bus_lock_get_dev_id(dev_handle);
	// SPI_CHECK(freecs != -1, "no free cs pins for the host", ESP_ERR_NOT_FOUND);

	//input parameters to calculate timing configuration
	int half_duplex = dev_config->flags & SPI_DEVICE_HALFDUPLEX ? 1 : 0;
	int no_compensate = dev_config->flags & SPI_DEVICE_NO_DUMMY ? 1 : 0;
	int duty_cycle = (dev_config->duty_cycle_pos == 0) ? 128 : dev_config->duty_cycle_pos;
	int use_gpio = !(bus_attr->flags & SPICOMMON_BUSFLAG_IOMUX_PINS);
	spi_hal_timing_param_t timing_param{
		.half_duplex = half_duplex,
		.no_compensate = no_compensate,
		.clk_src_hz = clock_source_hz,
		.expected_freq = dev_config->clock_speed_hz,
		.duty_cycle = duty_cycle,
		.input_delay_ns = dev_config->input_delay_ns,
		.use_gpio = use_gpio,
	};
	spi_hal_timing_conf_t temp_timing_conf;
	int freq;
	esp_err_t ret = spi_hal_cal_clock_conf(&timing_param, &freq, &temp_timing_conf);
	temp_timing_conf.clock_source = clk_src;
	SPI_CHECK(ret == ESP_OK, "assigned clock speed not supported", ret);

	// dev->id = freecs;

	// dev->cfg.duty_cycle_pos = duty_cycle;
	// dev->real_clk_freq_hz = freq;
	// TODO: if we have to change the apb clock among transactions, re-calculate this each time the apb clock lock is locked.

	// Set CS pin, CS options
	if(dev_config->spics_io_num >= 0) {
		spicommon_cs_initialize(host_id, dev_config->spics_io_num, freecs, use_gpio);
	}

	// initialise the device specific configuration
	spi_hal_dev_config_t* hal_dev = &(dev->hal_dev);
	hal_dev->mode = dev_config->mode;
	hal_dev->cs_setup = dev_config->cs_ena_pretrans;
	hal_dev->cs_hold = dev_config->cs_ena_posttrans;
	//set hold_time to 0 will not actually append delay to CS
	//set it to 1 since we do need at least one clock of hold time in most cases
	if(hal_dev->cs_hold == 0) {
		hal_dev->cs_hold = 1;
	}
	hal_dev->cs_pin_id = dev->id;
	hal_dev->timing_conf = temp_timing_conf;
	hal_dev->sio = (dev_config->flags) & SPI_DEVICE_3WIRE ? 1 : 0;
	hal_dev->half_duplex = dev_config->flags & SPI_DEVICE_HALFDUPLEX ? 1 : 0;
	hal_dev->tx_lsbfirst = dev_config->flags & SPI_DEVICE_TXBIT_LSBFIRST ? 1 : 0;
	hal_dev->rx_lsbfirst = dev_config->flags & SPI_DEVICE_RXBIT_LSBFIRST ? 1 : 0;
	hal_dev->no_compensate = dev_config->flags & SPI_DEVICE_NO_DUMMY ? 1 : 0;
#if SOC_SPI_AS_CS_SUPPORTED
	hal_dev->as_cs = dev_config->flags & SPI_DEVICE_CLK_AS_CS ? 1 : 0;
#endif
	hal_dev->positive_cs = dev_config->flags & SPI_DEVICE_POSITIVE_CS ? 1 : 0;

	//
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

	int spics_io_num = handle->cfg.spics_io_num;
	if(spics_io_num >= 0) {
		spicommon_cs_free_io(spics_io_num);
	}

	// auto err = spi_bus_remove_device(spi_device_handle_t(dev.config.handle));
	// dev.config.handle = nullptr;
	if(err == ESP_OK) {
		debug_i("[SPI] Bus %u, CS #%u released", unsigned(busId), dev.chipSelect);
	} else {
		debug_e("[SPI] Problem releasing bus %u, CS #%u", unsigned(busId), dev.chipSelect);
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

uint32_t Controller::setClockSpeed(Device& dev, uint32_t freq)
{
	if(dev.config.handle) {
		// Need to remove and re-initialise the device
		auto pinSet = dev.pinSet;
		auto chipSelect = dev.chipSelect;
		// TODO
	} else {
		dev.speed = freq;
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

	req.next = nullptr;
	req.busy = true;

	// Packet transfer already in progress?
	/*
	   Note: Interrupt needs to be disabled whilst updating the queue.
	   This call does a bit more than that unfortunately.
	   Calling portDISABLE_INTERRUPTS() doesn't do the job.
	 */
	// auto err = spi_device_acquire_bus(req.device->config.handle, portMAX_DELAY);
	// if(err) {
	// 	debug_e("[HSPI] spi_device_acquire_bus failed, %d", err);
	// }
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
		if(errcode) {
			debug_e("[HSPI] TRANSACTION FAILURE %d", errcode);
		}
	}
	// spi_device_release_bus(req.device->config.handle);

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

	spi_dev_t* hw = hal->hw;

	// errcode = spi_device_queue_trans_from_isr(dev.config.handle, &t.base);
	if(selected device changed) {
		// void spi_hal_setup_device(spi_hal_context_t *hal, const spi_hal_dev_config_t *dev)
		// Configure clock settings
#if SOC_SPI_AS_CS_SUPPORTED
		spi_ll_master_set_cksel(hw, dev->cs_pin_id, dev->as_cs);
#endif
		spi_ll_master_set_pos_cs(hw, dev->cs_pin_id, dev->positive_cs);
		spi_ll_master_set_clock_by_reg(hw, &dev->timing_conf.clock_reg);
		// Configure bit order
		spi_ll_set_rx_lsbfirst(hw, dev->rx_lsbfirst);
		spi_ll_set_tx_lsbfirst(hw, dev->tx_lsbfirst);
		spi_ll_master_set_mode(hw, dev->mode);
		// Configure misc stuff
		spi_ll_set_half_duplex(hw, dev->half_duplex);
		spi_ll_set_sio_mode(hw, dev->sio);
		// Configure CS pin and timing
		spi_ll_master_set_cs_setup(hw, dev->cs_setup);
		spi_ll_master_set_cs_hold(hw, dev->cs_hold);
		spi_ll_master_select_cs(hw, dev->cs_pin_id);
		// Clock
		spi_ll_set_clk_source(hal->hw, hal_dev->timing_conf.clock_source);
	}

	// void spi_hal_setup_trans(spi_hal_context_t *hal, const spi_hal_dev_config_t *dev, const spi_hal_trans_config_t *trans)
	// clear int bit
	spi_ll_clear_int_stat(hal->hw);
	// We should be done with the transmission.
	HAL_ASSERT(spi_ll_get_running_cmd(hw) == 0);
	// set transaction line mode
	spi_ll_master_set_line_mode(hw, trans->line_mode);

	int extra_dummy = 0;
	// when no_dummy is not set and in half-duplex mode, sets the dummy bit if RX phase exist
	if(trans->rcv_buffer && !dev->no_compensate && dev->half_duplex) {
		extra_dummy = dev->timing_conf.timing_dummy;
	}

	// SPI iface needs to be configured for a delay in some cases.
	// configure dummy bits
	spi_ll_set_dummy(hw, extra_dummy + trans->dummy_bits);

	uint32_t miso_delay_num = 0;
	uint32_t miso_delay_mode = 0;
	if(dev->timing_conf.timing_miso_delay < 0) {
		//if the data comes too late, delay half a SPI clock to improve reading
		switch(dev->mode) {
		case 0:
			miso_delay_mode = 2;
			break;
		case 1:
			miso_delay_mode = 1;
			break;
		case 2:
			miso_delay_mode = 1;
			break;
		case 3:
			miso_delay_mode = 2;
			break;
		}
		miso_delay_num = 0;
	} else {
		//if the data is so fast that dummy_bit is used, delay some apb clocks to meet the timing
		miso_delay_num = extra_dummy ? dev->timing_conf.timing_miso_delay : 0;
		miso_delay_mode = 0;
	}
	spi_ll_set_miso_delay(hw, miso_delay_mode, miso_delay_num);

	spi_ll_set_mosi_bitlen(hw, trans->tx_bitlen);

	if(dev->half_duplex) {
		spi_ll_set_miso_bitlen(hw, trans->rx_bitlen);
	} else {
		// rxlength is not used in full-duplex mode
		spi_ll_set_miso_bitlen(hw, trans->tx_bitlen);
	}

	//Configure bit sizes, load addr and command
	int cmdlen = trans->cmd_bits;
	int addrlen = trans->addr_bits;
	if(!dev->half_duplex && dev->cs_setup != 0) {
		/* The command and address phase is not compatible with cs_ena_pretrans
         * in full duplex mode.
         */
		cmdlen = 0;
		addrlen = 0;
	}

	spi_ll_set_addr_bitlen(hw, addrlen);
	spi_ll_set_command_bitlen(hw, cmdlen);

	spi_ll_set_command(hw, trans->cmd, cmdlen, dev->tx_lsbfirst);
	spi_ll_set_address(hw, trans->addr, addrlen, dev->tx_lsbfirst);

	//Configure keep active CS
	spi_ll_master_keep_cs(hw, trans->cs_keep_active);

	//
	nextTransaction();
}

void IRAM_ATTR Controller::nextTransaction()
{
#if CONFIG_IDF_TARGET_ESP32
		if(bus_attr->dma_enabled && (cur_trans_buf->buffer_to_rcv || cur_trans_buf->buffer_to_send)) {
			// mark channel as active, so that the DMA will not be reset by the slave
			// This workaround is only for esp32, where tx_dma_chan and rx_dma_chan are always same
			spicommon_dmaworkaround_transfer_active(bus_attr->tx_dma_chan);
		}
#endif //#if CONFIG_IDF_TARGET_ESP32

	auto& req = *trans.request;
	auto& dev = *req.device;

	auto& t = esp_trans->ext;

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
	// void spi_hal_prepare_data(spi_hal_context_t *hal, const spi_hal_dev_config_t *dev, const spi_hal_trans_config_t *trans)
	spi_dev_t* hw = hal->hw;

	// Fill DMA descriptors
	if(trans->rcv_buffer) {
		if(!hal->dma_enabled) {
			// No need to setup anything; we'll copy the result out of the work registers directly later.
		} else {
			s_spi_hal_dma_desc_setup_link(hal->dmadesc_rx, trans->rcv_buffer, ((trans->rx_bitlen + 7) / 8), true);

			spi_dma_ll_rx_reset(hal->dma_in, hal->rx_dma_chan);
			spi_ll_dma_rx_fifo_reset(hal->hw);
			spi_ll_infifo_full_clr(hal->hw);
			spi_ll_dma_rx_enable(hal->hw, 1);
			spi_dma_ll_rx_start(hal->dma_in, hal->rx_dma_chan, (lldesc_t*)hal->dmadesc_rx);
		}

	} else {
#if CONFIG_IDF_TARGET_ESP32
		// DMA temporary workaround: let RX DMA work somehow to avoid the issue in ESP32 v0/v1 silicon
		if(hal->dma_enabled && !dev->half_duplex) {
			spi_ll_dma_rx_enable(hal->hw, 1);
			spi_dma_ll_rx_start(hal->dma_in, hal->rx_dma_chan, 0);
		}
#endif
	}

	if(trans->send_buffer) {
		if(!hal->dma_enabled) {
			// Need to copy data to registers manually
			spi_ll_write_buffer(hw, trans->send_buffer, trans->tx_bitlen);
		} else {
			s_spi_hal_dma_desc_setup_link(hal->dmadesc_tx, trans->send_buffer, (trans->tx_bitlen + 7) / 8, false);

			spi_dma_ll_tx_reset(hal->dma_out, hal->tx_dma_chan);
			spi_ll_dma_tx_fifo_reset(hal->hw);
			spi_ll_outfifo_empty_clr(hal->hw);
			spi_ll_dma_tx_enable(hal->hw, 1);
			spi_dma_ll_tx_start(hal->dma_out, hal->tx_dma_chan, (lldesc_t*)hal->dmadesc_tx);
		}
	}

	// in ESP32 these registers should be configured after the DMA is set
	if((!dev->half_duplex && trans->rcv_buffer) || trans->send_buffer) {
		spi_ll_enable_mosi(hw, 1);
	} else {
		spi_ll_enable_mosi(hw, 0);
	}
	spi_ll_enable_miso(hw, (trans->rcv_buffer) ? 1 : 0);

	//
	req->device->transferStarting(*req);

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
	assert(spi_ll_usr_is_done(hw));

#if CONFIG_IDF_TARGET_ESP32
		// This workaround is only for esp32, where tx_dma_chan and rx_dma_chan are always same
		spicommon_dmaworkaround_idle(bus_attr->tx_dma_chan);
#endif

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
