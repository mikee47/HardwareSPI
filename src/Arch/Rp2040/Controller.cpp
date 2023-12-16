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
 * 
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this library.
 * If not, see <https://www.gnu.org/licenses/>.
 *
 * @author: December 2021 - mikee47 <mike@sillyhouse.net>
 *
 ****/

#include <HSPI/Controller.h>
#include <HSPI/Device.h>
#include <hardware/structs/spi.h>
#include <hardware/clocks.h>
#include <hardware/address_mapped.h>
#include <hardware/resets.h>
#include <hardware/gpio.h>
#include <hardware/regs/dreq.h>
#include <hardware/regs/intctrl.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/sync.h>
#include <Interrupts.h>
#include <esp_system.h>
#include <debug_progmem.h>
#include <Platform/Timers.h>

// #define HSPI_DEBUG

#include <hardware/structs/uart.h>

#define SPI_PERIPH_NUM 2

namespace HSPI
{
#ifdef HSPI_ENABLE_STATS
volatile Controller::Stats Controller::stats;
#endif

struct PeriphDef {
	SpiPins pins;
	uint8_t dreq_tx;
	uint8_t dreq_rx;
};

namespace
{
#ifdef HSPI_DEBUG

#define spi_debug(fmt, ...) spi_uart_printf(fmt "\r\n", ##__VA_ARGS__)

void spi_uart_printf(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	char buf[256];
	unsigned len = m_vsnprintf(buf, sizeof(buf), fmt, args);

	auto dev = uart0_hw;

	for(unsigned i = 0; i < len; ++i) {
		while((dev->fr & UART_UARTFR_TXFF_BITS) != 0) {
		}
		dev->dr = buf[i];
	}

	va_end(args);
}

#else

#define spi_debug(fmt, ...)

#endif

constexpr PeriphDef peripheralDefs[SPI_PERIPH_NUM]{
	{
		{
			.sck = PICO_DEFAULT_SPI_SCK_PIN,
			.miso = PICO_DEFAULT_SPI_RX_PIN,
			.mosi = PICO_DEFAULT_SPI_TX_PIN,
		},
		.dreq_tx = DREQ_SPI0_TX,
		.dreq_rx = DREQ_SPI0_RX,
	},
	{
		{
			.sck = 10,
			.miso = 12,
			.mosi = 11,
		},
		.dreq_tx = DREQ_SPI1_TX,
		.dreq_rx = DREQ_SPI1_RX,
	},
};

struct SpiPreDiv {
	unsigned freq;
	uint8_t prescale;
	uint8_t postdiv;
};

struct SpiPeriph {
	spi_hw_t* const hw;
	Controller* controller{};

	void reset()
	{
		reset_block(hw == spi0_hw ? RESETS_RESET_SPI0_BITS : RESETS_RESET_SPI1_BITS);
	}

	void unreset()
	{
		unreset_block_wait(hw == spi0_hw ? RESETS_RESET_SPI0_BITS : RESETS_RESET_SPI1_BITS);
	}

	void init(Controller& controller)
	{
		this->controller = &controller;
		reset();
		hw->cr1 = 0;
		unreset();

		// Enable DREQ signals
		hw->dmacr = SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS;

		// Enable hardware
		hw_set_bits(&hw->cr1, SPI_SSPCR1_SSE_BITS);
	}

	void deinit()
	{
		hw->cr1 = 0;
		hw->dmacr = 0;
		reset();
		controller = nullptr;
	}

	union SSPCR0 {
		struct {
			uint32_t dss : 4; ///< Data Size Select
			uint32_t frf : 2; ///< Frame Format
			uint32_t spo : 1; ///< SSPCLKOUT polarity
			uint32_t sph : 1; ///< SSPCLKOUT phase
			uint32_t scr : 8; ///< Serial clock rate
		};
		uint32_t val;
	};

	enum FrameFormat {
		FRAME_FORMAT_MOTOROLA = 0,
		FRAME_FORMAT_TI = 1,
		FRAME_FORMAT_MICROWIRE = 2,
		FRAME_FORMAT_UNDEFINED = 3,
	};

	void __forceinline setClockPrescale(uint8_t clk_prescale)
	{
		hw->cpsr = clk_prescale;
	}

	void __forceinline setWordSize(uint32_t cr0val, uint8_t data_bits)
	{
		hw->cr0 = cr0val | (data_bits - 1);
	}

	static uint32_t makeCR0(ClockMode mode, uint8_t postdiv)
	{
		SSPCR0 cr0{{
			.dss = 0,
			.frf = FRAME_FORMAT_MOTOROLA,
			.spo = (mode == ClockMode::mode2 || mode == ClockMode::mode3) ? 1U : 0U,
			.sph = (mode == ClockMode::mode1 || mode == ClockMode::mode3) ? 1U : 0U,
			.scr = uint8_t(postdiv - 1),
		}};
		return cr0.val;
	}

	void loopback(bool enable)
	{
		hw_write_masked(&hw->cr1, enable << SPI_SSPCR1_LBM_LSB, SPI_SSPCR1_LBM_BITS);
	}

	bool __forceinline isBusy() const
	{
		return hw->sr & SPI_SSPSR_BSY_BITS;
	}

	bool __forceinline canRead() const
	{
		return hw->sr & SPI_SSPSR_RNE_BITS;
	}

	bool __forceinline canWrite() const
	{
		return hw->sr & SPI_SSPSR_TNF_BITS;
	}

	uint16_t __forceinline read()
	{
		return hw->dr;
	}

	void __forceinline write(uint16_t c)
	{
		hw->dr = c;
	}

	void __forceinline flush()
	{
		while(canRead()) {
			(void)read();
		}
	}
};

SpiPeriph peripherals[SPI_PERIPH_NUM] = {
	{spi0_hw},
	{spi1_hw},
};

__forceinline SpiPeriph& getPeriph(SpiBus busId)
{
	return peripherals[unsigned(busId) - 1];
}

SpiPreDiv calculateClock(unsigned baudrate)
{
	auto freq_in = clock_get_hz(clk_peri);
	if(baudrate > freq_in) {
		return SpiPreDiv{freq_in, 2, 1};
	}

	/*
	 * Find smallest prescale value which puts output frequency in range of post-divide.
	 * Prescale is an even number from 2 to 254 inclusive.
	 */
	unsigned prescale;
	for(prescale = 2; prescale <= 254; prescale += 2) {
		if(freq_in < (prescale + 2) * 256 * (uint64_t)baudrate)
			break;
	}
	if(prescale > 254) {
		// Frequency too low
		prescale = 254;
	}

	/*
	 * Find largest post-divide which makes output <= baudrate.
	 * Post-divide is an integer in the range 1 to 256 inclusive.
	 */
	unsigned postdiv;
	for(postdiv = 256; postdiv > 1; --postdiv) {
		if(freq_in / (prescale * (postdiv - 1)) > baudrate)
			break;
	}

	// Return actual frequency and corresponding settings
	return SpiPreDiv{freq_in / (prescale * postdiv), uint8_t(prescale), uint8_t(postdiv)};
}

// Cortex M0+ doesn't support the rbit instruction
// __forceinline uint32_t reverseBits(uint32_t value)
// {
// 	uint32_t result;
// 	__asm__("rbit %0, %1" : "=r"(result) : "r"(value));
// 	return result;
// }

uint8_t IRAM_ATTR reverseBits(uint8_t n)
{
	static constexpr IRAM_ATTR uint8_t rev_nybble[16]{
		0b0000, 0b1000, 0b0100, 0b1100, 0b0010, 0b1010, 0b0110, 0b1110,
		0b0001, 0b1001, 0b0101, 0b1101, 0b0011, 0b1011, 0b0111, 0b1111,
	};
	return (rev_nybble[n & 0x0f] << 4) | rev_nybble[n >> 4];
}

uint16_t IRAM_ATTR reverseBits(uint16_t n)
{
	return (reverseBits(uint8_t(n)) << 8) | reverseBits(uint8_t(n >> 8));
}

uint32_t IRAM_ATTR reverseBits(uint32_t n)
{
	return (reverseBits(uint16_t(n)) << 16) | reverseBits(uint16_t(n >> 16));
}

void reverseBits(void* buffer, size_t length)
{
	auto bufptr = static_cast<uint8_t*>(buffer);
	while(length--) {
		*bufptr = reverseBits(*bufptr);
		++bufptr;
	}
}

} // namespace

bool Controller::begin()
{
	if(flags.initialised) {
		return true;
	}

	unsigned busIndex = unsigned(busId) - 1;
	if(busIndex >= ARRAY_SIZE(peripheralDefs)) {
		debug_e("Invalid bus ID");
		return false;
	}

	auto& periph = peripherals[busIndex];
	if(periph.controller != nullptr) {
		debug_e("[SPI] Bus #%u already assigned", busId);
		return false;
	}

	periph.init(*this);

	auto& def = peripheralDefs[busIndex];
	configure_dma(&periph.hw->dr, def.dreq_tx, def.dreq_rx);

	/* Configure pins */
	assignDefaultPins(def.pins);

	gpio_set_function(pins.sck, GPIO_FUNC_SPI);
	gpio_set_function(pins.miso, GPIO_FUNC_SPI);
	gpio_set_function(pins.mosi, GPIO_FUNC_SPI);
	gpio_disable_pulls(pins.sck);
	gpio_pull_up(pins.miso);
	gpio_disable_pulls(pins.mosi);

	flags.initialised = true;
	return true;
}

IoModes Controller::getSupportedIoModes(const Device& dev) const
{
	// Only support basic modes
	return dev.getSupportedIoModes() & (IoMode::SPI | IoMode::SPIHD);
}

void Controller::configure_dma(volatile void* fifo_addr, uint8_t dreq_tx, uint8_t dreq_rx)
{
	dma.tx_channel = dma_claim_unused_channel(true);
	dma.rx_channel = dma_claim_unused_channel(true);
	dma.ctrl_channel = dma_claim_unused_channel(true);

	debug_d("[SPI] Got DMA TX %u, RX %u, CTRL %u", dma.tx_channel, dma.rx_channel, dma.ctrl_channel);

	// TX DMA
	dma.tx_config = dma_channel_get_default_config(dma.tx_channel);
	channel_config_set_transfer_data_size(&dma.tx_config, DMA_SIZE_8);
	channel_config_set_dreq(&dma.tx_config, dreq_tx);
	dma_channel_set_write_addr(dma.tx_channel, fifo_addr, false);
	dma_channel_set_config(dma.tx_channel, &dma.tx_config, false);
	dma.tx_config_dummy = dma.tx_config;
	channel_config_set_ring(&dma.tx_config_dummy, false, 2);

	// RX DMA
	dma.rx_config = dma_channel_get_default_config(dma.rx_channel);
	channel_config_set_transfer_data_size(&dma.rx_config, DMA_SIZE_8);
	channel_config_set_dreq(&dma.rx_config, dreq_rx);
	channel_config_set_read_increment(&dma.rx_config, false);
	channel_config_set_write_increment(&dma.rx_config, true);
	dma.rx_config_dummy = dma.rx_config;
	channel_config_set_ring(&dma.rx_config_dummy, true, 2);
	dma.rx_config_chain = dma.rx_config;
	channel_config_set_chain_to(&dma.rx_config_chain, dma.ctrl_channel);
	channel_config_set_irq_quiet(&dma.rx_config_chain, true);
	dma_channel_set_read_addr(dma.rx_channel, fifo_addr, false);

	// Control
	dma.ctrl_config = dma_channel_get_default_config(dma.ctrl_channel);
	channel_config_set_transfer_data_size(&dma.ctrl_config, DMA_SIZE_32);
	channel_config_set_read_increment(&dma.ctrl_config, true);
	channel_config_set_write_increment(&dma.ctrl_config, true);
	channel_config_set_ring(&dma.ctrl_config, true, 3); // 8-byte wrap (2 words)
	dma_channel_set_config(dma.ctrl_channel, &dma.ctrl_config, false);
	dma_channel_set_write_addr(dma.ctrl_channel, &dma_hw->ch[dma.rx_channel].al1_write_addr, false);
	dma_channel_set_trans_count(dma.ctrl_channel, 2, false);

	// Fire interrupt when transaction is done (share between all controllers)
	dma_channel_acknowledge_irq0(dma.rx_channel);
	dma_channel_set_irq0_enabled(dma.rx_channel, true);
	irq_set_exclusive_handler(DMA_IRQ_0, staticInterruptHandler);
	irq_set_enabled(DMA_IRQ_0, true);
}

void Controller::release_dma()
{
	dma_channel_set_irq0_enabled(dma.rx_channel, false);
	dma_channel_unclaim(dma.ctrl_channel);
	dma_channel_unclaim(dma.rx_channel);
	dma_channel_unclaim(dma.tx_channel);
}

void Controller::end()
{
	if(!flags.initialised) {
		return;
	}

	// Check all devices have been released
	if(deviceCount != 0) {
		debug_e("ERROR: Devices not released");
		abort();
	}

	release_dma();

	auto& periph = getPeriph(busId);
	periph.deinit();

	flags.initialised = false;
}

void IRAM_ATTR Controller::interruptHandler()
{
	if(dma_channel_get_irq0_status(dma.rx_channel)) {
		dma_channel_acknowledge_irq0(dma.rx_channel);
		transactionDone();
	}
}

void IRAM_ATTR Controller::staticInterruptHandler()
{
	if(auto ctrl = peripherals[0].controller) {
		ctrl->interruptHandler();
	}
	if(auto ctrl = peripherals[1].controller) {
		ctrl->interruptHandler();
	}
}

bool Controller::startDevice(Device& dev, PinSet pinSet, uint8_t chipSelect, uint32_t clockSpeed)
{
	if(!flags.initialised) {
		debug_e("[SPI] Controller not initialised");
		return false;
	}

	if(dev.pinSet != PinSet::none) {
		debug_e("[SPI] device already started on bus %u, CS #%u", unsigned(busId), dev.chipSelect);
		return false;
	}

	if(pinSet != PinSet::normal) {
		debug_e("[SPI] PinSet not supported");
		return false;
	}

	// Chip select
	gpio_put(chipSelect, true);
	gpio_set_dir(chipSelect, true);
	gpio_disable_pulls(chipSelect);
	gpio_set_function(chipSelect, GPIO_FUNC_SIO);

	// Calculate clock setting
	setClockSpeed(dev, clockSpeed);
	dev.config.dirty = true;

	++deviceCount;
	dev.pinSet = pinSet;
	dev.chipSelect = chipSelect;

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

	detachInterrupt(dev.chipSelect);

	dev.pinSet = PinSet::none;
	dev.chipSelect = 255;
}

void Controller::configChanged(Device& dev)
{
	dev.config.dirty = true;
}

void Controller::updateConfig(Device& dev)
{
	auto& cfg = dev.config;
	cfg.cr0val = SpiPeriph::makeCR0(dev.getClockMode(), cfg.clk_postdiv);
	cfg.dirty = false;
}

uint32_t Controller::setClockSpeed(Device& dev, uint32_t freq)
{
	auto prediv = calculateClock(freq);
	dev.config.clk_prescale = prediv.prescale;
	dev.config.clk_postdiv = prediv.postdiv;
	dev.speed = prediv.freq; // Set actual clock speed in use
	return prediv.freq;
}

void Controller::execute(Request& req)
{
	if(!flags.initialised || req.device == nullptr || req.device->pinSet == PinSet::none) {
		debug_e("[SPI] Device not initialised");
		return;
	}

	// debug_i("[SPI] execute cmd %u 0x%04x, addr %u, mosi %u, dummy %u, miso %u, async %u", req.cmdLen, req.cmd,
	// 		req.addrLen, req.out.length, req.dummyLen, req.in.length, req.async);

	req.next = nullptr;
	req.busy = true;

	auto dev = req.device;
	if(dev->config.dirty) {
		updateConfig(*dev);
	}

	// Packet transfer already in progress?
	dma_channel_set_irq0_enabled(dma.rx_channel, false);
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
	dma_channel_set_irq0_enabled(dma.rx_channel, true);

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

	uint8_t controlBitCount = req.cmdLen + req.addrLen + req.dummyLen;
	if(controlBitCount != 0 && (controlBitCount < 4 || controlBitCount > 16)) {
		spi_debug("[SPI] ctrl %u, mosi %p %u, miso %p %u", controlBitCount, req.out.get(), req.out.length, req.in.get(),
				  req.in.length);
	}
	// spi_debug("[SPI] cmd %u 0x%04x, addr %u, mosi %u, dummy %u, miso %u", req.cmdLen, req.cmd, req.addrLen,
	// 		  req.out.length, req.dummyLen, req.in.length);

	/*
	 * Only SPI, SPIHD modes supported.
	 * SPI3WIRE requires read/write on same wire but that doesn't look possible.
	 *
	 * Command/address/dummy phase implemented by writing directly to FIFO,
	 * DMA transfers then handle MOSI/MISO phase.
	 * Bits read during this initial phase are discarded using a separate RX DMA.
	 * These are chained using a control DMA channel.
	 *
	 * SPI: Both TX/RX DMA occur simultaneously
	 * SPIHD: TX DMA first, chain to RX DMA on completion.
	 */
	switch(dev.ioMode) {
	case IoMode::SPI:
	case IoMode::SPIHD:
		break;
	default:
		debug_e("[SPI] Hardware doesn't support requested mode");
		assert(false);
	}

	/*
		Full-duplex mode (SPI) has MOSI and MISO phases. For half-duplex transfers:

			command
			address
			dummy
			mosi
			miso

		How to handle dummy? e.g.
			cmd8 - addr24 - dummy 1 - miso N
				Total OUT is 33 bits, can send as 16+8+9
				Using DMA we'd use 8+8+8+9

		e.g. ILI9341 uses 1 dummy bit for reads, RAM might use 8, etc.

		We've got 12 DMA channels so using a third channel for RX control isn't unreasonable.
	*/

	if(selectDeviceCallback) {
		selectDeviceCallback(dev.chipSelect, true);
	}
	dev.transferStarting(req);

	// Assert CS
	gpio_put(dev.chipSelect, false);

	trans.busy = true;
	trans.bitOrder = dev.bitOrder;

	auto& periph = getPeriph(busId);
	periph.setClockPrescale(dev.config.clk_prescale);

	uint8_t fifoCount{0}; // Words written directly into FIFO
	if(controlBitCount == 0) {
		periph.setWordSize(dev.config.cr0val, 8);
	} else {
		// Note: Could handle this by prepending to first MOSI byte
		assert(controlBitCount >= 4);

		uint64_t controlBits{0};
		if(trans.bitOrder == MSBFIRST) {
			controlBits = req.cmd;
			if(req.addrLen != 0) {
				controlBits <<= req.addrLen;
				controlBits |= req.addr;
			}
		} else {
			if(req.cmdLen != 0) {
				controlBits = reverseBits(req.cmd) >> (16 - req.cmdLen);
			}
			if(req.addrLen != 0) {
				controlBits <<= req.addrLen;
				controlBits |= reverseBits(req.addr) >> (32 - req.addrLen);
			}
		}
		controlBits <<= req.dummyLen;

		if(controlBitCount <= 16) {
			periph.setWordSize(dev.config.cr0val, controlBitCount);
			periph.write(controlBits);
			fifoCount = 1;
			// Require at least 1 cycle before changing word size
			__asm__ volatile("nop" :::);
			periph.setWordSize(dev.config.cr0val, 8);
		} else {
			// Write an initial word from 8-15 bits
			uint8_t initialWordSize = (controlBitCount % 8) + 8;
			periph.setWordSize(dev.config.cr0val, initialWordSize);
			controlBitCount -= initialWordSize;
			periph.write(controlBits >> controlBitCount);
			++fifoCount;
			// All following words are 8 bits
			__asm__ volatile("nop" :::);
			periph.setWordSize(dev.config.cr0val, 8);
			while(controlBitCount != 0) {
				controlBitCount -= 8;
				periph.write(controlBits >> controlBitCount);
				++fifoCount;
			}
		}
	}

	uint16_t channel_mask{0};

	// Setup outgoing data
	if(req.out.length != 0) {
		if(trans.bitOrder != MSBFIRST) {
			reverseBits(req.out.get(), req.out.length);
		}
		channel_mask |= BIT(dma.tx_channel);
		size_t outlen = req.out.length;
		if(dev.ioMode != IoMode::SPI) {
			/*
			 * Half-duplex, transmit bytes to match MISO
			 * TODO: For simplicity just read past end of provided buffer
			 * since we don't care what data is output.
			 */
			outlen += req.in.length;
		}
		dma_channel_set_read_addr(dma.tx_channel, req.out.get(), false);
		dma_channel_set_trans_count(dma.tx_channel, outlen, false);
		dma_channel_set_config(dma.tx_channel, &dma.tx_config, false);
	} else if(req.in.length != 0) {
		// Clock dummy bits
		channel_mask |= BIT(dma.tx_channel);
		dma.buffer[0] = 0xffffffff;
		dma_channel_set_read_addr(dma.tx_channel, dma.buffer, false);
		dma_channel_set_trans_count(dma.tx_channel, req.in.length, false);
		dma_channel_set_config(dma.tx_channel, &dma.tx_config_dummy, false);
	}

	// Setup incoming data
	if(req.in.length == 0) {
		// Dummy reads to match control + MOSI
		channel_mask |= BIT(dma.rx_channel);
		dma_channel_set_write_addr(dma.rx_channel, dma.buffer, false);
		dma_channel_set_trans_count(dma.rx_channel, fifoCount + req.out.length, false);
		dma_channel_set_config(dma.rx_channel, &dma.rx_config_dummy, false);
	} else if(dev.ioMode != IoMode::SPI) {
		// Half-duplex, use dummy read followed by MISO read
		channel_mask |= BIT(dma.ctrl_channel);
		dma.control_blocks[0] = RxControlBlock{dma.buffer, size_t(fifoCount + req.out.length)};
		dma.control_blocks[1] = RxControlBlock{req.in.get(), req.in.length};
		dma_channel_set_read_addr(dma.ctrl_channel, &dma.control_blocks[0], false);
		dma_channel_set_config(dma.rx_channel, &dma.rx_config_chain, false);
	} else if(fifoCount != 0) {
		// Dummy read followed by MISO read
		channel_mask |= BIT(dma.ctrl_channel);
		dma.control_blocks[0] = RxControlBlock{dma.buffer, fifoCount};
		dma.control_blocks[1] = RxControlBlock{req.in.get(), req.in.length};
		dma_channel_set_read_addr(dma.ctrl_channel, &dma.control_blocks[0], false);
		dma_channel_set_config(dma.rx_channel, &dma.rx_config_chain, false);
	} else {
		// MISO read
		channel_mask |= BIT(dma.rx_channel);
		dma_channel_set_write_addr(dma.rx_channel, req.in.get(), false);
		dma_channel_set_trans_count(dma.rx_channel, req.in.length, false);
		dma_channel_set_config(dma.rx_channel, &dma.rx_config, false);
	}

#ifdef HSPI_ENABLE_STATS
	++stats.transCount;
#endif

	/* Execute now */

	// start channels simultaneously to avoid races (in extreme cases the FIFO could overflow)
	dma_start_channel_mask(channel_mask);
}

/*
 * Read incoming data, if there is any, and start next transaction.
 * Called from interrupt context at completion of transaction.
 */
void IRAM_ATTR Controller::transactionDone()
{
	auto& req = *trans.request;
	auto& dev = *req.device;

	// De-assert CS
	gpio_put(dev.chipSelect, true);

	if(selectDeviceCallback) {
		selectDeviceCallback(dev.chipSelect, false);
	}

	// Transform received data bit order (if required)
	if(req.in.length != 0 && trans.bitOrder != MSBFIRST) {
		reverseBits(req.in.get(), req.in.length);
	}

	trans.busy = false;
	req.busy = false;
#ifdef HSPI_ENABLE_STATS
	++stats.requestCount;
#endif

	// Note next packet in chain and de-queue this one
	trans.request = req.next;
	req.next = nullptr;

	if(!dev.transferComplete(req)) {
		trans.request = reQueueRequest(trans.request, &req);
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
	auto& periph = getPeriph(busId);
	periph.loopback(enable);
	return true;
}

} // namespace HSPI
