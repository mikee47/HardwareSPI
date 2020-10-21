/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * Controller.cpp
 *
 * @author: 11 December 2018 - mikee47 <mike@sillyhouse.net>
 *
 * Pin connections:
 *
 *				NodeMCU
 *		HSPI			Overlapped			S1D13781 eval.
 * MISO GPIO12 D6		GPIO7 SD0			J2.1
 * MOSI GPIO13 D7		GPIO8 SD1			J2.4
 * CLK  GPIO14 D5		GPIO6 CLK			J2.3
 * CS   GPIO15 D8		GPIO0 D3 (CS2)		J3.5
 *
 * # Interrupt tests
 *
 * Measured 1.6us from CS high to ISR clearing GPIO.
 *
 * # Setup time
 *
 * Tried using inline 32-bit copy operations for FIFO but memcpy was consistently faster.
 * Remove the default 'volatile' from SPI struct, and added only those required for it to work correctly.
 * Reduced the setup time very slightly, but safer to leave as-is.
 *
 */

#include <HSPI/Controller.h>
#include <HSPI/Device.h>
#include <esp_clk.h>
#include <esp_systemapi.h>
#include <espinc/spi_register.h>
#include <espinc/spi_struct.h>
#include <espinc/gpio_struct.h>
#include <espinc/pin_mux_register.h>
#include <Platform/Timers.h>
#include "Debug.h"

namespace HSPI
{
#ifdef HSPI_ENABLE_STATS
volatile Controller::Stats Controller::stats;
#endif

// GPIO pin numbers for SPI
#define PIN_HSPI_MISO 12
#define PIN_HSPI_MOSI 13
#define PIN_HSPI_CLK 14
#define PIN_HSPI_CS0 15
#define PIN_SPI_CS1 1
#define PIN_SPI_CS2 0

// SPI interrupt status register address definition for determining the interrupt source
#define DPORT_SPI_INT_STATUS_REG 0x3ff00020
#define DPORT_SPI_INT_STATUS_SPI0 BIT4
#define DPORT_SPI_INT_STATUS_SPI1 BIT7

// Enable the given pin as a chip select
#define CS_ENABLE(cspin, func)                                                                                         \
	{                                                                                                                  \
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(cspin), func);                                                            \
	}

// Revert chip select to GPIO, output HIGH (CS inactive)
#define CS_DISABLE(cspin, func)                                                                                        \
	{                                                                                                                  \
		GPIO.enable_w1ts = BIT(cspin);                                                                                 \
		GPIO.out_w1ts = BIT(cspin);                                                                                    \
		GPIO.pin[cspin].val = 0;                                                                                       \
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(cspin), func);                                                            \
	}

namespace
{
const spi_dev_t::clock_t clkEquSys{{
	.clkcnt_l{0},
	.clkcnt_h{0},
	.clkcnt_n{0},
	.clkdiv_pre{0},
	.clk_equ_sysclk{true},
}};
const spi_dev_t::clock_t clkMin{{
	.clkcnt_l{0x3f},
	.clkcnt_h{0x1f},
	.clkcnt_n{0x3f},
	.clkdiv_pre{0x1fff},
	.clk_equ_sysclk{false},
}};
const spi_dev_t::clock_t clkDiv2{{
	.clkcnt_l{1},
	.clkcnt_h{0},
	.clkcnt_n{1},
	.clkdiv_pre{0},
	.clk_equ_sysclk{0},
}};

/**
 * @brief Enable or disable overlap of SPI1 controller onto SPI0 pins
 * @param enable
 */
void overlapEnable(bool enable)
{
	if(enable) {
		SET_PERI_REG_MASK(HOST_INF_SEL, PERI_IO_CSPI_OVERLAP);
		// Prioritise SPI over HSPI transactions
		SPI0.ext3.int_hold_ena = 1;
		SPI1.ext3.int_hold_ena = 3;
	} else {
		CLEAR_PERI_REG_MASK(HOST_INF_SEL, PERI_IO_CSPI_OVERLAP);
		// De-prioritise SPI vs HSPI
		SPI0.ext3.int_hold_ena = 0;
		SPI1.ext3.int_hold_ena = 0;
	}
}

/*
 * @brief Set function of SPI1 pins to SPI or GPIO
 * @param enable
 */
void enableSpi0Pins(bool enable)
{
	if(enable) {
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_MISO), FUNC_HSPIQ_MISO);
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_MOSI), FUNC_HSPID_MOSI);
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_CLK), FUNC_HSPI_CLK);
	} else {
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_MISO), FUNC_GPIO12);
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_MOSI), FUNC_GPIO13);
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_CLK), FUNC_GPIO14);
	}
}

/** @brief Calculate SPI clock Frequency based on the register value
 * 	@param regVal
 * 	@retval uint32_t clock frequency in Hz
 */
uint32_t getClockFrequency(const spi_dev_t::clock_t clk)
{
	return APB_CLK_FREQ / ((clk.clkdiv_pre + 1) * (clk.clkcnt_n + 1));
}

/** @brief Determine the best clock register value for a desired bus frequency
 *  @param frequency Desired SPI clock frequency in Hz
 *  @param clockReg the appropriate clock register value
 *  @retval uint32_t The actual frequency which will be used
 *  @note
 *  	Speed settings are pre-calculated to optimise switching between devices.
 *  	It is guaranteed that the frequency will not exceed the given target
 */
uint32_t calculateClock(uint32_t frequency, spi_dev_t::clock_t& clockReg)
{
	if(frequency >= APB_CLK_FREQ) {
		clockReg = clkEquSys;
		return APB_CLK_FREQ;
	}

	if(frequency < getClockFrequency(clkMin)) {
		// use minimum possible clock
		clockReg = clkMin;
		return getClockFrequency(clkMin);
	}

	clockReg.val = 0;
	uint8_t calN{1};
	uint32_t clockFreq{0};

	// find the best match
	while(calN <= 0x3F) { // 0x3F max for N

		uint32_t calFreq{0};
		spi_dev_t::clock_t reg{};
		reg.clkcnt_n = calN;

		// Test different variants for prescale
		int8_t calPreVari{-2};
		while(calPreVari++ <= 1) {
			int calPre = (((APB_CLK_FREQ / (reg.clkcnt_n + 1)) / frequency) - 1) + calPreVari;
			if(calPre > 0x1FFF) {
				reg.clkdiv_pre = 0x1FFF; // 8191
			} else if(calPre <= 0) {
				reg.clkdiv_pre = 0;
			} else {
				reg.clkdiv_pre = calPre;
			}

			reg.clkcnt_l = ((reg.clkcnt_n + 1) / 2);

			// Test calculation
			calFreq = getClockFrequency(reg);

			if(calFreq == frequency) {
				// accurate match use it!
				clockFreq = calFreq;
				clockReg = reg;
				break;
			}

			if(calFreq < frequency) {
				// never go over the requested frequency
				if(abs(int(frequency - calFreq)) < abs(int(frequency - clockFreq))) {
					clockFreq = calFreq;
					clockReg = reg;
				}
			}
		}

		if(calFreq == frequency) {
			// accurate match use it!
			break;
		}

		calN++;
	}

	return clockFreq;
}

} // namespace

void Controller::begin()
{
	// Pinset and chip selects are device-dependent and not initialised here - see startPacket()

	SPI0.slave.val &= ~0x000003FF; // Don't want interrupts from SPI0
	SPI1.slave.val &= ~0x000003FF; // Clear all interrupt sources
	SPI1.slave.trans_inten = 1;	// Interrupt on command completion
	SPI1.slave.slave_mode = false;
	SPI1.slave.sync_reset = true;
	SPI1.ctrl1.val = 0;

	TESTPIN_SETUP();

	ETS_SPI_INTR_ATTACH(ets_isr_t(isr), this);
	ETS_SPI_INTR_ENABLE();
}

void Controller::end()
{
	ETS_SPI_INTR_DISABLE();

	// Disable all hardware chip selects
	SPI1.pin.cs0_dis = 1;
	SPI1.pin.cs1_dis = 1;
	SPI1.pin.cs2_dis = 1;

	// Check all devices have been released
	assert(normalDevices == 0 && overlapDevices == 0);
}

bool Controller::startDevice(Device& dev, PinSet pinSet, uint8_t chipSelect)
{
	if(dev.pinSet != PinSet::none) {
		debug_e("SPI device already started at %u, CS #%u", unsigned(dev.pinSet), dev.chipSelect);
		return false;
	}

	auto csMax = (pinSet == PinSet::overlap) ? 2 : 0;
	if(chipSelect > csMax) {
		debug_e("SPI pinSet %u, CS #%u invalid", unsigned(pinSet), chipSelect);
		return false;
	}

	if(chipSelectsInUse[chipSelect]) {
		debug_e("SPI CS #%u in use", chipSelect);
		return false;
	}

	switch(pinSet) {
	case PinSet::overlap:
		if(overlapDevices++ == 0) {
			// From ESP32 code: 'we do need at least one clock of hold time in most cases'
			spi_dev_t::ctrl2_t ctrl2{};
			//			ctrl2.miso_delay_mode = 3;
			//			ctrl2.miso_delay_num = 4;
			//			ctrl2.mosi_delay_mode = 3;
			//			ctrl2.mosi_delay_num = 4;
			//			ctrl2.cs_delay_mode = 1;
			//			ctrl2.cs_delay_num = 4;
			SPI1.ctrl2.val = ctrl2.val;
		}
		break;

	case PinSet::normal:
		if(normalDevices++ == 0) {
			enableSpi0Pins(true);
		}
		break;

	case PinSet::none:
	default:
		return false;
	}

	// Enable Hardware CS
	switch(chipSelect) {
	case 0:
		CS_ENABLE(PIN_HSPI_CS0, FUNC_HSPI_CS0);
		break;
	case 1:
		CS_ENABLE(PIN_SPI_CS1, FUNC_SPICS1);
		break;
	case 2:
		CS_ENABLE(PIN_SPI_CS2, FUNC_SPICS2);
		break;
	}

	dev.pinSet = pinSet;
	dev.chipSelect = chipSelect;
	chipSelectsInUse[chipSelect] = true;
	dev.config.dirty = true;

	debug_i("SPI pinSet %u, CS #%u acquired", unsigned(pinSet), chipSelect);
	return true;
}

void Controller::stopDevice(Device& dev)
{
	switch(dev.pinSet) {
	case PinSet::overlap:
		assert(overlapDevices > 0);
		--overlapDevices;
		overlapEnable(false);
		break;

	case PinSet::normal:
		assert(normalDevices > 0);
		--normalDevices;
		if(normalDevices == 0) {
			enableSpi0Pins(false);
		}
		break;

	case PinSet::none:
		return;

	default:
		assert(false);
	}

	auto cs = dev.chipSelect;
	switch(cs) {
	case 0:
		CS_DISABLE(PIN_HSPI_CS0, FUNC_GPIO15);
		break;
	case 1:
		CS_DISABLE(PIN_SPI_CS1, FUNC_GPIO1);
		break;
	case 2:
		CS_DISABLE(PIN_SPI_CS2, FUNC_GPIO0);
		break;
	default:
		assert(false);
	}

	debug_i("SPI pinSet %u, CS #%u released", dev.pinSet, cs);

	if(cs < chipSelectsInUse.size()) {
		chipSelectsInUse[cs] = false;
	}

	dev.pinSet = PinSet::none;
	dev.chipSelect = 255;
}

void Controller::configChanged(Device& dev)
{
	dev.config.dirty = true;
}

uint32_t Controller::setSpeed(Device& dev, uint32_t frequency)
{
	spi_dev_t::clock_t reg;
	frequency = calculateClock(frequency, reg);
	dev.config.reg.clock = reg.val;
	return frequency;
}

uint32_t Controller::getSpeed(Device& dev) const
{
	return getClockFrequency({val: dev.config.reg.clock});
}

void Controller::updateConfig(Device& dev)
{
	struct {
		spi_dev_t::user_t user;
		spi_dev_t::ctrl_t ctrl;
		spi_dev_t::pin_t pin;
	} reg;
	reg.user.val = 0;
	reg.user.cs_setup = true;
	reg.user.cs_hold = true;
	reg.ctrl.val = 0;
	reg.ctrl.wp_reg = true;
	reg.pin.val = 0;

	// Enable Hardware CS
	switch(dev.chipSelect) {
	case 0:
		reg.pin.cs1_dis = true;
		reg.pin.cs2_dis = true;
		reg.pin.cs0_dis = false;
		break;
	case 1:
		reg.pin.cs0_dis = true;
		reg.pin.cs2_dis = true;
		reg.pin.cs1_dis = false;
		break;
	case 2:
		reg.pin.cs0_dis = true;
		reg.pin.cs1_dis = true;
		reg.pin.cs2_dis = false;
		break;
	default:
		// TODO: Call method for manual CS control, NON-OVERLAPPED mode ONLY!
		reg.pin.cs0_dis = true;
		reg.pin.cs1_dis = true;
		reg.pin.cs2_dis = true;
	}

	// Bit order
	auto bitOrder = dev.getBitOrder();
	reg.ctrl.rd_bit_order = (bitOrder == MSBFIRST) ? 0 : 1;
	reg.ctrl.wr_bit_order = (bitOrder == MSBFIRST) ? 0 : 1;

	// Byte order
	auto byteOrder = LSBFIRST;
	reg.user.wr_byte_order = (byteOrder == MSBFIRST) ? 1 : 0;
	reg.user.rd_byte_order = (byteOrder == MSBFIRST) ? 1 : 0;

	// Data mode
	auto ioMode = dev.getIoMode();
	reg.user.duplex = (ioMode == IoMode::SPI);
	switch(ioMode) {
	case IoMode::SPI:
	case IoMode::SPIHD:
		break;
	case IoMode::SDI:
	case IoMode::DIO:
		reg.ctrl.fastrd_mode = true;
		reg.ctrl.fread_dio = true;
		reg.user.fwrite_dio = true;
		break;
	case IoMode::DUAL:
		reg.ctrl.fastrd_mode = true;
		reg.ctrl.fread_dual = true;
		reg.user.fwrite_dual = true;
		break;
	case IoMode::SQI:
	case IoMode::QIO:
		reg.ctrl.fastrd_mode = true;
		reg.ctrl.fread_qio = true;
		reg.user.fwrite_qio = true;
		break;
	case IoMode::QUAD:
		reg.ctrl.fastrd_mode = true;
		reg.ctrl.fread_quad = true;
		reg.user.fwrite_quad = true;
		break;
	default:
		assert(false);
	}

	// Clock phase/polarity
	auto clockMode = uint8_t(dev.getClockMode());
	reg.user.ck_out_edge = (clockMode & 0x01) ? 1 : 0; // CPHA
	reg.pin.ck_idle_edge = (clockMode & 0x10) ? 1 : 0; // CPOL

	auto& cfg = dev.config;
	cfg.reg.user = reg.user.val;
	cfg.reg.ctrl = reg.ctrl.val;
	cfg.reg.pin = reg.pin.val;
	cfg.dirty = false;
}

/*
 * OK, so we have two versions of execute() to deal with single FIFO and multiple. For longer
 * transactions we have some options:
 *
 * 	1. As at present, we repeat the operation. It's fine for addressed devices, but may not be for others.
 * 	2a. After the first iteration we set up an ISR to kick off subsequent transactions. We can add a flag
 * 	to the packet to request this behaviour. Completion would be indicated by setting a flag in the packet.
 * 	If execute() is called again it would block until all previous transactions have completed.
 * 	2b. Queue multiple requests. This extends 2a to allow a chain of transactions to be submitted, as a
 * 	linked list of packets. The entire chain would need to complete before processing another.
 *
 *  The problem with (1) is splitting a single request into multiple transactions. We should instead keep
 *  CS asserted, which implies we don't use hardware CS control for that. This isn't an option in overlapped
 *  mode. Interleaving smaller transactions would be better, hence option (2).
 *
 *  Also bear in mind that once a transaction has been submitted to the hardware, it could be delayed by
 *  higher-priority flash accesses on SPI0. It would not cause blocking in the ISR though, as arbitration
 *  happens in hardware.
 *
 *  With the ESP32 we have both regular FIFO operation and the alternative DMA operation. In both cases
 *  a transaction is set up as usual, command, address, etc. with the only difference with the data
 *  transfer. It's not only faster but there's no interrupt overhead and the processor doesn't need
 *  to do any memory copies. The ESP32 can handle a single transfer of up to 4092 bytes.
 *
 *  It is likely we'll have a few well-defined ways to deal with this, and the caller can choose which is
 *  most appropriate for the application and hardware.
 *
 *  UPDATE: Using interrupts to handle transfers works very well, the most benefit being in larger burst
 *  transfers which require splitting out over multiple packets.
 *
 */
void Controller::execute(Request& req)
{
	req.next = nullptr;
	req.busy = true;

	auto dev = req.device;
	if(dev->config.dirty) {
		updateConfig(*dev);
	}

	// For high clock speeds don't use transaction interrupts
	if(req.async && dev->speed >= 16000000U) {
		req.task = true;
	}

	// Packet transfer already in progress?
	ETS_SPI_INTR_DISABLE();
	if(trans.busy) {
		// Tack new packet onto end of chain
		auto pkt = trans.request;
		while(pkt->next) {
			pkt = pkt->next;
		}
		pkt->next = &req;
		if(req.async) {
			if(!flags.taskQueued) {
				ETS_SPI_INTR_ENABLE();
			}
			return;
		}
	} else {
		// Not currently running, so do this one now
		trans.request = &req;
		startRequest();
		if(req.async) {
			if(req.task) {
				queueTask();
			} else {
				ETS_SPI_INTR_ENABLE();
			}
			return;
		}
	}

	// Otherwise block and poll
#ifdef HSPI_ENABLE_STATS
	CpuCycleTimer timer;
#endif
	while(req.busy) {
		isr(this);
	}
#ifdef HSPI_ENABLE_STATS
	stats.waitCycles += timer.elapsedTicks();
#endif
}

void IRAM_ATTR Controller::queueTask()
{
	if(!flags.taskQueued) {
		System.queueCallback([](void* param) { static_cast<Controller*>(param)->executeTask(); }, this);
		flags.taskQueued = true;
#ifdef HSPI_ENABLE_STATS
		++stats.tasksQueued;
#endif
	}
}

/*
 * Called from task context to execute request in blocking mode.
 * This is appropriate when transaction time is very short due to relative ISR overhead.
 */
void Controller::executeTask()
{
	flags.taskQueued = false;

	auto req = trans.request;
	if(req == nullptr) {
		// No current transaction - blocking call to execute() will have pre-empted this task
#ifdef HSPI_ENABLE_STATS
		++stats.tasksCancelled;
#endif
		return;
	}

	// Block and poll, but only on this request
#ifdef HSPI_ENABLE_STATS
	CpuCycleTimer timer;
#endif
	while(req == trans.request) {
		isr(this);
	}
	assert(!req->busy);
#ifdef HSPI_ENABLE_STATS
	stats.waitCycles += timer.elapsedTicks();
#endif
}

/*
 * Start transfer of a new request (trans.request)
 * May be called from interrupt context at completion of previous request
 */
void IRAM_ATTR Controller::startRequest()
{
	TESTPIN1_HIGH();

	auto& req = *trans.request;
	auto& dev = *req.device;
	auto& cfg = dev.config;

	trans.addr = req.addr;
	trans.outOffset = 0;
	trans.inOffset = 0;
	trans.inlen = 0;
	trans.ioMode = dev.getIoMode();
	trans.bitOrder = dev.getBitOrder();
	trans.busy = true;

	auto pinSet = dev.pinSet;
	if(pinSet != activePinSet) {
		if(activePinSet == PinSet::overlap) {
			overlapEnable(false);
		}

		// New pin set
		if(pinSet == PinSet::overlap) {
			overlapEnable(true);
		}

		activePinSet = pinSet;
	}

	// Clock
	auto ioMux = READ_PERI_REG(PERIPHS_IO_MUX_CONF_U);
	spi_dev_t::clock_t clk{.val{cfg.reg.clock}};
	if(clk.clk_equ_sysclk) {
		ioMux |= SPI1_CLK_EQU_SYS_CLK;
	} else {
		ioMux &= ~SPI1_CLK_EQU_SYS_CLK;

		// In overlap mode, SPI0 sysclock selection overrides SPI1
		if(!flags.spi0ClockChanged && activePinSet == PinSet::overlap) {
			if(ioMux & SPI0_CLK_EQU_SYS_CLK) {
				SPI0.clock.val = clkDiv2.val;
				ioMux &= ~SPI0_CLK_EQU_SYS_CLK;
				flags.spi0ClockChanged = true;
			}
		}
	}
	WRITE_PERI_REG(PERIPHS_IO_MUX_CONF_U, ioMux);
	SPI1.clock.val = cfg.reg.clock;

	SPI1.ctrl.val = cfg.reg.ctrl;
	SPI1.pin.val = cfg.reg.pin;

	spi_dev_t::user_t user{.val{cfg.reg.user}};
	spi_dev_t::user1_t user1{.val{cfg.reg.user1}};

	if(trans.ioMode == IoMode::SDI || trans.ioMode == IoMode::SQI) {
		// Setup command bits
		if(req.cmdLen == 16) {
			auto cmd = req.cmd;
			if(trans.bitOrder == MSBFIRST) {
				trans.cmd[0] = (req.cmd >> 8);
				trans.cmd[1] = req.cmd;
			} else {
				trans.cmd[0] = req.cmd;
				trans.cmd[1] = (req.cmd >> 8);
			}
		} else if(req.cmdLen == 8) {
			trans.cmd[0] = req.cmd;
		}

		user.usr_command = false;
		user.usr_addr = false;
		user.usr_mosi = true;
	} else {
		// Setup command bits
		if(req.cmdLen != 0) {
			uint16_t cmd{req.cmd};
			if(trans.bitOrder == MSBFIRST) {
				// Command sent bit 7->0 then 15->8 so adjust ordering
				cmd = bswap16(cmd << (16 - req.cmdLen));
			}
			spi_dev_t::user2_t user2{};
			user2.usr_command_value = cmd;
			user2.usr_command_bitlen = req.cmdLen - 1;
			SPI1.user2.val = user2.val;
			user.usr_command = true;
		} else {
			user.usr_command = false;
		}

		// Setup address bits
		if(req.addrLen != 0) {
			user1.usr_addr_bitlen = req.addrLen - 1;
			user.usr_addr = true;
		} else {
			user.usr_addr = false;
		}
	}

	// Setup dummy bits
	if(req.dummyLen != 0) {
		user1.usr_dummy_cyclelen = req.dummyLen - 1;
		user.usr_dummy = true;
	} else {
		user.usr_dummy = false;
	}

	cfg.reg.user = user.val;
	cfg.reg.user1 = user1.val;

	if(trans.ioMode == IoMode::SDI || trans.ioMode == IoMode::SQI) {
		nextTransactionSDQI();
	} else {
		nextTransaction();
	}
}

void IRAM_ATTR Controller::nextTransaction()
{
	auto& req = *trans.request;
	auto& cfg = req.device->config;

	spi_dev_t::user_t user{.val{cfg.reg.user}};
	spi_dev_t::user1_t user1{.val{cfg.reg.user1}};

	// Setup outgoing data (MOSI)
	unsigned outlen = req.out.length - trans.outOffset;
	if(outlen != 0) {
		if(req.out.isPointer) {
			outlen = std::min(outlen, SPI_BUFSIZE);
			memcpy((void*)SPI1.data_buf, req.out.ptr8 + trans.outOffset, ALIGNUP4(outlen));
		} else {
			SPI1.data_buf[0] = req.out.data32;
		}
		user1.usr_mosi_bitlen = (outlen * 8) - 1;
		trans.outOffset += outlen;
		user.usr_mosi = true;
	} else {
		user.usr_mosi = false;
	}

	// Setup incoming data (MISO)
	unsigned inlen = req.in.length - trans.inOffset;
	if(inlen != 0) {
		inlen = std::min(inlen, SPI_BUFSIZE);
		trans.inlen = inlen;
		// In duplex mode data is read during MOSI stage
		if(user.duplex) {
			if(inlen > outlen) {
				user1.usr_mosi_bitlen = (inlen * 8) - 1;
			}
			user.usr_miso = false;
		} else {
			user1.usr_miso_bitlen = (inlen * 8) - 1;
			user.usr_miso = true;
		}
	} else {
		user.usr_miso = false;
	}

	// Setup address
	if(req.addrLen != 0) {
		uint32_t addr = trans.addr;

		if(trans.bitOrder == MSBFIRST) {
			// Address sent MSB to LSB of register value, so shift up as required
			addr <<= 32 - req.addrLen;
		}

		SPI1.addr = addr;

		trans.addr += std::max(outlen, inlen);
	}

	SPI1.user1.val = user1.val;
	SPI1.user.val = user.val;

	// Execute now
	TESTPIN2_HIGH();
	SPI1.cmd.usr = true;

#ifdef HSPI_ENABLE_STATS
	++stats.transCount;
#endif
}

/*
 * Hardware only supports 1-bit command phase so emulation required for SDI and SQI modes
 */
void IRAM_ATTR Controller::nextTransactionSDQI()
{
	auto& req = *trans.request;
	auto& cfg = req.device->config;

	spi_dev_t::user_t user{.val{cfg.reg.user}};
	spi_dev_t::user1_t user1{.val{cfg.reg.user1}};

	uint8_t buffer[SPI_BUFSIZE];
	buffer[0] = trans.cmd[0];
	buffer[1] = trans.cmd[1];
	auto bufPtr = &buffer[req.cmdLen / 8];

	// Setup address
	if(req.addrLen != 0) {
		uint32_t addr = trans.addr;
		if(trans.bitOrder == MSBFIRST) {
			addr = bswap32(addr) >> (32 - req.addrLen);
		}
		memcpy(bufPtr, &addr, sizeof(addr));
		bufPtr += req.addrLen / 8;
	}

	// Setup outgoing data (MOSI)
	size_t outlen = req.out.length - trans.outOffset;
	if(outlen != 0) {
		if(req.out.isPointer) {
			outlen = std::min(outlen, SPI_BUFSIZE - (bufPtr - buffer));
			memcpy(bufPtr, req.out.ptr8 + trans.outOffset, outlen);
		} else {
			memcpy(bufPtr, req.out.data, sizeof(uint32_t));
		}
		bufPtr += outlen;
		trans.outOffset += outlen;
	}

	auto buflen = bufPtr - buffer;
	memcpy((void*)SPI1.data_buf, buffer, ALIGNUP4(buflen));
	user1.usr_mosi_bitlen = (buflen * 8) - 1;

	// Setup incoming data (MISO)
	size_t inlen = req.in.length - trans.inOffset;
	if(inlen != 0) {
		inlen = std::min(inlen, SPI_BUFSIZE);
		trans.inlen = inlen;
		user1.usr_miso_bitlen = (inlen * 8) - 1;
		user.usr_miso = true;
	} else {
		user.usr_miso = false;
	}

	trans.addr += std::max(outlen, inlen);

	SPI1.user1.val = user1.val;
	SPI1.user.val = user.val;

	// Execute now
	TESTPIN2_HIGH();
	SPI1.cmd.usr = true;

#ifdef HSPI_ENABLE_STATS
	++stats.transCount;
#endif
}

/*
 * Interrupt on transaction complete
 */
void IRAM_ATTR Controller::isr(Controller* spi)
{
	if(READ_PERI_REG(DPORT_SPI_INT_STATUS_REG) & DPORT_SPI_INT_STATUS_SPI1) {
		SPI1.slave.trans_done = 0;
		spi->transactionDone();
	}
}

/*
 * Read incoming data, if there is any, and start next transac/tion.
 * Called from interrupt context at completion of transaction.
 */
void IRAM_ATTR Controller::transactionDone()
{
	TESTPIN2_LOW();

	if(trans.request == nullptr) {
		return;
	}

	auto& req = *trans.request;

	// Read incoming data
	if(trans.inlen != 0) {
		if(req.in.isPointer) {
			memcpy(req.in.ptr8 + trans.inOffset, (const void*)SPI1.data_buf, ALIGNUP4(trans.inlen));
		} else {
			req.in.data32 = SPI1.data_buf[0];
		}
		trans.inOffset += trans.inlen;
		trans.inlen = 0;
	}

	// Packet complete?
	if(trans.inOffset >= req.in.length && trans.outOffset >= req.out.length) {
		TESTPIN1_LOW();
		trans.busy = false;
		req.busy = false;
#ifdef HSPI_ENABLE_STATS
		++stats.requestCount;
#endif
		// Note next packet in chain before invoking callback
		trans.request = req.next;
		req.next = nullptr;
		req.device->transferComplete(req);
		// Start the next packet, if there is one
		if(trans.request != nullptr) {
			if(trans.request->task) {
				ETS_SPI_INTR_DISABLE();
				startRequest();
				queueTask();
			} else {
				startRequest();
				ETS_SPI_INTR_ENABLE();
			}
		} else if(flags.spi0ClockChanged) {
			// All transfers have completed, set SPI0 clock back to full speed
			SET_PERI_REG_MASK(PERIPHS_IO_MUX_CONF_U, SPI0_CLK_EQU_SYS_CLK);
			flags.spi0ClockChanged = false;
		}
	} else if(trans.ioMode == IoMode::SDI || trans.ioMode == IoMode::SQI) {
		nextTransactionSDQI();
	} else {
		nextTransaction();
	}
}

} // namespace HSPI
