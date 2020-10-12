/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * SpiMaster.cpp
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

#include "SpiMaster.h"
#include "SpiDevice.h"
#include <esp_clk.h>
#include <esp_systemapi.h>
#include <espinc/spi_register.h>
#include <espinc/spi_struct.h>
#include <espinc/pin_mux_register.h>
#include <Platform/Timers.h>

volatile SpiStats SpiMaster::stats;

// GPIO pin numbers for SPI
#define PIN_SPI_CS2 0
#define PIN_HSPI_MISO 12
#define PIN_HSPI_MOSI 13
#define PIN_HSPI_CLK 14
#define PIN_HSPI_CS 15

/* SPI interrupt status register address definition for determining the interrupt source */
#define DPORT_SPI_INT_STATUS_REG 0x3ff00020
#define DPORT_SPI_INT_STATUS_SPI0 BIT4
#define DPORT_SPI_INT_STATUS_SPI1 BIT7

//#define SPI_ENABLE_TEST_PIN

#ifdef SPI_ENABLE_TEST_PIN
// For testing ISR latency, etc.
#define PIN_ISR_TEST 4

#define TESTPIN_HIGH()
#define TESTPIN_LOW()
#define TESTPIN_TOGGLE() GPO ^= BIT(PIN_ISR_TEST)
#else
#define TESTPIN_HIGH()
#define TESTPIN_LOW()
#define TESTPIN_TOGGLE()
#endif

// SPI FIFO
#define SPI_BUFSIZE sizeof(SPI1.data_buf)

/* SpiMaster */

void SpiMaster::begin(SpiPinSet pinSet)
{
	// Don't proceed unless hardware configuration is being changed
	if(this->pinSet == pinSet) {
		return;
	}

	// Disable current pin set first
	end();

	SPI1.user.val = 0;
	//	SPIUMOSI | SPIUDUPLEX; // | SPIUSSE;

	switch(pinSet) {
	case SPI_PINS_OVERLAP:
		SET_PERI_REG_MASK(HOST_INF_SEL, PERI_IO_CSPI_OVERLAP);

		// Enable Hardware CS
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_SPI_CS2), FUNC_SPICS2); // GPI0 to SPICS2 mode
		SPI1.pin.cs0_dis = 1;
		SPI1.pin.cs1_dis = 1;
		SPI1.pin.cs2_dis = 0;
		// Prioritise SPI over HSPI transactions
		SPI0.ext3.int_hold_ena = 1;
		SPI1.ext3.int_hold_ena = 3;
		SPI1.user.cs_setup = 0; //1;
		SPI1.user.cs_hold = 0;  //1;

		// From ESP32 code: 'we do need at least one clock of hold time in most cases'
		SPI1.ctrl2.val = 0;
		//		SPI1.ctrl2.X_hold_time = 2;
		//		SPI1.ctrl2.X_setup_time = 2;
		break;

	case SPI_PINS_NORMAL_CS_AUTO:
	case SPI_PINS_NORMAL_CS_MANUAL:
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_MISO), FUNC_HSPIQ_MISO); // HSPIQ MISO == GPIO12
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_MOSI), FUNC_HSPID_MOSI); // HSPID MOSI == GPIO13
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_CLK), FUNC_HSPI_CLK);	// CLK		 == GPIO14

		// CS setup
		SPI1.pin.cs1_dis = 1;
		SPI1.pin.cs2_dis = 1;

		if(pinSet == SPI_PINS_NORMAL_CS_AUTO) {
			// Enable hardware CS
			PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_CS), FUNC_HSPI_CS0); // HSPICS == GPIO15
			CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX_CONF_U, SPI1_CLK_EQU_SYS_CLK);
			SPI1.pin.cs0_dis = 0;
			SPI1.user.cs_setup = 1;
			SPI1.user.cs_hold = 1;
		} else {
			SPI1.pin.cs0_dis = 1;
			PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_CS), FUNC_GPIO15);
		}
		break;

	case SPI_PINS_NONE:
	default:
		return; // Do nothing
	}

	SPI1.ctrl.val = 0;
	//	SPI1C = 0;
	//	setFrequency(1000000); ///< 1MHz
	//	SPI1.user1.usr_mosi_bitlen = 8 - 1;
	//	SPI1.user1.usr_miso_bitlen = 8 - 1;
	//	SPI1U1 = (7 << SPILMOSI) | (7 << SPILMISO);
	SPI1.ctrl1.val = 0;

	this->pinSet = pinSet;

	// Configure interrupts

	//	debug_i("SPI0.slave = 0x%08x", SPI0.slave.val);
	SPI0.slave.val &= ~0x000003FF; // Don't want interrupts from SPI0

	SPI1.slave.val &= ~0x000003FF; // Clear all interrupt sources
	SPI1.slave.trans_inten = 1;	// Interrupt on command completion
								   // For testing, we'll be toggling a pin
#ifdef SPI_ENABLE_TEST_PIN
	GPIO_OUTPUT_SET(PIN_ISR_TEST, 0);
#endif
	ETS_SPI_INTR_ATTACH(ets_isr_t(isr), this);
	ETS_SPI_INTR_ENABLE();
}

void IRAM_ATTR SpiMaster::isr(SpiMaster* spi)
{
	if(READ_PERI_REG(DPORT_SPI_INT_STATUS_REG) & DPORT_SPI_INT_STATUS_SPI1) {
		SPI1.slave.trans_done = 0;
		spi->transfer();
	}

	//   	if (READ_PERI_REG(DPORT_SPI_INT_STATUS_REG) & DPORT_SPI_INT_STATUS_SPI0) {
	//   		SPI0.slave.val &= ~0x000003FF;
	//    }
}

void SpiMaster::end()
{
	switch(pinSet) {
	case SPI_PINS_NORMAL_CS_AUTO:
	case SPI_PINS_NORMAL_CS_MANUAL:
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_CLK), FUNC_GPIO14);
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_MISO), FUNC_GPIO12);
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_HSPI_MOSI), FUNC_GPIO13);

		// Disable hardware CS
		if(pinSet == SPI_PINS_NORMAL_CS_AUTO) {
			SPI1.pin.cs0_dis = 1;
			SPI1.pin.cs1_dis = 1;
			SPI1.pin.cs2_dis = 1;
			GPIO_OUTPUT_SET(PIN_HSPI_CS, 1);
			SPI1.user.cs_setup = 0;
			SPI1.user.cs_hold = 0;
		}
		break;

	case SPI_PINS_OVERLAP:
		// Disable HSPI overlap
		CLEAR_PERI_REG_MASK(HOST_INF_SEL, PERI_IO_CSPI_OVERLAP);

		// Disable hardware CS
		SPI1.pin.cs0_dis = 1;
		SPI1.pin.cs1_dis = 1;
		SPI1.pin.cs2_dis = 1;
		PIN_FUNC_SELECT(PERIPHS_GPIO_MUX_REG(PIN_SPI_CS2), FUNC_GPIO0);
		// De-prioritise SPI vs HSPI
		SPI0.ext3.int_hold_ena = 0;
		SPI1.ext3.int_hold_ena = 0;
		SPI1.user.cs_setup = 0;
		SPI1.user.cs_hold = 0;
		break;

	case SPI_PINS_NONE:
	default:; // Do nothing
	}

	pinSet = SPI_PINS_NONE;
}

static uint32_t getClockFrequency(const spi_dev_t::clock_t clk)
{
	return APB_CLK_FREQ / ((clk.clkdiv_pre + 1) * (clk.clkcnt_n + 1));
}

uint32_t SpiMaster::clkRegToFreq(uint32_t regVal)
{
	return getClockFrequency({val: regVal});
}

uint32_t SpiMaster::frequencyToClkReg(uint32_t freq)
{
	if(freq >= APB_CLK_FREQ) {
		return SPI_CLK_EQU_SYSCLK;
	}

	spi_dev_t::clock_t minClock{.val = 0x7FFFF000};
	uint32_t minFreq = getClockFrequency(minClock);
	if(freq < minFreq) {
		// use minimum possible clock
		return minClock.val;
	}

	uint8_t calN = 1;

	spi_dev_t::clock_t bestReg{};
	int32_t bestFreq = 0;

	// find the best match
	while(calN <= 0x3F) { // 0x3F max for N

		spi_dev_t::clock_t reg{};
		int32_t calFreq;
		int32_t calPre;
		int8_t calPreVari = -2;

		reg.clkcnt_n = calN;

		// Test different variants for prescale
		while(calPreVari++ <= 1) {
			calPre = (((APB_CLK_FREQ / (reg.clkcnt_n + 1)) / freq) - 1) + calPreVari;
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

			if(calFreq == (int32_t)freq) {
				// accurate match use it!
				bestReg = reg;
				break;
			} else if(calFreq < (int32_t)freq) {
				// never go over the requested frequency
				if(abs(int(freq - calFreq)) < abs(int(freq - bestFreq))) {
					bestFreq = calFreq;
					bestReg = reg;
				}
			}
		}
		if(calFreq == (int32_t)freq) {
			// accurate match use it!
			break;
		}
		calN++;
	}

	//	debug_i("[0x%08X][%d]\t EQU: %d\t Pre: %d\t N: %d\t H: %d\t L: %d\t - Real Frequency: %d\n", bestReg.val, freq,
	//			bestReg.clk_equ_sysclk, bestReg.clkdiv_pre, bestReg.clkcnt_n, bestReg.clkcnt_h, bestReg.clkcnt_l,
	//			getClockFrequency(bestReg));

	return bestReg.val;
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
void SpiMaster::execute(SpiPacket& packet)
{
	packet.next = nullptr;
	packet.busy = 1;
	// Packet transfer already in progress?
	ETS_SPI_INTR_DISABLE();
	if(trans.busy) {
		// Tack new packet onto end of chain
		auto pkt = trans.packet;
		while(pkt->next) {
			pkt = pkt->next;
		}
		pkt->next = &packet;
	} else {
		// Not currently running, so do this one now
		trans.packet = &packet;
		startPacket();
	}
	ETS_SPI_INTR_ENABLE();

	if(!packet.async) {
		CpuCycleTimer timer;
		while(packet.busy) {
			//
		}
		stats.waitCycles += timer.elapsedTicks();
	}
}

void IRAM_ATTR SpiMaster::startPacket()
{
	TESTPIN_TOGGLE();

	if(trans.packet->device != activeDevice) {
		activeDevice = trans.packet->device;

		// Bit order
		trans.bitOrder = activeDevice->getBitOrder();
		SPI1.ctrl.rd_bit_order = (trans.bitOrder == MSBFIRST) ? 0 : 1;
		SPI1.ctrl.wr_bit_order = (trans.bitOrder == MSBFIRST) ? 0 : 1;

		// Byte order
		auto byteOrder = LSBFIRST;
		SPI1.user.wr_byte_order = (byteOrder == MSBFIRST) ? 1 : 0;
		SPI1.user.rd_byte_order = (byteOrder == MSBFIRST) ? 1 : 0;

		// Mode
		auto mode = activeDevice->getMode();
		SPI1.user.ck_out_edge = (mode & 0x01) ? 1 : 0; // CPHA
		SPI1.pin.ck_idle_edge = (mode & 0x10) ? 1 : 0; // CPOL

		// Clock
		auto clockReg = activeDevice->getClockReg();
		auto ioMux = READ_PERI_REG(PERIPHS_IO_MUX_CONF_U);
		if(clockReg == SPI_CLK_EQU_SYSCLK) {
			ioMux |= SPI1_CLK_EQU_SYS_CLK;
		} else {
			ioMux &= ~SPI1_CLK_EQU_SYS_CLK;

			// In overlap mode, SPI0 sysclock selection overrides SPI1
			if(!spi0ClockChanged && pinSet == SPI_PINS_OVERLAP) {
				if(ioMux & SPI0_CLK_EQU_SYS_CLK) {
					spi_dev_t::clock_t div2{{
						.clkcnt_l = 1,
						.clkcnt_h = 0,
						.clkcnt_n = 1,
						.clkdiv_pre = 0,
						.clk_equ_sysclk = 0,
					}};
					SPI0.clock.val = div2.val;
					ioMux &= ~SPI0_CLK_EQU_SYS_CLK;
					spi0ClockChanged = true;
				}
			}
		}
		WRITE_PERI_REG(PERIPHS_IO_MUX_CONF_U, ioMux);
		SPI1.clock.val = clockReg;
	}

	trans.addrOffset = 0;
	trans.outOffset = 0;
	trans.inOffset = 0;
	trans.inlen = 0;
	trans.busy = 1;

	transfer();
}

/** @brief called from ISR
 *  @retval bool true when transaction has finished
 */
void IRAM_ATTR SpiMaster::transfer()
{
	TESTPIN_LOW();
	TESTPIN_HIGH();

	SpiPacket& packet = *trans.packet;

	// Read incoming data
	if(trans.inlen) {
		if(packet.in.isPointer) {
			memcpy(packet.in.ptr8 + trans.inOffset, (const void*)SPI1.data_buf, ALIGNUP4(trans.inlen));
		} else {
			packet.in.data32 = SPI1.data_buf[0];
		}
		trans.inOffset += trans.inlen;
		TESTPIN_HIGH();
	}

	// Packet complete?
	unsigned inlen = packet.in.length - trans.inOffset;
	unsigned outlen = packet.out.length - trans.outOffset;
	if(inlen == 0 && outlen == 0) {
		TESTPIN_LOW();
		trans.busy = 0;
		packet.busy = 0;
		// Note next packet in chain before invoking callback
		trans.packet = packet.next;
		packet.next = nullptr;
		activeDevice->transferComplete(packet);
		// Start the next packet, if there is one
		if(trans.packet != nullptr) {
			startPacket();
		} else {
			// All transfers have completed, set SPI0 clock back to full speed
			if(spi0ClockChanged) {
				SET_PERI_REG_MASK(PERIPHS_IO_MUX_CONF_U, SPI0_CLK_EQU_SYS_CLK);
				spi0ClockChanged = false;
			}
			activeDevice = nullptr;
		}
		return;
	}

	// Set up next transfer

	// Build register values in a temp is faster than modifying registers directly
	struct {
		spi_dev_t::user_t user;
		spi_dev_t::user1_t user1;
	} reg{SPI1.user.val, 0};
	reg.user.duplex = packet.duplex;

	// Setup command bits
	if(packet.cmdLen != 0) {
		uint16_t cmd = packet.cmd;
		if(trans.bitOrder == MSBFIRST) {
			// Command sent bit 7->0 then 15->8 so adjust ordering
			cmd = bswap16(cmd << (16 - packet.cmdLen));
		}
		spi_dev_t::user2_t tmp{};
		tmp.usr_command_value = cmd;
		tmp.usr_command_bitlen = packet.cmdLen - 1;
		SPI1.user2.val = tmp.val;
		reg.user.usr_command = 1;
	} else {
		reg.user.usr_command = 0;
	}

	// Setup address bits
	if(packet.addrLen) {
		uint32_t addr = packet.addr + trans.addrOffset;
		if(trans.bitOrder == MSBFIRST) {
			// Address sent MSB to LSB of register value, so shift up as required
			addr <<= 32 - packet.addrLen;
		}

		reg.user1.usr_addr_bitlen = packet.addrLen - 1;
		SPI1.addr = addr;
		reg.user.usr_addr = 1;
	} else {
		reg.user.usr_addr = 0;
	}

	// Setup dummy bits
	if(packet.dummyLen) {
		reg.user1.usr_dummy_cyclelen = packet.dummyLen - 1;
		reg.user.usr_dummy = 1;
	} else {
		reg.user.usr_dummy = 0;
	}

	// Setup outgoing data (MOSI)
	if(outlen) {
		if(packet.out.isPointer) {
			if(outlen > SPI_BUFSIZE) {
				outlen = SPI_BUFSIZE;
			}
			memcpy((void*)SPI1.data_buf, packet.out.ptr8 + trans.outOffset, ALIGNUP4(outlen));
		} else {
			SPI1.data_buf[0] = packet.out.data32;
		}
		reg.user1.usr_mosi_bitlen = (outlen * 8) - 1;
		trans.outOffset += outlen;
		reg.user.usr_mosi = 1;
	} else {
		reg.user.usr_mosi = 0;
	}

	// Setup incoming data (MISO)
	if(inlen) {
		if(inlen > SPI_BUFSIZE) {
			inlen = SPI_BUFSIZE;
		}
		trans.inlen = inlen;
		// In duplex mode data is read during MOSI stage
		if(reg.user.duplex) {
			reg.user.usr_miso = 0;
			reg.user1.usr_mosi_bitlen = (std::max(inlen, outlen) * 8) - 1;
		} else {
			reg.user1.usr_miso_bitlen = (inlen * 8) - 1;
			reg.user.usr_miso = 1;
		}
	} else {
		reg.user.usr_miso = 0;
	}

	SPI1.user1.val = reg.user1.val;
	SPI1.user.val = reg.user.val;

	// Execute now
	TESTPIN_LOW();
	SPI1.cmd.usr = 1;
	TESTPIN_HIGH();

	/*
	 * This caters for in-only, out-only or (for full duplex modes) in/out transactions.
	 * @todo We should probably identify invalid requests and reject them.
	 */
	trans.addrOffset += std::max(outlen, inlen);

	++stats.transCount;
}
