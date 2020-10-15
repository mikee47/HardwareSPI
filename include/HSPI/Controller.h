/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * Controller.h
 *
 * @author: 11 December 2018 - mikee47 <mike@sillyhouse.net>
 *
 * # SPI master-mode hardware controller
 *
 * The ESP8266 SPI hardware is capable of very high transfer speeds and has a number of
 * features which require a more flexible driver to take advantage of. This module,
 * together with Device, provide the following features:
 *
 *  - Support multiple slave devices sharing the same bus
 * 	- Custom CS multiplexing supported via callbacks. For example, routing CS2 via HC138
 * 	3:8 decoder allows 8 (or more) SPI devices to share the same bus.
 * 	- Use of HSPI (SPI1) using either its own pins or sharing pins with SPI0 (overlapped)
 * 	- (Potentially) enabling use of dual/quad operating modes when overlapped
 * 	- Making use of hardware command/address/data phases for best efficiency
 * 	- Pre-calculation of all register values to optimise switching between slave devices
 *  - Write-only transactions can return immediately rather than waiting for the transfer to
 *  complete. The time spent waiting can be used to prepare the next transaction which can
 *  potentially double the throughput
 *  - Interrupt callback on transaction completion. This can be used to improve system efficiency
 *  on slower devices.
 *  - (Potentially) Using DMA for larger read/write transactions. The SDK only demonstrates
 *  DMA for I2S services but it _may_ be possible to use it for SPI.
 *
 *
 * # Transactions
 *
 * Applications call Controller to perform a transfer, or sequence of transfers, as follows:
 *
 *	- Session setup
 *		- Wait for any HSPI transaction to complete (WAIT_READY)
 * 		- Configure clock & mode settings
 * 	- Transaction
 * 		- WAIT_READY
 * 		- Configure command / address / data
 * 		- Start operation
 * 		- If read required:
 * 			- WAIT_READY
 * 			- Copy data from FIFO
 *
 *	Transaction may be repeated for subsequent transfers on same device
 *	CS will be asserted/de-asserted by hardware so not need to end a transaction
 *
 * # Overlapped operation
 *
 * Both SPI controllers are able to share the pin signals from the flash SPI interface (SPI0).
 * This is handled through hardware.
 *
 * Advantages:
 * 	- Gain three pins (GPIO12-14), which liberates the I2S controller
 * 	- Dual and quad SPI modes can be used with HSPI
 *
 * Disadvantages:
 * 	- Slow SPI devices may reduce retrieval speed of program code from Flash memory
 *
 *	A primary IO MUX (PERIPHS_IO_MUX_CONF_U) selects whether the CPU clock goes through the
 * 	SPI clock divider circuitry or not. In overlapped mode the SPI0 setting is used for both,
 * 	therefore as most SPI slave devices will not operate at 80MHz this setting has to be disabled
 * 	to allow the clocks to be set independently. See PERIPHS_IO_MUX_CONF_U.
 *
 * The ESP32 Technical Reference manual gives a very useful insight into how the two SPI
 * devices work together, as the hardware appears largely similar.
 *
 */

/** @defgroup hw_spi SPI Hardware support
 *  @brief    Provides hardware SPI support
 */

#pragma once

#include <stdint.h>
#include <esp_attr.h>
#include "Request.h"
#include <bitset>

namespace HSPI
{
class Device;

//#define SPI_DEBUG  1

/**
 * @brief SPI clock polarity (CPOL) and phase (CPHA)
 */
enum class ClockMode : uint8_t {
	Mode0 = 0x00, ///<  CPOL: 0  CPHA: 0
	Mode1 = 0x01, ///<  CPOL: 0  CPHA: 1
	Mode2 = 0x10, ///<  CPOL: 1  CPHA: 0
	Mode3 = 0x11, ///<  CPOL: 1  CPHA: 1
};

/**
 * @brief Mode of data transfer
 */
enum class IoMode : uint8_t {
	SPI,   ///< One bit per clock, MISO stage concurrent with MISO (full-duplex)
	SPIHD, ///< One bit per clock, MISO stage follows MOSI (half-duplex)
	SDI,   ///< Two bits per clock for Command, Address and Data
	DIO,   ///< Two bits per clock for Address and Data, 1-bit for Command
	DUAL,  ///< Two bits per clock for Data, 1-bit for Command and Address
	SQI,   ///< Four bits per clock for Command, Address and Data
	QIO,   ///< Four bits per clock for Address and Data, 1-bit for Command
	QUAD,  ///< Four bits per clock  for Data, 1-bit for Command and Address
};

/*
 * TODO: Add dual/quad modes, etc.
 */

// 0 for LSBFIRST, non-zero for MSBFIRST
using ByteOrder = uint8_t;
using BitOrder = uint8_t;

/** @brief How SPI hardware pins are connected
 *  @note
 * 		Normal: MISO = GPIO12, MOSI = GPIO13, SCLK = GPIO14
 * 			CS_AUTO: pin 15 HSPI CS managed by hardware
 * 			CS_MANUAL: CS managed by device at start/end of transaction
 *
 * 		Overlap: MISO = SD0, MOSI = SDD1, SCLK = CLK
 * 			CS = CS2, always managed by hardware
 */
enum class PinSet {
	None,	///< Disabled
	Normal,  ///< Standard HSPI pins
	Overlap, ///< Overlapped with SPI 0
};

struct Stats {
	uint32_t waitCycles;
	uint32_t transCount;

	void clear() volatile
	{
		waitCycles = 0;
		transCount = 0;
	}
};

inline uint16_t bswap16(uint16_t data)
{
	return __builtin_bswap16(data);
}

inline uint32_t bswap24(uint32_t data)
{
	return __builtin_bswap32(data) >> 8;
}

inline uint32_t bswap32(uint32_t data)
{
	return __builtin_bswap32(data);
}

/**
 * Transform unsigned integer of length <= 32 bits to the format which can be
 * sent by the SPI driver directly.
 *
 * E.g. to send 9 bits of data, you can:
 *
 *      uint16_t data = SPI_SWAP_DATA_TX(0x145, 9);
 *
 * Then points tx_buffer to ``&data``.
 *
 * @param data Data to be sent, can be uint8_t, uint16_t or uint32_t. @param
 *  len Length of data to be sent, since the SPI peripheral sends from the MSB,
 *  this helps to shift the data to the MSB.
 */
inline uint32_t swapDataTx(uint32_t data, uint8_t len)
{
	return bswap32(data << (32 - len));
}

/**
 * Transform received data of length <= 32 bits to the format of an unsigned integer.
 *
 * E.g. to transform the data of 15 bits placed in a 4-byte array to integer:
 *
 *      uint16_t data = SPI_SWAP_DATA_RX(*(uint32_t*)t->rx_data, 15);
 *
 * @param data Data to be rearranged, can be uint8_t, uint16_t or uint32_t.
 * @param len Length of data received, since the SPI peripheral writes from
 *      the MSB, this helps to shift the data to the LSB.
 */
inline uint32_t swapDataRx(uint32_t data, uint8_t len)
{
	return bswap32(data) >> (32 - len);
}

class Controller
{
public:
	/** @brief  Instantiate hardware SPI object
	 *  @addtogroup hw_spi
	 *  @{
	 */
	Controller()
	{
	}

	virtual ~Controller()
	{
	}

	/* @brief Initialize the HSPI controller
	 */
	void begin();

	/** @brief Disable HSPI controller
	 * 	@note Reverts HSPI pins to GPIO and disables the controller
	 */
	void end();

	bool startDevice(Device& dev, PinSet pinSet, uint8_t chipSelect);

	void stopDevice(Device& dev);

	/** @brief Determine the best clock register value for a desired bus frequency
	 *  @param frequency Desired SPI clock frequency in Hz
	 *  @retval uint32_t opaque 32-bit register setting value
	 *  @note
	 *  	Speed settings are pre-calculated to optimise switching between devices.
	 *  	It is guaranteed that the frequency will not exceed the given target
	 *  	This method must be called with the CPU set to the correct clock frequency
	 *  	in use; if this changes then the calculation will be wrong.
	 */
	virtual uint32_t frequencyToClkReg(uint32_t frequency);

	/** @brief Calculate SPI clock Frequency based on the register value
	 * 	@param regVal
	 * 	@retval uint32_t clock frequency in Hz
	 */
	virtual uint32_t clkRegToFreq(uint32_t regVal);

	static volatile Stats stats;

protected:
	friend Device;

	virtual void execute(Request& request);

private:
	void configurePins(PinSet pinSet);

	/**
	 * @brief Transfer up to SPI_FIFO_LEN bytes
	 */
	static void IRAM_ATTR isr(Controller* spi);

	/**
	 * @brief Start transfer of a new request (trans.request)
	 * @note May be called from interrupt context at completion of previous request
	 */
	void IRAM_ATTR startRequest();

	/**
	 * @brief Read incoming data, if there is any, and start next transaction
	 * @note May be called from interrupt context at completion of previous transaction
	 */
	void IRAM_ATTR transfer();

	Device* activeDevice{nullptr};
	PinSet activePinSet{PinSet::None};
	uint8_t overlapDevices{0};
	uint8_t normalDevices{0};
	std::bitset<8> chipSelectsInUse;
	struct Flags {
		bool spi0ClockChanged : 1; ///< SPI0 clock MUX setting was changed for a transaction
		bool cs0Configured : 1;
		bool cs1Configured : 1;
		bool cs2Configured : 1;
	};
	Flags flags{};

	// State of the current transaction in progress
	struct Transaction {
		Request* request;	///< The current request being executed
		uint16_t addrOffset; ///< Address for next transfer, added to request start address
		uint16_t outOffset;  ///< Where to read data for next outgoing transfer
		uint16_t inOffset;   ///< Where to write incoming data from current transfer
		uint8_t inlen;		 ///< Incoming data for current transfer
		// Flags
		uint8_t bitOrder : 1;
		volatile uint8_t busy : 1;
	};
	Transaction trans{};
};

} // namespace HSPI
