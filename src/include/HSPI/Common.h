/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * Common.h - Architecture-independent definitions
 *
 * @author: 11 December 2018 - mikee47 <mike@sillyhouse.net>
 *
 */

#pragma once

#include <WString.h>

namespace HSPI
{
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
	DUAL,  ///< Two bits per clock for Data, 1-bit for Command and Address
	DIO,   ///< Two bits per clock for Address and Data, 1-bit for Command
	SDI,   ///< Two bits per clock for Command, Address and Data
	QUAD,  ///< Four bits per clock  for Data, 1-bit for Command and Address
	QIO,   ///< Four bits per clock for Address and Data, 1-bit for Command
	SQI,   ///< Four bits per clock for Command, Address and Data
};

/*
 * Details for each IO Mode
 *
 * Mode, clock bits, address bits, databits, duplex
 */
#define HSPI_IOMODE_MAP(XX)                                                                                            \
	XX(SPI, 1, 1, 1, true)                                                                                             \
	XX(SPIHD, 1, 1, 1, false)                                                                                          \
	XX(DUAL, 1, 1, 2, false)                                                                                           \
	XX(DIO, 1, 2, 2, false)                                                                                            \
	XX(SDI, 2, 2, 2, false)                                                                                            \
	XX(QUAD, 1, 1, 4, false)                                                                                           \
	XX(QIO, 1, 4, 4, false)                                                                                            \
	XX(SQI, 4, 4, 4, false)

struct IoModeInfo {
	const FlashString* name;
	IoMode mode;
	uint8_t clockBits : 3;
	uint8_t addrressBits : 3;
	uint8_t dataBits : 3;
	bool duplex : 1;
};

const IoModeInfo getIoModeInfo(IoMode mode);

inline String toString(IoMode mode)
{
	return *getIoModeInfo(mode).name;
}

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

// Note: These are faster than __builtin_bswapNN functions

inline uint16_t bswap16(uint16_t value)
{
	return (value >> 8) | (value << 8);
}

inline uint32_t bswap32(uint32_t value)
{
	return (value >> 24) | ((value >> 8) & 0xff00) | ((value << 8) & 0xff0000) | (value << 24);
}

} // namespace HSPI
