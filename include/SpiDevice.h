/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * SpiDevice.h
 *
 * @author: 11 December 2018 - mikee47 <mike@sillyhouse.net>
 *
 * SPI slave device management on hardware SPI bus (HSPI).
 *
 * Device specifies how bus will be used:
 *
 * 	- Bit order
 * 	- Frequency
 * 	- Data mode
 * 	- Port selection: SPI1 is the regular HSPI pin connections, SPI0 uses overlapped mode
 * 	- Chip select: Automatic (CS2) which is mandatory for SPI0, or manual
 *
 * Device classes may be defined to add specific functionality, such as mode switching.
*/

/** @defgroup hw_spi SPI Hardware support
 *  @brief    Provides hardware SPI support
 */

#pragma once

#include "SpiMaster.h"
#include <WConstants.h>

class SpiDevice
{
public:
	SpiDevice()
	{
	}

	virtual ~SpiDevice()
	{
	}

	/** @brief Initialise the device but don't perform any transactions
	 *  @param spi
	 *  @note Call this before setting clock speed or starting any transactions
	 */
	void begin(SpiMaster* spi)
	{
		this->spi = spi;
	}

	/** @brief Set maximum operating speed for device
	 *  @param speed in Hz
	 */
	void setSpeed(uint32_t speed)
	{
		clockReg = spi->frequencyToClkReg(speed);
	}

	uint32_t getSpeed()
	{
		return spi->clkRegToFreq(clockReg);
	}

	uint32_t getClockReg()
	{
		return clockReg;
	}

	/*
	 * Byte ordering is consistent with processor, i.e. always LSB first, but bit ordering
	 * is variable.
	 */
	void setBitOrder(SpiBitOrder bitOrder)
	{
		this->bitOrder = bitOrder;
	}

	SpiBitOrder getBitOrder()
	{
		return bitOrder;
	}

	void setMode(SpiMode mode)
	{
		this->mode = mode;
	}

	SpiMode getMode()
	{
		return mode;
	};

	void execute(SpiPacket& packet)
	{
		packet.device = this;
		spi->execute(packet);
	}

protected:
	friend SpiMaster;

	virtual void transferComplete(SpiPacket& packet)
	{
		if(packet.callback) {
			packet.callback(packet);
		}
	}

private:
	SpiMaster* spi{nullptr};
	uint32_t clockReg{0}; ///< Computed value for a given bus speed
	SpiBitOrder bitOrder{MSBFIRST};
	SpiMode mode{SPI_MODE0};
};
