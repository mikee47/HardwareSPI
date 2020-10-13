/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * Device.h
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

#include "Controller.h"
#include <WConstants.h>

namespace HSPI
{
class Device
{
public:
	Device(Controller& controller) : controller(controller)
	{
	}

	virtual ~Device()
	{
	}

	/** @brief Set maximum operating speed for device
	 *  @param speed in Hz
	 */
	void setSpeed(uint32_t speed)
	{
		clockReg = controller.frequencyToClkReg(speed);
	}

	uint32_t getSpeed()
	{
		return controller.clkRegToFreq(clockReg);
	}

	uint32_t getClockReg()
	{
		return clockReg;
	}

	/*
	 * Byte ordering is consistent with processor, i.e. always LSB first, but bit ordering
	 * is variable.
	 */
	void setBitOrder(BitOrder bitOrder)
	{
		this->bitOrder = bitOrder;
	}

	BitOrder getBitOrder()
	{
		return bitOrder;
	}

	void setMode(Mode mode)
	{
		this->mode = mode;
	}

	Mode getMode()
	{
		return mode;
	};

	void execute(Packet& packet)
	{
		packet.device = this;
		controller.execute(packet);
	}

protected:
	friend Controller;

	virtual void transferComplete(Packet& packet)
	{
		if(packet.callback) {
			packet.callback(packet);
		}
	}

private:
	Controller& controller;
	uint32_t clockReg{0}; ///< Computed value for a given bus speed
	BitOrder bitOrder{MSBFIRST};
	Mode mode{Mode0};
};

} // namespace HSPI
