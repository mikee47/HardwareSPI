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
		// Set a default speed
		setSpeed(1000000U);
	}

	bool begin(PinSet pinSet, uint8_t chipSelect)
	{
		return controller.startDevice(*this, pinSet, chipSelect);
	}

	void end()
	{
		controller.stopDevice(*this);
	}

	virtual ~Device()
	{
		end();
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

	void setClockMode(ClockMode mode)
	{
		clockMode = mode;
	}

	ClockMode getClockMode() const
	{
		return clockMode;
	};

	void setIoMode(IoMode mode)
	{
		ioMode = mode;
	}

	IoMode getIoMode() const
	{
		return ioMode;
	}

	void execute(Request& request)
	{
		request.device = this;
		controller.execute(request);
	}

protected:
	friend Controller;

	virtual void transferComplete(Request& request)
	{
		if(request.callback) {
			request.callback(request);
		}
	}

private:
	Controller& controller;
	PinSet pinSet{PinSet::None};
	uint8_t chipSelect;
	uint32_t clockReg; ///< Computed value for a given bus speed
	BitOrder bitOrder{MSBFIRST};
	ClockMode clockMode{0};
	IoMode ioMode{IoMode::SPI};
};

} // namespace HSPI
