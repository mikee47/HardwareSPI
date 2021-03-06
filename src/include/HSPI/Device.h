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

#pragma once

#include "Controller.h"

namespace HSPI
{
/**
 * @brief Manages a specific SPI device instance attached to a controller
 *
 * @ingroup hw_spi
 */
class Device
{
public:
	Device(Controller& controller) : controller(controller)
	{
		// Set a default speed
		setSpeed(1000000U);
	}

	virtual ~Device()
	{
		end();
	}

	bool begin(PinSet pinSet, uint8_t chipSelect)
	{
		return controller.startDevice(*this, pinSet, chipSelect);
	}

	void end()
	{
		controller.stopDevice(*this);
	}

	/**
	 * @brief Determine if the device is initialised
	 * @retval bool
	 */
	bool isReady() const
	{
		return pinSet != PinSet::none;
	}

	PinSet getPinSet() const
	{
		return pinSet;
	}

	uint8_t getChipSelect() const
	{
		return chipSelect;
	}

	/** @brief Set maximum operating speed for device
	 *  @param speed in Hz
	 */
	void setSpeed(uint32_t frequency)
	{
		speed = controller.setSpeed(*this, frequency);
	}

	uint32_t getSpeed()
	{
		return controller.getSpeed(*this);
	}

	/*
	 * Byte ordering is consistent with processor, i.e. always LSB first, but bit ordering
	 * is variable.
	 */
	void setBitOrder(BitOrder bitOrder)
	{
		if(this->bitOrder != bitOrder) {
			this->bitOrder = bitOrder;
			controller.configChanged(*this);
		}
	}

	BitOrder getBitOrder()
	{
		return bitOrder;
	}

	void setClockMode(ClockMode mode)
	{
		if(clockMode != mode) {
			clockMode = mode;
			controller.configChanged(*this);
		}
	}

	ClockMode getClockMode() const
	{
		return clockMode;
	}

	virtual IoModes getSupportedIoModes() const = 0;

	bool isSupported(IoMode mode) const
	{
		return getSupportedIoModes()[mode];
	}

	virtual bool setIoMode(IoMode mode)
	{
		if(ioMode != mode) {
			ioMode = mode;
			controller.configChanged(*this);
		}
		return true;
	}

	IoMode getIoMode() const
	{
		return ioMode;
	}

	size_t getBitsPerClock() const
	{
		switch(ioMode) {
		case IoMode::SPI:
		case IoMode::SPIHD:
			return 1;
		case IoMode::DUAL:
		case IoMode::DIO:
		case IoMode::SDI:
			return 2;
		case IoMode::QUAD:
		case IoMode::QIO:
		case IoMode::SQI:
			return 4;
		default:
			return 1;
		}
	}

	void execute(Request& request)
	{
		request.device = this;
		controller.execute(request);
	}

	Controller& controller;

protected:
	friend Controller;

	void IRAM_ATTR transferComplete(Request& request)
	{
		if(request.callback) {
			request.callback(request);
		}
	}

private:
	Controller::Config config{}; ///< Private config used by Controller
	PinSet pinSet{PinSet::none};
	uint8_t chipSelect{255};
	uint32_t speed{0};
	BitOrder bitOrder{MSBFIRST};
	ClockMode clockMode{};
	IoMode ioMode{};
};

} // namespace HSPI
