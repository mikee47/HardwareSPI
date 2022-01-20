/****
 * Device.h
 *
 * Copyright 2018 mikee47 <mike@sillyhouse.net>
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
 ****/

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
	}

	virtual ~Device()
	{
		end();
	}

	/**
	 * @brief Register device with controller and prepare for action
	 * @param pinSet Use PinSet::normal for Esp32, other values for Esp8266
	 * @param chipSelect Identifies the CS number for ESP8266, or the GPIO pin for ESP32
	 * @param clockSpeed Bus speed
	 */
	bool begin(PinSet pinSet, uint8_t chipSelect, uint32_t clockSpeed)
	{
		return controller.startDevice(*this, pinSet, chipSelect, clockSpeed);
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

	uint32_t getSpeed() const
	{
		return speed;
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

	BitOrder getBitOrder() const
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

	/**
	 * @brief Return set of IO modes supported by a device implementation
	 */
	virtual IoModes getSupportedIoModes() const = 0;

	/**
	 * @brief Determine if the device/controller combination supports an IO mode
	 * Must be called after `begin()` as other settings (e.g. pinset) can affect support.
	 */
	bool isSupported(IoMode mode) const
	{
		return controller.getSupportedIoModes(*this)[mode];
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

	/**
	 * @brief Set a callback to be invoked before a request is started, and when it has finished
	 * @param callback Invoked in interrupt context, MUST be in IRAM
	 */
	void onTransfer(Callback callback)
	{
		transferCallback = callback;
	}

	void wait(Request& request)
	{
		if(request.busy) {
			controller.wait(request);
		}
	}

	Controller& controller;

protected:
	friend Controller;

	void IRAM_ATTR transferStarting(Request& request)
	{
		if(transferCallback) {
			transferCallback(request);
		}
	}

	bool IRAM_ATTR transferComplete(Request& request)
	{
		if(transferCallback && !transferCallback(request)) {
			// Re-submit this request
			return false;
		}
		if(request.callback && !request.callback(request)) {
			return false;
		}
		// All done with this request
		return true;
	}

private:
	DeviceConfig config{}; ///< Private config used by Controller
	PinSet pinSet{PinSet::none};
	uint8_t chipSelect{255};
	uint32_t speed{0};
	BitOrder bitOrder{MSBFIRST};
	ClockMode clockMode{};
	IoMode ioMode{};
	Callback transferCallback{nullptr};
};

} // namespace HSPI
