/****
 * ControllerBase.h - Esp8266
 *
 * Copyright 2018 mikee47 <mike@sillyhouse.net>
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
 ****/

#pragma once

#include <bitset>

namespace HSPI
{
/**
 * @brief Identifies bus selection
 */
enum class SpiBus {
	INVALID = 0,
	MIN = 1,
	SPI1 = 1,
	MAX = 1,
	DEFAULT = SPI1,
};

struct DeviceConfig {
	// Pre-calculated register values - see updateConfig()
	struct {
		uint32_t clock{0};
		uint32_t ctrl{0};
		uint32_t pin{0};
		uint32_t user{0};
		uint32_t user1{0};
	} reg;
	bool dirty{true}; ///< Set when values require updating
};

class ControllerBase
{
protected:
	struct Flags {
		bool initialised : 1;
		bool spi0ClockChanged : 1; ///< ESP8266: SPI0 clock MUX setting was changed for a transaction
		bool taskQueued : 1;	   ///< ESP8266
	};

	uint8_t normalDevices{0};		 ///< Number of registered devices using dedicated SPI pins
	uint8_t overlapDevices{0};		 ///< Number of registered devices using overlap pins (SPI0)
	std::bitset<8> chipSelectsInUse; ///< Ensures each CS is used only once
	Flags flags{};
};

} // namespace HSPI
