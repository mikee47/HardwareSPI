/****
 * ControllerBase.h - Host
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

#include <cstddef>
#include <memory>

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
};

class AsyncThread;

class ControllerBase
{
public:
	ControllerBase();
	~ControllerBase();

protected:
	struct Flags {
		bool initialised : 1;
	};

	std::unique_ptr<AsyncThread> thread;
	Flags flags{};
};

} // namespace HSPI
