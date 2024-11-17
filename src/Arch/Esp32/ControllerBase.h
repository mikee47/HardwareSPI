/****
 * ControllerBase.h - Esp32
 *
 * Copyright 2021 mikee47 <mike@sillyhouse.net>
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

#include <memory>
#include <cstdint>
#include <soc/soc_caps.h>

struct intr_handle_data_t;

namespace HSPI
{
/**
 * @brief Identifies bus selection
 */
enum class SpiBus {
	INVALID = 0,
	MIN = 1,
	SPI1 = 1,
	SPI2 = 2,
	SPI3 = 3,
	MAX = SOC_SPI_PERIPH_NUM,
	DEFAULT = SPI2,
};

struct DeviceConfig {
};

struct EspTransaction;

class ControllerBase
{
public:
	ControllerBase();
	~ControllerBase();

	/**
	 * @brief Get the active ESP32 SPI host identifier
	 */
	uint8_t getHost() const;

protected:
	struct Flags {
		bool initialised : 1;
	};

	intr_handle_data_t* intr_handle;
	std::unique_ptr<EspTransaction> esp_trans;
	std::unique_ptr<uint32_t[]> dmaBuffer;
	uint8_t deviceCount{0};
	Flags flags{};
	int errcode{};
};

} // namespace HSPI
