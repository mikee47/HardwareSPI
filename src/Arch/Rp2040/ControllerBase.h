/****
 * ControllerBase.h - Rp2040
 *
 * Copyright 2022 mikee47 <mike@sillyhouse.net>
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
 * @author: January 2022 - mikee47 <mike@sillyhouse.net>
 *
 ****/

#pragma once

#include <cstdint>
#include <hardware/dma.h>

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
	MAX = 2,
	DEFAULT = SPI1,
};

struct DeviceConfig {
	uint16_t cr0val{0};
	uint8_t clk_prescale;
	uint8_t clk_postdiv;
	bool dirty{true}; ///< Set when values require updating
};

class ControllerBase
{
protected:
	struct Flags {
		bool initialised : 1;
	};
	struct RxControlBlock {
		const void* buffer;
		uint32_t len;
	};
	struct DMA {
		dma_channel_config tx_config;
		dma_channel_config tx_config_dummy;
		dma_channel_config rx_config;
		dma_channel_config rx_config_dummy;
		dma_channel_config rx_config_chain;
		dma_channel_config ctrl_config;
		uint8_t tx_channel;
		uint8_t rx_channel;
		uint8_t ctrl_channel;
		alignas(8) RxControlBlock control_blocks[3];
		alignas(8) uint32_t buffer[2];
	};

	DMA dma{};
	uint8_t deviceCount{0};
	Flags flags{};
};

} // namespace HSPI
