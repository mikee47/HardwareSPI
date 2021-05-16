/**
 * Common.cpp
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
 */

#include "include/HSPI/Common.h"
#include <FlashString/Array.hpp>

namespace HSPI
{
namespace
{
#define XX(name, clk, addr, data, duplex) DEFINE_FSTR_LOCAL(ioModeStr_##name, #name)
HSPI_IOMODE_MAP(XX)
#undef XX

#define XX(name, clk, addr, data, duplex) {&ioModeStr_##name, IoMode::name, clk, addr, data, duplex},
DEFINE_FSTR_ARRAY(ioModeInfo, IoModeInfo, HSPI_IOMODE_MAP(XX))
#undef XX
} // namespace

const IoModeInfo getIoModeInfo(IoMode mode)
{
	return ioModeInfo[unsigned(mode)];
}

} // namespace HSPI
