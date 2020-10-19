/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * Common.cpp
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
