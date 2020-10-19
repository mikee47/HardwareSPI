/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * Controller.cpp
 *
 * @author: 11 December 2018 - mikee47 <mike@sillyhouse.net>
 *
 */

#include <HSPI/Controller.h>
#include <HSPI/Device.h>
#include <debug_progmem.h>

namespace HSPI
{
#ifdef HSPI_ENABLE_STATS
volatile Controller::Stats Controller::stats;
#endif

void Controller::begin()
{
}

void Controller::end()
{
}

#define FUNC(fmt, ...) debug_i("Controller::%s(" fmt ")", __FUNCTION__, __VA_ARGS__);

bool Controller::startDevice(Device& dev, PinSet pinSet, uint8_t chipSelect)
{
	FUNC("%p, %u, %u", &dev, pinSet, chipSelect)
	dev.pinSet = pinSet;
	dev.chipSelect = chipSelect;
	return true;
}

void Controller::stopDevice(Device& dev)
{
	FUNC("%p, %u, %u", &dev, dev.pinSet, dev.chipSelect);
	dev.pinSet = PinSet::None;
	dev.chipSelect = 255;
}

void Controller::configChanged(Device& dev)
{
	dev.config.dirty = true;
}

void Controller::setSpeed(Device& dev, uint32_t frequency)
{
	dev.config.reg.clock = frequency;
}

uint32_t Controller::getSpeed(Device& dev) const
{
	return dev.config.reg.clock;
}

void Controller::execute(Request& req)
{
	FUNC("%p", &req);
}

} // namespace HSPI
