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
#include <Timer.h>

namespace HSPI
{
namespace
{
SimpleTimer asyncTimer;
}

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
	dev.pinSet = PinSet::none;
	dev.chipSelect = 255;
}

void Controller::configChanged(Device& dev)
{
	dev.config.dirty = true;
}

uint32_t Controller::setSpeed(Device& dev, uint32_t frequency)
{
	dev.config.reg.clock = frequency;
	return frequency;
}

uint32_t Controller::getSpeed(Device& dev) const
{
	return dev.config.reg.clock;
}

void Controller::execute(Request& req)
{
	FUNC("%p", &req);

	assert(!req.busy);
	assert(req.device != nullptr);

	debug_i("req .out = %p, %u; .in = %p, %u; .callback = %p, %p; async = %u", req.out.get(), req.out.length,
			req.in.get(), req.in.length, req.callback, req.param, req.async);
	if(req.out.length > 0) {
		debug_hex(INFO, "OUT", req.out.get(), std::min(req.out.length, uint16_t(32)), -1, 32);
	}

	req.busy = true;

	// Packet transfer already in progress?
	if(trans.busy) {
		// Tack new packet onto end of chain
		auto pkt = trans.request;
		while(pkt->next) {
			pkt = pkt->next;
		}
		pkt->next = &req;
	} else {
		// Not currently running, so do this one now
		trans.request = &req;
		startRequest();
	}

	if(req.async) {
		asyncTimer.initializeMs<10>(
			[](void* param) {
				auto spi = static_cast<Controller*>(param);
				spi->transactionDone();
			},
			this);
		asyncTimer.startOnce();
	} else {
		transactionDone();
	}
}

void Controller::startRequest()
{
	trans.busy = true;
	auto& req = *trans.request;
	auto& dev = *req.device;
	if(selectDeviceCallback) {
		selectDeviceCallback(dev.chipSelect, true);
	}
	dev.transferStarting(req);
}

void Controller::transactionDone()
{
	asyncTimer.stop();

	while(trans.request != nullptr) {
		auto& req = *trans.request;
		auto& dev = *req.device;
		if(selectDeviceCallback) {
			selectDeviceCallback(dev.chipSelect, false);
		}
		trans.request = req.next;
		req.next = nullptr;
		req.busy = false;
		dev.transferComplete(req);
	}
	trans.busy = false;
};

} // namespace HSPI
