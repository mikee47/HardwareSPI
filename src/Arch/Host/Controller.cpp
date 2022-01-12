/****
 * Controller.cpp - Host
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

#include <hostlib/threads.h>
#include <HSPI/Controller.h>
#include <HSPI/Device.h>
#include <debug_progmem.h>
#include <Platform/Timers.h>
#include <cassert>

#define enableInterrupts() thread->enable()
#define disableInterrupts() thread->disable()

static constexpr size_t hardwareBufferSize{64};

namespace HSPI
{
class AsyncThread : public CThread
{
public:
	AsyncThread(Delegate<void()> isr) : CThread("HSPI", 1), isr(isr)
	{
		execute();
	}

	void enable()
	{
		setState(State::enabled);
	}

	void disable()
	{
		setState(State::disabled);
	}

	void terminate()
	{
		setState(State::terminating);
		join();
	}

protected:
	void* thread_routine() override
	{
		while(state != State::terminating) {
			if(nextState != state) {
				state = nextState;
				continue;
			}
			if(state == State::disabled) {
				sem.wait();
				continue;
			}
			sched_yield();
			if(state == State::enabled) {
				isr();
			}
		}

		return nullptr;
	}

private:
	enum class State {
		disabled,
		enabled,
		terminating,
	};

	void setState(State newState)
	{
		if(nextState == newState) {
			return;
		}

		if(isCurrent()) {
			state = nextState = newState;
			return;
		}

		nextState = newState;
		sched_yield();

		if(newState != State::disabled) {
			sem.post();
			return;
		}

		while(state != newState) {
			//
		}
	}

	Delegate<void()> isr;
	CSemaphore sem; // Signals state change
	volatile State state{};
	volatile State nextState{};
};

namespace
{
void printRequest(Request& req)
{
	debug_d("req .cmd = 0x%04x, %u, .out = %p, %u; .in = %p, %u; .callback = %p, %p; async = %u", req.cmd, req.cmdLen,
			req.out.get(), req.out.length, req.in.get(), req.in.length, req.callback, req.param, unsigned(req.async));
	if(req.out.length > 0) {
		debug_hex(DBG, "OUT", req.out.get(), std::min(req.out.length, uint16_t(32)), -1, 32);
	}
}

} // namespace

#ifdef HSPI_ENABLE_STATS
volatile Controller::Stats Controller::stats;
#endif

ControllerBase::ControllerBase()
{
}

ControllerBase::~ControllerBase()
{
}

bool Controller::begin()
{
	if(!flags.initialised) {
		thread.reset(new AsyncThread([this]() { transactionDone(); }));
		flags.initialised = true;
	}

	return true;
}

void Controller::end()
{
	if(flags.initialised) {
		thread->terminate();
		flags.initialised = false;
	}
}

#define FUNC(fmt, ...) debug_i("Controller::%s(" fmt ")", __FUNCTION__, __VA_ARGS__);

bool Controller::startDevice(Device& dev, PinSet pinSet, uint8_t chipSelect, uint32_t clockSpeed)
{
	FUNC("%p, %u, %u", &dev, pinSet, chipSelect)

	if(!flags.initialised) {
		debug_e("SPI Controller not initialised");
		return false;
	}

	dev.pinSet = pinSet;
	dev.chipSelect = chipSelect;
	dev.speed = clockSpeed;
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
}

void Controller::execute(Request& req)
{
#if DEBUG_VERBOSE_LEVEL == DBG
	FUNC("%p", &req);
#endif

	if(!flags.initialised || req.device == nullptr || req.device->pinSet == PinSet::none) {
		debug_e("SPI device not initialised");
		return;
	}

	assert(!req.busy);
	assert(req.device != nullptr);

	req.busy = true;

	// Packet transfer already in progress?
	disableInterrupts();
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
	enableInterrupts();

	if(!req.async) {
		wait(req);
	}
}

void Controller::wait(Request& request)
{
	if(!request.busy) {
		return;
	}
#ifdef HSPI_ENABLE_STATS
	CpuCycleTimer timer;
#endif
	do {
	} while(request.busy);
#ifdef HSPI_ENABLE_STATS
	stats.waitCycles += timer.elapsedTicks();
#endif
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

void Controller::isr(Controller* spi)
{
	spi->transactionDone();
}

void Controller::transactionDone()
{
	assert(trans.request != nullptr);
	if(trans.request == nullptr) {
		return;
	}

	auto& req = *trans.request;
	auto& dev = *req.device;

#ifdef HSPI_ENABLE_STATS
	auto datalen = std::max(req.out.length, req.in.length);
	auto datatrans = (datalen + hardwareBufferSize - 1) / hardwareBufferSize;
	stats.transCount += std::max(1U, datatrans);
#endif

	printRequest(req);
	if(selectDeviceCallback) {
		selectDeviceCallback(dev.chipSelect, false);
	}

	trans.busy = false;

	// Note next packet in chain and de-queue this one
	trans.request = req.next;
	req.next = nullptr;

#ifdef HSPI_ENABLE_STATS
	++stats.requestCount;
#endif

	if(dev.transferComplete(req)) {
		req.busy = false;
	} else {
		trans.request = reQueueRequest(trans.request, &req);
	}

	// Feed the hardware
	if(trans.request == nullptr) {
		disableInterrupts();
	} else {
		startRequest();
	}
}

bool Controller::loopback(bool enable)
{
	(void)enable;
	return false;
}

} // namespace HSPI
