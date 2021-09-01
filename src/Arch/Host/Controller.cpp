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

#include <hostlib/threads.h>
#include <HSPI/Controller.h>
#include <HSPI/Device.h>
#include <debug_progmem.h>
#include <Platform/Timers.h>
#include <cassert>

#define ETS_SPI_INTR_ATTACH(func, arg) asyncThread.attach(func, arg)
#define ETS_SPI_INTR_ENABLE() asyncThread.enable()
#define ETS_SPI_INTR_DISABLE() asyncThread.disable()

namespace HSPI
{
namespace
{
class AsyncThread : public CThread
{
public:
	using Isr = void (*)(Controller* controller);

	AsyncThread() : CThread("HSPI", 1)
	{
		execute();
	}

	void attach(Isr isr, Controller* controller)
	{
		auto cur = state;
		setState(State::disabled);
		this->isr = isr;
		this->controller = controller;
		setState(cur);
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
			// msleep(1);
			sched_yield();
			if(nextState != State::enabled) {
				continue;
			}
			// interrupt_begin();
			isr(controller);
			// interrupt_end();
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

		if(newState == State::disabled) {
			sem.post();
			while(state != newState) {
				//
			}
		}
	}

	Isr isr;
	Controller* controller;
	CSemaphore sem; // Signals state change
	volatile State state{};
	volatile State nextState{};
};

AsyncThread asyncThread;

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

Controller::~Controller()
{
}

bool Controller::begin()
{
	if(!flags.initialised) {
		ETS_SPI_INTR_ATTACH(isr, this);
		flags.initialised = true;
	}

	return true;
}

void Controller::end()
{
	ETS_SPI_INTR_DISABLE();
	flags.initialised = false;
}

#define FUNC(fmt, ...) debug_i("Controller::%s(" fmt ")", __FUNCTION__, __VA_ARGS__);

bool Controller::startDevice(Device& dev, PinSet pinSet, uint8_t chipSelect)
{
	FUNC("%p, %u, %u", &dev, pinSet, chipSelect)

	if(!flags.initialised) {
		debug_e("SPI Controller not initialised");
		return false;
	}

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

	if(!flags.initialised || req.device == nullptr || req.device->pinSet == PinSet::none) {
		debug_e("SPI device not initialised");
		return;
	}

	assert(!req.busy);
	assert(req.device != nullptr);

	req.busy = true;

	// Packet transfer already in progress?
	ETS_SPI_INTR_DISABLE();
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
		ETS_SPI_INTR_ENABLE();
		return;
	}

	wait(req);
}

void Controller::wait(Request& request)
{
	if(!request.busy) {
		return;
	}
#ifdef HSPI_ENABLE_STATS
	CpuCycleTimer timer;
#endif
	ETS_SPI_INTR_DISABLE();
	do {
		isr(this);
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
	auto datatrans = (datalen + SPI_BUFSIZE - 1) / SPI_BUFSIZE;
	stats.transCount += std::max(1U, datatrans);
#endif

	printRequest(req);
	if(selectDeviceCallback) {
		selectDeviceCallback(dev.chipSelect, false);
	}

	trans.busy = false;
	req.busy = false;
#ifdef HSPI_ENABLE_STATS
	++stats.requestCount;
#endif

	// Note next packet in chain and de-queue this one
	trans.request = req.next;
	req.next = nullptr;

#ifdef HSPI_ENABLE_STATS
	++stats.requestCount;
#endif

	if(!dev.transferComplete(req)) {
		trans.request = reQueueRequest(trans.request, &req);
		req.busy = true;
	}

	// Feed the hardware
	if(trans.request == nullptr) {
		ETS_SPI_INTR_DISABLE();
	} else {
		startRequest();
		ETS_SPI_INTR_ENABLE();
	}
}

} // namespace HSPI
