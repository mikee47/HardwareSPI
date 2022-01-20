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

#ifdef HSPI_DEBUG
#define hspi_debug(fmt, ...)                                                                                           \
	host_printf("%u [%s] " fmt "\r\n", system_get_time(), CThread::getCurrentName(), ##__VA_ARGS__);
#else
#define hspi_debug(fmt, ...)
#endif

#define FUNC(fmt, ...) hspi_debug("Controller::%s(" fmt ")", __FUNCTION__, ##__VA_ARGS__)

static constexpr size_t hardwareBufferSize{64};

namespace HSPI
{
class AsyncThread : public CThread
{
public:
	using IsrCallback = Delegate<void()>;

	AsyncThread(IsrCallback isr) : CThread("HSPI", 1), isr(isr)
	{
		execute();
	}

	void terminate()
	{
		done = true;
		sem.post();
		join();
	}

	void interruptAfter(unsigned us)
	{
		this->transactionTime = us;
		sem.post();
	}

protected:
	void* thread_routine() override
	{
		while(true) {
			sem.wait();
			if(done) {
				break;
			}

			if(transactionTime != 0) {
				OneShotFastUs timer;
				timer.reset(transactionTime);
				while(!timer.expired()) {
				}
			}

			interrupt_begin();
			isr();
			interrupt_end();
		}

		return nullptr;
	}

private:
	IsrCallback isr;
	CSemaphore sem; // Signals state change
	uint32_t transactionTime{0};
	bool done{false};
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
	if(!thread) {
		thread.reset(new AsyncThread([this]() { transactionDone(); }));
	}

	return true;
}

void Controller::end()
{
	if(thread) {
		thread->terminate();
		thread.reset();
	}
}

IoModes Controller::getSupportedIoModes(const Device& dev) const
{
	// Hardware supports all modes
	return dev.getSupportedIoModes();
}

bool Controller::startDevice(Device& dev, PinSet pinSet, uint8_t chipSelect, uint32_t clockSpeed)
{
	FUNC("%p, %u, %u", &dev, pinSet, chipSelect)

	if(!thread) {
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

	if(!thread || req.device == nullptr || req.device->pinSet == PinSet::none) {
		debug_e("SPI device not initialised");
		return;
	}

	assert(!req.busy);
	assert(req.device != nullptr);

	req.busy = true;

	// Packet transfer already in progress?
	thread->suspend();
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
	thread->resume();

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
		sched_yield();
	} while(request.busy);
#ifdef HSPI_ENABLE_STATS
	stats.waitCycles += timer.elapsedTicks();
#endif
}

void Controller::startRequest()
{
	FUNC("%p, %u", trans.request, trans.request->busy);

	trans.busy = true;
	auto& req = *trans.request;
	auto& dev = *req.device;
	if(selectDeviceCallback) {
		selectDeviceCallback(dev.chipSelect, true);
	}
	dev.transferStarting(req);

	unsigned bitCount = req.cmdLen + req.addrLen;
	if(dev.ioMode == IoMode::SPI) {
		bitCount += std::max(req.out.length, req.in.length) * 8;
	} else {
		bitCount += (req.out.length + req.in.length) * 8;
	}
	unsigned us = muldiv(1000000U, bitCount, dev.speed);
	thread->interruptAfter(us);
}

void Controller::transactionDone()
{
	FUNC("%p", trans.request);

	assert(trans.request != nullptr);
	if(trans.request == nullptr) {
		return;
	}

	auto& req = *trans.request;
	assert(req.busy);
	if(!req.busy) {
		return;
	}
	assert(req.device != nullptr);
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
	if(trans.request != nullptr) {
		startRequest();
	}
}

bool Controller::loopback(bool enable)
{
	(void)enable;
	return false;
}

} // namespace HSPI
