/**
 * MemCheckState.h
 *
 * Copyright 2021 mikee47 <mike@sillyhouse.net>
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
 ****/

#pragma once

#include <S1D13781/Driver.h>
#include <S1D13781/registers.h>

namespace HSPI
{
namespace Test
{
/**
 * @brief Class to test memory devices by writing/reading random data blocks
 *
 * B: Build outgoing block
 * C: Check received block
 * W: Start write of outgoing block
 * R: Start read of incoming block
 *
 * CPU   B, B     C, B    C, B
 *       |        ^       ^
 *       |        |       |
 * SPI   -> W-> R-> W-> R-> W...
 *
 *
 * Interrupt version, 433ms
 * Additional time due to setup code. A 512 byte buffer creates 772 requests, for each of write/read,
 * a total of 1544 requests. So the additional (433 - 383) = 50ms is from 32us per request.
 * 1024 byte: 443ms
 * 256 byte: 423ms
 * 128 byte: 404ms
 * 64 bytes: ms FAIL
 *
 * With task queue and packet chaining:
 *
 * 1024 byte: 387ms
 * 512 byte: 391ms
 * 256 byte: 400ms
 * 128 byte: 418ms
 * 64 byte: 452ms
 *
 * So how come it's faster? Who cares - awesome, way to go :-)
 *
 */
class MemCheckState
{
public:
	MemCheckState(MemoryDevice& device) : device(device), maxAddr(device.getSize())
	{
	}

	virtual ~MemCheckState()
	{
	}

	void execute()
	{
		device.controller.stats.clear();
		buildBlock();
		writeBlock();
		readBlock();
		buildBlock();
	}

	InterruptDelegate onComplete;

protected:
	/**
	 * @brief Fill buffer with test data
	 * @param addr Starting address where data will be written
	 * @param buffer Buffer to store data
	 * @param size Size of buffer in 32-bit words (i.e. byte count = size * 4)
	 * 
	 * Method is virtual so display devices can be tested which may have quirks
	 * in their memory layout.
	 */
	virtual void fillBlock(uint32_t addr, uint32_t* buffer, size_t size)
	{
		os_get_random(reinterpret_cast<uint8_t*>(buffer), size * 4);
	}

private:
	void buildBlock()
	{
		fillBlock(writeAddr, writeBuffer[bufIndex], bufSize / 4);
	}

	void writeBlock()
	{
		device.write(reqWr, writeAddr, writeBuffer[bufIndex], bufSize);
		writeAddr += bufSize;
		bufIndex = 1 - bufIndex;
	}

	void readBlock()
	{
		auto callback = [](Request& request) {
			System.queueCallback([](void* param) { static_cast<MemCheckState*>(param)->blockRead(); }, request.param);
		};
		device.read(reqRd, readAddr, readBuffer, bufSize, callback, this);
	}

	void blockRead()
	{
		if(writeAddr < maxAddr) {
			writeBlock();
		} else {
			bufIndex = 1 - bufIndex;
		}

		checkBlock();
		readAddr += bufSize;
		if(readAddr < maxAddr) {
			readBlock();
			if(writeAddr < maxAddr) {
				buildBlock();
			}
		} else {
			complete();
		}
	}

	void checkBlock()
	{
		if(memcmp(readBuffer, writeBuffer[bufIndex], bufSize)) {
			debug_e("Mem check failed between 0x%08x and 0x%08x", readAddr, readAddr + bufSize - 1);

			unsigned checklen = ARRAY_SIZE(readBuffer);
			for(unsigned i = 0; i < checklen; ++i) {
				auto in = readBuffer[i];
				auto out = writeBuffer[bufIndex][i];
				if(in != out) {
					debug_e("  @ 0x%08x: out 0x%08x in 0x%08x", readAddr + (i * 4), out, in);
					break;
				}
			}
		}
	}

	void complete()
	{
		auto& stats = device.controller.stats;
		debug_i("Memory check complete, %s, waitCycles = %u, trans = %u", timer.elapsedTime().toString().c_str(),
				stats.waitCycles, stats.transCount);

		debug_i("out = %u, in = %u", reqWr.busy, reqRd.busy);

		auto callback = onComplete;

		delete this;

		if(callback) {
			callback();
		}
	}

private:
	MemoryDevice& device;
	uint32_t maxAddr;
	static const unsigned bufSize{512};
	uint32_t writeBuffer[2][bufSize / 4];
	uint32_t readBuffer[bufSize / 4];
	ElapseTimer timer;
	uint32_t writeAddr{0};
	uint32_t readAddr{0};
	uint8_t bufIndex{0};
	Request reqRd, reqWr;
};

} // namespace Test
} // namespace HSPI
