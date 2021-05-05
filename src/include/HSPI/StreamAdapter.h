/**
 * StreamAdapter.h
 *
 * Copyright 2020 mikee47 <mike@sillyhouse.net>
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
 * @author: October 2020 - mikee47 <mike@sillyhouse.net>
 *
 ****/

#pragma once

#include "MemoryDevice.h"
#include <Data/Stream/ReadWriteStream.h>
#include <Interrupts.h>

namespace HSPI
{
/**
 * @brief Helper class for streaming data to/from SPI devices
 * @ingroup hw_spi
 */
class StreamAdapter
{
public:
	StreamAdapter(MemoryDevice& device);

	bool write(IDataSourceStream* source, uint32_t address, size_t len, InterruptDelegate callback);

	bool read(ReadWriteStream* dest, uint32_t address, size_t len, InterruptDelegate callback);

	bool getIsWrite() const
	{
		return isWrite;
	}

	size_t getBytesRequested() const
	{
		return bytesRequested;
	}

	size_t getBytesTransferred() const
	{
		return bytesTransferred;
	}

private:
	struct Buffer {
		static constexpr size_t size{1024};
		HSPI::Request req;
		char data[size];
	};

	void task();
	unsigned writeChunks();
	bool writeChunk();
	unsigned readChunks();
	bool readChunk();
	static bool requestComplete(HSPI::Request& req);

	MemoryDevice& device;
	InterruptDelegate callback;
	//	Stream* stream{nullptr};
	IDataSourceStream* stream{nullptr};
	bool isWrite{false};
	uint32_t address{0};
	size_t bytesRequested{0};
	size_t bytesTransferred{0};
	static constexpr size_t bufCount{2};
	Buffer buffers[bufCount];
	uint8_t index{0};
	bool taskQueued{false};
};

} // namespace HSPI
