/**
 * StreamAdapter.cpp
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

#include "include/HSPI/StreamAdapter.h"
#include <Platform/System.h>

namespace HSPI
{
StreamAdapter::StreamAdapter(MemoryDevice& device) : device(device)
{
	for(auto& buf : buffers) {
		buf.req.setAsync(requestComplete, this);
	}
}

bool StreamAdapter::write(IDataSourceStream* source, uint32_t address, size_t len, InterruptDelegate callback)
{
	assert(this->stream == nullptr);

	this->callback = callback;
	this->stream = source;
	this->address = address;
	bytesRequested = len;
	bytesTransferred = 0;
	isWrite = true;

	return writeChunks() != 0;
}

bool StreamAdapter::read(ReadWriteStream* dest, uint32_t address, size_t len, InterruptDelegate callback)
{
	assert(this->stream == nullptr);

	this->callback = callback;
	this->stream = dest;
	this->address = address;
	bytesRequested = len;
	bytesTransferred = 0;
	isWrite = false;

	return readChunks() != 0;
}

void StreamAdapter::task()
{
	taskQueued = false;

	if(isWrite) {
		unsigned n = writeChunks();
		for(auto& buf : buffers) {
			n += buf.req.busy;
		}
		if(n != 0) {
			return;
		}
	} else {
		readChunks();
		if(bytesTransferred < bytesRequested) {
			return;
		}
	}

	delete stream;
	stream = nullptr;

	if(callback) {
		callback();
	}
}

unsigned StreamAdapter::writeChunks()
{
	unsigned n{0};
	while(writeChunk()) {
		++n;
	}
	return n;
}

bool StreamAdapter::writeChunk()
{
	auto& buf = buffers[index];
	if(buf.req.busy) {
		return false;
	}

	auto len = std::min(bytesRequested - bytesTransferred, Buffer::size);
	if(stream == nullptr || len == 0) {
		return false;
	}

	//		len = stream->readBytes(buf.data, len);
	len = stream->readMemoryBlock(buf.data, len);

	if(len == 0) {
		return false;
	}
	stream->seek(len);
	device.prepareWrite(buf.req, address, buf.data, len);
	device.execute(buf.req);
	address += len;
	bytesTransferred += len;
	index = (index + 1) % bufCount;
	return true;
}

unsigned StreamAdapter::readChunks()
{
	unsigned ret{0};

	for(unsigned i = 0; i < bufCount; ++i) {
		auto& buf = buffers[index];
		if(buf.req.busy) {
			break;
		}

		auto& data = buf.req.in;
		if(data.length == 0) {
			break;
		}

		auto len = reinterpret_cast<ReadWriteStream*>(stream)->write(reinterpret_cast<uint8_t*>(data.ptr), data.length);
		if(len != data.length) {
			debug_e("Stream write failed: %u written, %u expected", len, data.length);
		}
		bytesTransferred += data.length;
		data.length = 0;
		index = (index + 1) % bufCount;
		++ret;
	}

	for(unsigned i = 0; i < bufCount; ++i) {
		auto& buf = buffers[(index + i) % bufCount];
		if(buf.req.busy) {
			break;
		}

		auto len = std::min(bytesRequested - bytesTransferred, Buffer::size);
		if(len == 0) {
			break;
		}

		device.prepareRead(buf.req, address, buf.data, len);
		device.execute(buf.req);
		address += len;
		bytesTransferred += len;
		++ret;
	}

	return ret;
} // namespace HSPI

void IRAM_ATTR StreamAdapter::requestComplete(HSPI::Request& req)
{
	auto self = static_cast<StreamAdapter*>(req.param);
	if(!self->taskQueued) {
		System.queueCallback([](void* param) { static_cast<StreamAdapter*>(param)->task(); }, self);
		self->taskQueued = true;
	}
}

} // namespace HSPI
