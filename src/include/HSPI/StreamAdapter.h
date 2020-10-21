/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * StreamAdapter.h
 *
 * @author: October 2020 - mikee47 <mike@sillyhouse.net>
 *
*/

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
	static void requestComplete(HSPI::Request& req);

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
