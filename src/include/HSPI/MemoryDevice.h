/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * MemoryDevice.h
 *
 * @author: October 2020 - mikee47 <mike@sillyhouse.net>
 *
*/

#pragma once

#include "Device.h"

namespace HSPI
{
/**
 * @brief Base class for read/write addressable devices
 * @ingroup hw_spi
 */
class MemoryDevice : public Device
{
public:
	using Device::Device;

	/**
	  * @name Prepare a write request
	  * @{
	  */

	/**
	 * @param request
	 * @param address
	 */
	virtual void prepareWrite(HSPI::Request& req, uint32_t address) = 0;

	/**
	 * @param request
	 * @param address
	 * @param data
	 * @param len
	 */
	void prepareWrite(HSPI::Request& req, uint32_t address, const void* data, size_t len)
	{
		prepareWrite(req, address);
		req.out.set(data, len);
		req.in.clear();
	}
	/** @} */

	/**
	 * @brief Write a block of data
	 * @param address
	 * @param data
	 * @param len
	 * @note Limited by current operating mode
	 */
	void write(uint32_t address, const void* data, size_t len)
	{
		Request req;
		prepareWrite(req, address, data, len);
		execute(req);
	}

	void write(Request& req, uint32_t address, const void* data, size_t len, Callback callback = nullptr,
			   void* param = nullptr)
	{
		prepareWrite(req, address, data, len);
		req.setAsync(callback, param);
		execute(req);
	}

	void write8(uint32_t address, uint8_t value)
	{
		Request req;
		prepareWrite(req, address);
		req.out.set8(value);
		execute(req);
	}

	void write8(Request& req, uint32_t address, uint8_t value, Callback callback = nullptr, void* param = nullptr)
	{
		prepareWrite(req, address);
		req.out.set8(value);
		req.in.clear();
		req.setAsync(callback, param);
		execute(req);
	}

	void write16(uint32_t address, uint16_t value)
	{
		Request req;
		prepareWrite(req, address);
		req.out.set16(value);
		execute(req);
	}

	void write16(Request& req, uint32_t address, uint16_t value, Callback callback = nullptr, void* param = nullptr)
	{
		prepareWrite(req, address);
		req.out.set16(value);
		req.in.clear();
		req.setAsync(callback, param);
		execute(req);
	}

	void write32(uint32_t address, uint32_t value)
	{
		Request req;
		prepareWrite(req, address);
		req.out.set32(value);
		execute(req);
	}

	void write32(Request& req, uint32_t address, uint32_t value, Callback callback = nullptr, void* param = nullptr)
	{
		prepareWrite(req, address);
		req.out.set32(value);
		req.in.clear();
		req.setAsync(callback, param);
		execute(req);
	}

	void writeWord(Request& req, uint32_t address, uint32_t value, unsigned byteCount)
	{
		prepareWrite(req, address);
		req.out.set32(value, byteCount);
		req.in.clear();
		execute(req);
	}

	/**
	  * @name Prepare a read request
	  * @{
	  */

	/**
	 * @param req
	 * @param address
	 */
	virtual void prepareRead(HSPI::Request& req, uint32_t address) = 0;

	/**
	 * @param req
	 * @param address
	 * @param data
	 * @param len
	 */
	void prepareRead(HSPI::Request& req, uint32_t address, void* buffer, size_t len)
	{
		prepareRead(req, address);
		req.out.clear();
		req.in.set(buffer, len);
	}

	/** @} */

	/**
	 * @brief Read a block of data
	 * @param address
	 * @param data
	 * @param len
	 * @note Limited by current operating mode
	 */
	void read(uint32_t address, void* buffer, size_t len)
	{
		Request req;
		prepareRead(req, address, buffer, len);
		execute(req);
	}

	uint8_t read8(uint32_t address)
	{
		Request req;
		prepareRead(req, address);
		req.in.set8(address);
		execute(req);
		return req.in.data8;
	}

	uint16_t read16(uint32_t address)
	{
		Request req;
		prepareRead(req, address);
		req.in.set16(address);
		execute(req);
		return req.in.data16;
	}

	uint32_t read32(uint32_t address)
	{
		Request req;
		prepareRead(req, address);
		req.in.set32(address);
		execute(req);
		return req.in.data32;
	}

	uint32_t readWord(uint32_t address, unsigned byteCount)
	{
		Request req;
		prepareRead(req, address);
		req.in.set32(0, byteCount);
		execute(req);
		return req.in.data32;
	}

	void read(Request& req, uint32_t address, void* buffer, size_t len, Callback callback = nullptr,
			  void* param = nullptr)
	{
		prepareRead(req, address, buffer, len);
		req.setAsync(callback, param);
		execute(req);
	}
};

} // namespace HSPI
