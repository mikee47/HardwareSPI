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
	 * @brief Prepare a write request
	 * @param request
	 * @param address
	 * @param data
	 * @param len
	 */
	virtual void prepareWrite(HSPI::Request& req, uint32_t address, const void* data, size_t len) = 0;

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

	/**
	 * @brief Read a block of data
	 * @param req
	 * @param address
	 * @param data
	 * @param len
	 */
	virtual void prepareRead(HSPI::Request& req, uint32_t address, void* buffer, size_t len) = 0;

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
};

} // namespace HSPI
