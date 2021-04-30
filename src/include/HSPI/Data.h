/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * Data.h
 *
 * @author: 11 December 2018 - mikee47 <mike@sillyhouse.net>
 *
 * Definitions for an SPI Request Packet containing data and settings for a transfer.
 * A single transfer may use 1 or more transactions.
 *
 */

#pragma once

#include <stdint.h>

namespace HSPI
{
/**
 * @brief Specifies a block incoming or outgoing data
 *
 * Data can be specified directly within `Data`, or as a buffer reference.
 *
 * Command or address are stored in native byte order and rearranged according to the requested
 * byteOrder setting. Data is always sent and received LSB first (as stored in memory) so any re-ordering
 * must be done by the device or application.
 *
 * @ingroup hw_spi
 */
struct Data {
	union {
		uint8_t data8;
		uint16_t data16;
		uint32_t data32;
		uint8_t data[4];
		void* ptr; ///< Pointer to data
		const void* cptr;
		uint8_t* ptr8;
	};
	uint16_t length : 15;   ///< Number of bytes of data
	uint16_t isPointer : 1; ///< If set, data is referenced indirectly, otherwise it's stored directly

	Data()
	{
		clear();
	}

	void* get()
	{
		return isPointer ? ptr : data;
	}

	/**
	 * @brief Reset to zero-length
	 */
	void clear()
	{
		data32 = 0;
		length = 0;
		isPointer = 0;
	}

	/**
	 * @brief Set to reference external data block
	 * @param data Location of data
	 * @param count Number of bytes
	 */
	void set(const void* data, uint16_t count)
	{
		cptr = data;
		length = count;
		isPointer = 1;
	}

	void* get()
	{
		return isPointer ? ptr : data;
	}

	/**
	 * @name Set internal data value of 1-4 bytes
	 * @note Data is sent LSB, MSB (native byte order)
	 * @{
	 */

	/**
	 * @brief Set to single 8-bit value
	 * @param data
	 */
	void set8(uint8_t data)
	{
		set32(data, 1);
	}

	/**
	 * @brief Set to single 16-bit value
	 * @param data
	 */
	void set16(uint16_t data)
	{
		set32(data, 2);
	}

	/**
	 * @brief Set to 32-bit data
	 * @param data
	 * @param len Length in bytes (1 - 4)
	 */
	void set32(uint32_t data, uint8_t len = 4)
	{
		data32 = data;
		length = len;
		isPointer = 0;
	}

	/** @} */
};

} // namespace HSPI
