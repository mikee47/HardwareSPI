/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * Request.h
 *
 * @author: 11 December 2018 - mikee47 <mike@sillyhouse.net>
 *
 * Definitions for an SPI Request Packet containing data and settings for a transfer.
 * A single transfer may use 1 or more transactions.
 *
 */

/** @defgroup hw_spi SPI Hardware support
 *  @brief    Provides hardware SPI support
 */

#pragma once

#include <stdint.h>
#include <esp_attr.h>

namespace HSPI
{
class Device;

/**
 * @brief Data can be specified directly within `Data`, or as a buffer reference
 * @note Regular SPI is full duplex, so we send the same number of bytes as we receive.
 * We consider the command/address phases separately, because in master mode they're always a write operation.
 * For dual/quad modes the bus is half-duplex, however like regular SPI a transaction is typically
 * (but not always) either read OR write, not both.
 * Therefore we need to cater for sending and receiving arbitrary numbers of bytes in a transaction,
 * regardless of mode.
 *
 * @note Command or address are stored in native byte order and rearranged according to the requested
 * byteOrder setting. Data is always sent and received LSB first (as stored in memory) so any re-ordering
 * must be done by the device or application.
 *
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

	void clear()
	{
		data32 = 0;
		length = 0;
		isPointer = 0;
	}

	void set(const void* data, uint16_t count)
	{
		cptr = data;
		length = count;
		isPointer = 1;
	}

	void set8(uint8_t data)
	{
		set32(data, 1);
	}

	void set16(uint16_t data)
	{
		set32(data, 2);
	}

	/** @brief Set 32-bit data
	 *  @param data
	 *  @param len in bytes (1 - 4)
	 */
	void set32(uint32_t data, uint8_t len = 4)
	{
		data32 = data;
		length = len;
		isPointer = 0;
	}
};

struct Request;

/** @brief SPI callback routine */
typedef void (*Callback)(Request& request);

/** @brief Defines an SPI Request Packet
 *  @todo SPI Master can keep a list of these to allow queueing of transfers
 *  Client still needs to deal with buffer allocation though, so may not be necessary as
 *  a better model is for device/client to queue another transaction via callback.
 *  At present we're looking just at address-based transactions.
 *
 *  Request fields may be accessed directly or by use of helper methods.
 *
 *  Note that we chain requests, but typically the chain will be short.
 *  For example, an application may use two Requests, so one can be prepared whilst the other
 *  is in flight. This helps to minimises the setup latency between SPI transactions.
 */
struct Request {
	Device* device{nullptr};	///< Target device for this request
	Request* next{nullptr};		///< Controller uses this to chain requests
	uint16_t cmd{0};			///< Command value
	uint8_t cmdLen{0};			///< Command bits, 0 - 16
	uint8_t async : 1;			///< Set for asynchronous operation
	volatile uint8_t busy : 1;  ///< Request in progress
	uint32_t addr{0};			///< Address value
	uint8_t addrLen{0};			///< Address bits, 0 - 32
	uint8_t dummyLen{0};		///< Dummy read bits between address and read data, 0 - 255
	Data out;					///< Outgoing data
	Data in;					///< Incoming data
	Callback callback{nullptr}; ///< Completion routine
	void* param{nullptr};		///< User parameter

	Request() : async(false), busy(false)
	{
	}

	/**
	 * @brief MUST call this first before attempting to re-use a request
	 * @note If the request is already queued then this method will block until it's completed
	 */
	void prepare()
	{
		while(busy) {
			;
		}
		busy = false;
	}

	void setCommand(uint16_t command, uint8_t bitCount)
	{
		cmd = command;
		cmdLen = bitCount;
	}

	void setCommand8(uint8_t command)
	{
		setCommand(command, 8);
	}

	void setCommand16(uint16_t command)
	{
		setCommand(command, 16);
	}

	void setAddress(uint32_t address, uint8_t bitCount)
	{
		addr = address;
		addrLen = bitCount;
	}

	void setAddress24(uint32_t address)
	{
		setAddress(address, 24);
	}
};

} // namespace HSPI
