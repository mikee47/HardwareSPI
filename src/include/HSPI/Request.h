/****
 * Request.h
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
 * Definitions for an SPI Request Packet containing data and settings for a transfer.
 * A single transfer may use 1 or more transactions.
 *
 ****/

#pragma once

#include "Data.h"
#include <cstddef>

namespace HSPI
{
class Device;
struct Request;

/**
 * @brief SPI completion callback routine
 * @param request
 * @retval bool Return true if request is finished, false to re-queue it immediately
 * @ingroup hw_spi
 */
using Callback = bool (*)(Request& request);

/**
 * @brief Defines an SPI Request Packet
 *
 * Request fields may be accessed directly or by use of helper methods.
 *
 * Application is responsible for managing Request object construction/destruction.
 * Queuing is managed as a linked list so the objects aren't copied.
 *
 * Applications will typically only require a couple of Request objects, so one can be
 * prepared whilst the other is in flight. This helps to minimises the setup latency
 * between SPI transactions.
 *
 * @ingroup hw_spi
 */
struct Request {
	Device* device{nullptr};	///< Target device for this request
	Request* next{nullptr};		///< Controller uses this to queue requests
	uint16_t cmd{0};			///< Command value
	uint8_t cmdLen{0};			///< Command bits, 0 - 16
	uint8_t async : 1;			///< Set for asynchronous operation
	uint8_t task : 1;			///< Controller will execute this request in task mode
	volatile uint8_t busy : 1;  ///< Request in progress
	uint32_t addr{0};			///< Address value
	uint8_t addrLen{0};			///< Address bits, 0 - 32
	uint8_t dummyLen{0};		///< Dummy read bits between address and read data, 0 - 255
	uint8_t sizeAlign{0};		///< Required size alignment of each transaction (if split up)
	Data out;					///< Outgoing data
	Data in;					///< Incoming data
	Callback callback{nullptr}; ///< Completion routine
	void* param{nullptr};		///< User parameter

	Request() : async(false), task(false), busy(false)
	{
	}

	/**
	 * @name Set value for command phase
	 * @{
	 */

	/**
	 * @param command
	 * @param bitCount Length of command in bits
	 */
	__forceinline void setCommand(uint16_t command, uint8_t bitCount)
	{
		cmd = command;
		cmdLen = bitCount;
	}

	/**
	 * @brief Set 8-bit command
	 * @param command
	 */
	__forceinline void setCommand8(uint8_t command)
	{
		setCommand(command, 8);
	}

	/**
	 * @brief Set 16-bit command
	 * @param command
	 */
	__forceinline void setCommand16(uint16_t command)
	{
		setCommand(command, 16);
	}

	/** @} */

	/**
	 * @name Set value for address phase
	 * @{
	 */

	/**
	 * @param address
	 * @param bitCount Length of address in bits
	 */
	__forceinline void setAddress(uint32_t address, uint8_t bitCount)
	{
		addr = address;
		addrLen = bitCount;
	}

	/**
	 * @brief Set 24-bit address
	 * @param address
	 */
	__forceinline void setAddress24(uint32_t address)
	{
		setAddress(address, 24);
	}

	/** @} */

	/**
	 * @brief Set request to asynchronous execution with optional callback
	 */
	void setAsync(Callback callback = nullptr, void* param = nullptr)
	{
		async = true;
		this->callback = callback;
		this->param = param;
	}
};

/**
 * @brief Support function for fast request re-queuing
 * @param head The current queue head
 * @param request The request to append
 * @retval Request* The new queue head
 *
 * Append a request back onto the queue.
 *
 * Don't just put it at the front of the queue though as it will block any other requests.
 * So it goes at the end. BUT! we must preserve execution order for all requests for a given device,
 * so any other requests for the same device must go after it.
 *
 * The easiest way to do this is to build two queues, one off the current request (A) and the other containing all other requests (B).
 * We then append (A) to (B) and set the head to the start of (B).
 * 
 * This is kind of laborious but fast as it's just pointer manipulation.
 */
Request* reQueueRequest(Request* head, Request* request);

} // namespace HSPI
