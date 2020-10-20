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

#pragma once

#include "Data.h"

namespace HSPI
{
class Device;
struct Request;

/**
 * @brief SPI completion callback routine
 * @ingroup hw_spi
 */
typedef void (*Callback)(Request& request);

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
	Data out;					///< Outgoing data
	Data in;					///< Incoming data
	Callback callback{nullptr}; ///< Completion routine
	void* param{nullptr};		///< User parameter

	Request() : async(false), task(false), busy(false)
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
		task = false;
	}

	/**
	 * @name Set value for command phase
	 * @{
	 */

	/**
	 * @param command
	 * @param bitCount Length of command in bits
	 */
	void setCommand(uint16_t command, uint8_t bitCount)
	{
		cmd = command;
		cmdLen = bitCount;
	}

	/**
	 * @brief Set 8-bit command
	 * @param command
	 */
	void setCommand8(uint8_t command)
	{
		setCommand(command, 8);
	}

	/**
	 * @brief Set 16-bit command
	 * @param command
	 */
	void setCommand16(uint16_t command)
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
	void setAddress(uint32_t address, uint8_t bitCount)
	{
		addr = address;
		addrLen = bitCount;
	}

	/**
	 * @brief Set 24-bit address
	 * @param address
	 */
	void setAddress24(uint32_t address)
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

} // namespace HSPI
