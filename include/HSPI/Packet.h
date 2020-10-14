/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * Packet.h
 *
 * @author: 11 December 2018 - mikee47 <mike@sillyhouse.net>
 *
 * Definitions for an SPI packet containing data and settings for a transfer.
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

/** @brief Data can be specified directly within SpiPacket, or as a buffer reference
 *  @note Regular SPI is full duplex, so we send the same number of bytes as we receive.
 *  We consider the command/address phases separately, because in master mode they're always a write operation.
 *  For dual/quad modes the bus is half-duplex, however like regular SPI a transaction is typically
 *  (but not always) either read OR write, not both.
 *  Therefore we need to cater for sending and receiving arbitrary numbers of bytes in a transaction,
 *  regardless of mode.
 *
 *  @todo Where 16/32-bit data is used we need to know what the slave device memory byte ordering is,
 *  can be added as a device parameter. This may not necessarily be the same as the SPI byte order.
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

struct Packet;

/** @brief SPI callback routine */
typedef void (*Callback)(Packet& packet);

/** @brief Defines an SPI transaction packet
 *  @todo SPI Master can keep a list of these to allow queueing of transfers
 *  Client still needs to deal with buffer allocation though, so may not be necessary as
 *  a better model is for device/client to queue another transaction via callback.
 *  At present we're looking just at address-based transactions.
 *
 *  Can add methods to this packet to make it easier to setup with various types of request.
 *
 *  Note that we chain requests, but typically the chain will be short. A specific SPI driver
 *  might use two packets for transmit, so that one can be set up while the other is in flight.
 *  That way the setup latency between SPI transactions is minimised for maximum throughput.
 */
struct Packet {
	Device* device{nullptr};	///< SPI device for this packet
	Packet* next{nullptr};		///< SPI master uses this to chain requests
	uint16_t cmd{0};			///< Command value
	uint8_t cmdLen{0};			///< Command bits, 0 - 16
	uint8_t async : 1;			///< Set for asynchronous operation
	volatile uint8_t busy : 1;  ///< Packet in use
	uint32_t addr{0};			///< Address value
	uint8_t addrLen{0};			///< Address bits, 0 - 32
	uint8_t dummyLen{0};		///< Dummy read bits between address and read data, 0 - 255
	Data out;					///< Outgoing data
	Data in;					///< Incoming data
	Callback callback{nullptr}; ///< Completion routine
	void* param{nullptr};		///< User parameter

	Packet() : async(0), busy(0)
	{
	}

	/** @brief MUST call this first before attempting to use a packet
	 *  @note If the packet is already queued then this method will block until it's completed
	 */
	void prepare()
	{
		while(busy) {
			;
		}
		busy = 0;
	}

	void setCommand8(uint8_t command)
	{
		cmd = command;
		cmdLen = 8;
	}

	void setCommand16(uint16_t command)
	{
		cmd = command;
		cmdLen = 16;
	}

	void setAddress24(uint32_t address)
	{
		addr = address;
		addrLen = 24;
	}
};

/*
 * Experimental re-ordering
 struct SpiPacket {
 uint32_t addr;	 ///< Address value
 uint16_t cmd;	  ///< Command value
 uint16_t outLen;
 uint16_t inLen;
 uint8_t cmdLen;	///< Command bits, 0 - 16
 uint8_t addrLen: 6;   ///< Address bits, 0 - 32
 uint8_t inPtr: 1;
 uint8_t outPtr: 1;
 uint8_t dummyLen;  ///< Dummy read bits between address and read data, 0 - 255
 Data out; ///< Outgoing data (32 bits)
 Data in;		   ///< Incoming data (32 bits)
 SpiCallback callback; ///< Completion routine
 };
 */

} // namespace HSPI
