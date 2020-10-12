/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * Controller.h
 *
 * @author: 11 December 2018 - mikee47 <mike@sillyhouse.net>
 *
 * # SPI master-mode hardware controller
 *
 * The ESP8266 SPI hardware is capable of very high transfer speeds and has a number of
 * features which require a more flexible driver to take advantage of. This module,
 * together with Device, provide the following features:
 *
 *  - Support multiple slave devices sharing the same bus
 * 	- Custom CS multiplexing supported via callbacks. For example, routing CS2 via HC138
 * 	3:8 decoder allows 8 (or more) SPI devices to share the same bus.
 * 	- Use of HSPI (SPI1) using either its own pins or sharing pins with SPI0 (overlapped)
 * 	- (Potentially) enabling use of dual/quad operating modes when overlapped
 * 	- Making use of hardware command/address/data phases for best efficiency
 * 	- Pre-calculation of all register values to optimise switching between slave devices
 *  - Write-only transactions can return immediately rather than waiting for the transfer to
 *  complete. The time spent waiting can be used to prepare the next transaction which can
 *  potentially double the throughput
 *  - Interrupt callback on transaction completion. This can be used to improve system efficiency
 *  on slower devices.
 *  - (Potentially) Using DMA for larger read/write transactions. The SDK only demonstrates
 *  DMA for I2S services but it _may_ be possible to use it for SPI.
 *
 *
 * # Transactions
 *
 * Applications call Controller to perform a transfer, or sequence of transfers, as follows:
 *
 *	- Session setup
 *		- Wait for any HSPI transaction to complete (WAIT_READY)
 * 		- Configure clock & mode settings
 * 	- Transaction
 * 		- WAIT_READY
 * 		- Configure command / address / data
 * 		- Start operation
 * 		- If read required:
 * 			- WAIT_READY
 * 			- Copy data from FIFO
 *
 *	Transaction may be repeated for subsequent transfers on same device
 *	CS will be asserted/de-asserted by hardware so not need to end a transaction
 *
 * # Overlapped operation
 *
 * Both SPI controllers are able to share the pin signals from the flash SPI interface (SPI0).
 * This is handled through hardware.
 *
 * Advantages:
 * 	- Gain three pins (GPIO12-14), which liberates the I2S controller
 * 	- Dual and quad SPI modes can be used with HSPI
 *
 * Disadvantages:
 * 	- Slow SPI devices may reduce retrieval speed of program code from Flash memory
 *
 *	A primary IO MUX (PERIPHS_IO_MUX_CONF_U) selects whether the CPU clock goes through the
 * 	SPI clock divider circuitry or not. In overlapped mode the SPI0 setting is used for both,
 * 	therefore as most SPI slave devices will not operate at 80MHz this setting has to be disabled
 * 	to allow the clocks to be set independently. See PERIPHS_IO_MUX_CONF_U.
 *
 * The ESP32 Technical Reference manual gives a very useful insight into how the two SPI
 * devices work together, as the hardware appears largely similar.
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

//#define SPI_DEBUG  1

#undef SPI_MODE0
#undef SPI_MODE1
#undef SPI_MODE2
#undef SPI_MODE3

enum Mode : uint8_t {
	Mode0 = 0x00, ///<  CPOL: 0  CPHA: 0
	Mode1 = 0x01, ///<  CPOL: 0  CPHA: 1
	Mode2 = 0x10, ///<  CPOL: 1  CPHA: 0
	Mode3 = 0x11, ///<  CPOL: 1  CPHA: 1
};

/*
 * TODO: Add dual/quad modes, etc.
 */

// 0 for LSBFIRST, non-zero for MSBFIRST
using ByteOrder = uint8_t;
using BitOrder = uint8_t;

/** @brief How SPI hardware pins are connected
 *  @note
 * 		Normal: MISO = GPIO12, MOSI = GPIO13, SCLK = GPIO14
 * 			CS_AUTO: pin 15 HSPI CS managed by hardware
 * 			CS_MANUAL: CS managed by device at start/end of transaction
 *
 * 		Overlap: MISO = SD0, MOSI = SDD1, SCLK = CLK
 * 			CS = CS2, always managed by hardware
 */
enum class PinSet {
	None,
	Normal_CS_Auto,
	Normal_CS_Manual,
	Overlap,
};

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
	uint8_t duplex : 1;			///< Set for 4-wire bi-directional transfer
	uint8_t async : 1;			///< Set for asynchronous operation
	volatile uint8_t busy : 1;  ///< Packet in use
	uint32_t addr{0};			///< Address value
	uint8_t addrLen{0};			///< Address bits, 0 - 32
	uint8_t dummyLen{0};		///< Dummy read bits between address and read data, 0 - 255
	Data out;					///< Outgoing data
	Data in;					///< Incoming data
	Callback callback{nullptr}; ///< Completion routine
	void* param{nullptr};		///< User parameter

	Packet() : duplex(0), async(0), busy(0)
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

struct Stats {
	uint32_t waitCycles;
	uint32_t transCount;

	void clear() volatile
	{
		waitCycles = 0;
		transCount = 0;
	}
};

inline uint16_t bswap16(uint16_t data)
{
	return __builtin_bswap16(data);
}

inline uint32_t bswap24(uint32_t data)
{
	return __builtin_bswap32(data) >> 8;
}

inline uint32_t bswap32(uint32_t data)
{
	return __builtin_bswap32(data);
}

/**
 * Transform unsigned integer of length <= 32 bits to the format which can be
 * sent by the SPI driver directly.
 *
 * E.g. to send 9 bits of data, you can:
 *
 *      uint16_t data = SPI_SWAP_DATA_TX(0x145, 9);
 *
 * Then points tx_buffer to ``&data``.
 *
 * @param data Data to be sent, can be uint8_t, uint16_t or uint32_t. @param
 *  len Length of data to be sent, since the SPI peripheral sends from the MSB,
 *  this helps to shift the data to the MSB.
 */
inline uint32_t swapDataTx(uint32_t data, uint8_t len)
{
	return bswap32(data << (32 - len));
}

/**
 * Transform received data of length <= 32 bits to the format of an unsigned integer.
 *
 * E.g. to transform the data of 15 bits placed in a 4-byte array to integer:
 *
 *      uint16_t data = SPI_SWAP_DATA_RX(*(uint32_t*)t->rx_data, 15);
 *
 * @param data Data to be rearranged, can be uint8_t, uint16_t or uint32_t.
 * @param len Length of data received, since the SPI peripheral writes from
 *      the MSB, this helps to shift the data to the LSB.
 */
inline uint32_t swapDataRx(uint32_t data, uint8_t len)
{
	return bswap32(data) >> (32 - len);
}

class Controller
{
public:
	/** @brief  Instantiate hardware SPI object
	 *  @addtogroup hw_spi
	 *  @{
	 */
	Controller()
	{
	}

	virtual ~Controller()
	{
	}

	/* @brief Initializes the HSPI controller using the specified set of pins
	 * @param pinSet Which set of pins to use (see PinSet definition)
	 * @note MUST only be called once
	 */
	void begin(PinSet pinSet);

	/** @brief Disables HSPI controller
	 * 	@note Reverts HSPI pins to GPIO and disables the controller
	 */
	void end();

	/** @brief Determine the best clock register value for a desired bus frequency
	 *  @param frequency Desired SPI clock frequency in Hz
	 *  @retval uint32_t opaque 32-bit register setting value
	 *  @note
	 *  	Speed settings are pre-calculated to optimise switching between devices.
	 *  	It is guaranteed that the frequency will not exceed the given target
	 *  	This method must be called with the CPU set to the correct clock frequency
	 *  	in use; if this changes then the calculation will be wrong.
	 */
	virtual uint32_t frequencyToClkReg(uint32_t frequency);

	/** @brief Calculate SPI clock Frequency based on the register value
	 * 	@param regVal
	 * 	@retval uint32_t clock frequency in Hz
	 */
	virtual uint32_t clkRegToFreq(uint32_t regVal);

	static volatile Stats stats;

protected:
	friend Device;

	virtual void execute(Packet& packet);

	/** @brief get an empty packet ready for a new request
	 *  @param wait if true, will block until a packet becomes free, if false will return nullptr immediately
	 *  @retval SpiPacket*
	 *  @note packets are all pre-allocated
	 *  until one becomes available.
	 *
	 *  Packets don't have to be obtained here, but having a central allocation is more efficient.
	 *  Consider making the packet a class, so it can be overridden for custom behaviour. Not sure what
	 *  that behaviour might be though...
	 */
	Packet* getPacket(bool wait = true);

private:
	/** @brief Transfer up to SPI_FIFO_LEN bytes */
	static void IRAM_ATTR isr(Controller* spi);
	/** @brief Start transfer of a new packet (trans.packet)
	 *  @note May be called from interrupt context at completion of previous transfer
	 */
	void IRAM_ATTR startPacket();

	/** @brief ead incoming data, if there is any, and start next transaction
	 *  @note May be called from interrupt context at completion of previous transaction
	 */
	void IRAM_ATTR transfer();

	PinSet pinSet = PinSet::None;
	Device* activeDevice = nullptr;
	bool spi0ClockChanged = false;

	// State of the current transaction in progress
	struct Transaction {
		Packet* packet;		 ///< The packet being executed
		uint16_t addrOffset; ///< Address for next transfer, added to packet start address
		uint16_t outOffset;  ///< Where to read data for next outgoing transfer
		uint16_t inOffset;   ///< Where to write data from current transfer when it arrives
		uint8_t inlen;		 ///< Incoming data for current transfer
		// Flags
		BitOrder bitOrder : 1;
		volatile uint8_t busy : 1;
	};
	Transaction trans{};
};

} // namespace HSPI
