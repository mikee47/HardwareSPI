/****
 * Controller.h
 *
 * Copyright 2018 mikee47 <mike@sillyhouse.net>
 *
 * This file is part of the HardwareSPI Library
 *
 * This library is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, version 3 or later.
 *
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
 ****/

#pragma once

#include <cstdint>
#include <esp_attr.h>
#include "Request.h"
#include <bitset>
#include "Common.h"
#include <ControllerBase.h>

namespace HSPI
{
class Device;

static constexpr uint8_t SPI_PIN_NONE{0xff};
static constexpr uint8_t SPI_PIN_DEFAULT{0xfe};

/**
 * @brief SPI pin connections
 */
struct SpiPins {
	uint8_t sck{SPI_PIN_DEFAULT};
	uint8_t miso{SPI_PIN_DEFAULT};
	uint8_t mosi{SPI_PIN_DEFAULT};
};

/**
 * @brief Manages access to SPI hardware
 */
class Controller : public ControllerBase
{
public:
	/**
	 * @brief Interrupt callback for custom Controllers
	 * @param chipSelect The value passed to `startDevice()`
	 * @param active true when transaction is about to start, false when completed
	 *
	 * For manual CS (PinSet::manual) the actual CS GPIO must be asserted/de-asserted.
	 *
	 * Expanding the SPI bus using a HC138 3:8 multiplexer, for example, can also
	 * be handled here, setting the GPIO address lines appropriately.
	 */
	using SelectDevice = void (*)(uint8_t chipSelect, bool active);

	Controller(SpiBus id = SpiBus::DEFAULT) : ControllerBase(), busId(id)
	{
	}

	Controller(SpiBus id, SpiPins pins) : ControllerBase(), busId(id), mPins(pins)
	{
	}

	virtual ~Controller()
	{
		end();
	}

	/* @brief Initialize the HSPI controller
	 */
	bool begin();

	/** @brief Disable HSPI controller
	 * 	@note Reverts HSPI pins to GPIO and disables the controller
	 */
	void end();

	/**
	 * @brief Determine which IO modes are supported for the given device
	 *
	 * May be restricted by both controller and device capabilities.
	 */
	IoModes getSupportedIoModes(const Device& dev) const;

	/**
	 * @brief Set interrupt callback to use for manual CS control (PinSet::manual)
	 * or if CS pin is multiplexed.
	 *
	 * @note Callback MUST be marked IRAM_ATTR
	 */
	void onSelectDevice(SelectDevice callback)
	{
		selectDeviceCallback = callback;
	}

	/**
	 * @brief Assign a device to a CS# using a specific pin set.
	 * Only one device may be assigned to any CS.
	 *
	 * Custom controllers should override this method to verify/configure chip selects,
	 * and also provide a callback (via `onSelectDevice()`).
	 */
	virtual bool startDevice(Device& dev, PinSet pinSet, uint8_t chipSelect, uint32_t clockSpeed);

	/**
	 * @brief Release CS for a device.
	 */
	virtual void stopDevice(Device& dev);

	/**
	 * @brief Devices call this method to tell the Controller about configuration changes.
	 * Internally, we just set a flag and update the register values when required.
	 */
	void configChanged(Device& dev);

	/**
	 * @brief Get the active bus identifier
	 *
	 * On successful call to begin() returns actual bus in use.
	 */
	SpiBus getBusId() const
	{
		return busId;
	}

#ifdef HSPI_ENABLE_STATS
	struct Stats {
		uint32_t requestCount;   ///< Completed requests
		uint32_t transCount;	 ///< Completed SPI transactions
		uint32_t waitCycles;	 ///< Total blocking CPU cycles
		uint32_t tasksQueued;	///< Number of times task callback registered for async execution (no interrupts)
		uint32_t tasksCancelled; ///< Tasks cancelled by blocking requests

		void clear() volatile
		{
			requestCount = 0;
			transCount = 0;
			waitCycles = 0;
			tasksQueued = 0;
			tasksCancelled = 0;
		}
	};
	static volatile Stats stats;
#endif

	PinSet IRAM_ATTR getActivePinSet() const
	{
		return activePinSet;
	}

	void wait(Request& request);

	/**
	 * @brief For testing, tie MISO <-> MOSI internally
	 * @brief enable true to enable loopback, false for normal receive operation
	 * @retval true on success, false if loopback not supported
	 */
	bool loopback(bool enable);

	const SpiPins& pins{mPins};

protected:
	friend Device;

	virtual void execute(Request& request);

	/**
	 * @brief Assign any default pins
	 */
	void assignDefaultPins(const SpiPins& defPins)
	{
		if(pins.sck == SPI_PIN_DEFAULT) {
			mPins.sck = defPins.sck;
		}
		if(pins.miso == SPI_PIN_DEFAULT) {
			mPins.miso = defPins.miso;
		}
		if(pins.mosi == SPI_PIN_DEFAULT) {
			mPins.mosi = defPins.mosi;
		}
	}

private:
#if defined(ARCH_ESP32)
	static void IRAM_ATTR pre_transfer_callback(spi_transaction_t* t);
	static void IRAM_ATTR post_transfer_callback(spi_transaction_t* t);
#elif defined(ARCH_ESP8266) || defined(ARCH_HOST)
	static void isr(Controller* spi);
#endif

	static void updateConfig(Device& dev);

	void queueTask();
	void executeTask();
	void startRequest();
	void nextTransaction();
	void transactionDone();

	SpiBus busId;
	SpiPins mPins;
	PinSet activePinSet{PinSet::none};
	SelectDevice selectDeviceCallback{nullptr}; ///< Callback for custom controllers

	// State of the current transaction in progress
	struct Transaction {
		Request* request; ///< The current request being executed
		uint32_t addr;		///< Address for next transfer
		uint16_t outOffset; ///< Where to read data for next outgoing transfer
		uint16_t inOffset;  ///< Where to write incoming data from current transfer
		uint8_t inlen;		///< Incoming data for current transfer
		IoMode ioMode;
		// Flags
		uint8_t bitOrder : 1;
		volatile uint8_t busy : 1;
#ifdef ARCH_ESP8266
		uint8_t addrShift;	///< How many bits to shift address left
		uint32_t addrCmdMask; ///< In SDI/SQI modes this is combined with address
#endif
	};
	Transaction trans{};
};

} // namespace HSPI
