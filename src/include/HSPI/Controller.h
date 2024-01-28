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
	uint8_t miso{SPI_PIN_DEFAULT}; ///< or IO0
	uint8_t mosi{SPI_PIN_DEFAULT}; ///< or IO1
	uint8_t io2{SPI_PIN_NONE};	 ///< or WP (Write Protect)
	uint8_t io3{SPI_PIN_NONE};	 ///< or HD (Hold) for flash devices
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
	 * @brief Dynamically change clock speed for device
	 * @param freq Clock speed in Hz
	 * @retval uint32_t Actual clock speed in use
	 * @note Always return actual clock speed, even if dynamic changes aren't supported
	 */
	uint32_t setClockSpeed(Device& dev, uint32_t freq);

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
		if(pins.io2 == SPI_PIN_DEFAULT) {
			mPins.io2 = defPins.io2;
		}
		if(pins.io3 == SPI_PIN_DEFAULT) {
			mPins.io3 = defPins.io3;
		}
	}

private:
#if defined(ARCH_ESP32)
	static void IRAM_ATTR pre_transfer_callback(spi_transaction_t* t);
	static void IRAM_ATTR post_transfer_callback(spi_transaction_t* t);
#elif defined(ARCH_ESP8266) || defined(ARCH_HOST)
	static void isr(Controller* spi);
#elif defined(ARCH_RP2040)
	void configure_dma(volatile void* fifo_addr, uint8_t dreq_tx, uint8_t dreq_rx);
	void release_dma();
	void interruptHandler();
	static void staticInterruptHandler();
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
#ifdef ARCH_RP2040
		// RP2040 doesn't have a restriction on transaction size, so doesn't need to split requests
#else
		uint32_t addr;		///< Address for next transfer
		uint16_t outOffset; ///< Where to read data for next outgoing transfer
		uint16_t inOffset;  ///< Where to write incoming data from current transfer
		uint8_t inlen;		///< Incoming data for current transfer
		IoMode ioMode;
#endif
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
