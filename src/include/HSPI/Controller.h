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
#include "Request.h"
#include <bitset>
#include "Common.h"

namespace HSPI
{
class Device;

class Controller
{
public:
	struct Config {
		bool dirty{true}; ///< Set when values require updating
		// Pre-calculated register values - see updateConfig()
		struct {
			uint32_t clock{0};
			uint32_t ctrl{0};
			uint32_t pin{0};
			uint32_t user{0};
			uint32_t user1{0};
		} reg;
	};

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

	/* @brief Initialize the HSPI controller
	 */
	void begin();

	/** @brief Disable HSPI controller
	 * 	@note Reverts HSPI pins to GPIO and disables the controller
	 */
	void end();

	bool startDevice(Device& dev, PinSet pinSet, uint8_t chipSelect);

	void stopDevice(Device& dev);

	void configChanged(Device& dev);

	void setSpeed(Device& dev, uint32_t frequency);

	uint32_t getSpeed(Device& dev) const;

#ifdef HSPI_ENABLE_STATS
	struct Stats {
		uint32_t waitCycles;
		uint32_t transCount;

		void clear() volatile
		{
			waitCycles = 0;
			transCount = 0;
		}
	};
	static volatile Stats stats;
#endif

protected:
	friend Device;

	virtual void execute(Request& request);

private:
	static void updateConfig(Device& dev);

	/**
	 * @brief Start transfer of a new request (trans.request)
	 * @note May be called from interrupt context at completion of previous request
	 */
	void startRequest();

	void nextTransaction();
	void nextTransactionSDQI();

	/**
	 * @brief Interrupt on transaction complete
	 */
	static void isr(Controller* spi);

	/**
	 * @brief Read incoming data, if there is any, and start next transaction
	 * @note Called from interrupt context at completion of transaction
	 */
	void transactionDone();

	PinSet activePinSet{PinSet::none};
	uint8_t overlapDevices{0};		 ///< Number of registered devices using overlap pins (SPI0)
	uint8_t normalDevices{0};		 ///< Number of registered devices using HSPI pins (SPI1)
	std::bitset<8> chipSelectsInUse; ///< Ensures each CS is used only once
	struct Flags {
		bool spi0ClockChanged : 1; ///< SPI0 clock MUX setting was changed for a transaction
	};
	Flags flags{};

	static constexpr size_t SPI_BUFSIZE{64};

	// State of the current transaction in progress
	struct Transaction {
		Request* request;   ///< The current request being executed
		uint16_t addr;		///< Address for next transfer
		uint16_t outOffset; ///< Where to read data for next outgoing transfer
		uint16_t inOffset;  ///< Where to write incoming data from current transfer
		uint8_t inlen;		///< Incoming data for current transfer
		IoMode ioMode;
		// Flags
		uint8_t bitOrder : 1;
		volatile uint8_t busy : 1;
		// Used in SDI/SQI modes
		uint8_t cmd[2];
	};
	Transaction trans{};
};

} // namespace HSPI
