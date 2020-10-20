/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * SpiRam.h
 *
 * @author: October 2020 - mikee47 <mike@sillyhouse.net>
 *
*/

#pragma once

#include "Device.h"

namespace HSPI
{
/**
 * @brief IS62/65WVS2568GALL fast serial RAM
 * @ingroup hw_spi
 */
class SpiRam : public Device
{
public:
	using Device::Device;

	/**
	 * @brief Memory operating mode determines how read/write operations are performed
	 */
	enum class OpMode {
		Byte = 0x00,	   ///< Limited to one byte
		Page = 0x80,	   ///< Limited to single 32-bit page
		Sequential = 0x40, ///< Access entire memory array (DEFAULT)
	};

	/**
	 * @brief Configure the RAM into a known operating mode
	 */
	bool begin(PinSet pinSet, uint8_t chipSelect)
	{
		if(!Device::begin(pinSet, chipSelect)) {
			return false;
		}

		setSpeed(40000000U);
		setBitOrder(MSBFIRST);
		setClockMode(ClockMode::mode0);

		// Ensure device is in SPI mode
		Device::setIoMode(IoMode::SQI);
		Request req;
		req.out.set8(0xFF);
		execute(req);
		Device::setIoMode(IoMode::SDI);
		execute(req);
		Device::setIoMode(IoMode::SPIHD);

		debug_i("RDMR = 0x%08x", getOpMode());

		setOpMode(OpMode::Sequential);
		setIoMode(IoMode::SQI);

		return true;
	}

	/**
	 * @brief IO Mode switching requires device support
	 * @param mode Requested new mode
	 * @retval IoMode Previous mode
	 *
	 * If requested mode is not supported the current mode will be returned.
	 */
	IoMode setIoMode(IoMode mode)
	{
		auto oldMode = Device::getIoMode();
		if(oldMode == mode) {
			return oldMode;
		}

		if(mode != IoMode::SPIHD && mode != IoMode::SDI && mode != IoMode::SQI) {
			debug_e("setIoMode(): Mode %u invalid", unsigned(mode));
			return oldMode;
		}

		Request req;
		if(oldMode != IoMode::SPIHD) {
			req.out.set8(0xFF); // Exit SDI/SQI mode
			execute(req);
		}

		if(mode != IoMode::SPIHD) {
			req.out.set8((mode == IoMode::SDI) ? 0x3B : 0x38);
			execute(req);
		}

		Device::setIoMode(mode);
		return oldMode;
	}

	void setOpMode(OpMode mode)
	{
		auto savedIoMode = setIoMode(IoMode::SPIHD);

		if(getIoMode() != IoMode::SPIHD) {
			debug_e("writeMode() requires SPIHD IO");
			return;
		}

		debug_i("WRMR(%u)", unsigned(mode));
		Request req;
		req.setCommand8(0x01); // WRMR
		req.out.set8(uint8_t(mode));
		execute(req);
		this->opMode = mode;

		setIoMode(savedIoMode);
	}

	/**
	 * @brief Get current operating mode (cached value)
	 * @retval OpMode
	 *
	 * No device access is performed.
	 */
	OpMode getOpMode() const
	{
		return opMode;
	}

	/**
	 * @brief Read current operating mode from device
	 * @retval OpMode
	 */
	OpMode readOpMode()
	{
		// requires SPIHD
		auto savedIoMode = setIoMode(IoMode::SPIHD);

		Request req;
		req.setCommand8(0x05); // RDMR
		req.in.set8(0);
		execute(req);
		opMode = OpMode(req.in.data8);

		setIoMode(savedIoMode);
		return opMode;
	}

	/**
	 * @brief Prepare a write request
	 * @param request
	 * @param address
	 * @param data
	 * @param len
	 */
	void prepareWrite(HSPI::Request& req, uint32_t address, const void* data, size_t len)
	{
		req.setCommand8(0x02); // Write
		req.setAddress24(address);
		req.out.set(data, len);
	}

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
	void prepareRead(HSPI::Request& req, uint32_t address, void* buffer, size_t len)
	{
		req.setCommand8(0x03); // Read
		req.setAddress24(address);
		req.dummyLen = 8 / getBitsPerClock();
		req.in.set(buffer, len);
	}

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

private:
	OpMode opMode{OpMode::Sequential};
	HSPI::Request req1;
	HSPI::Request req2;
};

} // namespace HSPI
