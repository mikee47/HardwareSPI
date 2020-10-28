/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * IS62-65.h
 *
 * @author: October 2020 - mikee47 <mike@sillyhouse.net>
 *
*/

#pragma once

#include "../MemoryDevice.h"

namespace HSPI
{
namespace RAM
{
/**
 * @brief IS62/65WVS2568GALL fast serial RAM
 * @ingroup hw_spi
 */
class IS62_65 : public MemoryDevice
{
public:
	using MemoryDevice::MemoryDevice;

	/**
	 * @brief Memory operating mode determines how read/write operations are performed
	 */
	enum class OpMode {
		Byte = 0x00,	   ///< Limited to one byte
		Page = 0x80,	   ///< Limited to single 32-bit page
		Sequential = 0x40, ///< Access entire memory array (DEFAULT)
	};

	size_t getSize() const override
	{
		return 256 * 1024;
	}

	IoModes getSupportedIoModes() const override
	{
		return IoMode::SPIHD | IoMode::SDI | IoMode::SQI;
	}

	/**
	 * @brief Configure the RAM into a known operating mode
	 */
	bool begin(PinSet pinSet, uint8_t chipSelect)
	{
		if(!MemoryDevice::begin(pinSet, chipSelect)) {
			return false;
		}

		setBitOrder(MSBFIRST);
		setClockMode(ClockMode::mode0);

		// Ensure device is in SPI mode
		MemoryDevice::setIoMode(IoMode::SQI);
		Request req;
		req.out.set8(0xFF);
		execute(req);
		MemoryDevice::setIoMode(IoMode::SDI);
		execute(req);
		MemoryDevice::setIoMode(IoMode::SPIHD);

		debug_i("RDMR = 0x%08x", getOpMode());

		setOpMode(OpMode::Sequential);
		setIoMode(IoMode::SQI);

		return true;
	}

	bool setIoMode(IoMode mode) override
	{
		auto oldMode = MemoryDevice::getIoMode();
		if(oldMode == mode) {
			return true;
		}

		if(!isSupported(mode)) {
			debug_e("setIoMode(): Mode %u invalid", unsigned(mode));
			return false;
		}

		Request req;
		if(oldMode != IoMode::SPIHD) {
			req.out.set8(0xFF); // Exit SDI/SQI mode
			execute(req);
			MemoryDevice::setIoMode(IoMode::SPIHD);
		}

		if(mode != IoMode::SPIHD) {
			req.out.set8((mode == IoMode::SDI) ? 0x3B : 0x38);
			execute(req);
		}

		return MemoryDevice::setIoMode(mode);
	}

	void setOpMode(OpMode mode)
	{
		auto savedIoMode = getIoMode();
		if(!setIoMode(IoMode::SPIHD)) {
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
		auto savedIoMode = getIoMode();
		setIoMode(IoMode::SPIHD);

		Request req;
		req.setCommand8(0x05); // RDMR
		req.in.set8(0);
		execute(req);
		opMode = OpMode(req.in.data8);

		setIoMode(savedIoMode);
		return opMode;
	}

	void prepareWrite(HSPI::Request& req, uint32_t address) override
	{
		req.prepare();
		req.setCommand8(0x02); // Write
		req.setAddress24(address);
		req.dummyLen = 0;
	}

	void prepareRead(HSPI::Request& req, uint32_t address) override
	{
		req.prepare();
		req.setCommand8(0x03); // Read
		req.setAddress24(address);
		req.dummyLen = 8 / getBitsPerClock();
	}

private:
	OpMode opMode{OpMode::Sequential};
	HSPI::Request req1;
	HSPI::Request req2;
};

} // namespace RAM
} // namespace HSPI
