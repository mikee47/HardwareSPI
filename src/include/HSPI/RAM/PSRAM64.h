/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 *
 * PSRAM64.h
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
 * @brief PSRAM64(H) pseudo-SRAM
 * @ingroup hw_spi
 */
class PSRAM64 : public MemoryDevice
{
public:
	using MemoryDevice::MemoryDevice;

	size_t getSize() const override
	{
		return 8 * 1024 * 1024;
	}

	IoModes getSupportedIoModes() const override
	{
		return IoModes(IoMode::SPIHD | IoMode::QIO | IoMode::SQI);
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

		// Exit QUAD mode
		Request req;
		MemoryDevice::setIoMode(IoMode::SQI);
		req.setCommand8(0xF5);
		execute(req);

		MemoryDevice::setIoMode(IoMode::SPIHD);

		// Issue RESET
		req.setCommand8(0x66);
		execute(req);
		req.setCommand8(0x99);
		execute(req);

		readId();

		return true;
	}

	uint8_t readId()
	{
		auto savedIoMode = getIoMode();
		if(!setIoMode(IoMode::SPIHD)) {
			debug_e("readId() requires SPIHD IO");
			return 0;
		}

		uint8_t buffer[8];
		Request req;
		req.setCommand8(0x9F); // Read ID
		req.setAddress24(0);
		req.in.set(buffer, sizeof(buffer));
		execute(req);

		setIoMode(savedIoMode);

		debug_hex(ERR, "ID", buffer, sizeof(buffer));

		return buffer[0];
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
		if(oldMode == IoMode::SQI) {
			req.setCommand8(0xF5); // Exit Quad Mode
			execute(req);
		} else if(mode == IoMode::SQI) {
			MemoryDevice::setIoMode(IoMode::SPIHD);
			req.setCommand8(0x35); // Enter Quad Mode
			execute(req);
		}

		return MemoryDevice::setIoMode(mode);
	}

	void prepareWrite(HSPI::Request& req, uint32_t address) override
	{
		bool quad = (getIoMode() != IoMode::SPIHD);
		req.prepare();
		req.setCommand8(quad ? 0x38 : 0x02);
		req.setAddress24(address);
		req.dummyLen = 0;
	}

	void prepareRead(HSPI::Request& req, uint32_t address) override
	{
		bool quad = (getIoMode() != IoMode::SPIHD);
		req.prepare();
		req.setCommand8(quad ? 0xEB : 0x0B);
		req.setAddress24(address);
		req.dummyLen = quad ? 6 : 8;
	}

private:
	HSPI::Request req1;
	HSPI::Request req2;
};

} // namespace RAM
} // namespace HSPI
