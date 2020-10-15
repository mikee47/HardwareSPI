#pragma once

#include "Device.h"

namespace HSPI
{
/**
 * @brief IS62/65WVS2568GALL fast serial RAM
 */
class SpiRam : public Device
{
public:
	using Device::Device;

	enum class Mode {
		Byte = 0x00,
		Page = 0x80,
		Sequential = 0x40,
	};

	bool begin(PinSet pinSet, uint8_t chipSelect)
	{
		if(!Device::begin(pinSet, chipSelect)) {
			return false;
		}

		setSpeed(40000000U);
		setBitOrder(MSBFIRST);
		setClockMode(ClockMode::Mode0);

		// Ensure device is in SPI mode
		Device::setIoMode(IoMode::SQI);
		Request req;
		req.out.set8(0xFF);
		execute(req);
		Device::setIoMode(IoMode::SDI);
		execute(req);
		Device::setIoMode(IoMode::SPIHD);

		debug_i("RDMR = 0x%08x", readMode());

		//		setIoMode(IoMode::SQI);

		return true;
	}

	/**
	 * @retval IoMode Previous mode
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

	void writeMode(Mode mode)
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
		this->mode = mode;

		setIoMode(savedIoMode);
	}

	Mode readMode()
	{
		auto savedIoMode = setIoMode(IoMode::SPIHD);

		if(getIoMode() != IoMode::SPIHD) {
			debug_e("readMode() requires SPIHD IO");
			return mode;
		}

		Request req;
		req.setCommand8(0x05); // RDMR
		req.in.set8(0);
		execute(req);
		mode = Mode(req.in.data8);

		setIoMode(savedIoMode);
		return mode;
	}

	void write(uint32_t address, const void* data, size_t len)
	{
		Request req;
		if(getIoMode() == IoMode::SPIHD) {
			req.setCommand8(0x02); // Write
			req.setAddress24(address);
			req.out.set(data, len);
			execute(req);
		} else {
			auto buf = new uint8_t[4 + len];
			buf[0] = 0x02;
			buf[1] = address >> 16;
			buf[2] = address >> 8;
			buf[3] = address;
			memcpy(&buf[4], data, len);
			req.out.set(buf, 4 + len);
			execute(req);
			delete buf;
		}
	}

	void read(uint32_t address, void* buffer, size_t len)
	{
		Request req;
		if(getIoMode() == IoMode::SPIHD) {
			req.setCommand8(0x03); // Read
			req.setAddress24(address);
			req.dummyLen = 8;
		} else {
			req.out.set32(0);
			auto& buf = req.out.data;
			buf[0] = 0x03;
			buf[1] = address >> 16;
			buf[2] = address >> 8;
			buf[3] = address;
			req.dummyLen = (getIoMode() == IoMode::DIO) ? 4 : 2;
		}
		req.in.set(buffer, len);
		execute(req);
	}

private:
	Mode mode{Mode::Sequential};
};

} // namespace HSPI
