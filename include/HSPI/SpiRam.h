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

	void init()
	{
		setSpeed(10000000U);
		setBitOrder(MSBFIRST);
		setClockMode(ClockMode::Mode0);

		// Ensure device is in SPI mode
		Device::setIoMode(IoMode::SQI);
		Packet packet;
		packet.out.set8(0xFF);
		execute(packet);
		Device::setIoMode(IoMode::SDI);
		execute(packet);
		Device::setIoMode(IoMode::SPIHD);

		debug_i("RDMR = 0x%08x", readMode());

		setIoMode(IoMode::SQI);
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

		Packet packet;
		if(oldMode != IoMode::SPIHD) {
			packet.out.set8(0xFF); // Exit SDI/SQI mode
			execute(packet);
		}

		if(mode != IoMode::SPIHD) {
			packet.out.set8((mode == IoMode::SDI) ? 0x3B : 0x38);
			execute(packet);
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
		Packet packet;
		packet.setCommand8(0x01); // WRMR
		packet.out.set8(uint8_t(mode));
		execute(packet);
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

		Packet packet;
		packet.setCommand8(0x05); // RDMR
		packet.in.set8(0);
		execute(packet);
		mode = Mode(packet.in.data8);

		setIoMode(savedIoMode);
		return mode;
	}

	void write(uint32_t address, const void* data, size_t len)
	{
		Packet packet;
		if(getIoMode() == IoMode::SPIHD) {
			packet.setCommand8(0x02); // Write
			packet.setAddress24(address);
			packet.out.set(data, len);
			execute(packet);
		} else {
			auto buf = new uint8_t[4 + len];
			buf[0] = 0x02;
			buf[1] = address >> 16;
			buf[2] = address >> 8;
			buf[3] = address;
			memcpy(&buf[4], data, len);
			packet.out.set(buf, 4 + len);
			execute(packet);
			delete buf;
		}
	}

	void read(uint32_t address, void* buffer, size_t len)
	{
		Packet packet;
		if(getIoMode() == IoMode::SPIHD) {
			packet.setCommand8(0x03); // Read
			packet.setAddress24(address);
			packet.dummyLen = 8;
		} else {
			packet.out.set32(0);
			auto& buf = packet.out.data;
			buf[0] = 0x03;
			buf[1] = address >> 16;
			buf[2] = address >> 8;
			buf[3] = address;
			packet.dummyLen = (getIoMode() == IoMode::DIO) ? 4 : 2;
		}
		packet.in.set(buffer, len);
		execute(packet);
	}

private:
	Mode mode{Mode::Sequential};
};

} // namespace HSPI
