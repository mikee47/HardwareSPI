#include <SmingTest.h>
#include <HSPI/Device.h>

namespace
{
/*
void printBin(const char* tag, uint32_t value)
{
	char buf[40];
	ultoa_wp(value, buf, 2, 32, '0');
	m_puts(tag);
	m_putc(' ');
	m_nputs(buf, 8);
	m_putc(' ');
	m_nputs(buf + 8, 8);
	m_putc(' ');
	m_nputs(buf + 16, 8);
	m_putc(' ');
	m_nputs(buf + 24, 8);
	m_puts("\r\n");
}
*/

class TestDevice : public HSPI::Device
{
public:
	using Device::Device;

	HSPI::IoModes getSupportedIoModes() const
	{
		return HSPI::IoModes::domain();
	}
};

#if defined(ARCH_ESP32)
constexpr uint8_t chipSelect{2};
#elif defined(ARCH_RP2040)
constexpr uint8_t chipSelect{20};
#else
constexpr uint8_t chipSelect{0};
#endif

constexpr uint32_t clockSpeed{100000};

} // namespace

class HardwareSpiTest : public TestGroup
{
public:
	HardwareSpiTest() : TestGroup(F("HardwareSPI")), dev(spi)
	{
	}

	void execute() override
	{
		REQUIRE(spi.begin());
		REQUIRE(dev.begin(HSPI::PinSet::normal, chipSelect, clockSpeed));
		looped = spi.loopback(true);

		debug_w("Connected SCK %u, MISO %u, MOSI %u, clock %u", spi.pins.sck, spi.pins.miso, spi.pins.mosi,
				dev.getSpeed());

		largeTransfer();
	}

	void largeTransfer()
	{
		TEST_CASE("Large transfer")
		{
			// constexpr size_t bufSize{6666};
			lt.reset(new LargeTransfer);
			lt->init();
			lt->req.setAsync(
				[](HSPI::Request& request) {
					System.queueCallback(
						[](void* param) {
							auto self = static_cast<HardwareSpiTest*>(param);
							self->largeTransferComplete();
						},
						request.param);
					return true;
				},
				this);
			dev.execute(lt->req);
		}

		pending();
	}

	void largeTransferComplete()
	{
		debug_i("Async request complete");
		if(looped) {
			REQUIRE(memcmp(lt->inbuf, lt->outbuf, lt->bufSize) == 0);
		}
		lt.reset();
		testCombos();
		testBitPatterns();
	}

	void testCombos()
	{
		using namespace HSPI;

		for(unsigned i = 0; i < unsigned(HSPI::IoMode::MAX); ++i) {
			auto mode = HSPI::IoMode(i);
			if(!dev.isSupported(mode)) {
				continue;
			}
			Serial.print(F("\r\n\n***  IoMode::"));
			Serial.print(toString(mode));
			Serial.println(F("  ***"));
			dev.setIoMode(mode);

			TEST_CASE("Command only")
			{
				Request req;
				req.setCommand(0xCA, 8);
				dev.execute(req);
			}

			TEST_CASE("Command + Address + MOSI")
			{
				Request req;
				req.setCommand(0xCA, 8);
				req.setAddress(0x123456, 24);
				req.out.set16(0xabcd);
				dev.execute(req);
			}

			TEST_CASE("Command + Address + MISO")
			{
				Request req;
				req.setCommand(0xCA, 8);
				req.setAddress(0x123456, 24);
				req.in.set32(0);
				dev.execute(req);
				debug_i("<< 0x%08x", req.in.data32);
			}

			TEST_CASE("Command + Address + DUMMY + MISO")
			{
				Request req;
				req.setCommand(0xCA, 8);
				req.setAddress(0x123456, 24);
				req.dummyLen = 1;
				req.in.set32(0);
				dev.execute(req);
				debug_i("<< 0x%08x", req.in.data32);
			}

			TEST_CASE("MOSI only")
			{
				Request req;
				req.out.set32(0x12345678);
				dev.execute(req);
			}

			TEST_CASE("MISO only")
			{
				Request req;
				req.in.set32(0);
				dev.execute(req);
				debug_i("<< 0x%08x", req.in.data32);
			}
		}
	}

	void testBitPatterns()
	{
		TEST_CASE("Bit Patterns")
		{
			debug_w("Connect scope and observe bit pattern");
			constexpr unsigned duration{10};
			constexpr unsigned loopInterval{250};
			loopCount = duration * 1000 / loopInterval;
			timer.initializeMs<loopInterval>([this]() { bitPatterns(); });
			timer.start();
			return pending();
		}
	}

	/*
	 * 6 transactions:
	 *
	 * - MODE0 MSBFIRST       0xAA00AA00    0x12345678    0x12 0x34 0x56 0x78 0x9A
	 * 	   Viewed MSB first:    00 AA 00 AA   12 34 56 78   12 34 56 78 9A
	 *     Viewed LSB first:    00 55 00 55   48 2C 6A 1E   48 2C 6A 1E 59
	 * - MODE0 LSBFIRST       0xAA00AA00    0x12345678    0x12 0x34 0x56 0x78 0x9A
	 *     Viewed MSB first:    55 00 55 00   1E 6A 2C 48   48 2C 6A 1E 59
	 * 	   Viewed LSB first:    AA 00 AA 00   78 56 34 12   12 34 56 78 9A
	 * - MODE0 MSBFIRST 0xAA (13 bits)
	 *     Clock idles LOW, latch on RISING edge
	 * - MODE1 MSBFIRST 0xAA (13 bits)
	 *     Clock idles LOW, latch on FALLING edge
	 * - MODE2 MSBFIRST 0xAA (13 bits)
	 *     Clock idles HIGH, latch on FALLINGg edge
	 * - MODE3 MSBFIRST 0xAA (13 bits)
	 *     Clock idles HIGH, latch on RISING edge
	 */
	void bitPatterns()
	{
		using namespace HSPI;

		dev.setClockMode(ClockMode::mode0);
		for(uint8_t bitOrder : {MSBFIRST, LSBFIRST}) {
			dev.setBitOrder(bitOrder);
			Request req;
			req.out.set32(0x00AA00AA);
			dev.execute(req);
			req.out.set32(0x92345679);
			dev.execute(req);
			uint8_t data[]{0x12, 0x34, 0x56, 0x78, 0x9A};
			req.out.set(data, sizeof(data));
			req.in.set(data, sizeof(data));
			dev.execute(req);
			delayMicroseconds(100);
		}

		delayMicroseconds(200);
		for(auto clockMode : {ClockMode::mode0, ClockMode::mode1, ClockMode::mode2, ClockMode::mode3}) {
			dev.setBitOrder(MSBFIRST);
			dev.setClockMode(clockMode);
			uint8_t data[]{0xCB, 0x15, 0x47};
			Request req;
			req.out.set(data, sizeof(data));
			uint8_t buf[3]{};
			req.in.set(buf, sizeof(buf));
			dev.execute(req);
			delayMicroseconds(100);
		}

		// Leave bus in mode 0
		dev.setClockMode(ClockMode::mode0);

		m_putc('.');

		if(loopCount-- != 0) {
			return;
		}

		timer.stop();
		m_puts("\r\n");

		complete();
	}

	/*
	void loopbackTests()
	{
		if(!spi.loopback(true)) {
			debug_w("WARNING: SPI loopback not supported. Manual connection required.");
			debug_w("Connect MISO (GPIO%u) <-> MOSI (GPIO%u)", spi.pins.miso, spi.pins.mosi);
			allowFailure = true;
		}

		settings.speed = 8000000;
		settings.dataMode = SPI_MODE0;

		TEST_CASE("32-bit values")
		{
#ifdef ARCH_HOST
			SPI.setDebugIoCallback(ioCallback);
#endif
			clearStats();

			 * Note: Single-bit transfers fail on esp32c3, and RP2040 doesn't
			 * support less than 4 bits. So start at 4.
			for(auto bitOrder : {MSBFIRST, LSBFIRST}) {
				settings.bitOrder = bitOrder;
				for(auto bits : {4, 7, 8, 9, 15, 16, 17, 19, 23, 24, 25, 29, 30, 31}) {
					send(0, bits);
					send(0xffffffff, bits);
					send(0xaaaaaaaa, bits);
					send(0x55555555, bits);
					send(0x12345678, bits);
					send(~0x12345678, bits);
				}
			}
#ifdef ARCH_HOST
			SPI.setDebugIoCallback(nullptr);
#endif

			printStats();
		}

		TEST_CASE("Byte sequences")
		{
			clearStats();

			DEFINE_FSTR_LOCAL(seq1, "This is a longer sequence but no more than 64 bytes");
			for(auto bitOrder : {MSBFIRST, LSBFIRST}) {
				settings.bitOrder = bitOrder;
				for(unsigned offset = 0; offset < 4; ++offset) {
					// Small packet, fits in FIFO
					send(seq1, offset);

					// Packet larger than FIFO
					String seq2 = seq1;
					seq2 += seq2;
					seq2 += seq2;
					seq2 += seq2;
					seq2 += seq2;
					send(seq2, offset);
				}
			}

			printStats();
		}
	}

	void send(uint32_t outValue, uint8_t bits)
	{
		outValue &= BIT(bits) - 1;
		Serial.print("TX 0x");
		Serial.print(outValue, HEX);
		Serial.print(", ");
		Serial.print(bits);
		Serial.print(" bits, ");
		Serial.print(settings.bitOrder == MSBFIRST ? 'M' : 'L');
		Serial.println("SB first");
		printBin(">", outValue);

		beginTrans();
		spi.beginTransaction(settings);
		uint32_t inValue = spi.transfer32(outValue, bits);
		endTrans(bits);

		if(inValue != outValue) {
			printBin("<", inValue);
			Serial.print("RX 0x");
			Serial.println(inValue, HEX);
			if(allowFailure) {
				fail(__PRETTY_FUNCTION__);
				return;
			}
		}

		REQUIRE(inValue == outValue);
	}

	void send(const String& outData, unsigned startOffset)
	{
		String inData = outData;
		auto bufptr = reinterpret_cast<uint8_t*>(inData.begin() + startOffset);
		auto length = inData.length() - startOffset;

		debug_i("TX %u bytes, startOffset %u, %cSB first", length, startOffset,
				settings.bitOrder == MSBFIRST ? 'M' : 'L');

		beginTrans();
		spi.transfer(bufptr, length);
		endTrans(length * 8);

		auto outptr = outData.c_str() + startOffset;
		if(memcmp(outptr, bufptr, length) != 0) {
			length = std::min(length, 64U);
			m_printHex(">", outptr, length);
			m_printHex("<", bufptr, length);
			if(allowFailure) {
				fail(__PRETTY_FUNCTION__);
			} else {
				TEST_ASSERT(false);
			}
		}
	}

*/

private:
	HSPI::Controller spi;
	TestDevice dev;
	Timer timer;
	unsigned loopCount;
	bool looped{false};

	struct LargeTransfer {
		static constexpr size_t bufSize{8192};
		HSPI::Request req;
		uint8_t outbuf[bufSize];
		uint8_t inbuf[bufSize];

		void init()
		{
			os_get_random(outbuf, bufSize);
			req.out.set(outbuf, bufSize);
			req.in.set(inbuf, bufSize);
		}
	};
	std::unique_ptr<LargeTransfer> lt;
};

void REGISTER_TEST(HSPI)
{
	registerGroup<HardwareSpiTest>();
}
