#include <SmingCore.h>
#include <Platform/Timers.h>
#include <HSPI/RAM/PSRAM64.h>
#include <HSPI/RAM/IS62-65.h>
#include <BasicDevice.h>
#include <Services/Profiling/CpuUsage.h>
#include <HSPI/StreamAdapter.h>

namespace
{
HSPI::Controller spi;
// HSPI::RAM::PSRAM64 ram(spi);
HSPI::RAM::IS62_65 ram(spi);
SimpleTimer ramTimer;
BasicDevice dev0(spi), dev1(spi), dev2(spi);
Profiling::CpuUsage cpuUsage;
CpuCycleTimer streamTimer;
HSPI::StreamAdapter adapter(ram);

DEFINE_FSTR(smingLogo, "sming_logo.raw")

// Chip select to use for our device(s)
#define CS_RAM 2

void printSpiStats()
{
	auto& stats = spi.stats;
	Serial.print(F("HSPI stats: requests = "));
	Serial.print(stats.requestCount);

	Serial.print(F(", trans = "));
	Serial.print(stats.transCount);
	Serial.print(F(", waitCycles = "));
	Serial.print(stats.waitCycles);
	Serial.print(F(", tasks queued = "));
	Serial.print(stats.tasksQueued);
	Serial.print(F(", tasks cancelled = "));
	Serial.println(stats.tasksCancelled);

	Serial.print(F("Cpu Usage: "));
	Serial.print(cpuUsage.getUtilisation() / 100.0);
	Serial.println('%');

	stats.clear();
}

String getSpeed(uint32_t cycles, uint32_t byteCount)
{
	String s;
	s += cycles;
	s += " for ";
	s += byteCount / 1024;
	s += "K (";
	auto time = CpuCycleTimer::ticksToTime(cycles);
	double mbps = (8 * 1e9 / 1e6) * byteCount / time;
	s += mbps;
	s += "Mbit/s)";
	return s;
}

uint32_t countDiffs(const uint8_t* p1, const uint8_t* p2, size_t length)
{
	uint32_t count{0};
	for(size_t i = 0; i < length; ++i) {
		if(p1[i] != p2[i]) {
			++count;
		}
	}
	return count;
}

void memTest()
{
	Serial.print(_F("Testing memory: IO Mode = "));
	auto& info = HSPI::getIoModeInfo(ram.getIoMode());
	Serial.print(*info.name);
	Serial.print(_F(", speed = "));
	Serial.println(ram.getSpeed());

	const unsigned blockSize{1024};
	const unsigned ramSize = ram.getSize();

	std::unique_ptr<uint8_t[]> wrData(new uint8_t[blockSize]);
	std::unique_ptr<uint8_t[]> rdData(new uint8_t[blockSize]);

	uint8_t id[16];
	ram.readId(id, sizeof(id));

	bool ok{true};
	CpuCycleTimer timer;
	uint32_t writeCycles{0};
	uint32_t readCycles{0};
	const uint32_t startAddress{0};
	uint32_t address = startAddress;
	uint32_t failCount{0};
	uint32_t failedBlockCount{0};
	for(; address < ramSize; address += blockSize) {
		os_get_random(wrData.get(), blockSize);

		// Write
		CpuCycleTimer tmr;
		tmr.start();
		ram.write(address, wrData.get(), blockSize);
		writeCycles += tmr.elapsedTicks();

		// Read
		memset(rdData.get(), 0, blockSize);
		tmr.start();
		ram.read(address, rdData.get(), blockSize);
		readCycles += tmr.elapsedTicks();

		// Compare
		auto diffCount = countDiffs(wrData.get(), rdData.get(), blockSize);

		if(diffCount != 0) {
			ok = false;
			++failedBlockCount;

			if(failCount == 0) {
				debug_e("memTest failed at 0x%08x", address);
				auto len = std::min(blockSize, 256U);
				m_printHex("WR", wrData.get(), len);
				m_printHex("RD", rdData.get(), len);
			}
			//									break;

			failCount += diffCount;
		}

		WDT.alive();
	}

	auto byteCount = address - startAddress;

	if(ok) {
		Serial.println(F("memTest complete"));
	} else {
		Serial.print(F("Mismatch in "));
		Serial.print(failCount);
		Serial.print(F(" bytes ("));
		Serial.print(100.0 * failCount / byteCount);
		Serial.print(F("%), "));
		Serial.print(failedBlockCount);
		Serial.print('/');
		Serial.print(byteCount / blockSize);
		Serial.println(F(" failed blocks"));
	}

	Serial.print(F("Elapsed: "));
	Serial.print(timer.elapsedTicks());
	Serial.print(F(" cycles, I/O write: "));
	Serial.print(getSpeed(writeCycles, byteCount));
	Serial.print(F(", I/O read: "));
	Serial.println(getSpeed(readCycles, byteCount));

	Serial.print(F("Equivalent 4K cycles, I/O write: "));
	Serial.print(4096.0 * writeCycles / byteCount);
	Serial.print(F(", I/O read: "));
	Serial.println(4096.0 * readCycles / byteCount);

	printSpiStats();
}

void spiRamTest()
{
	Serial.println(F("SPI RAM test"));

	for(unsigned i = 0; i < 8; ++i) {
		auto m = HSPI::IoMode(i);
		if(!ram.isSupported(m)) {
			continue;
		}
		assert(ram.setIoMode(m));
		memTest();
	}
}

void basicTests(HSPI::PinSet pinSet)
{
	dev0.begin(pinSet, 0);
	dev1.begin(pinSet, 1);
	dev2.begin(pinSet, 2);

	dev0.write();
	dev0.read();
	dev1.write();
	dev1.read();
	dev2.write();
	dev2.read();

	dev2.end();
	dev1.end();
	dev0.end();
}

void basicTests()
{
	//	basicTests(HSPI::PinSet::overlap);
	basicTests(HSPI::PinSet::normal);
}

void printAdapterStats()
{
	auto requested = adapter.getBytesRequested();
	auto transferred = adapter.getBytesTransferred();
	String speed = getSpeed(streamTimer.elapsedTime(), transferred);

	Serial.print(F("Stream "));
	Serial.print(adapter.getIsWrite() ? "WRITE" : "READ");
	Serial.print(F(" all done, "));
	Serial.print(requested);
	Serial.print(F(" bytes requested, "));
	Serial.print(transferred);
	Serial.print(F(" bytes transferred, elapsed = "));
	Serial.println(speed);

	Serial.print(F("Max tasks = "));
	Serial.println(System.getMaxTaskCount());
}

void ramStreamTest()
{
	Serial.println(F("RAM Stream Test"));

	cpuUsage.reset();
	streamTimer.start();
	auto stream = new FileStream(smingLogo);
	adapter.write(stream, 0, 409600, []() {
		printAdapterStats();
		printSpiStats();

		auto len = adapter.getBytesTransferred();
		// auto fs = new FileStream("logo.out", eFO_CreateNewAlways | eFO_ReadWrite);
		auto fs = new LimitedMemoryStream(1024);
		streamTimer.start();
		adapter.read(fs, 0, len, []() {
			printAdapterStats();
			printSpiStats();
		});
	});
}

void ready()
{
	spiRamTest();
	//ramTimer.initializeMs<10000>(spiRamTest).start();
	//	System.queueCallback(spiRamTest);

	ramTimer.initializeMs<2000>(ramStreamTest).startOnce();
}

} // namespace

void init()
{
// Start serial for serial monitor
// CS #1 conflicts with regular serial TX pin, so switch to secondary debug
#if !defined(ENABLE_GDB) && !defined(ARCH_HOST) && (CS_LCD == 1 || CS_RAM == 1)
	Serial.setPort(UART_ID_1);
#endif
	Serial.setTxBufferSize(2048);
	Serial.setTxWait(true);
	Serial.begin(SERIAL_BAUD_RATE);
	Serial.systemDebugOutput(true);

	fwfs_mount();

	spi.begin();

	// Initialise RAM
	ram.setSpeed(40000000U);
	if(!ram.begin(HSPI::PinSet::overlap, CS_RAM)) {
		Serial.println(F("Failed to start RAM device"));
	}
	Serial.print(F("RAM clock = "));
	Serial.println(ram.getSpeed());

	cpuUsage.begin(ready);
}
