#include <SmingCore.h>
#include <HSPI/Device.h>

/*
Pinout display koblet til geekreit:
MISO SD0        (oransj -> 8H)
LED             (gul    -> +)
CLK  CLK        (grønn  -> 7H)
MOSI SD1        (blå    -> 10H)
DC   ~D1/GPIO5  (lilla  -> 14V)
RST  ~D2/GPIO4  (grå    -> 13V)
CS   ~D3/GPIO0  (hvit   -> 12V)
GND             (svart  -> -)
VCC             (brun   -> +)
*/

#define PIN_DC 5
#define PIN_RESET 4

#define PINSET_OVERLAP_HSPI_CS 0 // = GPIO15 = TXD2 = ~D8
#define PINSET_OVERLAP_SPI_CS1 1 // = GPIO1 = TXD1 = ~D4
#define PINSET_OVERLAP_SPI_CS2 2 // = GPIO0 = UART0_TXD = ~D3

namespace
{
class ILI9341_SPI : public HSPI::Device
{
public:
	ILI9341_SPI(HSPI::Controller& controller) : Device(controller)
	{
	}

	HSPI::IoModes getSupportedIoModes() const override
	{
		return HSPI::IoMode::SPI;
	}

	bool begin(HSPI::PinSet pinSet, uint8_t chipSelect)
	{
		if(!Device::begin(pinSet, chipSelect)) {
			return false;
		}
		setSpeed(1000000U);
		setBitOrder(MSBFIRST);
		setClockMode(HSPI::ClockMode::mode0);
		setIoMode(HSPI::IoMode::SPI);
		return true;
	}
};

HSPI::Controller spi;
ILI9341_SPI screen(spi);

void dumpBuffer(const String& debugText, void* buffer, size_t length)
{
	Serial.print(debugText);
	m_printHex("RX", buffer, length);
}

void close()
{
	Serial.println("*** close");
	screen.end();
	spi.end();
	Serial.println("*** done");
}

void IRAM_ATTR print_request(const String& txt, HSPI::Request& r)
{
	Serial.println(txt);
	if(r.cmdLen > 0) {
		Serial.printf("Command : %02X\n", r.cmd);
	}
	if(r.addrLen > 0) {
		Serial.printf("Address : %0x (%d)\n", r.addr, r.addrLen);
	}
	if(r.in.length > 0) {
		Serial.printf("Data in : %0x (%d)\n", r.in.data32, r.in.length);
	}
	if(r.out.length > 0) {
		Serial.printf("Data out: %0x (%d)\n", r.out.data32, r.out.length);
	}
}

void IRAM_ATTR request_complete(HSPI::Request& r)
{
	System.queueCallback(close);
}

void testSequence()
{
	Serial.println("****************************************");
	Serial.println("*** start");
	pinMode(PIN_DC, OUTPUT);
	pinMode(PIN_RESET, OUTPUT);

	// reset display
	digitalWrite(PIN_RESET, HIGH);
	digitalWrite(PIN_RESET, LOW);
	delay(50);
	digitalWrite(PIN_RESET, HIGH);
	delay(100);

	// init controller
	spi.begin();

	// assign device to CS-pin
	screen.begin(HSPI::PinSet::overlap, PINSET_OVERLAP_SPI_CS2);

	HSPI::Request req;
	req.setCommand8(0x0c);
	req.in.set8(0);
	digitalWrite(PIN_DC, LOW);
	screen.execute(req);
	print_request("*** Request done", req);

	// //r2.setCommand(0xd3, 8);
	// r2.device = &screen;
	// r2.in.set32(0x0, 4);
	// r2.callback = request_complete;
	// print_request("*** Request 2", r2);
	// digitalWrite(PIN_DC, 0);
	// screen.execute(r2);
	// print_request("*** Request 2 done", r2);
}

} // namespace

void init()
{
	Serial.begin(SERIAL_BAUD_RATE);
	Serial.systemDebugOutput(true);

	debug_i("ILI9341 comms test using HardwareSPI library");

	testSequence();
}
