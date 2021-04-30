#include <SmingCore.h>
#include <HSPI/Device.h>

/*
Pinout display koblet til geekreit:
MISO SD0        (oransj -> 8H)m
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

		pinMode(PIN_DC, OUTPUT);
		pinMode(PIN_RESET, OUTPUT);

		reset();
		return true;
	}

	void reset()
	{
		digitalWrite(PIN_RESET, HIGH);
		digitalWrite(PIN_RESET, LOW);
		delay(50);
		digitalWrite(PIN_RESET, HIGH);
		delay(100);
	}
};

HSPI::Controller spi;
ILI9341_SPI screen(spi);
SimpleTimer timer;

#define STATE_MAP(XX)                                                                                                  \
	XX(powerOn)                                                                                                        \
	XX(softwareReset)                                                                                                  \
	XX(sleepOut)                                                                                                       \
	XX(diagnostic)                                                                                                     \
	XX(run)

enum class State {
#define XX(s) s,
	STATE_MAP(XX)
#undef XX
};

String toString(State state)
{
	switch(state) {
#define XX(s)                                                                                                          \
	case State::s:                                                                                                     \
		return F(#s);
		STATE_MAP(XX)
#undef XX
	default:
		assert(false);
	}
}

State state;

// void dumpBuffer(const String& debugText, void* buffer, size_t length)
// {
// 	Serial.print(debugText);
// 	m_printHex("RX", buffer, length);
// }

// void close()
// {
// 	Serial.println("*** close");
// 	screen.end();
// 	spi.end();
// 	Serial.println("*** done");
// }

void IRAM_ATTR print_request(HSPI::Request& r)
{
	Serial.printf(_F("*** Request done - %s\r\n"), toString(state).c_str());
	if(r.cmdLen > 0) {
		Serial.printf("Command : %02X\n", r.cmd);
	}
	if(r.addrLen > 0) {
		Serial.printf("Address : %0x (%d)\n", r.addr, r.addrLen);
	}
	if(r.out.length > 0) {
		m_printHex("DATA OUT", r.out.get(), r.out.length);
	}
	if(r.in.length > 0) {
		m_printHex("DATA IN", r.in.get(), r.in.length);
	}
}

// void IRAM_ATTR request_complete(HSPI::Request& r)
// {
// 	System.queueCallback(close);
// }

void sendRequest()
{
	HSPI::Request req;

	switch(state) {
	case State::powerOn:
		// Command
		digitalWrite(PIN_DC, LOW);
		req.out.set8(0x01);
		req.in.set8(0);
		screen.execute(req);
		print_request(req);
		state = State::softwareReset;
		break;

	case State::softwareReset:
		// Command
		digitalWrite(PIN_DC, LOW);
		req.out.set8(0x11);
		req.in.set8(0);
		screen.execute(req);
		print_request(req);
		state = State::sleepOut;
		break;

	case State::sleepOut:
		digitalWrite(PIN_DC, LOW);
		req.out.set8(0x0f);
		req.in.set8(0);
		screen.execute(req);
		print_request(req);
		digitalWrite(PIN_DC, HIGH);
		req.out.set16(0);
		req.in.set16(0);
		screen.execute(req);
		print_request(req);
		state = State::diagnostic;
		break;

	case State::diagnostic:
	case State::run:
		state = State::run;
		// Command
		digitalWrite(PIN_DC, LOW);
		req.out.set8(0x04);
		req.in.set8(0);
		screen.execute(req);
		digitalWrite(PIN_DC, HIGH);
		req.out.set32(0);
		req.in.set32(0);
		screen.execute(req);
		print_request(req);
		break;
	}

	// Command

	// Command
	// digitalWrite(PIN_DC, LOW);
	// req.out.set8(0x09);
	// req.in.set8(0);
	// screen.execute(req);
	// print_request(req);

	// //r2.setCommand(0xd3, 8);
	// r2.device = &screen;
	// r2.in.set32(0x0, 4);
	// r2.callback = request_complete;
	// print_request("*** Request 2", r2);
	// digitalWrite(PIN_DC, 0);
	// screen.execute(r2);
	// print_request("*** Request 2 done", r2);
}

void spiInit()
{
	Serial.println("****************************************");
	Serial.println("*** start");

	// init controller
	spi.begin();

	// assign device to CS-pin
	screen.begin(HSPI::PinSet::overlap, PINSET_OVERLAP_SPI_CS2);

	timer.initializeMs<1000>(sendRequest).start();
}

} // namespace

void init()
{
	Serial.begin(SERIAL_BAUD_RATE);
	Serial.systemDebugOutput(true);

	debug_i("ILI9341 comms test using HardwareSPI library");

	spiInit();
}
