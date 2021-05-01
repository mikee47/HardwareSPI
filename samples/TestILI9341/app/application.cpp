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

#define TFT_DC_DATA digitalWrite(PIN_DC, HIGH)
#define TFT_DC_COMMAND digitalWrite(PIN_DC, LOW)

void transmitCmdData(uint8_t cmd, uint8_t* data, uint8_t numDataByte)
{
	HSPI::Request req;
	TFT_DC_COMMAND;
	req.out.set8(cmd);
	req.in.set8(0);
	screen.execute(req);

	TFT_DC_DATA;
	req.out.set(data, numDataByte);
	req.in.set(data, numDataByte);
	screen.execute(req);
}

// void transmitData(uint16_t data)
// {
// 	TFT_CS_ACTIVE;
// 	SPI.transfer((uint8_t*)&data, 2);
// 	TFT_CS_DEACTIVE;
// }

// void transmitCmdData(uint8_t cmd, uint32_t data)
// {
// 	TFT_DC_COMMAND;

// 	TFT_CS_ACTIVE;
// 	SPI.transfer(cmd);
// 	TFT_CS_DEACTIVE;

// 	TFT_DC_DATA;
// 	TFT_CS_ACTIVE;
// 	SPI.transfer32(data);
// 	TFT_CS_DEACTIVE;
// }

// void transmitData(uint16_t data, int32_t repeats)
// {
// 	TFT_CS_ACTIVE;
// 	while(repeats--) {
// 		SPI.transfer16(data);
// 	}
// 	TFT_CS_DEACTIVE;
// }

void transmitCmd(uint8_t cmd)
{
	HSPI::Request req;
	TFT_DC_COMMAND;
	req.out.set8(cmd);
	req.in.set8(0);
	screen.execute(req);
	TFT_DC_DATA;
}

void spiInit()
{
	Serial.println("****************************************");
	Serial.println("*** start");

	// init controller
	spi.begin();

	// assign device to CS-pin
	screen.begin(HSPI::PinSet::overlap, PINSET_OVERLAP_SPI_CS2);

	uint8_t data[15]{};

	data[0] = 0x39;
	data[1] = 0x2C;
	data[2] = 0x00;
	data[3] = 0x34;
	data[4] = 0x02;
	transmitCmdData(0xCB, data, 5);

	data[0] = 0x00;
	data[1] = 0XC1;
	data[2] = 0X30;
	transmitCmdData(0xCF, data, 3);

	data[0] = 0x85;
	data[1] = 0x00;
	data[2] = 0x78;
	transmitCmdData(0xE8, data, 3);

	data[0] = 0x00;
	data[1] = 0x00;
	transmitCmdData(0xEA, data, 2);

	data[0] = 0x64;
	data[1] = 0x03;
	data[2] = 0X12;
	data[3] = 0X81;
	transmitCmdData(0xED, data, 4);

	data[0] = 0x20;
	transmitCmdData(0xF7, data, 1);

	data[0] = 0x23;					//VRH[5:0]
	transmitCmdData(0xC0, data, 1); //Power control

	data[0] = 0x10;					//SAP[2:0];BT[3:0]
	transmitCmdData(0xC1, data, 1); //Power control

	data[0] = 0x3e; //Contrast
	data[1] = 0x28;
	transmitCmdData(0xC5, data, 2); //VCM control

	data[0] = 0x86;					//--
	transmitCmdData(0xC7, data, 1); //VCM control2

	data[0] = 0x48; //C8

	//This command works with ili9341
	//transmitCmdData(0x36, data, 1);    	// Memory Access Control

	//This commands works with ili9341-9340-9340c
	transmitCmdData(0x40, data, 1);
	transmitCmdData(0x08, data, 1);

	data[0] = 0x55;
	transmitCmdData(0x3A, data, 1);

	data[0] = 0x00;
	data[1] = 0x18;
	transmitCmdData(0xB1, data, 2);

	data[0] = 0x08;
	data[1] = 0x82;
	data[2] = 0x27;
	transmitCmdData(0xB6, data, 3); // Display Function Control

	data[0] = 0x00;
	transmitCmdData(0xF2, data, 1); // 3Gamma Function Disable

	data[0] = 0x01;
	transmitCmdData(0x26, data, 1); //Gamma curve selected

	data[0] = 0x0F;
	data[1] = 0x31;
	data[2] = 0x2B;
	data[3] = 0x0C;
	data[4] = 0x0E;
	data[5] = 0x08;
	data[6] = 0x4E;
	data[7] = 0xF1;
	data[8] = 0x37;
	data[9] = 0x07;
	data[10] = 0x10;
	data[11] = 0x03;
	data[12] = 0x0E;
	data[13] = 0x09;
	data[14] = 0x00;
	transmitCmdData(0xE0, data, 15); //Set Gamma

	data[0] = 0x00;
	data[1] = 0x0E;
	data[2] = 0x14;
	data[3] = 0x03;
	data[4] = 0x11;
	data[5] = 0x07;
	data[6] = 0x31;
	data[7] = 0xC1;
	data[8] = 0x48;
	data[9] = 0x08;
	data[10] = 0x0F;
	data[11] = 0x0C;
	data[12] = 0x31;
	data[13] = 0x36;
	data[14] = 0x0F;
	transmitCmdData(0xE1, data, 15); //Set Gamma

	transmitCmd(0x11); //Exit Sleep
	delayMicroseconds(120000);

	transmitCmd(0x29); //Display on
	transmitCmd(0x2c);

	// timer.initializeMs<1000>(sendRequest).start();
}

} // namespace

void init()
{
	Serial.begin(SERIAL_BAUD_RATE);
	Serial.systemDebugOutput(true);

	debug_i("ILI9341 comms test using HardwareSPI library");

	spiInit();
}
