#include <SmingTest.h>

#define TEST_GROUP_INTERVAL 500
#define RESTART_DELAY 10000

extern void REGISTER_TEST(HSPI);

namespace
{
void testsComplete()
{
#ifdef ARCH_HOST
	System.restart();
#else
	SmingTest::runner.execute(testsComplete, RESTART_DELAY);
#endif
}

} // namespace

void init()
{
	Serial.setTxBufferSize(1024);
	Serial.begin(SERIAL_BAUD_RATE);
	Serial.systemDebugOutput(true);

	debug_e("HardwareSPI test application");

	REGISTER_TEST(HSPI);
	SmingTest::runner.setGroupIntervalMs(TEST_GROUP_INTERVAL);
	System.onReady([]() { SmingTest::runner.execute(testsComplete); });
}
