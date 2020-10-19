#pragma once

// For testing ISR latency, etc.
#ifdef HSPI_ENABLE_TESTPINS

#include <Digital.h>

// For testing, we'll be toggling a pin
#define TESTPIN_SETUP()                                                                                                \
	{                                                                                                                  \
		pinMode(HSPI_TESTPIN1, OUTPUT);                                                                                 \
		pinMode(HSPI_TESTPIN2, OUTPUT);                                                                                 \
		TESTPIN1_LOW();                                                                                                \
		TESTPIN2_LOW();                                                                                                \
	}

// Set HIGH during request execution
#define TESTPIN1_HIGH() GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, BIT(HSPI_TESTPIN1))
#define TESTPIN1_LOW() GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, BIT(HSPI_TESTPIN1))

// Set HIGH during transaction execution
#define TESTPIN2_HIGH() GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, BIT(HSPI_TESTPIN2))
#define TESTPIN2_LOW() GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, BIT(HSPI_TESTPIN2))

#else

#define TESTPIN_SETUP()
#define TESTPIN1_HIGH()
#define TESTPIN1_LOW()
#define TESTPIN2_HIGH()
#define TESTPIN2_LOW()

#endif
