COMPONENT_INCDIRS = src/include
COMPONENT_SRCDIRS = src src/Arch/$(SMING_ARCH)

COMPONENT_VARS += HSPI_ENABLE_STATS
HSPI_ENABLE_STATS ?= 0
ifeq ($(HSPI_ENABLE_STATS),1)
GLOBAL_CFLAGS += -DHSPI_ENABLE_STATS=1
endif

COMPONENT_VARS += HSPI_ENABLE_TESTPINS HSPI_TESTPIN1 HSPI_TESTPIN2
HSPI_ENABLE_TESTPINS ?= 0
HSPI_TESTPIN1 ?= 4 # Set HIGH during request execution
HSPI_TESTPIN2 ?= 5 # Set HIGH during transaction
ifeq ($(HSPI_ENABLE_TESTPINS),1)
COMPONENT_CXXFLAGS += \
	-DHSPI_ENABLE_TESTPINS=1 \
	-DHSPI_TESTPIN1=$(HSPI_TESTPIN1) \
	-DHSPI_TESTPIN2=$(HSPI_TESTPIN2)
endif
