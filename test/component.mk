COMPONENT_DEPENDS := \
	HardwareSPI \
	SmingTest

#
HOST_NETWORK_OPTIONS := --nonet
DISABLE_NETWORK := 1
DEBUG_VERBOSE_LEVEL := 2

.PHONY: execute
execute: flash run
