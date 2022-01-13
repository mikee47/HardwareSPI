SPI master-mode hardware controller
===================================

The ESP8266 SPI hardware is capable of very high transfer speeds and has a number of
features which require a more flexible driver to take advantage of.

Controller Class
================

Provides the following features:

- Support access to multiple slave devices sharing the same bus
  - Custom CS multiplexing supported via callbacks. For example, routing CS2 via HC138
    3:8 decoder allows 8 (or more) SPI devices to share the same bus.
  - Use of HSPI (SPI1) using either its own pins or sharing pins with SPI0 (overlapped)
  - (Potentially) enabling use of dual/quad operating modes when overlapped
  - Making use of hardware command/address/data phases for best efficiency
  - Pre-calculation of all register values to optimise switching between slave devices

- Write-only transactions can return immediately rather than waiting for the transfer to
  complete. The time spent waiting can be used to prepare the next transaction which can
  potentially double the throughput

- Interrupt callback on transaction completion. This can be used to improve system efficiency
  on slower devices.

Note that DMA is not available on the ESP8266 (only for I2S).

Transactions
------------

Applications call Controller to perform a transfer, or sequence of transfers, as follows:
- Session setup
- Wait for any HSPI transaction to complete (WAIT_READY)
  - Configure clock & mode settings
- Transaction
  - WAIT_READY
  - Configure command / address / data
  - Start operation
  - If read required:
    - WAIT_READY
    - Copy data from FIFO

Transaction may be repeated for subsequent transfers on same device.
CS will be asserted/de-asserted by hardware so not need to end a transaction.

Overlapped operation
--------------------

Both SPI controllers are able to share the pin signals from the flash SPI interface (SPI0).
This is handled through hardware.

Advantages:

- Gain three pins (GPIO12-14), which liberates the I2S controller
- Dual and quad SPI modes can be used with HSPI

Disadvantages:

- Slow SPI devices may reduce retrieval speed of program code from Flash memory

A primary IO MUX (PERIPHS_IO_MUX_CONF_U) selects whether the CPU clock goes through the
SPI clock divider circuitry or not. In overlapped mode the SPI0 setting is used for both,
therefore as most SPI slave devices will not operate at 80MHz this setting has to be disabled
to allow the clocks to be set independently. See PERIPHS_IO_MUX_CONF_U.

The ESP32 Technical Reference manual gives a very useful insight into how the two SPI
devices work together, as the hardware appears largely similar.
