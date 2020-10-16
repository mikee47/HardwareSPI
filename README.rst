HardwareSPI
===========

Asynchronous SPI device stack for Sming.

Problem statement
-----------------

The ESP8266 has limited I/O and the most useful way to expand it is using serial devices. There are several types available:

   I2C: Can be fast-ish but more limited than SPI, however slave devices generally cheaper, more widely available and simpler to interface as they only require 2 pins. There does not appear to be actual hardware support though, so very inefficient.
   I2S: Designed for audio devices. Apparently the FIFO size is 1Kbyte.
   RS232: Tied in with RS485, Modbus, DMX512, etc. Well-supported with asynchronous driver.
   SPI: Speed generally limited by slave device capability, can be multiplexed ('overlapped') onto SPI0 (flash memory) pins. The two SPI modules are identical, but the additional pins for quad/dual modes are only brought out for SPI0. Three user chip selects are available in this mode.

The purpose of this driver is to provide a consistent interface which makes better use of the hardware capabilities.

-  Applications use drivers with a high-level interface to support specific devices (e.g. Flash memory, SPI RAM, LCD controllers, etc.)
-  Device objects are attached to the stack via specified PinSet (overlapped or normal) and chip select
-  Dual and Quad modes are supported via overlap pins. There are three variants of each depending on whether the command and address phases are required to be 1-bit transfers.
-  A request object supports transfers of up to 64K. The controller splits these into smaller transactions as dictated by hardware.
-  Asynchronous execution supported so application does not block during SPI transfer. Application may register callback to be notified when request has completed.
-  For high-speed devices, blocking calls may be more appropriate and run with SPI interrupts disabled to minimise overhead


SPI IO Modes:

IO Mode  |  Command  |  Address   |  Data   | Notes
-------  |  -------  |  -------   |  -----  |
SPI      |     1     |     1      |    1    | Full-duplex
SPIHD    |     1     |     1      |    1    | Half-duplex
DUAL     |     1     |     1      |    2    | Half-duplex
DIO      |     1     |     2      |    2    | Half-duplex
SDI      |     2     |     2      |    2    | Half-duplex *
QUAD     |     1     |     1      |    4    | Half-duplex
QIO      |     1     |     4      |    4    | Half-duplex
SQI      |     4     |     4      |    4    | Half-duplex *

Note that SDI and SQI are not supported directly by hardware, but is implemented within the driver using the data phase only.
For 8-bit command and 24-bit address, this limits each transaction to 60 bytes.


WS2811
~~~~~~
   
Both I2S and SPI can be used to drive 'Pixel' LED lighting. For example, using SPI to drive WS2811 requires a 10MHz bitstream (from MOSI) for 0.1us timing accuracy. Times are:

   WS2811 (10MHz)       WS2812 (20MHz)
   | time | us | cycles  
   T0H 0.5  5           0.4  8      3 (2x + 2)
   T0L 2.0  20          0.85 17     8 (2x + 1)
   T1H   1.2   12          0.8  16     7 (2x + 2)
   T1L 1.3  13          0.45 9      5 (2x - 1)
   RES > 50 500
   
So bit time is 2.5us, 25 cycles. A command is 24 bits, total of 600 cycles or 75 bytes. Bugger. That's (slightly) too much... Looks like I2S is the better option then.


Primary use case for system expansion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  MC23S017 SPI bus expander. Operates at 10MHz (regular SPI) and provides 16 additional I/O with interrupt capability.
-  SPI display devices, a multitude are available. We'll focus on the Bridgetek FT813 EVE, which supports dual/quad modes and clocks up to 30MHz. (On the ESP8266 we can use 26.7MHz.)
-  NRF24L01 RF transceiver
-  Serial memory devices (RAM, e.g. 23LC1024 / FRAM / EEPROM / FLASH)

Hardware
~~~~~~~~

Overlapped onto SPI0 (flash) pins using CS2 for HSPI devices. This is multiplxed using a HC138 3:8 decoder onto the actual devices. The slave device is therefore pre-selected before the HSPI command is executed. CS2 is under SPI hardware control so when idle all HC138 outputs are disabled.

Software operation
------------------

Overview
~~~~~~~~

Hardware FIFO is limited to 64 bytes so larger transfers must be broken into chunks, for example when performing a bulk transfer into SPI display memory. This could be direct from Flash storage, via SPIFFS or RAM buffer if streaming via WiFi. Therefore a callback (from interrupt context) can deal with queueing the next transfer.

As an example, consider moving a 128KByte file from flash storage into display memory. A 64-byte data transfer (with 1 command byte and 3-byte address) at 26MHz would take 21us (5.25us in quad mode) or 1680 (420) CPU cycles. A file of, say, 128Kbytes would take 2048 such transactions, 43ms (11ms), not including memory copy overheads. Performing these asynchronously with an interrupt callback would actually be quite inefficient because of the overhead of context switching (pushing/popping registers).

The I/O expander 'only' runs at 10MHz, but those transactions are much smaller.

There is some documentation on using DMA with I2S for data transfer, so the question of whether we can use it for SPI remains.

The model is very similar to the IOControl system, but that is intended for user-level requests. SPI is a high-performance system bus so will have a considerably higher throughput. Things like dynamic memory allocation, for example, may not be appropriate, and JSON is unlikely to have any part to play here as we are dealing with internal hardware which is generally fixed.

Objects
~~~~~~~

We have:

-  Controller: SPI hardware
-  Device: Slave device on the SPI bus
-  Packet: Details of a transaction for a specific device

The Controller maintains an active list of requests (as a linked-list), but does not own the request objects. These will be allocated by the application, configured then submitted to the Controller for execution.

The Device specifies how a slave is connected to the bus, and that may change dynamically. For example, at reset an SPI RAM may be in SPI mode but can be switched into SDI/SQI modes.
This is device-specific so would be implemented by superclassing HSPI::Device.


Command sequences
-----------------

There are four phases and the sequence is always command - address - MOSI - delay - MISO, though all phases are optional.
Delay bits are specified in clocks so dependent on data mode. Perhaps API should express this in data bits?

Possible sub-modes are:

SPI:
command 1-bit
address 1-bit
data 1-bit

SDI/SQI:
command 1-bit
address 1-bit
data 2/4-bit

command 1-bit
address 2/4-bit
data 2/4-bit

command 2/4-bit
address 2/4-bit
data 2/4-bit

This last one isn't directly supported by hardware as command is always 1-bit and there isn't any obvious register setting available.
This seems to be consistent with the ESP32 IDF driver, as in ``spi_ll.h``::

   /** IO modes supported by the master. */
   typedef enum {
       SPI_LL_IO_MODE_NORMAL = 0,  ///< 1-bit mode for all phases
       SPI_LL_IO_MODE_DIO,         ///< 2-bit mode for address and data phases, 1-bit mode for command phase
       SPI_LL_IO_MODE_DUAL,        ///< 2-bit mode for data phases only, 1-bit mode for command and address phases
       SPI_LL_IO_MODE_QIO,         ///< 4-bit mode for address and data phases, 1-bit mode for command phase
       SPI_LL_IO_MODE_QUAD,        ///< 4-bit mode for data phases only, 1-bit mode for command and address phases
   } spi_ll_io_mode_t;

Somne devices (e.g. W25Q32 flash) have specific commands to support these modes, but others (e.g. IS62/65WVS2568GALL fast serial RAM) do not,
and the SDI/SQI mode setting applies to all phases. This needs to be implemented in the driver as otherwise the user code is more complex than
necesssary and performance suffers considerably.

