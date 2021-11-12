HardwareSPI
===========

Asynchronous SPI device stack for Sming.

Problem statement
-----------------

The ESP8266 has limited I/O and the most useful way to expand it is using serial devices. There are several types available:

I2C
   Can be fast-ish but more limited than SPI, however slave devices generally cheaper, more widely available and simpler to interface as they only require 2 pins.
   The ESP8266 does have hardware support however, so requires a bit-banging solution. Inefficient.

I2S
   Designed for streaming audio devices but has other uses. See :component-esp8266:`driver`.

RS232
   Tied in with RS485, Modbus, DMX512, etc. Well-supported with asynchronous driver.

SPI
   Speed generally limited by slave device capability, can be multiplexed ('overlapped') onto SPI0 (flash memory) pins.
   The two SPI modules are identical, but the additional pins for quad/dual modes are only brought out for SPI0.
   In addition, three user chip selects are available in this mode; there is only one in normal mode although this
   can be handled using a regular GPIO.


The purpose of this driver is to provide a consistent interface which makes better use of the hardware capabilities.

-  Classes may inherit from :cpp:class:`HSPI::Device` to provide support for specific devices such as
   Flash memory, SPI RAM, LCD controllers, etc.
-  Device objects are attached to the stack via specified PinSet (overlapped or normal) and chip select.
-  Multiple concurrent devices are supported, limited only by available chip selects and physical constraints such
   as wire length, bus speeds.
-  2 and 4-bit modes are supported via overlap pins.
-  A :cpp:class:`HSPI::Request` object supports transfers of up to 64K. The controller splits these into smaller transactions as required.
-  Asynchronous execution so application does not block during SPI transfer.
   Application may provide a per-request callback to be notified when request has completed.
-  Blocking requests are also supported.
-  Support for moving data between Sming :doc:`/framework/core/data/streams/index` and SPI memory devices
   using the :cpp:class:`HSPI::StreamAdapter`.


SPI system expansion
~~~~~~~~~~~~~~~~~~~~

A primary use-case for this driver is to provide additional resources for the ESP8266 using SPI devices such as:

-  MC23S017 SPI bus expander. Operates at 10MHz (regular SPI) and provides 16 additional I/O with interrupt capability.
-  High-speed shift registers. These can be wired directly to SPI busses to expand GPIO capability.
-  Epson S1D13781 display controller.  See :library:`TFT_S1D13781`.
   Evaluation boards are inexpensive and is a useful way to evaluate display modules with TFT interfaces.
   The `Newhaven NHD-5.0-800480TF-ATXL#-CTP <https://www.newhavendisplay.com/nhd50800480tfatxlctp-p-6062.html>`__
   was used during development of this driver.
-  Bridgetek FT813 EVE TFT display controller. This supports dual/quad modes and clocks up to 30MHz.
-  NRF24L01 RF transceiver. Rated bus speed is 8MHz but it seems to work fine at 40MHz.
-  Serial memory devices. The library contains a driver for the IS62/65WVS2568GALL fast serial RAM, which clocks up to 45MHz and
   supports SDI/SQI modes.


Software operation
------------------

Overview
~~~~~~~~

We have:

-  Controller: SPI hardware
-  Device: Slave device on the SPI bus
-  Request: Details of a transaction for a specific device

The Controller maintains an active list of requests (as a linked-list), but does not own the request objects. These will be allocated by the application, configured then submitted to the Controller for execution.

The Device specifies how a slave is connected to the bus, and that may change dynamically. For example, at reset an SPI RAM may be in SPI mode but can be switched into SDI/SQI modes.
This is device-specific so would be implemented by superclassing :cpp:class:`HSPI::Device`.


Requests
--------

Each :cpp:class:`HSPI::Request` is split into transactions.
A transaction has four phases: command - address - MOSI - dummy - MISO.
All phases are optional.
The dummy bits are typically used in read modes and specified by the device datasheet.
No data is transferred during this phase.

The ESP8266 hardware FIFO is used for MOSI/MISO phases and is limited to 64 bytes,
so larger transfers must be broken into chunks. The driver handles this automatically.

Requests may be executed asynchronously so the call will not block and the CPU can continue
with normal operations. An optional callback is invoked when the request has completed.
As an example, consider moving a 128KByte file from flash storage into FT813 display memory:

1. Read the first file chunk into a RAM buffer, submit an SPI request1 to transfer it asynchronously
2. Read the second file chunk into another RAM buffer, and prepare request2 for that (but do not submit it yet)
3. When request1 has completed, submit request2 (from the interrupt callback).
   Schedule a task to read the next chunk and prepare request1.
4. When request2 has completed, continue from step (2) to submit request1, etc.

Timing
------

A 64-byte data transfer (full hardware FIFO with 1 command byte and 3-byte address) at 26MHz would take 21us (5.25us in QIO mode)
or 1680 (420) CPU cycles. To transfer 128Kbytes would take 2048 such transactions, 43ms (11ms for QIO), not including memory copy overheads.

In practice request sizes will be much smaller due to RAM constraints.
Nevertheless, at high clock speeds the interrupt rate increases to the point where it consumes more CPU cycles than the
actual transfer. The driver therefore disables interrupts in these situations and executes the request in task mode.

Bear in mind that issuing a blocking request will also require all queued requests to complete.

The driver does not currently support out-of-order execution, which might prioritise faster devices.


Pin Set
-------

To avoid confusion, we'll refer to the flash memory SPI bus as SPI0, and the user bus as SPI1.
This driver doesn't support direct use of SPI0 as on most devices it is reserved for flash
memory. However, an overlap mode is supported which makes use of hardware arbitration to
perform SPI1 transactions using SPI0 pins. This has several advantages:

-  Liberates three GPIO which would normally be required for MOSI, MISO and SCLK.
-  Only one additional pin is required for chip select.
-  Additional 2/4 bits-per-clock modes are available for supported devices.

For the ESP8266, these are the :cpp:enum:`HSPI::PinSet` assignments:

PinSet::normal
   MISO=GPIO12, MOSI=GPIO13, SCLK=GPIO14. One chip select:
      0. GPIO15 (HSPI CS)

PinSet::overlap
   MISO=SD0, MOSI=SD1, IO2=SD3, IO3=SD2, SCLK = CLK. Three chip selects:
      0. GPIO15 (HSPI_CS)
      1. GPIO1 (SPI_CS1 / UART0_TXD).
         This conflicts with the normal serial TX pin which should be swapped to GPIO2 if required.
      2. GPIO0 (SPI_CS2)

PinSet::manual
   Typically a GPIO will be assigned to perform chip select (CS).
   The application should register a callback function via :cpp:func:`HSPI::onSelectDevice`
   which performs the actual switching. This **MUST** be in IRAM.

.. note::

   The connections for IO2/3 look wrong above, but on two different models of SPI RAM chip these
   have been verified as correct by writing in SPIHD mode and reading in quad mode.

Multiplexed CS
--------------

Multiple devices can be supported on a single CS using, for example using a HC138 3:8 decoder.
The CS line is connected to an enable input, with three GPIO outputs setting A0-2.

A custom controller should be created like this::

   class CustomController: public HSPI::Controller
   {
   public:
      bool startDevice(Device& dev, PinSet pinSet, uint8_t chipSelect) override
      {
         /*
          * You should perform any custom validation here and return false on failure.
          * For example, if we're only using 3 of the 8 available outputs.
          */
         auto addr = chipSelect & 0x07;
         if(addr > 3) {
            debug_e("Invalid CS addr: %u", addr);
            return false;
         }
         
         /*
          * Provide a callback to route chip select signal as required.
          * Note this only needs to be done once, so could be called externally without
          * subclassing HSPI::Controller if the other mechanisms aren't required.
          */
         onSelectDevice(selectDevice);

         /*
          * Initialise hardware Controller
          */
         auto cs = chipSelect >> 3;
         return HSPI::Controller::startDevice(dev, pinSet, cs);
      }

   private:
      static void IRAM_ATTR selectDevice(uint8_t chipSelect, bool active);

      uint8_t activeChipSelect{0};
   };

Now in the .cpp file::

   CustomController spi;

   void IRAM_ATTR CustomController::selectDevice(uint8_t chipSelect, bool active)
   {
      // Only perform GPIO if CS changes as GPIO is expensive
      if(active && chipSelect != activeChipSelect) {
         auto addr = chipSelect & 0x07;
         digitalWrite(PIN_MUXADDR0, addr & 0x01);
         digitalWrite(PIN_MUXADDR1, addr & 0x02);
         // As we only need 2 address lines, can leave this one
         // digitalWrite(PIN_MUXADDR2, addr & 0x03);

         activeChipSelect = chipSelect;
      }

      // If using a hardware CS output then we're done.

      if(spi.getActivePinSet() == HSPI::PinSet::manual) {
         // For manual chip select operation, set the state directly here
      }
   }


IO Modes
--------

Not to be confused with :cpp:enum:`HSPI::ClockMode`, the :cpp:enum:`HSPI::IoMode` determines how
the command, address and data phases are transferred:

   =======     =======     =======     ====     ======
   .                   Bits per clock           .
   -------     ----------------------------     ------
   IO Mode     Command     Address     Data     Duplex
   =======     =======     =======     ====     ======
   SPI         1           1           1        Full
   SPIHD       1           1           1        Half
   SPI3WIRE    1           1           1        Half
   DUAL        1           1           2        Half
   DIO         1           2           2        Half
   SDI         2           2           2        Half
   QUAD        1           1           4        Half
   QIO         1           4           4        Half
   SQI         4           4           4        Half
   =======     =======     =======     ====     ======

.. note::

   SDI and SQI are not supported directly by hardware, but is implemented within the driver using the address phase.
   In these modes, commands are limited to 8 bits.

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


Streaming
---------

The :cpp:class:`HSPI::StreamAdapter` provides support for streaming of data to/from memory devices.

This would be used, for example, to transfer content to or from a :cpp:class:`FileStream`
or :cpp:class:`FlashMemoryStream` to SPI RAM asynchronously.

Supported devices must inherit from :cpp:class:`HSPI::MemoryDevice`.



API
---

.. doxygenenum:: HSPI::ClockMode
.. doxygenenum:: HSPI::IoMode
.. doxygenenum:: HSPI::PinSet

.. doxygenstruct:: HSPI::Request
   :members:

.. doxygenstruct:: HSPI::Data
   :members:

.. doxygenclass:: HSPI::Device
   :members:

.. doxygenclass:: HSPI::MemoryDevice
   :members:

.. doxygenclass:: HSPI::RAM::PSRAM64
.. doxygenclass:: HSPI::RAM::IS62_65

.. doxygenclass:: HSPI::Controller
   :members:

.. doxygenclass:: HSPI::StreamAdapter
   :members:

