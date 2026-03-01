.. zephyr:board:: usbtingo

Overview
********

The USBtingo is an USB to CAN FD adapter board made and direct distributed
by Thomas Fischl.

Hardware
********

The USBtingo board is equipped with a LPC5514 (non-secure) microcontroller
and features an USB connector, a PCB footprint for an 100mil screw terminal
block or direct cable soldering for the CAN bus, one jumper for CAN termination
resistor, one jumper for ISP boot selection, and one red user LED.

Schematic, component placement drawing and datasheet are available on the
`USBtingo homepage`_.

Supported Features
==================

.. zephyr:board-supported-hw::

System Clock
============

The LPC5514 PLL is driven by the external crystal running at 16 MHz and
configured to provide a system clock of 144 MHz.

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Build and flash applications as usual (see :ref:`build_an_application` and
:ref:`application_run` for more details).

If flashing via USB MCU Bootloader, short jumper ``JP1`` when applying power
to the USBtingo in order to enter the built-in Boot ROM (ISP mode).

Here is an example for the :zephyr:code-sample:`blinky` application.

.. zephyr-app-commands::
   :zephyr-app: samples/basic/blinky
   :board: usbtingo
   :goals: flash

.. _USBtingo homepage:
   https://www.ﬁschl.de/usbtingo/
