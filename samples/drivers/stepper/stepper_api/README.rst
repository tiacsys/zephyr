.. zephyr:code-sample:: stepper_api
   :name: Stepper API
   :relevant-api: stepper_interface

   Move a stepper motor using the api, allowing for different stepper drivers

Overview
********

The Stepper API sample attempts to move a stepper motor using three different api calls using the :ref:`Stepper API <stepper_api>`.

The source code shows how to:

#. Enable and configure a stepper motor using the api
#. Use ``enable_constant_velocity_mode`` to move the motor
#. Use ``move`` to move the motor
#. Use ``set_target_position`` to move the mootor


.. _stepper_api-sample-requirements:

Requirements
************

You must:

#. Have a board with at least 4 free gpio ports to use the gpio stepper driver.
#. OR Have a stepper driver that uses the stepper api connected to your board.
#. A devicetree binding that gives a stepper driver the ``stepper0`` alias (see the rpi_pico files for
   an example).

Building and Running
********************

Build and flash Stepper API as follows, changing ``rpi_pico`` for your board:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/stepper/stepper_api
   :board: rpi_pico
   :goals: build flash
   :compact:

After flashing, the motor moves up to 3 times, depending on the driver capabilities, and a
corresponding output is printed to the shell.

Build errors
************

You will see a build error at the source code line defining the ``struct
device *stepper`` variable if you try to build the sample for an currently unsupported board and
stepper driver combination. In this case, a new overlay file will need to be written.

On GCC-based toolchains, the error looks like this:

.. code-block:: none

   error: '__device_dts_ord_DT_N_ALIAS_stepper0_ORD' undeclared here (not in a function)

Adding board support
********************

To add support for your board, add something like this to your devicetree (the example is for the
gpio_stepper_controller:

.. code-block:: DTS

   / {
   	aliases {
   		stepper0 = &motor_1;
   	};
   };

  / {
	   stepper_motor: stepper_motor {
		   compatible = "zephyr,gpio-steppers";
		   status = "okay";
		   motor_1: motor_1 {
			   micro-step-res = <1>;
			   gpios = <&gpio0 12 GPIO_ACTIVE_HIGH>,
				   <&gpio0 13 GPIO_ACTIVE_HIGH>,
				   <&gpio0 14 GPIO_ACTIVE_HIGH>,
				   <&gpio0 15 GPIO_ACTIVE_HIGH>;
		   };
	   };
   };

The above sets the ``stepper0`` alias to the ``motor_1`` of the stepper controller and sets the motors
gpio pins to pins 12-15 on GPIO controller ``gpio0``.

Tips:

- See :zephyr_file:`dts/bindings/stepper` for more information on defining the used stepper driver in the
  devicetree.

- See the stepper drivers Kconfig file in :zephyr_file:`drivers/stepper` for the Kconfig options it requires.

