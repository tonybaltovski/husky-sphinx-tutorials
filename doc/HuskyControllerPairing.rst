Joystick Controller Pairing
============================

Joystick controllers are used for teleoperation; they allow you to remotely drive the Husky, whether it is a physical Husky robot, or a simulated Husky. The following instructions below detail how to pair different controllers to the Husky's computer; however, these instructions can also be used to pair these controllers to your own computer.

Logitech F710 Controller
---------------------------

.. Note::

  If your Husky comes with a Logitech F710 controller, it will be paired already. Simply turn on the Husky, plug the USB dongle into one of the Husky's USB ports, and turn on the controller.

To re-pair the Logitech F710 controller or pair a new Logitech F710 controller, plug the controller's USB dongle into the Husky's computer and turn on the controller. The controller automatically pair.

PS4 Controller
---------------

.. Note::

  If your Husky comes with a PS4 controller, it will be paired already. Simply turn on the Husky and turn on the controller.

To re-pair the PS4 controller or pair a new PS4 controller:

1. Install the ``python-ds4drv`` package if it is not installed already. In terminal, run:

.. code-block:: bash

  sudo apt-get install python-ds4drv

2. Put the controller in pairing mode. Press and hold the PS and Share buttons on your controller until the LED begins rapidly flashing white.

3. Run the ``ds4drv-pair`` script to pair the controller to the computer. In terminal, run:

.. code-block:: bash

  sudo ds4drv-pair

This script will scan for nearby Bluetooth devices, and pair automatically to the controller.

Alternatively, if ``ds4drv-pair`` fails to detect the controller, you can pair the controller using ``bluetoothctl``:

1. Install the ``bluez`` package if it is not installed already. In terminal, run:

.. code-block:: text

  sudo apt-get install bluez

2. Run the ``bluetoothctl`` command. In terminal, run:

.. code-block:: text

  sudo bluetoothctl

3. Use ``bluetoothctl`` to scan for nearby devices. In the bluetooth control application, run:

.. code-block:: text

  agent on
  scan on

4. Put the controller in pairing mode. Press and hold the PS and Share buttons on your controller until the LED begins rapidly flashing white.

5. The bluetooth scan will display the MAC addresses of nearby devices. Determine which MAC address corresponds to the
controller and copy it. In the bluetooth control application, run:

.. code-block:: text

  scan off
  pair <MAC Address>
  trust <MAC Address>
  connect <MAC Address>

The controller should now be paired.
