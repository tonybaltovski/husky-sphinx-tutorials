Installing a Jetson Nano
==========================

Step 1: Remove mini-ITX Computer
--------------------------------

(Skip this step if you don't have a computer)

If you have a mini-ITX computer installed it will need to be removed. Locate the power and communication cable connected to the computer and remove them. If you have any other sensors attached, remove those too.  Keep them connected to the Husky plaform.

.. image:: images/PCRemoval/PC_RM_1.JPG

Remove the four (4) hex screws from the computer case using a 2.5mm wrench.

.. image:: images/PCRemoval/PC_RM_2.JPG

.. image:: images/PCRemoval/PC_RM_3.JPG

Gently lift the computer out of the platform and clean the area.

.. image:: images/PCRemoval/PC_RM_4.JPG

Step 2: Install the Nano
------------------------
Custom mounting brackets are available `on Github <https://github.com/clearpathrobotics/jetson_setup/raw/melodic/models/JetsonNanoXavierHuskyMount.stl>`_

Print this mount off using a 3D printer.  A 0.2mm layer thickness should be sufficient.

.. image:: images/Nano/Hardware/1.JPG

Add 4 M3 stand-offs to the board mount points.  A 6mm height is recommended.

.. image:: images/Nano/Hardware/2.JPG

Use M3 screws to fasten the Nano to the mount

.. image:: images/Nano/Hardware/3.JPG

Attach the Xavier and mount to the two M3 holes opposite the platform serial connector

.. image:: images/Nano/Hardware/4.JPG

Re-attach the power and serial cables from the platrofm to the Jetson

Step 3: Installing the Software
--------------------------------

Download the latest version of the `Nano SD Image <https://developer.nvidia.com/jetson-nano-sd-card-image>`_
Download the latest version of `Balena Etcher <https://www.balena.io/etcher/>`_

.. image:: images/Nano/Software/1.png

Use Etcher to flash the image onto your SD card

Once it is installed, connect the nano to a keyboard, monitor, and power supply.  Ubuntu needs to be setup first.  Agree to the Terms


.. image:: images/Nano/Software/2.png

Select your language.

.. image:: images/Nano/Software/3.png

Select your keyboard layout.

.. image:: images/Nano/Software/4.png

Select your locaiton.

.. image:: images/Nano/Software/5.png

Pick a hostname, username, and password for the machine.

.. note::

    For compatibility with older versions of the Jetson Nano software, set the username and password to ``nvidia``.
    To standardize with other Clearpath Robotics products, set the username to ``administrator`` and the password to ``clearpath``.

.. image:: images/Nano/Software/6.png

Specify the size for the partition.  The default size should fill the whole SD card.  Make sure it matches the maximum possible size, unless you have other plans for that space.

.. image:: images/Nano/Software/7.png

It will install the remainder of the required default packages.

.. image:: images/Nano/Software/8.png

Once the OS has been written to the Nano, log into it and run the following commands to configure it for use with Husky:

.. code-block:: bash

    curl -s https://raw.githubusercontent.com/clearpathrobotics/jetson_setup/melodic/scripts/nano_setup.sh | bash -s --
    bash JACKAL_SETUP.sh

.. image:: images/Nano/Software/9.png

These commands will download and install ROS along with the necessary APT packages to get Husky up and running.  Depending on your network speed it may take a long time for everything to install.  Reboot the Nano after these commands are done to complete the configuration.

When the Jetson starts up again, it should be connected to the Husky. To see that the Husky is connected by opening a terminal and executing "rostopic echo /status". You should see a 1hz message containing the Husky's diagnostic information.

Your Jetson Nano should now be configured to operate as the Husky's main PC.

If you would like to pair a PS4 controller to drive the Husky, hold down the PS and Share buttons on the controller until the light bar starts to flash. In a terminal on the Husky, run ``bluetoothctl`` and then run the following commands:

.. code-block:: text

    agent on
    scan on
    < look for the MAC address of your controller; it will be identified by "Wireless Controller" or similar text >
    scan off
    pair <MAC ADDRESS>
    trust <MAC ADDRESS>
    connect <MAC ADDRESS>
    < ctrl + d to exit >

The light on the controller will turn solid blue once it is paired. With the controller paired you should be able to control the Husky by pressing L1 and using the left stick to drive. For more information see the Husky manual.

The light on the controller will turn solid blue once it is paired. With the controller paired you should be able to control the Husky by pressing L1 and using the left stick to drive. For more information see the Husky manual.

To use your host computer with the Husky first `install ROS <http://wiki.ros.org/melodic/Installation>`_.  Once ROS is installed, install the Husky packages with ``sudo apt install ros-melodic-Husky*``

Note the IP address of the Nano and `setup your host computer <http://wiki.ros.org/ROS/Tutorials/MultipleMachines>`_ to use it as the master.

You can then run ``roslaunch Husky_viz view_robot.launch`` on your host machine.  You should see a model of the robot and be able to move the Husky using the interactive markers. See: :doc:`Navigating with Husky <DrivingHusky>` for more information on using maps for navigation and localization.

The Nano will reboot and will have ROS Melodic installed along with the Husky drivers.

To setup the Jetson to work with the Husky, run ``bash ~/JACKAL_SETUP.sh`` on the Jetson and restart. When the Jetson starts up again, it should be connected to the Husky. To see that the Husky is connected by opening a terminal and executing "rostopic echo /status". You should see a 1hz message containing the Husky's diagnostic information.

If you would like to pair a PS4 controller to drive the Husky, hold down the PS and Share buttons on the controller until the light bar starts to flash. In a terminal on the Husky, run ``sudo ds4drv-pair`` and wait for the controller to connect.  With the controller paired you should be able to control the Husky by pressing L1 and using the left stick to drive. For more information see the Husky manual.

To use your host computer with the Husky first install ROS (http://wiki.ros.org/melodic/Installation) and setup a catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace). Clone the general Husky repo and the desktop specific repo in to the src folder and compile it. Installing rosdeps if necessary with "rosdep install --from-paths src --ignore-src -r -y". https://github.com/Husky/Husky and https://github.com/Husky/Husky_desktop. Note the network ip of the Nano and setup your host computer to use it as the master. http://wiki.ros.org/ROS/Tutorials/MultipleMachines

You can then run "roslaunch Husky_viz view_robot.launch" on your host machine.  You should see a model of the robot and be able to move the Husky using the interactive markers. See: http://www.clearpathrobotics.com/assets/guides/Husky/navigation.html
