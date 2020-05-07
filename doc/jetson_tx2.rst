Installing a Jetson TX2 on a Husky
=====================================

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

Step 2: Install the TX2
------------------------
Custom mounting brackets are available `on Github <https://github.com/clearpathrobotics/jetson_setup/raw/melodic/models/JetsonTX2HuskyMount.stl>`_

Download and 3D print the mounts.  A 0.2mm layer thickness should be sufficient.

Using M3 bolts, attach the plates inside the Husky platform.  On the plates, add 4 M3 stand-offs to the board mount points.  A 6mm height is recommended.

.. image:: images/JetsonTX2/img1.JPG

Once the mounts are installed, attach the TX2 onto the mounts using M3 bolts.

Take the power cable that powered the ITX computer and plug it into the TX2.  Then, attach the serial communication cable to the USB port on the TX2.  If you have multiple USB peripherals on the robot, you may need to use a USB hub.

.. image:: images/JetsonTX2/img2.JPG

Step 3: Installing the software
--------------------------------

`Download the latest version of Nvidia's SDK Manager <https://developer.nvidia.com/nvidia-sdk-manager>`_ on a PC running Ubuntu 18.04.  While that's downloading, put the TX2 into reovery mode by following these steps:

1.  Connect the TX2 to your PC using the provided microUSB cable.
2.  Make sure the TX2 is powered off
3.  Connect a monitor, mouse, and keyboard to the Jetson.  (The mouse is optional, but recommended.  If you do not have an all-in-one mouse+keyboard you will need to use a small USB hub, as the Jetson TX2 only has a single USB port.)
4.  Press and hold the REC button
5.  Press the power button.

Install the SDK Manager by running the following commands:

.. code-block:: bash

    cd <folder where you downloaded SDK manager>
    sudo dpkg -i sdkmanager_<version>_amd64.deb

.. note::

    If your system is missing dependencies you may see error messages in the output of the ``dpkg`` command.  To resolve these, run ``sudo apt-get -f install``.

Once SDK Manager is installed, run it with the ``sdkmanager`` command.  `Follow SDK Manager's instructions <https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html>`_ to install the operating system and Nvidia components on your Jetson TX2.  During installation you will be asked to configure the login information on the Jetson.  To do this, use the monitor & keyboard you connected before to enter the username and password you want to use.

.. note::

    For compatibility with older versions of the Jetson TX2 software, set the username and password to ``nvidia``.
    To standardize with other Clearpath Robotics products, set the username to ``administrator`` and the password to ``clearpath``.

Once the OS has been written to the TX2, log into it and run the following commands to configure it for use with Husky:

.. code-block:: bash

    curl -s https://raw.githubusercontent.com/clearpathrobotics/jetson_setup/melodic/scripts/tx2_setup.sh | bash -s --
    bash HUSKY_SETUP.sh

These commands will download and install ROS along with the necessary APT packages to get Husky up and running.  Depending on your network speed it may take a long time for everything to install.  Reboot the TX2 after these commands are done to complete the configuration.

When the Jetson starts up again, it should be connected to the Husky. To see that the Husky is connected by opening a terminal and executing "rostopic echo /status". You should see a 1hz message containing the Husky's diagnostic information.

Your Jetson TX2 should now be configured to operate as the Husky's main PC.

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

To use your host computer with the Husky first `install ROS <http://wiki.ros.org/melodic/Installation>`_.  Once ROS is installed, install the Husky packages with ``sudo apt install ros-melodic-husky*``

Note the IP address of the TX2 and `setup your host computer <http://wiki.ros.org/ROS/Tutorials/MultipleMachines>`_ to use it as the master.

You can then run ``roslaunch husky_viz view_robot.launch`` on your host machine.  You should see a model of the robot and be able to move the Husky using the interactive markers. See: :doc:`Navigating with Husky <navigation>` for more information on using maps for navigation and localization.
