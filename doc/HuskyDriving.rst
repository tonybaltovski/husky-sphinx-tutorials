Driving Husky
==============

There are two ways to drive the Husky: teleoperation using a joystick controller, or by manually publishing ROS2 messages. Both methods will work on a physical Husky robot as well as on a simulated Husky.

There is also a third way of driving the Husky: through autonomous navigation. This will be covered in this tutorial in the future!

Safety Precautions
-------------------

.. warning::

	Husky is a heavy, robot capable of reaching high speeds. Careless driving can cause harm to the operator, bystanders, the robot, or other property. Always remain vigilant, ensure you have a clear line of sight to the robot, and operate the robot at safe speeds. We strongly recommend driving in normal (slow) mode first, and only enabling turbo in large, open areas that are free of people and obstacles.

Teleoperation
--------------

.. note::

	For instructions on controller pairing, see :doc:`Joystick Controller Pairing <HuskyControllerPairing>`.

Before driving Husky via teleoperation, you first need to launch the Husky ROS2 teleoperation launch file in the ``husky_control`` package. In terminal, run:

.. code-block:: bash

  ros2 launch husky_control teleop_launch.py

Once launched, you can now drive Husky with the paired joystick controller.

To drive the Husky, Axis 0 controls the robot's steering, Axis 1 controls the forward/backward velocity, and Buttons 4 and 5 act as enable & enable-turbo respectively. On common controllers, these correspond to the following physical controls:

============= ==================================== ===== ===== =========
Axis/Button   Physical Input                       PS4   F710  Xbox One
============= ==================================== ===== ===== =========
Axis 0        Left thumb stick horizontal          LJ    LJ    LJ
Axis 1        Left thumb stick vertical            LJ    LJ    LJ
Button 4      Left shoulder button or trigger      L1    LB    LB
Button 5      Right shoulder button or trigger     R1    RB    RB
============= ==================================== ===== ===== =========

You must hold either Button 4 or Button 5 at all times while driving the robot.

Publishing ROS2 Messages
-------------------------

By default, Husky's velocity controller exposes the ROS2 topic ``/husky_velocity_controller/cmd_vel_unstamped``. You can manually publish ``geometry_msgs/msg/Twist`` messages to this topic to drive the Husky. 

For example, in terminal, run:

.. code-block:: bash

	ros2 topic pub --once /husky_velocity_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

The command above makes Husky drive forward at 0.5 m/s without any rotation. 
