Driving Husky
==============

Updating the Virtual Machine
------------------------------
Open a terminal window (Ctrl + Alt + T), and enter the following:

.. parsed-literal::
	sudo apt-get update
	sudo apt-get install ros-noetic-husky-desktop
	sudo apt-get install ros-noetic-husky-simulator


Running a Virtual Husky
------------------------

Open a terminal window, and enter:

.. parsed-literal::
	roslaunch husky_gazebo empty_world.launch

Open another terminal window, and enter:

.. parsed-literal::
	roslaunch husky_viz view_robot.launch

You should be given two windows, both showing a yellow, rugged robot (Husky!)

**RViz**

.. image:: images/Huskyviz.png
	:scale: 50%

**Gazebo**

.. image:: images/Huskysim.png
	:scale: 50%

The left one shown is Gazebo. This is where we get a realistic simulation of our robot, including wheel slippage, skidding, and inertia.
We can add objects to this simulation, such as the cube above, or even entire maps of real places.

The right window is RViz. This tool allows us to see sensor data from a robot, and give it commands (in a future post).

We can now command the robot to go forwards. Open a terminal window, and use the command below, copy pasting this one won't work! You can tab complete this command by hitting the tab key after geometry_msgs/Twist:

.. parsed-literal::
	rostopic pub /husky_velocity_controller/cmd_vel geometry_msgs/Twist "linear:
		x: 0.5
		y: 0.0
		z: 0.0
	angular:
		x: 0.0
		y: 0.0
		z: 0.0" -r 10

In the above command, we publish to the **/husky_velocity_controller/cmd_vel topic**, of topic type **geometry_msgs/Twist**.
The data we publish tells the simulated Husky to go forwards at 0.5m/s, without any rotation. You should see your Husky move forwards.
In the gazebo window, you might notice simulated wheel slip, and skidding.

You can also use a game controller to drive your robot in Gazebo.  To set up your computer for teleop using the game controller
follow these steps:

1.  Connect the controller to your PC.

2.  Set the ``HUSKY_JOY_DEVICE`` environment variable to point to your game controller device.  Normally this will be ``/dev/input/js0``.

3.  Launch gazebo as described above.

To drive the robot, Axis 0 controls the robot's steering, Axis 1 controls the forward/backward velocity,
and buttons 4 and 5 act as enable & enable-turbo respectively.  On common controllers these correspond to the following
physical controls:

============= ==================================== ===== ===== =========
Axis/Button   Physical Input                       PS4   F710  Xbox One
============= ==================================== ===== ===== =========
Axis 0        Left thumb stick horizontal          LJ    LJ    LJ
Axis 1        Left thumb stick vertical            LJ    LJ    LJ
Button 4      Left shoulder button or trigger      L1    LB    LB
Button 5      Right shoulder button or trigger     R1    RB    RB
============= ==================================== ===== ===== =========

Using rqt_graph
----------------
We can also see the structure of how topics are passed around the system. Leave the publishing window running, and open a terminal window. Type in:

.. parsed-literal::
	rosrun rqt_graph rqt_graph

This command generates a representation of how the nodes and topics running on the current ROS Master are related. You should get something similar to the following:

.. image:: images/rqtgraph.png

The highlighted node and arrow show the topic that you are publishing to the simulated Husky. This Husky then goes on to update the gazebo virtual environment,
which takes care of movement of the joints (wheels) and the physics of the robot.
The rqt_graph command is very handy to use, when you are unsure who is publishing to what in ROS.
Once you figure out what topic you are interested in, you can see the content of the topic using **rostopic echo**.

Using tf
-----------

In ROS, tf is a special topic that keeps track of coordinate frames, and how they relate to each other.
So, our simulated Husky starts at (0,0,0) in the world coordinate frame. When the Husky moves, it’s own coordinate frame changes.
Each wheel has a coordinate frame that tracks how it is rotating, and where it is. Generally, anything on the robot that is not fixed in space, will have a tf describing it.
In the **rqt_graph section**, you can see that the **/tf topic** is published to and subscribed from by many different nodes.

One intuitive way to see how the tf topic is structured for a robot is to use the **view_frames** tool provided by ROS. Open a terminal window. Type in:

.. parsed-literal::
	rosrun tf view_frames

Wait for this to complete, and then type in:

.. parsed-literal::
	evince frames.pdf

This will bring up something similar to the following image.

.. image:: images/tfframes.png

Here we can see that all four wheel are referenced to the **base_link**. We also see that the **odom topic** is driving the reference of the whole robot.
This means that if you write to the **odom topic** (IE, when you publish to the **/cmd_vel topic**) then the whole robot will move.


Operating a Physical Husky
===================================

Husky can either drive autonomously, be :doc:`controlled through ROS messages <SimulatingHusky>`, or you
can use the supplied remote control to teleoperate it.

Safety Precautions
----------------------

.. warning::

    Husky is a heavy, robot capable of reaching high speeds.  Careless driving can cause harm to the operator,
    bystanders, the robot, or other property.  Always remain vigilant, ensure you have a clear line of sight to the
    robot, and operate the robot at safe speeds.


Teleoperation
-------------------

.. note::

	For instructions on controller pairing, see :doc:`Controller Pairing <InstallHuskySoftware>`.

To enable the controller you must hold down either the left or right shoulder buttons on the controller (L1 and R1 on
the PS4 controller, or LB and RB on the Logitech F710).  The left button is for normal operation, while the right
button enables turbo speed.

.. warning::

	When familiarizing yourself with Husky's operation, always hold the left button (L1/LB).  Once you are comfortable
	with how Husky operates, and you are in a large area with plenty of open room, then you can use R1/RB to enable
	turbo mode.

Once the controller is paired, you can use the left thumb-stick to drive the robot.  The vertical axis controls
the robot's speed and the horizontal axis controls the robot's turning.
