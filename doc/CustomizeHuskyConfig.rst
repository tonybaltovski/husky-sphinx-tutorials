Customize Husky Configuration
======================================

.. Note:: These tutorials assume that you are familiar with ROS and the catkin build system. Please familiarize yourself using the `ROS <http://wiki.ros.org/ROS/Tutorials>`_ and `catkin <http://wiki.ros.org/catkin/Tutorials>`_ tutorials.

If upgrading from a prior ROS release, you should now re-examine your backed-up files from Backing Up Husky Configuration to determine if there's any customizations that need to be configured on your platform.

Environment Variables
--------------------------

Husky's standard peripherals can be configured using these environment variables, to be added to the robot-wide setup file (``/etc/ros/setup.bash``). These environment variables are loaded on boot.

.. raw:: html

	<table class="wy-table-responsive"><tbody><tr><td><p><strong>Variable</strong> </p></td>
 	 <td><p> <strong>Default</strong> </p></td>
 	 <td><p> <strong>Description</strong> </p></td>
	</tr>
	<tr>  <td><span  id="line-37"></span><p> <tt>ROBOT_NETWORK</tt> </p></td>
 	 <td><p> None </p></td>
	  <td><p>  Configure a network interface to trigger the husky-core</tt> job, and initialize ROS_IP</tt>. If not set, husky-core</tt> will define ROS_HOSTNAME</tt> instead (see <a href="/ROS/NetworkSetup#Name_resolution">Network Setup</a>) </p></td>
	</tr>
	<tr>  <td><span  id="line-38"></span><p> <tt>HUSKY_IMU_PORT</tt> </p></td>
 	 <td><p> <tt>/dev/clearpath/imu</tt> </p></td>
 	 <td><p>  Port for the Husky UM6 IMU if present, must be set before running husky_bringup&nbsp;install</tt> </p></td>
	</tr>
	<tr>  <td><span  id="line-39"></span><p> <tt>HUSKY_IMU_XYZ</tt> </p></td>
 	 <td><p> <tt>0.19&nbsp;0.0&nbsp;0.149</tt> </p></td>
 	 <td><p>  Pose offset for the Husky IMU's standard mounting location </p></td>
	</tr>
	<tr>  <td><span  id="line-40"></span><p> <tt>HUSKY_IMU_RPY</tt> </p></td>
  <td><p> <tt>0.0&nbsp;-1.5708&nbsp;3.1416</tt> </p></td>
  <td><p>  Orientation offset for the Husky IMU's standard mounting location </p></td>
	</tr>
	<tr>  <td><span  id="line-41"></span><p> <tt>HUSKY_NAVSAT_PORT</tt> </p></td>
	  <td><p> <tt>/dev/clearpath/gps</tt> </p></td>
	  <td><p> Port for the Husky GPS if present, must be set before running husky_bringup&nbsp;install</tt> </p></td>
	</tr>
	<tr>  <td><span  id="line-42"></span><p> <tt>HUSKY_NAVSAT_BAUD</tt> </p></td>
	  <td><p> <tt>19200</tt> </p></td>
	  <td><p> Baudrate for the Husky GPS </p></td>
	</tr>
	<tr>  <td><span  id="line-43"></span><p> <tt>HUSKY_UR5_IP</tt> </p></td>
	  <td><p> <tt>None </p></td>
	  <td><p> IP Address for the UR5 manipulator if present, must be set before running husky_bringup&nbsp;install</tt> </p></td>
	</tr>
	<tr>  <td><span  id="line-44"></span><p> <tt>HUSKY_UR5_ENABLED</tt> </p></td>
	  <td><p> <tt>false</tt> </p></td>
	  <td><p> Enable/disable the UR5 manipulator. </p></td>
	</tr>
	<tr>  <td><span  id="line-45"></span><p> <tt>HUSKY_LMS1XX_IP</tt> </p></td>
	  <td><p> None </p></td>
	  <td><p> IP Address for the SICK LMS1XX LIDAR if present, must be set before running husky_bringup&nbsp;install</tt> </p></td>
	</tr>
	<tr>  <td><span  id="line-46"></span><p> <tt>HUSKY_LMS1XX_ENABLED</tt> </p></td>
	  <td><p> <tt>false</tt> </p></td>
	  <td><p> Enable/disable the SICK LMS1XX LIDAR </p></td>
	</tr>
	<tr>  <td><span  id="line-47"></span><p> <tt>HUSKY_LMS1XX_XYZ</tt> </p></td>
	  <td><p> <tt>0.2206&nbsp;0.0&nbsp;0.00635</tt> </p></td>
	  <td><p>  Pose offset for the SICK LMS1XX LIDAR </p></td>
	</tr>
	<tr>  <td><span  id="line-48"></span><p> <tt>HUSKY_LMS1XX_RPY</tt> </p></td>
	  <td><p> <tt>0.0&nbsp;0.0&nbsp;0.0</tt> </p></td>
	  <td><p>  Orientation offset for the SICK LMS1XX LIDAR </p></td>
	</tr>
	<tr>  <td><span  id="line-48"></span><p> <tt>HUSKY_LMS1XX_IP</tt> </p></td>
	  <td><p> <tt>192.168.131.20</tt> </p></td>
	  <td><p>  IP address of the SICK LMS1XX LIDAR </p></td>
	</tr>
	<tr>  <td><span  id="line-46"></span><p> <tt>HUSKY_LMS1XX_SECONDARY_ENABLED</tt> </p></td>
	  <td><p> <tt>false</tt> </p></td>
	  <td><p> Enable/disable the secondary SICK LMS1XX LIDAR </p></td>
	</tr>
	<tr>  <td><span  id="line-47"></span><p> <tt>HUSKY_LMS1XX_SECONDARY_XYZ</tt> </p></td>
	  <td><p> <tt>-0.2206&nbsp;0.0&nbsp;0.00635</tt> </p></td>
	  <td><p>  Pose offset for the secondary SICK LMS1XX LIDAR </p></td>
	</tr>
	<tr>  <td><span  id="line-48"></span><p> <tt>HUSKY_LMS1XX_SECONDARY_RPY</tt> </p></td>
	  <td><p> <tt>0.0&nbsp;0.0&nbsp;0.0</tt> </p></td>
	  <td><p>  Orientation offset for the secondary SICK LMS1XX LIDAR </p></td>
	</tr>
	<tr>  <td><span  id="line-48"></span><p> <tt>HUSKY_LMS1XX_SECONDARY_IP</tt> </p></td>
	  <td><p> <tt>192.168.131.21</tt> </p></td>
	  <td><p>  IP address of the secondary SICK LMS1XX LIDAR </p></td>
	</tr>
	<tr>  <td><span  id="line-46"></span><p> <tt>HUSKY_LASER_3D_ENABLED</tt> </p></td>
	  <td><p> <tt>false</tt> </p></td>
	  <td><p> Enable/disable the main Velodyne VLP-16 </p></td>
	</tr>
	<tr>  <td><span  id="line-47"></span><p> <tt>HUSKY_LASER_3D_XYZ</tt> </p></td>
	  <td><p> <tt>0&nbsp;0&nbsp;0</tt> </p></td>
	  <td><p>  Pose offset for the Velodyne VLP-16 </p></td>
	</tr>
	<tr>  <td><span  id="line-48"></span><p> <tt>HUSKY_LASER_3D_RPY</tt> </p></td>
	  <td><p> <tt>0.0&nbsp;0.0&nbsp;0.0</tt> </p></td>
	  <td><p>  Orientation offset for the Velodyne VLP-16 </p></td>
	</tr>
	<tr>  <td><span  id="line-48"></span><p> <tt>HUSKY_LASER_3D_HOST</tt> </p></td>
	  <td><p> <tt>192.168.131.20</tt> </p></td>
	  <td><p>  IP address of the Velodyne VLP-16 </p></td>
	</tr>
	<tr>  <td><span  id="line-48"></span><p> <tt>HUSKY_LASER_3D_TOPIC</tt> </p></td>
	  <td><p> <tt>points</tt> </p></td>
	  <td><p>  ROS topic the Velodyne VLP-16 data publishes to </p></td>
	</tr>
	<tr>  <td><span  id="line-48"></span><p> <tt>HUSKY_MAG_CONFIG</tt> </p></td>
	  <td><p> <tt>$(find husky_bringup)/config<br/>/mag_config_default.yaml</tt> </p></td>
	  <td><p>  Path to the Husky's compass calibration configuration file </p></td>
	</tr>
	<tr>  <td><span  id="line-46"></span><p> <tt>HUSKY_REALSENSE_ENABLED</tt> </p></td>
	  <td><p> <tt>false</tt> </p></td>
	  <td><p> Enable/disable the main RealSense D435 camera </p></td>
	</tr>
	<tr>  <td><span  id="line-47"></span><p> <tt>HUSKY_REALSENSE_MOUNT_FRAME</tt> </p></td>
	  <td><p> <tt>sensor_arch_mount_link</tt> </p></td>
	  <td><p>  Mounting point for the RealSense D435 camera </p></td>
	</tr>
	<tr>  <td><span  id="line-48"></span><p> <tt>HUSKY_REALSENSE_OFFSET</tt> </p></td>
	  <td><p> <tt>0.0&nbsp;0.0&nbsp;0.0</tt> </p></td>
	  <td><p>  Pose offset for the RealSense D435 </p></td>
	</tr>
	<tr>  <td><span  id="line-48"></span><p> <tt>HUSKY_REALSENSE_OFFSET</tt> </p></td>
	  <td><p> <tt>0&nbsp;0&nbsp;0</tt> </p></td>
	  <td><p>  Orientation offset for the RealSense D435 </p></td>
	</tr>
	<tr>  <td><span  id="line-49"></span><p> <tt>HUSKY_TOP_PLATE_ENABLED</tt> </p></td>
	  <td><p> <tt>true</tt> </p></td>
	  <td><p> Enable/disable the standard Husky top plate. </p></td>
	</tr>
	<tr>  <td><span  id="line-49"></span><p> <tt>HUSKY_SENSOR_ARCH</tt> </p></td>
	  <td><p> <tt>false</tt> </p></td>
	  <td><p> Enable/disable the sensor/accessory arch. </p></td>
	</tr>
	<tr>  <td><span  id="line-49"></span><p> <tt>HUSKY_SENSOR_ARCH_HEIGHT</tt> </p></td>
	  <td><p> <tt>510</tt> </p></td>
	  <td><p> The height of the sensor arch in mm.  Must be either 510 or 300 </p></td>
	</tr>
	<tr>  <td><span  id="line-49"></span><p> <tt>HUSKY_SENSOR_ARCH_OFFSET</tt> </p></td>
	  <td><p> <tt>0&nbsp;0&nbsp;0</tt> </p></td>
	  <td><p> Pose offset for the sensor arch </p></td>
	</tr>
	<tr>  <td><span  id="line-49"></span><p> <tt>HUSKY_SENSOR_ARCH_RPY</tt> </p></td>
	  <td><p> <tt>0&nbsp;0&nbsp;0</tt> </p></td>
	  <td><p> Orientation offset for the sensor arch </p></td>
	</tr>
	<tr>  <td><span  id="line-49"></span><p> <tt>HUSKY_LOGITECH</tt> </p></td>
	  <td><p> <tt>0</tt> </p></td>
	  <td><p> This must be set to 1 if you use a Logitech F710 controller; otherwise a PS4 controller for teleop is assumed. </p></td>
	</tr>
	</tbody></table>

Adding a Source Workspace
---------------------------

Configuring non-standard peripherals requires a source workspace on the robot PC.

1.  Create a new workspace:

.. code:: bash

	 mkdir -p ~/husky_noetic_ws/src

2.  Add any custom source packages to the ~/husky_noetic_ws/src directory.

3.  After adding your packages, make sure any necessary dependencies are installed:

.. code:: bash

 	cd ~/husky_noetic_ws/
	rosdep install --from-paths src --ignore-src --rosdistro noetic -y

4.  Build the workspace:

.. code:: bash

 	cd ~/husky_noetic_ws/
	catkin_make

5.  Modify your robot-wide setup file (/etc/ros/setup.bash) to source your new workspace instead of the base noetic 		install:

.. code:: bash

 	source /home/administrator/husky_noetic_ws/devel/setup.bash

6.  Reinitialize your environment so that it picks up your new workspace:

.. code:: bash

 	source /etc/ros/setup.bash


Robot Description
-----------------------

In ROS Hydro and earlier, custom Husky descriptions (URDFs) were provided to customers in a workspace in their home folder. Since the Husky URDF has undergone some changes for Kinetic and later, your robot description from prior ROS releases will have to be slightly adapted.

First create a new URDF file in which you will define your custom Husky additions.  e.g. ``/home/administrator/husky-custom.xacro``.  Then modify ``/etc/ros/setup.bash`` to define the HUSKY_URDF_EXTRAS variable to point to your new file:

.. code:: bash

	export HUSKY_URDF_EXTRAS=/path/to/your/custom-file.xacro

So for the previous example, if we saved the customized file to ``/home/administrator/husky-custom.xacro`` we would put ``export HUSKY_URDF_EXTRAS=/home/administrator/husky-custom.xacro`` in the ``setup.bash`` file.

Modify your customized ``*.xacro`` file to add whatever additional features are required.  When finished, restart ROS by running `sudo systemctl restart ros`.  You can verify that your customized model is being used by running

.. code-block:: bash

		roslauch husky_viz view_robot.launch


Network Configuration
-----------------------

If upgrading from prior ROS releases, your old ``/etc/network/interfaces`` file may contain a static IP binding for your robot, or other customizations that should be replicated on your new setup.
