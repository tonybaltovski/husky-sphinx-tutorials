Husky Frontier Exploration Demo
======================================

This tutorial shows you how to use `move_base <http://wiki.ros.org/move_base>`_ with `gmapping <http://wiki.ros.org/gmapping>`_ and `frontier_exploration <http://wiki.ros.org/frontier_exploration>`_ to perform autonomous planning movement, and exploration with simultaneous localization and mapping (SLAM), on a simulated Husky, or a factory-standard Husky with a laser scanner publishing on the scan topic.

To adapt this demo to your own Husky, you may need to clone the `husky_navigation <http://wiki.ros.org/husky_navigation>`_ repository, and modify the relevant parameters. To learn about `move_base <http://wiki.ros.org/move_base>`_, `gmapping <http://wiki.ros.org/gmapping>`_, `frontier_exploration <http://wiki.ros.org/frontier_exploration>`_ and the `navigation stack <http://wiki.ros.org/navigation>`_, see the `Navigation Tutorials <http://wiki.ros.org/navigation/Tutorials>`_.


Preparation
------------------

.. warning::

    The ``frontier_exploration`` package is no longer officially supported in ROS Melodic.  These instructions
    require building the package from source.  Because the package is no longer officially supported support for
    any issues you encounter will be limited.

First, you must build the ``frontier_exploration`` package from source.  Create a catkin workspace and ``cd`` into it
(or ``cd`` into an existing workspace if you already have one) and clone the code from github:

.. code-block:: bash

  cd ~/catkin_ws/src
  git clone https://github.com/paulbovbel/frontier_exploration.git

Install any additional dependencies:

.. code-block:: bash

  cd ~/catkin_ws
  rosdep install --from-paths src --ignore-src -r -y

The make the ``exploration_msgs`` package:

.. code-block:: bash

  catkin_make --pkg exploration_msgs

Once that package is built, source your workspace and build the rest of the package:

.. code-block:: bash

  source devel/setup.bash
  catkin_make

Make sure you have the ``husky_navigation`` package installed by running

.. code-block:: bash

  sudo apt-get install ros-melodic-husky-navigation

Because ``frontier_exploration`` is not officially supported by ROS Melodic you will need to modify the following
launch files inside the ``husky_navigation`` package:

* launch/exploration.launch
* launch/exploration_demo.launch

Remove the commented-out sections so that the files look like this:

**exploration.launch**:

.. code-block:: xml

  <launch>

    <node pkg="frontier_exploration" type="explore_client" name="explore_client" output="screen"/>
    <node pkg="frontier_exploration" type="explore_server" name="explore_server" output="screen">
      <param name="frequency" value="1.0"/>

      <!-- Should be less than sensor range -->
      <param name="goal_aliasing" value="2.0"/>
      <rosparam file="$(find husky_navigation)/config/costmap_common.yaml" command="load" ns="explore_costmap" />
      <rosparam file="$(find husky_navigation)/config/costmap_exploration.yaml" command="load" ns="explore_costmap" />
    </node>
  </launch>

**exploration_demo.launch**:

.. code-block:: xml

  <launch>
    <!--- Run gmapping -->
    <include file="$(find husky_navigation)/launch/gmapping.launch" />

    <!--- Run Move Base -->
    <include file="$(find husky_navigation)/launch/move_base.launch" />

    <!-- Run Frontier Exploration -->
    <include file="$(find husky_navigation)/launch/exploration.launch" />
  </launch>


Running the demo
------------------

.. note::

  In each terminal window, make sure to source the catkin workspace where you built ``frontier_exploration``

1.  In three separate terminal windows:

  i.  Start the Clearpath-configured Husky simulation environment:

  .. code:: bash

    roslaunch husky_gazebo husky_playpen.launch

  ii. Start the Clearpath-configured rviz visualizer:

  .. code:: bash

    roslaunch husky_viz view_robot.launch

  iii.  Start the frontier_exploration demo:

  .. code:: bash

    roslaunch husky_navigation exploration_demo.launch

2.  In the Rviz visualizer, make sure the visualizers in the Navigation group are enabled.

3.  Use the Point tool in the top toolbar to draw a closed polygon on the map that the Husky should explore. Watch the terminal window for instructions.

4.  As the robot moves, you should see the grey static map (map topic) grow. Occasionally, the gmapping algorithm will relocalize the robot, causing a discrete jump in the map->odom transform.

5.  When the exploration goal is complete, you will see a feedback message in the terminal window. You can now issue a new exploration goal if you wish.

6.  To save the generated map, you can run the map_saver utility:

.. code:: bash

  rosrun map_server map_saver -f <filename>
