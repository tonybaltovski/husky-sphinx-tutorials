Setting Up Husky's Network
===========================

Husky is normally equipped with a combination Wifi + Bluetooth module, such as the `Intel Centrino Advanced-N 6235`__.
If this is your first unboxing, ensure that Husky's wireless antennas are firmly screwed on to the chassis.

.. _Centrino: http://www.intel.com/content/www/us/en/wireless-products/centrino-advanced-n-6235.html
__ Centrino_

Some Husky robots may only be equipped with a single antenna, depending on the exact model of PC installed in the robot.


First Connection
----------------

By default, Husky's wireless is in client mode, looking for the wireless network at the Clearpath factory. In
order to set it up to connect to your own network, you'll have to connect an ethernet cable to the Husky's PC and connect
the other end to your laptop.  Configure your laptop to have a static IP address on the ``192.168.131.x`` subnet, e.g.
``192.168.131.100``.  Then, make the connection to Husky's default static IP:

.. code-block:: bash

    ssh administrator@192.168.131.1

The default password is ``clearpath``. You should now be logged into Husky as the administrator user.

Changing the Default Password
-----------------------------

.. Note::

  All Clearpath robots ship from the factory with their login password set to ``clearpath``.  Upon receipt of your
  robot we recommend changing the password.

To change the password to log into your robot, run the

.. code-block:: bash

  passwd

command.  This will prompt you to enter the current password, followed by the new password twice.  While typing the
passwords in the ``passwd`` prompt there will be no visual feedback (e.g. "*" characters).

To further restrict access to your robot you can reconfigure the robot's SSH service to disallow logging in with a
password and require SSH certificates to log in.  This_ tutorial covers how to configure SSH to disable password-based
login.

.. _This: https://linuxize.com/post/how-to-setup-passwordless-ssh-login/


Connecting to Wifi Access Point
--------------------------------

Husky's standard wireless network manager is wicd_. To connect to an access point in your lab, run:

.. code-block:: bash

    wicd-curses

You should see a browsable list of networks which the robot has detected. Use arrow keys to select the one you
would like to connect to, and then press the right arrow to configure it. You can enter your network's password
near the bottom of the page, and note that you must select the correct encryption scheme; most modern networks
use ``WPA1/2 Passphrase``, so if that's you, make sure that option is selected. You also likely want to select
the option to automatically reconnect to this network, so that Husky will be there for you on your wireless
automatically in the future.

When you're finished, press F10 to save, and then C to connect.

Wicd will tell you in the footer what IP address it was given by your lab's access point, so you can now log out,
remove the network cable, and reconnect over wireless. When you've confirmed that all this is working as expected,
close up Husky's chassis.

.. _wicd: https://launchpad.net/wicd


.. _remote:

Remote ROS Connection
---------------------

To use ROS desktop tools, you'll need your computer to be able to connect to Husky's ROS master. This can be a
tricky process, but we've tried to make it as simple as possible.

In order for the ROS tools on your computer to talk to Husky, they need to know two things:

- How to find the ROS master, which is set in the ``ROS_MASTER_URI`` environment variable, and
- How processes on the other computer can find *your computer*, which is the ``ROS_IP`` environment variable.

The suggested pattern is to create a file in your home directory called ``remote-husky.sh`` with the following
contents:

.. code-block:: bash

    export ROS_MASTER_URI=http://cpr-husky-0001:11311  # Husky's hostname
    export ROS_IP=10.25.0.102                           # Your laptop's wireless IP address

If your network doesn't already resolve Husky's hostname to its wireless IP address, you may need to add
a corresponding line to your computer's ``/etc/hosts`` file:

.. code-block:: bash

    10.25.0.101 cpr-husky-0001

Then, when you're ready to communicate remotely with Husky, you can source that script like so, thus defining
those two key environment variables in the present context.

.. code-block:: bash

    source remote-husky.sh

Now, when you run commands like ``rostopic list``, ``rostopic echo``, ``rosnode list``, and others, the output
you see should reflect the activity on Husky's ROS master, rather than on your own machine. Once you've
verified the basics (list, echo) from the prompt, try launching some of the standard visual ROS tools:

.. code-block:: bash

    roslaunch husky_viz view_robot.launch
    rosrun rqt_robot_monitor rqt_robot_monitor
    rosrun rqt_console rqt_console

If there are particular :roswiki:`rqt` widgets you find yourself using a lot, you may find it an advantage to dock them together
and then export this configuration as the default RQT perspective. Then, to bring up your standard GUI, you can simply
run:

.. code-block:: bash

    rqt
