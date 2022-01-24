Installing Husky Software
==========================

.. note::

  The physical Husky robot comes pre-configured with ROS2 and the necessary Husky packages already installed; therefore, you will only need to follow the instructions below if re-installing software on the Husky.

.. note::

  If you wish to install the Husky packages on your computer (e.g. to interface with the physical Husky robot and/or to simulate Husky), then proceed with the following instructions below. A prequisite is to make sure you have a working ROS2 Foxy installation set up on your computer.

Add Clearpath Debian Package Repository
----------------------------------------

Before you can install the Husky packages, you need to configure Ubuntu's APT package manager to
add Clearpath's package server:

1. Install the authentication key for the packages.clearpathrobotics.com repository. In terminal, run:

.. code-block:: bash

    wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -

2. Add the debian sources for the repository. In terminal, run:

.. code-block:: bash

    sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'

3. Update your computer's package cache. In terminal, run:

.. code-block:: bash

    sudo apt-get update

Installing from Debian Packages
--------------------------------

The preferred way to install Husky’s packages is using precompiled Debian packages. These packages are available for Ubuntu 20.04.

After your computer is configured to use Clearpath's debian package repository, you can install the Husky packages needed for this tutorial. In terminal, run:

.. code-block :: bash

    sudo apt-get install ros-foxy-husky-desktop ros-foxy-husky-simulator

The ``husky_robot`` package only needs to be installed on the physical Husky robot; however, you can optionally install it on your computer too. In terminal, run:

.. code-block :: bash

    sudo apt-get install ros-foxy-husky-robot

Installing from Source
-----------------------

Husky packages are available on GitHub_, and can be compiled and installed from source if desired:

.. _GitHub: https://github.com/husky/

1. Create a workspace directory. In terminal, run:

.. code-block:: bash

    mkdir -p ~/husky_ws/src

2. Clone the Husky repositories into your workspace directory. In terminal, run:

.. code-block:: bash

    cd ~/husky_ws/src
    git clone -b foxy-devel https://github.com/husky/husky.git
    cd ..

3. Install additional dependencies. In terminal, run:

.. code-block:: bash

    rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

4. Build the workspace. In terminal, run:

.. code-block:: bash

    colcon build

5. You can now source your workspace to make use of the packages you just built. In terminal, run:

.. code-block:: bash

    source install/setup.bash
