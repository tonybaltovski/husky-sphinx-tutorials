Husky UGV Tutorials
=====================


.. image:: images/TJM_5949_00001.jpg


Husky is a rugged, outdoor-ready unmanned ground vehicle (UGV), suitable for research and rapid prototyping applications. Husky fully supports ROS—all of the packages are available in the `Husky github org <https://github.com/husky>`_.

For more information or to receive a quote, please `visit us online <http://clearpathrobotics.com/husky>`_.

.. Warning::
  These tutorials assume that you are comfortable working with ROS.  We recommend starting with our
  `ROS tutorial <./../ros>`_ if you are not familiar with ROS already.

.. note::

  These tutorials specifically target Husky robots running Ubuntu 20.04 with ROS Noetic, as it is the standard OS environment for Husky. If instead you have an older Husky robot running Ubuntu 18.04 with ROS Melodic, please follow `this tutorial <https://www.clearpathrobotics.com/assets/guides/melodic/melodic-to-noetic/index.html>`_ to upgrade its OS environment to Ubuntu 20.04 with ROS Noetic.

.. toctree::
    :maxdepth: 0
    :caption: Installation

    BackUpHusky
    InstallHuskySoftware
    HuskyNetwork
    CustomizeHuskyConfig

.. toctree::
    :maxdepth: 0
    :caption: Using Husky

    SimulatingHusky
    InterfacingWithHusky
    DrivingHusky

.. toctree::
    :maxdepth: 0
    :caption: Demo Applications: Navigation

    HuskyMove
    HuskyAMCL
    HuskyGmapping
    HuskyFrontiers

.. toctree::
    :maxdepth: 0
    :caption: NVIDIA Jetson

    jetson_tx2
    jetson_nano
    jetson_xavier

.. toctree::
    :maxdepth: 0
    :caption: Package Components

    HuskyPackages
    additional_sim_worlds
