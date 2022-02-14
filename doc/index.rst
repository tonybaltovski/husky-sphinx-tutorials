Husky UGV Tutorials
====================

.. image:: images/husky_banner.jpg
    :alt: Husky Robot

This package supplies Sphinx-based tutorial content to assist you with setting up and operating your Husky_ mobile robot. The tutorials topics are listed in the left column, and presented in the suggested reading order.

.. _Husky: https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/

.. Warning::
  These tutorials assume that you are comfortable working with `ROS2 <https://docs.ros.org/en/foxy/index.html>`_.

:doc:`Simulation <HuskySimulation>` is a logical place for most users to start, as this is universally applicable; understanding how to effectively operate Husky in simulation is valuable whether you are in the testing phase with software you intend to ultimately deploy on a real Husky, or you do not have one and are simply exploring the platform's capabilities.

:doc:`Driving <HuskyDriving>` covers how to teleoperate Husky using the remote control, a well as safety procedures for operating the real robot. Anyone working with a physical robot should be familiar with this section.

.. toctree::
    :maxdepth: 0
    :caption: Husky Overview

    Introduction <self>
    HuskyPackages

.. toctree::
    :maxdepth: 0
    :caption: Setting Up Husky

    ROSUpgrade
    HuskyInstallSoftware
    HuskyNetwork
    HuskyControllerPairing

.. toctree::
    :maxdepth: 0
    :caption: Using Husky

    HuskyDriving
    HuskySimulation
