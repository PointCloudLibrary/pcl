.. _ensenso_cameras:

==========================================
Grabbing point clouds from Ensenso cameras
==========================================

In this tutorial we will learn how to use the `IDS-Imaging <http://en.ids-imaging.com/>`_ Ensenso cameras within PCL. This tutorial will show you how to configure PCL
and how to use the examples to fetch point clouds from the `Ensenso <http://www.ensenso.de/>`_.

.. contents::

Install Ensenso drivers
=======================

The Ensenso drivers are free (as in beer) and available for download, for each of them follow the instructions provided:

  * `uEye <http://en.ids-imaging.com/download-ueye.html>`_
  * `Ensenso SDK <http://www.ensenso.de/download>`_

Plug-in the camera and test if the Ensenso is working, launch ``nxView`` in your terminal to check if you can actually use the camera.

Configuring PCL
===============

You need at least PCL 1.8.0 to be able to use the Ensenso cameras. You need to make sure ``WITH_ENSENSO`` is set to ``true`` in the CMake 
configuration (it should be set to true by default if you have followed the instructions before).

The default following values can be tweaked into cmake if you don't have a standard installation, for example:

.. code-block::

  ENSENSO_ABI_DIR     /opt/ensenso_test/development/c

You can deactivate building the Ensenso support by setting ``BUILD_ENSENSO`` to false.
Compile and install PCL.

Using the example
==================

The `pcl_ensenso_viewer <https://github.com/PointCloudLibrary/pcl/blob/master/visualization/tools/ensenso_viewer.cpp>`_ example shows how to
display a point cloud grabbed from an Ensenso device using the `EnsensoGrabber <http://docs.pointclouds.org/trunk/classpcl_1_1_ensenso_grabber.html>`_ class.

Note that this program opens the TCP port of the nxLib tree, this allows you to open the nxLib tree with the nxTreeEdit program (port 24000).
The capture parameters (exposure, gain etc..) are set to default values.
If you have performed and stored an extrinsic calibration it will be temporary reset.

.. code-block:: cpp

  ensenso_grabber->enumDevices ();
  ensenso_grabber->openTcpPort ();
  ensenso_grabber->openDevice ();
  ensenso_grabber->configureCapture ();
  ensenso_grabber->setExtrinsicCalibration ();

The code is very similar to the ``pcl_openni_viewer``.
All the Ensenso devices connected are listed and then the point cloud are fetched as fast as possible.

Here is an example of the terminal output ::
   
   $ pcl_ensenso_viewer 
   Initialising nxLib
   Number of connected cameras: 1
   Serial No    Model   Status
   140242   N10-1210-18   Available
   
   Opening Ensenso stereo camera id = 0
   
   FPS: 4.46927
   FPS: 5.01253
   FPS: 5.16351
   FPS: 5.27506
   FPS: 5.32569
   FPS: 5.51645
   FPS: 5.53013
   FPS: 5.56729
   Closing Ensenso stereo camera

.. image:: images/ensenso/ensenso_viewer.jpg
  :height: 550

