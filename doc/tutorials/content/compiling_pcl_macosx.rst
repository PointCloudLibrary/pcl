.. _compiling_pcl_macosx:

Compiling PCL and its dependencies from MacPorts and source on Mac OS X
=======================================================================

This tutorial explains how to build the Point Cloud Library
**from MacPorts and source** on Mac OS X platforms, and tries to guide you
through the download and building *of all the required dependencies*.

.. image:: images/macosx_logo.png
   :alt: Mac OS X logo
   :align: right


.. _macosx_prerequisites:

Prerequisites
=============

Before getting started download and install the following prerequisites for
Mac OS X:

- **XCode** (https://developer.apple.com/xcode/)
   Apple’s powerful integrated development environment


- **MacPorts** (http://www.macports.org)
   An open-source community initiative to design an easy-to-use
   system for compiling, installing, and upgrading  either command-line, X11 or
   Aqua based open-source software on the Mac OS X operating system.


.. _macosx_dependencies:

PCL Dependencies
================

In order to compile every component of the PCL library we need to download and
compile a series of 3rd party library dependencies.  We'll cover the building,
compiling and installing of everything in the following sections:

Required
--------

The following libraries are **Required** to build PCL.

- **CMake** version >= 3.5.0 (http://www.cmake.org)
   Cross-platform, open-source build system.

   .. note::
  
      Though not a dependency per se, the PCL community relies heavily on CMake
      for the libraries build process.

- **Boost** version >= 1.46.1 (http://www.boost.org/)
   Provides free peer-reviewed portable C++ source libraries.  Used for shared
   pointers, and threading.

- **Eigen** version >= 3.0.0 (http://eigen.tuxfamily.org/)
   Unified matrix library.  Used as the matrix backend for SSE optimized math.

- **FLANN** version >= 1.6.8
  (http://www.cs.ubc.ca/research/flann/)
  Library for performing fast approximate nearest neighbor searches in high
  dimensional spaces.  Used in `kdtree` for fast approximate nearest neighbors
  search.

- **Visualization ToolKit (VTK)** version >= 5.6.1 (http://www.vtk.org/)
   Software system for 3D computer graphics, image processing and visualization.
   Used in `visualization` for 3D point cloud rendering and visualization.

Optional
--------

The following libraries are **Optional** and provide extended functionality
within PCL, ie Kinect support.

- **Qhull** version >= 2011.1 (http://www.qhull.org/)
   computes the convex hull, Delaunay triangulation, Voronoi diagram, halfspace
   intersection about a point, furthest-site Delaunay triangulation, and
   furthest-site Voronoi diagram.  Used for convex/concave hull decompositions
   in `surface`.

- **libusb** (http://www.libusb.org/)
   A library that gives user level applications uniform access to USB devices
   across many different operating systems.

- **PCL Patched OpenNI/Sensor** (http://www.openni.org/)
   The OpenNI Framework provides the interface for physical devices and for
   middleware components. Used to grab point clouds from OpenNI compliant
   devices.

Advanced (Developers)
---------------------

The following libraries are **Advanced** and provide additional functionality
for PCL developers:

- **googletest** version >= 1.6.0 (http://code.google.com/p/googletest/)
   Google's framework for writing C++ tests on a variety of platforms. Used
   to build test units.

- **Doxygen** (http://www.doxygen.org)
   A documentation system for C++, C, Java, Objective-C, Python, IDL (Corba and
   Microsoft flavors), Fortran, VHDL, PHP, C#, and to some extent D.

- **Sphinx** (http://sphinx-doc.org/)
   A tool that makes it easy to create intelligent and beautiful
   documentation.


.. _macosx_building_prerequisites:

Building, Compiling and Installing PCL Dependencies
===================================================

By now you should have downloaded and installed the latest versions of XCode and
MacPorts under the :ref:`macosx_prerequisites` section.  We'll be installing most
dependencies available via MacPorts and the rest will be built from source.

Install CMake
-------------
::

   $ sudo port install cmake
   
   
Install Boost
-------------
::

   $ sudo port install boost
   
   
Install Eigen
-------------
::

   $ sudo port install eigen3
   
Install FLANN
-------------
::

   $ sudo port install flann
   
Install VTK
-----------

To install via MacPorts::

   $ sudo port install vtk5 +qt4_mac
   
To install from source download the source from
http://www.vtk.org/VTK/resources/software.html

Follow the README.html for compiling on UNIX / Cygwin / Mac OSX::

   $ cd VTK
   $ mkdir VTK-build
   $ cd VTK-build
   $ ccmake ../VTK

Within the CMake configuration:
   Press [c] for initial configuration

   Press [t] to get into advanced mode and change the following::
   
      VTK_USE_CARBON:OFF
      VTK_USE_COCOA:ON
      VTK_USE_X:OFF

   .. note::

      VTK *must* be built with Cocoa support and *must* be installed,
      in order for the visualization module to be able to compile. If you do
      not require visualisation, you may omit this step.

   Press [g] to generate the make files.
   
   Press [q] to quit.

Then run::
   
   $ make && make install
   
Install Qhull
-------------
::
   
   $ sudo port install qhull

Install libusb
--------------
::

   $ sudo port install libusb-devel +universal

Install Patched OpenNI and Sensor
---------------------------------

Download the patched versions of OpenNI and Sensor from the PCL downloads page
http://pointclouds.org/downloads/macosx.html

Extract, build, fix permissions and install OpenNI::

   $ unzip openni_osx.zip -d openni_osx
   $ cd openni_osx/Redist
   $ chmod -R a+r Bin Include Lib
   $ chmod -R a+x Bin Lib
   $ chmod a+x Include/MacOSX Include/Linux-*
   $ sudo ./install
   
In addition the following primesense xml config found within the patched OpenNI
download needs its permissions fixed and copied to the correct location to for
the Kinect to work on Mac OS X::

   $ chmod a+r openni_osx/Redist/Samples/Config/SamplesConfig.xml
   $ sudo cp openni_osx/Redist/Samples/Config/SamplesConfig.xml /etc/primesense/

Extract, build, fix permissions and install Sensor::

   $ unzip ps_engine_osx.zip -d ps_engine_osx
   $ cd ps_engine_osx/Redist
   $ chmod -R a+r Bin Lib Config Install
   $ chmod -R a+x Bin Lib
   $ sudo ./install


.. _macosx_building_pcl:

Building PCL
============

At this point you should have everything needed installed to build PCL with
almost no additional configuration.

Checkout the PCL source from the Github:

   $ git clone https://github.com/PointCloudLibrary/pcl
   $ cd pcl
   
Create the build directories, configure CMake, build and install::

   $ mkdir build
   $ cd build
   $ cmake ..
   $ make
   $ sudo make install
   
The customization of the build process is out of the scope of this tutorial and
is covered in greater detail in the :ref:`building_pcl` tutorial.


Using PCL
=========

We finally managed to compile the Point Cloud Library (PCL) for Mac OS X. You
can start using them in your project by following the :ref:`using_pcl_pcl_config` tutorial.


.. _macosx_advanced:

Advanced (Developers)
=====================

Testing (googletest)
--------------------


API Documentation (Doxygen)
---------------------------

Install Doxygen via MacPorts::

   $ sudo port install doxygen

Or install the Prebuilt binary for Mac OS X
(http://www.stack.nl/~dimitri/doxygen/download.html#latestsrc)

After installed you can build the documentation::

   $ make doc

Tutorials (Sphinx)
------------------

In addition to the API documentation there is also tutorial documentation built
using Sphinx.  The easiest way to get this installed is using pythons
`easy_install`::

   $ easy_install -U Sphinx

The Sphinx documentation also requires the third party contrib extension
`sphinxcontrib-doxylink` (https://pypi.python.org/pypi/sphinxcontrib-doxylink)
to reference the Doxygen built documentation.

To install from source you'll also need Mercurial::

   $ sudo port install mercurial
   $ hg clone http://bitbucket.org/birkenfeld/sphinx-contrib
   $ cd sphinx-contrib/doxylink
   $ python setup.py install

After installed you can build the tutorials::

   $ make Tutorials

.. note::
   
   Sphinx can be installed via MacPorts but is a bit of a pain getting all the
   PYTHON_PATH's in order

