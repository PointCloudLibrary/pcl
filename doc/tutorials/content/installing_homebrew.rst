.. _installing_homebrew:

Installing on Mac OS X using Homebrew
=====================================

This tutorial explains how to install the Point Cloud Library on Mac OS
X using Homebrew. Two approaches are described: using the existing PCL
formula to automatically install, and using Homebrew only for the
dependencies with PCL compiled and installed from source.

.. image:: images/macosx_logo.png
   :alt: Mac OS X logo
   :align: right

.. _homebrew_preqs:

Prerequisites
=============

No matter which method you choose, you will need to have Homebrew
installed. If you do not already have a Homebrew installation, see the
`Homebrew homepage`_ for installation instructions.

.. _`Homebrew homepage`:
   http://mxcl.github.com/homebrew/

.. _homebrew_all:

Using the formula
=================

Homebrew includes a formula for installing PCL. This will automatically
install all necessary dependencies and provides options for controlling
which parts of PCL are installed.

.. note::

   The PCL formula is currently in development. It will be submitted to
   Homebrew shortly. Until then, you can download it from
   `PCL.RB <http://dev.pointclouds.org/attachments/download/1052/pcl.rb>`_. To prepare it,
   follow these steps:


   #. Install Homebrew. See the Homebrew website for instructions.
   #. Execute ``brew update``.
   #. Download the formula and place it in
      ``/usr/local/Library/Formula`` (or an appropriate location if you
      installed Homebrew somewhere else).

To install using the formula, execute the following command::

  $ brew install pcl

You can specify options to control which parts of PCL are installed. For
example, to disable the Python bindings and visualisation, and enable the
documentation, execute the following command::

  $ brew install pcl --nopython --novis --doc

For a full list of the available options, see the formula's help::

  $ brew options pcl

You can test the installation by executing the tests included with PCL::

  $ brew test pcl

Once PCL is installed, you may wish to periodically upgrade it. Update
Homebrew and, if a PCL update is available, upgrade::

  $ brew update
  $ brew upgrade pcl


.. _homebrew_deps:

Installing from source
======================

In order to compile every component of PCL, several dependencies must be
installed. Homebrew includes formulae for all PCL dependencies except
OpenNI, so this step is relatively easy.

Dependency information
----------------------

Required
''''''''

The following libraries are **Required** to build PCL.

- **CMake** version >= 2.8.3 (http://www.cmake.org)
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
  (http://www.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN)
  Library for performing fast approximate nearest neighbor searches in high
  dimensional spaces.  Used in `kdtree` for fast approximate nearest neighbors
  search.

- **Visualization ToolKit (VTK)** version >= 5.6.1 (http://www.vtk.org/)
  Software system for 3D computer graphics, image processing and visualization.
  Used in `visualization` for 3D point cloud rendering and visualization.

  .. note::

     The current release of PCL (1.2) does not support visualisation on
     Mac OS X. PCL 1.3 is expected to correct this.

Optional
''''''''

The following libraries are *optional* and provide extended functionality
within PCL, such as Kinect support.

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

- **Doxygen** (http://www.doxygen.org)
   A documentation system for C++, C, Java, Objective-C, Python, IDL (Corba and
   Microsoft flavors), Fortran, VHDL, PHP, C#, and to some extent D.

- **Sphinx** (http://sphinx.pocoo.org/)
   A tool that makes it easy to create intelligent and beautiful
   documentation. PCL uses this and Doxygen to compile the
   documentation.

Advanced (Developers)
'''''''''''''''''''''

The following libraries are *advanced* and provide additional functionality
for PCL developers:

- **googletest** version >= 1.6.0 (http://code.google.com/p/googletest/)
   Google's framework for writing C++ tests on a variety of platforms. Used
   to build test units.

Installing dependencies
-----------------------

Most of the dependencies will be installed via Homebrew. The remainder,
we will compile from source.

Install CMake
'''''''''''''
::

  $ brew install cmake

Install Boost
'''''''''''''
::

  $ brew install boost

Install Eigen
'''''''''''''
::

  $ brew install eigen

Install FLANN
'''''''''''''
::

  $ brew install flann

Install VTK
'''''''''''

To install VTK, you need a modified Homebrew formula for VTK. Please
download it from `VTK.RB <http://dev.pointclouds.org/attachments/600/vtk.rb>`_.

::

  $ brew install vtk --qt OR --qt-extern [if you have your own Qt installation already]

.. note::

   If you are installing PCL 1.2, you may skip this dependency.

Install Qhull (optional)
''''''''''''''''''''''''
::

  $ brew install qhull

Install libusb (optional)
'''''''''''''''''''''''''
::

  $ brew install libusb

Install Doxygen (optional)
''''''''''''''''''''''''''
::

  $ brew install doxygen

Install Sphinx (optional)
'''''''''''''''''''''''''
::

  $ brew install sphinx

Install patched OpenNI and Sensor
'''''''''''''''''''''''''''''''''

Download the patched versions of OpenNI and Sensor: `openni_osx.zip
<http://dev.pointclouds.org/attachments/download/191/openni_osx.zip>`_ and
`ps_engine_osx.zip
<http://dev.pointclouds.org/attachments/download/192/ps_engine_osx.zip>`_.

Extract, build, fix permissions and install OpenNI::

   $ unzip openni_osx.zip -d openni_osx
   $ cd openni_osx/Redist
   $ chmod -R a+r Bin Include Lib
   $ chmod -R a+x Bin Lib
   $ chmod a+x Include/MacOSX Include/Linux-*
   $ sudo ./install.sh

In addition the PrimeSense XML configuration file found within the
patched OpenNI download needs its permissions fixed and to be copied to
the correct location to for the Kinect to work on Mac OS X::

   $ chmod a+r openni_osx/Redist/Samples/Config/SamplesConfig.xml
   $ sudo cp openni_osx/Redist/Samples/Config/SamplesConfig.xml /etc/primesense/

Extract, build, fix permissions and install Sensor::

   $ unzip ps_engine_osx.zip -d ps_engine_osx
   $ cd ps_engine_osx/Redist
   $ chmod -R a+r Bin Lib Config Install
   $ chmod -R a+x Bin Lib
   $ sudo ./install.sh

Compiling PCL
-------------

At this point you should have everything needed installed to build PCL
with almost no additional configuration.

Check out the PCL source from the Github:

   $ git clone https://github.com/PointCloudLibrary/pcl
   $ cd pcl

Create the build directories, configure CMake, build and install::

   $ mkdir build
   $ cd build
   $ cmake ..
   $ make
   $ sudo make install

.. note::

   If you are installing PCL 1.2, disable the visualisation module, or
   compilation will fail::

     $ cmake .. -DBUILD_visualization:BOOL=OFF

The customization of the build process is out of the scope of this tutorial and
is covered in greater detail in the :ref:`building_pcl` tutorial.

Compiling the documentation (optional)
--------------------------------------

If you installed the Doxygen and Sphinx dependencies, you can compile
the documentation after compiling PCL. To do so, use this command::

  $ make doc

The tutorials can be built using this command::

  $ make Tutorials

.. note::

  The Homebrew formula for Sphinx may not install the extension
  necessary to link to the Doxygen-generated documentation. In this
  case, you will need to install Sphinx and the extension manually.
  Start by installing Sphinx using easy_install::

    $ easy_install -U Sphinx

  Next, install Mercurial (see the Mercurial documentation) and the
  extension::

   $ hg clone http://bitbucket.org/birkenfeld/sphinx-contrib
   $ cd sphinx-contrib/doxylink
   $ python setup.py install

Using PCL
---------

Now that PCL in installed, you can start using the library in your own
projects by following the :ref:`using_pcl_pcl_config` tutorial.

