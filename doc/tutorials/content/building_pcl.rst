.. _building_pcl:

Customizing the PCL build process
---------------------------------

This tutorial explains how to modify the PCL cmake options and tweak your
building process to better fit the needs of your project and/or your system's
requirements.

Audience
--------

This tutorial targets users with a basic knowledge of CMake, C++ compilers,
linkers, flags and make.

Prerequisites
-------------

We assume you have checked out the last available revision of PCL.

PCL basic settings
------------------

Let's say PCL is placed under /PATH/TO/PCL, which we will refer to as PCL_ROOT::

  $ cd $PCL_ROOT
  $ mkdir build
  $ cmake ..

This will cause `cmake` to create a file called CMakeCache.txt in the build
directory with the default options.

Let's have a look at what `cmake` options got enabled::

  $ ccmake ..

You should see something like the following on screen::
   
 BUILD_TESTS                      ON
 BUILD_common                     ON
 BUILD_features                   ON
 BUILD_filters                    ON
 BUILD_global_tests               ON
 BUILD_io                         ON
 BUILD_kdtree                     ON
 BUILD_keypoints                  ON
 BUILD_octree                     ON
 BUILD_range_image                ON
 BUILD_registration               ON
 BUILD_sample_consensus           ON
 BUILD_segmentation               ON
 BUILD_surface                    ON
 BUILD_visualization              ON
 CMAKE_BUILD_TYPE                 
 CMAKE_INSTALL_PREFIX             /usr/local
 PCL_SHARED_LIBS                  ON
 PCL_VERSION                      1.0.0
 VTK_DIR                          /usr/local/lib/vtk-5.6
 wxWidgets_CONFIG_EXECUTABLE      /usr/bin/wx-config
 wxWidgets_wxrc_EXECUTABLE        /usr/bin/wxrc

   
The explanation
---------------

* BUILD_TESTS: this an option which can enables/disables building of
   tests

* BUILD_common: this an option which can enables/disables building of
   common library

* BUILD_features: this an option which can enables/disables building of
   features library

* BUILD_filters: this an option which can enables/disables building of
   filters library

* BUILD_global_tests: this an option which can enables/disables building of
   global unit tests

* BUILD_io: this an option which can enables/disables building of
   io library

* BUILD_kdtree: this an option which can enables/disables building of
   kdtree library

* BUILD_keypoints: this an option which can enables/disables building of
   keypoints library

* BUILD_octree: this an option which can enables/disables building of
   octree library

* BUILD_range_image: this an option which can enables/disables building of
   range_image library

* BUILD_registration: this an option which can enables/disables building of
   registration library

* BUILD_sample_consensus: this an option which can enables/disables building of
   sample_consensus library

* BUILD_segmentation: this an option which can enables/disables building of
   segmentation library

* BUILD_surface: this an option which can enables/disables building of
   surface library

* BUILD_visualization: this an option which can enables/disables building of
   visualization library

* CMAKE_BUILD_TYPE: here you specify which the build type. In CMake,
   a CMAKE_BUILD_TYPE corresponds to a set of options and flags passed
   to the compiler to activate/deactivate a functionality and to
   constrain the building process.

* CMAKE_INSTALL_PREFIX: where the built libraries and the headers
   will be installed

* PCL_SHARED_LIBS: option to enable building of shared
   libraries. Default is yes.

* PCL_VERSION: this is the PCL library version. It affects the built
   libraries names.

* VTK_DIR: directory of VTK library if found

* wxWidgets_CONFIG_EXECUTABLE: path to wx-config program

* wxWidgets_wxrc_EXECUTABLE: path to wxrc program

The above are called `cmake` cached variables. At this level we only looked at
the basic ones.

Tweaking basic settings
-----------------------

Depending on your project/system, you might want to enable/disable certain
options. For example, you can prevent the building of:

* tests: setting BUILD_TESTS and BUILD_global_tests to OFF

* a library: setting BUILD_LIBRARY_NAME to OFF

Note that if you disable a XXX library that is required for building
YYY then XXX will be built but won't appear in the cache. 

You can also change the build type:

* Debug: means that no optimization is done and all the debugging
   symbols are imbedded into the libraries file. This is plateform and
   compiler dependent. On Linux with gcc this is equivalent to running
   gcc with -O0 -g -ggdb -Wall

* Release: the compiled code is optimized and no debug information
   will be print out. This will lead to -O3 for gcc and -O5 for clang

* RelWithDebInfo: the compiled code is optimized but debugging data
   is also imbedded in the libraries. This is a tradeoff between the
   two former ones.

* MinSizeRel: this, normally, results in the smallest libraries you
   can build. This is interesting when building for Android or a
   restricted memory/space system.

A list of available CMAKE_BUILD_TYPEs can be found by typing::

  $ cmake --help-variable CMAKE_BUILD_TYPE

