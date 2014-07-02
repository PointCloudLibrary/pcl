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
   
 BUILD_common                     ON
 BUILD_features                   ON
 BUILD_filters                    ON
 BUILD_global_tests               OFF
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

   
The explanation
---------------

* `BUILD_common`: option to enable/disable building of common library

* `BUILD_features`: option to enable/disable building of features library

* `BUILD_filters`: option to enable/disable building of filters library

* `BUILD_global_tests`: option to enable/disable building of global unit tests

* `BUILD_io`: option to enable/disable building of io library

* `BUILD_kdtree`: option to enable/disable building of kdtree library

* `BUILD_keypoints`: option to enable/disable building of keypoints library

* `BUILD_octree`: option to enable/disable building of octree library

* `BUILD_range_image`: option to enable/disable building of range_image library

* `BUILD_registration`: option to enable/disable building of registration library

* `BUILD_sample_consensus`: option to enable/disable building of sample_consensus library

* `BUILD_segmentation`: option to enable/disable building of segmentation library

* `BUILD_surface`: option to enable/disable building of surface library

* `BUILD_visualization`: option to enable/disable building of visualization library

* `CMAKE_BUILD_TYPE`: here you specify the build type. In CMake, a CMAKE_BUILD_TYPE corresponds to a set of options and flags passed to the compiler to activate/deactivate a functionality and to constrain the building process.

* `CMAKE_INSTALL_PREFIX`: where the headers and the built libraries will be installed

* `PCL_SHARED_LIBS`: option to enable building of shared libraries. Default is yes.

* `PCL_VERSION`: this is the PCL library version. It affects the built libraries names.

* `VTK_DIR`: directory of VTK library if found

The above are called `cmake` cached variables. At this level we only looked at
the basic ones.

Tweaking basic settings
-----------------------

Depending on your project/system, you might want to enable/disable certain
options. For example, you can prevent the building of:

* tests: setting `BUILD_global_tests` to `OFF`

* a library: setting `BUILD_LIBRARY_NAME` to `OFF`

Note that if you disable a XXX library that is required for building
YYY then XXX will be built but won't appear in the cache. 

You can also change the build type:

* **Debug**: means that no optimization is done and all the debugging symbols are imbedded into the libraries file. This is plateform and compiler dependent. On Linux with gcc this is equivalent to running gcc with `-O0 -g -ggdb -Wall`

* **Release**: the compiled code is optimized and no debug information will be print out. This will lead to `-O3` for gcc and `-O5` for clang

* **RelWithDebInfo**: the compiled code is optimized but debugging data is also imbedded in the libraries. This is a tradeoff between the two former ones.

* **MinSizeRel**: this, normally, results in the smallest libraries you can build. This is interesting when building for Android or a restricted memory/space system.

A list of available CMAKE_BUILD_TYPEs can be found typing::

  $ cmake --help-variable CMAKE_BUILD_TYPE

Tweaking advanced settings
--------------------------

Now we are done with all the basic stuff. To turn on advanced cache
options hit `t` while in ccmake.
Advanced options become especially useful when you have dependencies
installed in unusal locations and thus cmake hangs with
`XXX_NOT_FOUND` this can even prevent you from building PCL although
you have all the dependencies installed. In this section we will
discuss each dependency entry so that you can configure/build or
update/build PCL according to your system. 

Building unit tests
^^^^^^^^^^^^^^^^^^^

If you want to contribute to PCL, or are modifying the code, you need
to turn on building of unit tests. This is accomplished by setting the `BUILD_global_tests`
option to `ON`, with a few caveats. If you're using `ccmake` and you find that `BUILD_global_tests`
is reverting to `OFF` when you configure, you can move the cursor up to the `BUILD_global_tests` line to see the 
error message.

Two options which will need to be turned ON before `BUILD_global_tests` are `BUILD_outofcore` and 
`BUILD_people`. Your mileage may vary.

Also required for unit tests is the source code for the Google C++ Testing Framework. That is
usually as simple as downloading the source, extracting it, and pointing the `GTEST_SRC_DIR` and `GTEST_INCLUDE_DIR` 
options to the applicable source locations. On Ubuntu, you can simply run `apt-get install libgtest-dev`.

These steps enable the `tests` make target, so you can use `make tests` to run tests.

General remarks
^^^^^^^^^^^^^^^
Under ${PCL_ROOT}/cmake/Modules there is a list of FindXXX.cmake files
used to locate dependencies and set their related variables. They have
a list of default searchable paths where to look for them. In addition,
if pkg-config is available then it is triggered to get hints on their
locations. If all of them fail, then we look for a CMake entry or
environment variable named **XXX_ROOT** to find headers and libraries.
We recommend setting an environment variable since it is independent
from CMake and lasts over the changes you can make to your
configuration.

The available ROOTs you can set are as follow:

* **BOOST_ROOT**: for boost libraries with value `C:/Program Files/boost-1.4.6` for instance
* **CMINPACK_ROOT**: for cminpack with value `C:/Program Files/CMINPACK 1.1.13` for instance
* **QHULL_ROOT**: for qhull with value `C:/Program Files/qhull 6.2.0.1373` for instance
* **FLANN_ROOT**: for flann with value `C:/Program Files/flann 1.6.8` for instance
* **EIGEN_ROOT**: for eigen with value `C:/Program Files/Eigen 3.0.0` for instance

To ensure that all the dependencies were correctly found, beside the
message you get from CMake, you can check or edit each dependency specific
variables and give it the value that best fits your needs. 

UNIX users generally don't have to bother with debug vs release versions
they are fully complient. You would just loose debug symbols if you use
release libraries version instead of debug while you will end up with much
more verbose output and slower execution. This said, Windows MSVC users
and Apple iCode ones can build debug/release from the same project, thus
it will be safer and more coherent to fill them accordingly.


Detailed description
^^^^^^^^^^^^^^^^^^^^

Below, each dependency variable is listed, its meaning is explained
then a sample value is given for reference.

* Boost

+----------------------------------+---------------------------------------------------------------+------------------------------------------+ 
| cache variable                   | meaning                                                       | sample value                             |
+==================================+===============================================================+==========================================+
| Boost_DATE_TIME_LIBRARY          | full path to boost_date-time.[so,lib,a]                       | /usr/local/lib/libboost_date_time.so     |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_DATE_TIME_LIBRARY_DEBUG    | full path to boost_date-time.[so,lib,a] (debug version)       | /usr/local/lib/libboost_date_time-gd.so  |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_DATE_TIME_LIBRARY_RELEASE  | full path to boost_date-time.[so,lib,a] (release version)     | /usr/local/lib/libboost_date_time.so     |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_FILESYSTEM_LIBRARY         | full path to boost_filesystem.[so,lib,a]                      | /usr/local/lib/libboost_filesystem.so    |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_FILESYSTEM_LIBRARY_DEBUG   | full path to boost_filesystem.[so,lib,a] (debug version)      | /usr/local/lib/libboost_filesystem-gd.so |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_FILESYSTEM_LIBRARY_RELEASE | full path to boost_filesystem.[so,lib,a] (release version)    | /usr/local/lib/libboost_filesystem.so    |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_INCLUDE_DIR                | path to boost headers directory                               | /usr/local/include                       |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_LIBRARY_DIRS               | path to boost libraries directory                             | /usr/local/lib                           |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_SYSTEM_LIBRARY             | full path to boost_system.[so,lib,a]                          | /usr/local/lib/libboost_system.so        |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_SYSTEM_LIBRARY_DEBUG       | full path to boost_system.[so,lib,a] (debug version)          | /usr/local/lib/libboost_system-gd.so     |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_SYSTEM_LIBRARY_RELEASE     | full path to boost_system.[so,lib,a] (release version)        | /usr/local/lib/libboost_system.so        |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_THREAD_LIBRARY             | full path to boost_thread.[so,lib,a]                          | /usr/local/lib/libboost_thread.so        |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_THREAD_LIBRARY_DEBUG       | full path to boost_thread.[so,lib,a] (debug version)          | /usr/local/lib/libboost_thread-gd.so     |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+
| Boost_THREAD_LIBRARY_RELEASE     | full path to boost_thread.[so,lib,a] (release version)        | /usr/local/lib/libboost_thread.so        |
+----------------------------------+---------------------------------------------------------------+------------------------------------------+


* CMinpack

+------------------------+--------------------------------------------------------+----------------------------------+ 
| cache variable         | meaning                                                | sample value                     |
+========================+========================================================+==================================+ 
| CMINPACK_INCLUDE_DIR   | path to cminpack headers directory                     | /usr/local/include/cminpack-1    |
+------------------------+--------------------------------------------------------+----------------------------------+
| CMINPACK_LIBRARY       | full path to cminpack.[so,lib,a] (release version)     | /usr/local/lib/libcminpack.so    |
+------------------------+--------------------------------------------------------+----------------------------------+
| CMINPACK_LIBRARY_DEBUG | full path to cminpack.[so,lib,a] (debug version)       | /usr/local/lib/libcminpack-gd.so |    
+------------------------+--------------------------------------------------------+----------------------------------+


* FLANN

+---------------------+------------------------------------------------------------+-----------------------------------+
| cache variable      | meaning                                                    | sample value                      |
+=====================+============================================================+===================================+
| FLANN_INCLUDE_DIR   | path to flann headers directory                            | /usr/local/include                |
+---------------------+------------------------------------------------------------+-----------------------------------+
| FLANN_LIBRARY       | full path to libflann_cpp.[so,lib,a] (release version)     | /usr/local/lib/libflann_cpp.so    |
+---------------------+------------------------------------------------------------+-----------------------------------+
| FLANN_LIBRARY_DEBUG | full path to libflann_cpp.[so,lib,a] (debug version)       | /usr/local/lib/libflann_cpp-gd.so |
+---------------------+------------------------------------------------------------+-----------------------------------+


* Eigen

+------------------+---------------------------------+---------------------------+
| cache variable   | meaning                         | sample value              |
+==================+=================================+===========================+ 
| EIGEN_INCLUDE_DIR| path to eigen headers directory | /usr/local/include/eigen3 |
+------------------+---------------------------------+---------------------------+

