.. _using_pcl:

Using PCL in your own project
-----------------------------

This tutorial explains how to use PCL from your own project.


Prerequisites
-------------

We assume you have downloaded, compiled and installed PCL on your
machine.

Project settings
----------------
Let us say the project is placed under /PATH/TO/MY/GRAND/PROJECT that
contains a lonely cpp file name ``pcd_write.cpp``. Do::


  $ cd /PATH/TO/MY/GRAND/PROJECT

  $ cp /PATH/TO/WHERE/YOU/BUILT/PCL/FindPCL.cmake .

And create a file named CMakeLists.txt that contains:

.. code-block:: cmake
	 
	 cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
	 project(MY_GRAND_PROJECT)
	 list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
	 find_package(PCL 1.0 REQUIRED COMPONENTS io)
	 include_directories(${PCL_INCLUDE_DIRS})
	 add_executable(pcd_write_test pcd_write.cpp)
	 target_link_libraries(pcd_write_test ${PCL_IO_LIBRARIES})
	 
The explanation
---------------

Now, let's see what we did.

.. code-block:: cmake
	 
	 cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
	 
This is mandatory for cmake, and since we are making very basic
project we don't need features from cmake 2.8 or higher.

.. code-block:: cmake
	 
	 project(MY_GRAND_PROJECT)	

This line names your project and sets some useful cmake variables
such as those to refer to the source directory
(MY_GRAND_PROJECT_SOURCE_DIR) and the directory from which you are
invoking cmake (MY_GRAND_PROJECT_BINARY_DIR).

.. code-block:: cmake

	 list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

We have placed FindPCL.cmake right on the project directory, so we need
to append the current source directory to the list of paths that cmake
will seek for any file named FindXXX.cmake.

.. code-block:: cmake

	 find_package(PCL 1.0 REQUIRED COMPONENTS io)

We are requesting to find the PCL package at minimum version 1.0. We
also says that it is ``REQUIRED`` meaning that cmake will fail
gracefully if it can't be found. As PCL is modular one can request:

* only one component: find_package(PCL 1.0 REQUIRED COMPONENTS io)
* several: find_package(PCL 1.0 REQUIRED COMPONENTS io common)
* all existing: find_package(PCL 1.0 REQUIRED)

.. code-block:: cmake

 	 include_directories(${PCL_INCLUDE_DIRS})

When PCL is found, several related variables are set:

* `PCL_FOUND`: set to 1 if PCL is found, otherwise unset
* `PCL_INCLUDE_DIRS`: set to the paths to PCL installed headers
* `PCL_LIBRARIES`: set to the file names of the built and installed PCL libraries
* `PCL_LINK_DIRECTORIES`: set to the paths to where PCL libraries reside
* `PCL_VERSION`: the version of the found PCL 

To let cmake know about external headers you include in your project,
one needs to use ``include_directories()`` macro. In our case
``PCL_INCLUDE_DIRS``, contains exactly what we need, thus we ask cmake
to search the paths it contains for a header potentially included.

.. code-block:: cmake

	 add_executable(pcd_write_test pcd_write.cpp)

Here, we tell cmake that we are trying to make an executable file
named ``pcd_write_test`` from one single source file
``pcd_write.cpp``. CMake will take care of the suffix (``.exe`` on
Windows platform and blank on UNIX) and the permissions.

.. code-block:: cmake

	 target_link_libraries(pcd_write_test ${PCL_IO_LIBRARIES})

The executable we are building makes call to PCL functions. So far, we
have only included the PCL headers so the compilers knows about the
methods we are calling. We need also to make the linker knows about
the libraries we are linking against. As said before the, PCL
found libraries are refered to using ``PCL_LIBRARIES`` variable, all
that remains is to trigger the link operation which we do calling
``target_link_libraries()`` macro.

Compiling and running the project
---------------------------------

Make a directory called ``build``, in which the compilation will be
done. Do::

  $ cd build

  $ cmake ..

You will see something similar to::

-- The C compiler identification is GNU
-- The CXX compiler identification is GNU
-- Check for working C compiler: /usr/bin/gcc
-- Check for working C compiler: /usr/bin/gcc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Found PCL_IO: /usr/local/lib/libpcl_io.so
-- Found PCL: /usr/local/lib/libpcl_io.so (Required is at least version "1.0")
-- Configuring done
-- Generating done
-- Build files have been written to: /PATH/TO/MY/GRAND/PROJECT/build

Now, we can build up our project, simply typing::

  $ make

The result should be as follow::

  Scanning dependencies of target pcd_write_test
  [100%] Building CXX object
  CMakeFiles/pcd_write_test.dir/pcd_write.cpp.o
  Linking CXX executable pcd_write_test
  [100%] Built target pcd_write_test

The project is now compiled, linked and ready to test::

  $ ./pcd_write_test


Which leads to this::

  Saved 5 data points to test_pcd.pcd.
    0.352222 -0.151883 -0.106395
    -0.397406 -0.473106 0.292602
    -0.731898 0.667105 0.441304
    -0.734766 0.854581 -0.0361733
    -0.4607 -0.277468 -0.916762
