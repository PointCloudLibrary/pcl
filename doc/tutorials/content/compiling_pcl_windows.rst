.. _compiling_pcl_windows:

Compiling PCL and its dependencies from source in Windows
---------------------------------------------------------

This tutorial explains how to build the Point Cloud Library **from source** on
Microsoft Windows platforms, and tries to guide you through the download and
the compilation *of all the required dependencies*. The illustrated compilation
process produces a static library. 

.. image:: images/windows_logo.png
   :alt: Microsoft Windows logo
   :align: right

Requirements
-------------

In order to compile every component of the PCL library we need to download a
compile a series of 3rd party library dependencies:

	- **Boost** version >= 1.46.1 (http://www.boost.org/)

    used for shared pointers, and threading. **mandatory**

	- **Eigen** version >= 3.0.0 (http://eigen.tuxfamily.org/)

    used as the matrix backend for SSE optimized math. **mandatory**

	- **CMINPACK** version >= 1.1.3 (http://devernay.free.fr/hacks/cminpack/cminpack.html)

    used in the `sample_consensus` and `registration` modules for non-linear
    (Levenberg-Marquardt) optimizations. **mandatory**

	- **FLANN** version >= 1.6.8 (http://www.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN)

    used in `kdtree` for fast approximate nearest neighbors search. **mandatory**

	- **Visualization ToolKit (VTK)** version >= 5.6.1 (http://www.vtk.org/)

    used in `visualization` for 3D point cloud rendering and visualization. **mandatory**

	- **QHULL** version >= 2011.1 (http://www.qhull.org/)

    used for convex/concave hull decompositions in `surface`. **optional**

	- **wxWidgets** version >= 2.8.1 (http://www.wxwidgets.org/)

    used for `pcl::RangeImage` visualizations. **optional**


.. note::
  
   Though not a dependency per se, don't forget that you also need the CMake
   build system (http://www.cmake.org/), at least version **2.8.3**. We recommend
   version **2.8.4**.

Building dependencies
---------------------
Let's unpack all our libraries in C:/PCL_dependencies so that it would like
like::

	C:/PCL_dependencies
	C:/PCL_dependencies/boost_1_46_1
	C:/PCL_dependencies/eigen
	C:/PCL_dependencies/cminpack-1.1.3
	C:/PCL_dependencies/flann-1.6.8-src
	C:/PCL_dependencies/qhull
	C:/PCL_dependencies/wxWidgets
	C:/PCL_dependencies/VTK
	
Let's start with `Boost`, which does not use CMake. To build Boost, open the
command prompt and navigate to the Boost folder::

	prompt> cd c:\PCL_dependencies\boost_1_46_1
	prompt> bootstrap
	
Depending on the compilation toolchain you intend to use to build PCL, you need
to change the following `--toolset` argument to msvc-8.0, msvc-9.0 (Visual
Studio 2008) or msvc-10.0 (Visual Studio 2010)::

	prompt> bjam --toolset=msvc-9.0 --with-thread --with-date_time --build-type=complete stage

This will take a while (time for coffee or something).

Once done, proceed to compile `Eigen`. Open CMake-gui and fill in the fields::

  Where is my source code: C:/PCL_dependencies/eigen
  Where to build binaries: C:/PCL_dependencies/eigen/bin .
  
Hit the "Configure" button and CMake will tell that the binaries folder doesn't exist yet 
(e.g., *C:/PCL_dependencies/eigen/bin*) and it will ask for a confirmation.
Proceed and be sure to choose the correct "Generator" on the next window. So,
if you've built Boost using the Visual Studio 2008 toolset you would choose the
same generator here. 

.. note::
  
  Don't forget that all the dependencies must be compiled using the same
  compiler options and architecture specifications, i.e. you can't mix 32 bit
  libraries with 64 bit libraries.

Hit "Configure" again and then go for the "Generate" button. This will generate
the required project files/makefiles to build the library. Now you can simply
go to `C:/PCL_dependencies/eigen/bin` and proceed with the compilation using
your toolchain. In case you use Visual Studio, you will find the Visual Studio
Solution file in that folder. 

Once done, let's continue with the `cminpack` library. Open CMake-gui and fill
in the fields::

  Where is my source code: C:/PCL_dependencies/cminpack-1.1.3
  Where to build binaries: C:/PCL_dependencies/cminpack-1.1.3/bin .
  
As before, when clicking on "Configure", CMake will ask to create the bin
folder. Confirm then hit again "Configure". After choosing the generator, if
CMake doesn't report any problem, continue generating the files and proceed
with the compilation step.

We are halfway the compilation process -- let's move on to `FLANN`. Setup the
CMake fields as usual::

  Where is my source code: C:/PCL_dependencies/flann-1.6.8-src
  Where to build binaries: C:/PCL_dependencies/flann-1.6.8-src/bin .

Hit "Configure" and as for the previous steps, confirm and choose the correct
"Generator". Now, on my machine I had to manually set the `BUILD_PYTHON_BINDINGS`
and `BUILD_MATLAB_BINDINGS` to OFF otherwise it would not continue to the next
step as it is complaining about unable to find Python and Matlab. Click on
"Advanced mode" and find them, or alternatively, add those entries by clicking
on the "Add Entry" button in the top right of the CMake-gui window.  Add one
entry named "BUILD_PYTHON_BINDINGS", set its type to "Bool" and its value to
unchecked. Do the same with the "BUILD_MATLAB_BINDINGS" entry. Now hit the
"Configure" button and it should work. Go for the "Generate" button and proceed
to the compilation phase.

Setup the CMake fields with the `qhull` paths::

  Where is my source code: C:/PCL_dependencies/qhull
  Where to build binaries: C:/PCL_dependencies/qhull/bin .
  
Then hit "Configure" twice and "Generate". Compile the generated project files.

The procedure is virtually the same for `VTK` so I won't show it again here.

Building `wxWidgets` is a bit different: go to the
C:\PCL_dependencies\wxWidgets\build\msw folder, open the project file and build
the "Debug" and "Release" configurations. Don't use the DLL* configurations.

That's it, we're done with the dependencies!

Building PCL
------------

This section assumes that you're downloading PCL trunk from svn, but the same
instuctions can be applied to a released version too (e.g. pcl-1.0).

Let's assume that the PCL source code is in C:/PCL. Run the CMake-gui
application and fill in the fields::

  Where is my source code: C:/PCL
  Where to build binaries: C:/PCL/bin .

Now hit the "Configure" button. You will be asked to select a generator for
this project, that is to say the toolchain which will be used to build the
sources. In my case, since I have Visual Studio 2008 installed, my choice is
"Visual Studio 9, 2008". Be sure to leave the "Use default native compilers"
radio button checked unless you really know what you are doing.

Because CMake is unable to find all the 3rd party libraries you installed by
itself, it will prompt you to input their paths manually. 

Because this tutorial is exemplifying the process of compiling PCL as a
**static** library, the first thing you have to do is to uncheck the
**PCL->PCL_SHARED_LIBS** checkbox. Also uncheck **BUILD->BUILD_TESTS** and
**BUILD->BUILD_global_tests** unless you plan to run the unit tests as a
developer.


Now examine the CMake-gui log window. You should see some red colored error
stating that CMake could not find library XXX. The GUI will also highlight in
red the items which need to be modified. In my case the first time it couldn't
find the EIGEN library, so it said::

	Could NOT find Eigen (missing: EIGEN_INCLUDE_DIR) .

Simply modify the **EIGEN_INCLUDE_DIR** key to point to the correct Eigen include
directory (C:/PCL_dependencies/eigen in our case). 

Now hit the "Configure" button again. Like before, look for the error in the
log area and sort it out by modifying the related key above the "Configure"
button. Repeat the process until you've done with the errors.

A little trick: when it comes to BOOST or wxWidgets errors, just set the
**wxWidgets_ROOT_DIR** and **Boost_INCLUDE_DIR** elements to the appropriate paths. The
CMake build system will figure out the other related paths automatically.

Once all the reported errors are sorted out the "Generate" button becomes
available. Hit it and a project will be generated in C:/PCL/bin.

Open that folder and use the generated project to finally build the PCL library
using the toolchain of your choice.

Using PCL
---------

We finally managed to compile the Point Cloud Library (PCL) as binaries for
Windows. You can start using them in your project by following the
:ref:`using_pcl` tutorial.

