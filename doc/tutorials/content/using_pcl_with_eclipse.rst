.. _using_pcl_with_eclipse:

Using PCL with Eclipse
---------------------------------

This tutorial explains how to use Eclipse as a PCL editor

Prerequisites
-------------

We assume you have downloaded, compiled and installed PCL trunk (see Downloads, experimental) on your machine.

Creating the eclipse project files
----------------------------------

Open a terminal window and do::

  $ cd /PATH/TO/MY/TRUNK/ROOT
  $  cmake -G"Eclipse CDT4 - Unix Makefiles" .

You will see something similar to::

-- The C compiler identification is GNU
-- The CXX compiler identification is GNU
-- Could not determine Eclipse version, assuming at least 3.6 (Helios). Adjust CMAKE_ECLIPSE_VERSION if this is wrong.
-- Check for working C compiler: /home/u0062536/bin/gcc
-- Check for working C compiler: /home/u0062536/bin/gcc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working CXX compiler: /home/u0062536/bin/c++
-- Check for working CXX compiler: /home/u0062536/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- -- GCC > 4.3 found, enabling -Wabi
-- Using CPU native flags for SSE optimization:  -march=native
-- Performing Test HAVE_MM_MALLOC
-- Performing Test HAVE_MM_MALLOC - Success
-- Performing Test HAVE_POSIX_MEMALIGN
-- Performing Test HAVE_POSIX_MEMALIGN - Success
-- Performing Test HAVE_SSE4_2_EXTENSIONS
-- Performing Test HAVE_SSE4_2_EXTENSIONS - Success
-- Performing Test HAVE_SSE4_1_EXTENSIONS
-- Performing Test HAVE_SSE4_1_EXTENSIONS - Success
-- Performing Test HAVE_SSE3_EXTENSIONS
-- Performing Test HAVE_SSE3_EXTENSIONS - Success
-- Performing Test HAVE_SSE2_EXTENSIONS
-- Performing Test HAVE_SSE2_EXTENSIONS - Success
-- Performing Test HAVE_SSE_EXTENSIONS
-- Performing Test HAVE_SSE_EXTENSIONS - Success
-- Found SSE4.2 extensions, using flags:  -march=native -msse4.2 -mfpmath=sse
-- Try OpenMP C flag = [-fopenmp]
-- Performing Test OpenMP_FLAG_DETECTED
-- Performing Test OpenMP_FLAG_DETECTED - Success
-- Try OpenMP CXX flag = [-fopenmp]
-- Performing Test OpenMP_FLAG_DETECTED
-- Performing Test OpenMP_FLAG_DETECTED - Success
-- Found OpenMP: -fopenmp 
-- Found OpenMP
-- Boost version: 1.46.1
-- The following subsystems will be built:
--   common
--   kdtree
--   octree
--   io
--   search
--   sample_consensus
--   filters
--   2d
--   features
--   keypoints
--   geometry
--   ml
--   segmentation
--   visualization
--   outofcore
--   stereo
--   surface
--   tracking
--   registration
--   people
--   recognition
--   global_tests
--   tools
-- The following subsystems will not be built:
--   examples: Code examples are disabled by default.
--   simulation: Disabled by default.
--   apps: Disabled by default.
-- Configuring done
-- Generating done
-- Build files have been written to: /data/git/pcl

Importing into Eclipse
----------------------

Now you launch your Eclipse editor and you select File->Import...
Out of the list you select General->Existing Projects into Workspace and then next.
At the top you select Select root directory to be the root of your pcl trunk installation and press Finish.

Setting the PCL code style in Eclipse
-------------------------------------

You can find a PCL code style file for Eclipse in trunk/doc/advanced/content/files/. 
In Eclipse go to Project->Properties, then select Code Style in the left field and Enable project specific settings, then Import and select where your trunk/doc/advanced/content/files/PCL_eclipse_profile.xml file is.

Where to get more information
-----------------------------

You can get more information here: http://www.vtk.org/Wiki/Eclipse_CDT4_Generator
