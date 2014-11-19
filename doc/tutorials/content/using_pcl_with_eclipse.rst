.. _using_pcl_with_eclipse:

======================
Using PCL with Eclipse
======================

This tutorial explains how to use Eclipse as an IDE to manage your PCL projects. It was tested under Ubuntu 14.04 with Eclipse Luna; 
do not hesitate to modify this tutorial by submitting a pull request on GitHub to add other configurations etc.

.. contents::

Prerequisites
=============

We assume you have downloaded and extracted a PCL version (either PCL trunk or a stable version) on your machine.
For the example, we will use the `pcl visualizer <http://www.pointclouds.org/documentation/tutorials/pcl_visualizer.php>`_ code.

Creating the eclipse project files
==================================

The files are organized like the following tree::

  .
  ├── build
  └── src
      ├── CMakeLists.txt
      └── pcl_visualizer_demo.cpp

Open a terminal, navigate to your project root folder and configure the project::

  $ cd /path_to_my_project/build
  $ cmake -G "Eclipse CDT4 - Unix Makefiles" ../src

You will see something that should look like::

  -- The C compiler identification is GNU 4.8.2
  -- The CXX compiler identification is GNU 4.8.2
  -- Could not determine Eclipse version, assuming at least 3.6 (Helios). Adjust CMAKE_ECLIPSE_VERSION if this is wrong.
  -- Check for working C compiler: /usr/lib/ccache/cc
  -- Check for working C compiler: /usr/lib/ccache/cc   -- works
  -- Detecting C compiler ABI info
  -- Detecting C compiler ABI info - done
  -- Check for working CXX compiler: /usr/lib/ccache/c++
  -- Check for working CXX compiler: /usr/lib/ccache/c++   -- works
  -- Detecting CXX compiler ABI info
  -- Detecting CXX compiler ABI info - done
  -- checking for module 'eigen3'
  --   found eigen3, version 3.2.0
  -- Found eigen: /usr/include/eigen3  
  -- Boost version: 1.54.0
  -- Found the following Boost libraries:
  --   system
  --   filesystem
  --   thread
  --   date_time
  --   iostreams
  --   mpi
  --   serialization
  --   chrono
  -- checking for module 'openni-dev'
  --   package 'openni-dev' not found
  -- Found openni: /usr/lib/libOpenNI.so  
  -- checking for module 'openni2-dev'
  --   package 'openni2-dev' not found
  -- Found OpenNI2: /usr/lib/libOpenNI2.so  
  ** WARNING ** io features related to pcap will be disabled
  ** WARNING ** io features related to png will be disabled
  -- Found libusb-1.0: /usr/include  
  -- checking for module 'flann'
  --   found flann, version 1.8.4
  -- Found Flann: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a  
  -- Found qhull: /usr/lib/x86_64-linux-gnu/libqhull.so  
  -- checking for module 'openni-dev'
  --   package 'openni-dev' not found
  -- checking for module 'openni2-dev'
  --   package 'openni2-dev' not found
  -- looking for PCL_COMMON
  -- Found PCL_COMMON: /usr/local/lib/libpcl_common.so  
  -- looking for PCL_OCTREE
  -- Found PCL_OCTREE: /usr/local/lib/libpcl_octree.so  
  -- looking for PCL_IO
  -- Found PCL_IO: /usr/local/lib/libpcl_io.so  
  -- looking for PCL_KDTREE
  -- Found PCL_KDTREE: /usr/local/lib/libpcl_kdtree.so  
  -- looking for PCL_SEARCH
  -- Found PCL_SEARCH: /usr/local/lib/libpcl_search.so  
  -- looking for PCL_SAMPLE_CONSENSUS
  -- Found PCL_SAMPLE_CONSENSUS: /usr/local/lib/libpcl_sample_consensus.so  
  -- looking for PCL_FILTERS
  -- Found PCL_FILTERS: /usr/local/lib/libpcl_filters.so  
  -- looking for PCL_2D
  -- Found PCL_2D: /usr/local/include/pcl-1.7  
  -- looking for PCL_FEATURES
  -- Found PCL_FEATURES: /usr/local/lib/libpcl_features.so  
  -- looking for PCL_GEOMETRY
  -- Found PCL_GEOMETRY: /usr/local/include/pcl-1.7  
  -- looking for PCL_KEYPOINTS
  -- Found PCL_KEYPOINTS: /usr/local/lib/libpcl_keypoints.so  
  -- looking for PCL_SURFACE
  -- Found PCL_SURFACE: /usr/local/lib/libpcl_surface.so  
  -- looking for PCL_REGISTRATION
  -- Found PCL_REGISTRATION: /usr/local/lib/libpcl_registration.so  
  -- looking for PCL_ML
  -- Found PCL_ML: /usr/local/lib/libpcl_ml.so  
  -- looking for PCL_SEGMENTATION
  -- Found PCL_SEGMENTATION: /usr/local/lib/libpcl_segmentation.so  
  -- looking for PCL_RECOGNITION
  -- Found PCL_RECOGNITION: /usr/local/lib/libpcl_recognition.so  
  -- looking for PCL_VISUALIZATION
  -- Found PCL_VISUALIZATION: /usr/local/lib/libpcl_visualization.so  
  -- looking for PCL_PEOPLE
  -- Found PCL_PEOPLE: /usr/local/lib/libpcl_people.so  
  -- looking for PCL_OUTOFCORE
  -- Found PCL_OUTOFCORE: /usr/local/lib/libpcl_outofcore.so  
  -- looking for PCL_TRACKING
  -- Found PCL_TRACKING: /usr/local/lib/libpcl_tracking.so  
  -- looking for PCL_STEREO
  -- Found PCL_STEREO: /usr/local/lib/libpcl_stereo.so  
  -- looking for PCL_GPU_CONTAINERS
  -- Found PCL_GPU_CONTAINERS: /usr/local/lib/libpcl_gpu_containers.so  
  -- looking for PCL_GPU_UTILS
  -- Found PCL_GPU_UTILS: /usr/local/lib/libpcl_gpu_utils.so  
  -- looking for PCL_GPU_OCTREE
  -- Found PCL_GPU_OCTREE: /usr/local/lib/libpcl_gpu_octree.so  
  -- looking for PCL_GPU_FEATURES
  -- Found PCL_GPU_FEATURES: /usr/local/lib/libpcl_gpu_features.so  
  -- looking for PCL_GPU_KINFU
  -- Found PCL_GPU_KINFU: /usr/local/lib/libpcl_gpu_kinfu.so  
  -- looking for PCL_GPU_KINFU_LARGE_SCALE
  -- Found PCL_GPU_KINFU_LARGE_SCALE: /usr/local/lib/libpcl_gpu_kinfu_large_scale.so  
  -- looking for PCL_GPU_SEGMENTATION
  -- Found PCL_GPU_SEGMENTATION: /usr/local/lib/libpcl_gpu_segmentation.so  
  -- looking for PCL_CUDA_COMMON
  -- Found PCL_CUDA_COMMON: /usr/local/include/pcl-1.7  
  -- looking for PCL_CUDA_FEATURES
  -- Found PCL_CUDA_FEATURES: /usr/local/lib/libpcl_cuda_features.so  
  -- looking for PCL_CUDA_SEGMENTATION
  -- Found PCL_CUDA_SEGMENTATION: /usr/local/lib/libpcl_cuda_segmentation.so  
  -- looking for PCL_CUDA_SAMPLE_CONSENSUS
  -- Found PCL_CUDA_SAMPLE_CONSENSUS: /usr/local/lib/libpcl_cuda_sample_consensus.so  
  -- Found PCL: /usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_mpi.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libpthread.so;optimized;/usr/local/lib/libpcl_common.so;debug;/usr/local/lib/libpcl_common.so;optimized;/usr/local/lib/libpcl_octree.so;debug;/usr/local/lib/libpcl_octree.so;/usr/lib/libOpenNI.so;/usr/lib/libOpenNI2.so;vtkCommon;vtkFiltering;vtkImaging;vtkGraphics;vtkGenericFiltering;vtkIO;vtkRendering;vtkVolumeRendering;vtkHybrid;vtkWidgets;vtkParallel;vtkInfovis;vtkGeovis;vtkViews;vtkCharts;optimized;/usr/local/lib/libpcl_io.so;debug;/usr/local/lib/libpcl_io.so;optimized;/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a;debug;/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a;optimized;/usr/local/lib/libpcl_kdtree.so;debug;/usr/local/lib/libpcl_kdtree.so;optimized;/usr/local/lib/libpcl_search.so;debug;/usr/local/lib/libpcl_search.so;optimized;/usr/local/lib/libpcl_sample_consensus.so;debug;/usr/local/lib/libpcl_sample_consensus.so;optimized;/usr/local/lib/libpcl_filters.so;debug;/usr/local/lib/libpcl_filters.so;optimized;/usr/local/lib/libpcl_features.so;debug;/usr/local/lib/libpcl_features.so;optimized;/usr/local/lib/libpcl_keypoints.so;debug;/usr/local/lib/libpcl_keypoints.so;optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so;optimized;/usr/local/lib/libpcl_surface.so;debug;/usr/local/lib/libpcl_surface.so;optimized;/usr/local/lib/libpcl_registration.so;debug;/usr/local/lib/libpcl_registration.so;optimized;/usr/local/lib/libpcl_ml.so;debug;/usr/local/lib/libpcl_ml.so;optimized;/usr/local/lib/libpcl_segmentation.so;debug;/usr/local/lib/libpcl_segmentation.so;optimized;/usr/local/lib/libpcl_recognition.so;debug;/usr/local/lib/libpcl_recognition.so;optimized;/usr/local/lib/libpcl_visualization.so;debug;/usr/local/lib/libpcl_visualization.so;optimized;/usr/local/lib/libpcl_people.so;debug;/usr/local/lib/libpcl_people.so;optimized;/usr/local/lib/libpcl_outofcore.so;debug;/usr/local/lib/libpcl_outofcore.so;optimized;/usr/local/lib/libpcl_tracking.so;debug;/usr/local/lib/libpcl_tracking.so;optimized;/usr/local/lib/libpcl_stereo.so;debug;/usr/local/lib/libpcl_stereo.so;optimized;/usr/local/lib/libpcl_gpu_containers.so;debug;/usr/local/lib/libpcl_gpu_containers.so;optimized;/usr/local/lib/libpcl_gpu_utils.so;debug;/usr/local/lib/libpcl_gpu_utils.so;optimized;/usr/local/lib/libpcl_gpu_octree.so;debug;/usr/local/lib/libpcl_gpu_octree.so;optimized;/usr/local/lib/libpcl_gpu_features.so;debug;/usr/local/lib/libpcl_gpu_features.so;optimized;/usr/local/lib/libpcl_gpu_kinfu.so;debug;/usr/local/lib/libpcl_gpu_kinfu.so;optimized;/usr/local/lib/libpcl_gpu_kinfu_large_scale.so;debug;/usr/local/lib/libpcl_gpu_kinfu_large_scale.so;optimized;/usr/local/lib/libpcl_gpu_segmentation.so;debug;/usr/local/lib/libpcl_gpu_segmentation.so;optimized;/usr/local/lib/libpcl_cuda_features.so;debug;/usr/local/lib/libpcl_cuda_features.so;optimized;/usr/local/lib/libpcl_cuda_segmentation.so;debug;/usr/local/lib/libpcl_cuda_segmentation.so;optimized;/usr/local/lib/libpcl_cuda_sample_consensus.so;debug;/usr/local/lib/libpcl_cuda_sample_consensus.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_mpi.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libpthread.so;optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so;/usr/lib/libOpenNI.so;/usr/lib/libOpenNI2.so;optimized;/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a;debug;/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a;vtkCommon;vtkFiltering;vtkImaging;vtkGraphics;vtkGenericFiltering;vtkIO;vtkRendering;vtkVolumeRendering;vtkHybrid;vtkWidgets;vtkParallel;vtkInfovis;vtkGeovis;vtkViews;vtkCharts (Required is at least version "1.7") 
  -- Configuring done
  -- Generating done
  -- Build files have been written to: /home/dell/visualizer/build

Importing into Eclipse
======================

- Launch `Eclipse CDT <http://eclipse.org/cdt/>`_ and select ``File > Import``.
- In the list select  ``General > Existing Projects into Workspace`` and then next.
- Browse (``Select root directory``) to the root folder of the project and select the ``build`` folder (in the example case, ``/home/dell/visualizer/build``).
- Click ``Finish``.

.. WARNING::
  The Eclipse indexer is going to parse the files in the project (and all the includes), this can take a lot of time and might crash Eclipse if it's not configured for big projects.
  Take a look at the bottom right of Eclipse's window to see the indexer status; it is advised not to do anything until the indexer has finished it's job.

Configuring Eclipse
-------------------

If Eclipse fails to open your PCL project you might need to change Eclipse configuration; here are some values that should solve all problems 
(but might not work on light hardware configurations)::

  $ sudo gedit /usr/lib/eclipse/eclipse.ini

Change the values in the last lines::

  org.eclipse.platform
  --launcher.XXMaxPermSize
  1024m
  --launcher.defaultAction
  openFile
  --launcher.appendVmargs
  -vmargs
  -Dosgi.requiredJavaVersion=1.7
  -XX:MaxPermSize=512m
  -Xms1024m
  -Xmx1024m

Restart Eclipse and go to ``Windows > Preferences``, then ``C/C++ > Indexer > Cache Limits``. Set the limits to [50% | 512 | 512].

Setting the PCL code style in Eclipse
=====================================

You can find a PCL code style file for Eclipse in `PCL GitHub trunk <https://github.com/PointCloudLibrary/pcl/blob/master/doc/advanced/content/files/PCL_eclipse_profile.xml>`_

Global
------
If you want to apply the PCL style guide to all projects: 
``Windows > Preferences > C/C++ > Code Style > Formatter``

Project specific
----------------
If you want to apply the style guide only to one project:
Go to ``Project > Properties``, then select ``Code Style`` in the left field and Enable ``project specific settings``, then ``Import`` and select where you profile file (.xml) is.

How to format the code
----------------------
If you want to format the whole project use ``Source > Format``. If you want to format only your selection use the shortcut ``Ctrl + Shift + F``

Launching the program
=====================

To build the project, click on the build icon

.. image:: images/pcl_with_eclipse/build_tab.gif
  :height: 16

- Create a launch configuration, select the project on the left panel (left click on the project name); ``Run > Run Configurations..``.
- Create a new ``C/C++ Application`` click on ``Search Project`` and choose the executable to be launched.
- Go the second tab (``Arguments``) and enter your arguments; remember this is not a terminal and ``~`` won't work to get to your home folder for example !

Run the program by clicking on the run icon

.. image:: images/pcl_with_eclipse/lrun_obj.gif
  :height: 16

The Eclipse console doesn't manage ANSI colours, you could use `an ANSI console plugin <http://www.mihai-nita.net/eclipse/>`_ to get rid of the "[0m" characters in the output.

Where to get more information
=============================

You can get more information about the Eclipse CDT4 Generator `here <http://www.vtk.org/Wiki/Eclipse_CDT4_Generator>`_.
