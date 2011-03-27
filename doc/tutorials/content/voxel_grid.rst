.. _voxel_grid:

Downsampling a PointCloud using a VoxelGrid filter
--------------------------------------------------

In this tutorial we will learn how to downsample -- that is, reduce the number
of points -- a point cloud dataset, using a voxelized grid approach. 

.. raw:: html
  
  <iframe title="Downsampling a PointCloud using a VoxelGrid filter" width="480" height="390" src="http://www.youtube.com/embed/YHR6_OIxtFI" frameborder="0" allowfullscreen></iframe>

The code
--------

First, download the dataset `table_scene_lms400.pcd
<http://dev.pointclouds.org/attachments/download/22/table_scene_lms400.pcd>`_
and save it somewhere to disk.

Then, create a file, let's say, ``voxel_grid.cpp`` in your favorite
editor, and place the following inside it:

.. code-block:: cpp
   :linenos:

   #include <iostream>
   #include "pcl/io/pcd_io.h"
   #include "pcl/point_types.h"
   #include "pcl/filters/voxel_grid.h"
  
   int
     main (int argc, char** argv)
   {
      sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2 ()), 
      sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ());
  
      // Fill in the cloud data
      pcl::PCDReader reader;
      // Replace the path below with the path where you saved your file
      reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

      std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
                << " data points (" << pcl::getFieldsList (*cloud) << ")."
    
      // Create the filtering object
      pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
      sor.setInputCloud (cloud);
      sor.setLeafSize (0.01, 0.01, 0.01);
      sor.filter (*cloud_filtered);
    
      std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
                << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."

      pcl::PCDWriter writer;
      writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
                    Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  
      return (0);
    }

The explanation
---------------

Now, let's break down the code piece by piece.

The following lines of code will read the point cloud data from disk.

.. code-block:: cpp

      // Fill in the cloud data
      pcl::PCDReader reader;
      // Replace the path below with the path where you saved your file
      reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

Then, a *pcl::VoxelGrid* filter is created with a leaf size of 1cm, the input
data is passed, and the output is computed and stored in *cloud_filtered*.

.. code-block:: cpp

      pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
      sor.setInputCloud (cloud);
      sor.setLeafSize (0.01, 0.01, 0.01);
      sor.filter (*cloud_filtered);

Finally, the data is written to disk for later inspection.

.. code-block:: cpp

      pcl::PCDWriter writer;
      writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
                    Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. code-block:: cmake
   
   add_executable (voxel_grid voxel_grid.cpp)
   target_link_libraries (voxel_grid pcl_io pcl_filters)

After you have made the executable, you can run it. Simply do::

  $ ./voxel_grid

You will see something similar to::

  PointCloud before filtering: 460400 data points (x y z intensity distance sid).
  PointCloud after filtering: 41049 data points (x y z intensity distance sid).

