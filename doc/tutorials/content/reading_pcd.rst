.. _reading_pcd:

Reading Point Cloud data from PCD files
---------------------------------------

In this tutorial we will learn how to read point cloud data from a PCD file.

The code
--------

First, create a file called, let's say, ``pcd_read.cpp`` in your favorite
editor, and place the following code inside it:

.. code-block:: cpp
   :linenos:

   #include <iostream>
   #include "pcl/io/pcd_io.h"
   #include "pcl/point_types.h"

   int
     main (int argc, char** argv)
   {
     sensor_msgs::PointCloud2 cloud_blob;
     pcl::PointCloud<pcl::PointXYZ> cloud;

     if (pcl::io::loadPCDFile ("test_pcd.pcd", cloud_blob) == -1)
     {
       std::cerr << "Couldn't read file test_pcd.pcd" << std::endl;
       return (-1);
     }
     std::cerr << "Loaded " 
               << cloud_blob.width * cloud_blob.height 
               << " data points from test_pcd.pcd with the following fields: " 
               << pcl::getFieldsList (cloud_blob) 
               << std::endl;

     // Convert to the templated message type
     pcl::fromROSMsg (cloud_blob, cloud);

     for (size_t i = 0; i < cloud.points.size (); ++i)
       std::cerr << "    " << cloud.points[i].x 
                 << " " << cloud.points[i].y 
                 << " " << cloud.points[i].z << std::endl;

     return (0);
   }

The explanation
---------------

Now, let's break down the code piece by piece.

.. code-block:: cpp

   sensor_msgs::PointCloud2 cloud_blob;

creates a binary blob sensor_msgs/PointCloud2 message. Note that due to the
dynamic nature of point clouds, we prefer to read them as binary blobs, and
then convert to the actual representation that we want to use.


.. code-block:: cpp

   if (pcl::io::loadPCDFile ("test_pcd.pcd", cloud_blob) == -1)
   {
     ROS_ERROR ("Couldn't read file test_pcd.pcd");
     return (-1);
   }

loads the PointCloud data from disk (we assume that test_pcd.pcd has already
been created from the previous tutorial) into the binary blob.


.. code-block:: cpp

   pcl::fromROSMsg (cloud_blob, cloud);

converts the binary blob into the templated PointCloud format, here using
pcl::PointXYZ as the underlying point type.

Finally:

.. code-block:: cpp

   for (size_t i = 0; i < cloud.points.size (); ++i)
     std::cerr << "    " << cloud.points[i].x 
               << " " << cloud.points[i].y 
               << " " << cloud.points[i].z << std::endl;

is used to show the data that was loaded from file.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. code-block:: cmake

   add_executable (pcd_read pcd_read.cpp)
   target_link_libraries (pcd_read ${PCL_IO_LIBRARIES})

After you have made the executable, you can run it. Simply do::

  $ ./pcd_read 

You will see something similar to::

  Loaded 5 data points from test_pcd.pcd with the following fields: x y z
    0.35222 -0.15188 -0.1064
    -0.39741 -0.47311 0.2926
    -0.7319 0.6671 0.4413
    -0.73477 0.85458 -0.036173
    -0.4607 -0.27747 -0.91676

Note that if the file test_pcd.pcd does not exist (either it hasn't been
created or it has been erased), you should get an error message such as::

  Couldn't read file test_pcd.pcd
