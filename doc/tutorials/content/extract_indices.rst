.. _extract_indices:

Extracting indices from a PointCloud
------------------------------------

In this tutorial we will learn how to extract a set of indices given by a
segmentation algorithm from a point cloud. In order to not complicate the
tutorial, the segmentation algorithm is not explained here. Please check
the :ref:`planar_segmentation` tutorial for more information.


.. raw:: html

   <iframe title="Extracting indices from a PointCloud" width="480" height="390" src="http://www.youtube.com/embed/ZTK7NR1Xx4c?rel=0" frameborder="0" allowfullscreen></iframe>

The code
--------

First, download the dataset `table_scene_lms400.pcd
<http://dev.pointclouds.org/attachments/download/157/table_scene_lms400.pcd>`_
and save it somewhere to disk.

Then, create a file, let's say, ``extract_indices.cpp`` in your favorite
editor, and place the following inside it:

.. code-block:: cpp
   :linenos:

   #include <iostream>
   #include "pcl/ModelCoefficients.h"
   #include "pcl/io/pcd_io.h"
   #include "pcl/point_types.h"
   #include "pcl/sample_consensus/method_types.h"
   #include "pcl/sample_consensus/model_types.h"
   #include "pcl/segmentation/sac_segmentation.h"
   #include "pcl/filters/voxel_grid.h"
   #include "pcl/filters/extract_indices.h"

   int
     main (int argc, char** argv)
   {
     sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2), cloud_filtered_blob (new sensor_msgs::PointCloud2);
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

     // Fill in the cloud data
     pcl::PCDReader reader;
     reader.read ("table_scene_lms400.pcd", *cloud_blob);

     std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

     // Create the filtering object: downsample the dataset using a leaf size of 1cm
     pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
     sor.setInputCloud (cloud_blob);
     sor.setLeafSize (0.01, 0.01, 0.01);
     sor.filter (*cloud_filtered_blob);

     // Convert to the templated PointCloud
     pcl::fromROSMsg (*cloud_filtered_blob, *cloud_filtered);

     std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
     
     // Write the downsampled version to disk
     pcl::PCDWriter writer;
     writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
     pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
     // Create the segmentation object
     pcl::SACSegmentation<pcl::PointXYZ> seg;
     // Optional
     seg.setOptimizeCoefficients (true);
     // Mandatory
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setMaxIterations (1000);
     seg.setDistanceThreshold (0.01);

     // Create the filtering object
     pcl::ExtractIndices<pcl::PointXYZ> extract;

     int i = 0, nr_points = cloud_filtered->points.size ();
     // While 30% of the original cloud is still there
     while (cloud_filtered->points.size () > 0.3 * nr_points)
     {
       // Segment the largest planar component from the remaining cloud
       seg.setInputCloud (cloud_filtered);
       seg.segment (*inliers, *coefficients);
       if (inliers->indices.size () == 0)
       {
         std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
         break;
       }

       // Extract the inliers
       extract.setInputCloud (cloud_filtered);
       extract.setIndices (inliers);
       extract.setNegative (false);
       extract.filter (*cloud_p);
       std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

       std::stringstream ss;
       ss << "table_scene_lms400_plane_" << i << ".pcd";
       writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

       // Create the filtering object
       extract.setNegative (true);
       extract.filter (*cloud_filtered);

       i++;
     }

     return (0);
   }

The explanation
---------------

Now, let's break down the code piece by piece, skipping the obvious.

After the data has been loaded from the input .PCD file, we create a
*VoxelGrid* filter, to downsample the data. The rationale behind data
downsampling here is just to speed things up -- less points means less time
needed to spend within the segmentation loop.

.. code-block:: cpp

      pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
      sor.setInputCloud (cloud_blob);
      sor.setLeafSize (0.01, 0.01, 0.01);
      sor.filter (*cloud_filtered_blob);

The next block of code deals with the parametric segmentation. To keep the
tutorial simple, its its explanation will be skipped for now. Please see the
**segmentation** tutorials (in particular :ref:`planar_segmentation`) for more
information.

.. code-block:: cpp

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (1000);
      seg.setDistanceThreshold (0.01);

The line

.. code-block:: cpp

      pcl::ExtractIndices<pcl::PointXYZ> extract;

and

.. code-block:: cpp

        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);

represent the actual indices extraction filter. To process multiple models, we
run the process in a loop, and after each model is extracted, we go back to
obtain the remaining points, and iterate. The *inliers* are obtained from the segmentation process, as follows:

.. code-block:: cpp

       seg.setInputCloud (cloud_filtered);
       seg.segment (*inliers, *coefficients);



Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. code-block:: cmake
   
   add_executable (extract_indices extract_indices.cpp)
   target_link_libraries (extract_indices ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})

After you have made the executable, you can run it. Simply do::

  $ ./extract_indices

You will see something similar to::

  PointCloud before filtering: 460400 data points.
  PointCloud after filtering: 41049 data points.
  PointCloud representing the planar component: 20164 data points.
  PointCloud representing the planar component: 12129 data points.

