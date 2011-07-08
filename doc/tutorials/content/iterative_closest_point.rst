.. _iterative_closest_point:

How to use iterative closest point
----------------------------------

This document demonstrates using the Iterative Closest Point algorithm in your code which can determine if one PointCloud is just a rigid transformation of another by minimizing the distances between the points of two pointclouds and rigidly transforming them.

The code
--------

.. literalinclude:: sources/iterative_closest_point/iterative_closest_point.cpp
   :language: cpp
   :linenos:

   .. code-block:: cpp

		#include <iostream>
		#include <pcl/io/pcd_io.h>
		#include <pcl/point_types.h>
		#include <pcl/registration/icp.h>

		int
		 main (int argc, char** argv)
		{
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new
		    pcl::PointCloud<pcl::PointXYZ>);
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new
		    pcl::PointCloud<pcl::PointXYZ>);
		
		  // Fill in the CloudIn data
		  cloud_in->width    = 5;
		  cloud_in->height   = 1;
		  cloud_in->is_dense = false;
		  cloud_in->points.resize (cloud_in->width * cloud_in->height);
		  for (size_t i = 0; i < cloud_in->points.size (); ++i)
		  {
		    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
		    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
		    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
		  }
		  std::cout << "Saved " << cloud_in->points.size () <<
		    " data points to input:" << std::endl;
		  for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout <<
		    "    " << cloud_in->points[i].x << " " << cloud_in->points[i].y <<
		    " " << cloud_in->points[i].z << std::endl;
		  *cloud_out = *cloud_in;
		  std::cout << "size:" << cloud_out->points.size() << std::endl;
		  for (size_t i = 0; i < cloud_in->points.size (); ++i)
		    cloud_out->points[i].x = cloud_in->points[i].x + .7;
		  std::cout << "Transformed " << cloud_in->points.size () <<
		    " data points:" << std::endl;
		  for (size_t i = 0; i < cloud_out->points.size (); ++i)
		    std::cout << "    " << cloud_out->points[i].x << " " <<
		    cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
		  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		  icp.setInputCloud(cloud_in);
		  icp.setInputTarget(cloud_out);
		  pcl::PointCloud<pcl::PointXYZ> Final;
		  icp.align(Final);
		  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		  icp.getFitnessScore() << std::endl;
		  std::cout << icp.getFinalTransformation() << std::endl;

		 return (0);
		}



The explanation
---------------

   Now, let's breakdown this code piece by piece.

   .. code-block:: cpp

      #include <iostream>
      #include <pcl/io/pcd_io.h>
      #include <pcl/point_types.h>
      #include <pcl/registration/icp.h>

   these are the header files that contain the definitions for all of the classes which we will use.

   .. code-block:: cpp

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new
		  pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new
		  pcl::PointCloud<pcl::PointXYZ>);

   Creates two pcl::PointCloud<pcl::PointXYZ> boost shared pointers and initializes them. The type of each point is set to PointXYZ in the pcl namespace which is:

   .. code-block:: cpp

      // \brief A point structure representing Euclidean xyz coordinates.
      struct PointXYZ
      {
        float x;
        float y;
        float z;
      };

   The lines:

   .. code-block:: cpp

		// Fill in the cloud_in data
		cloud_in->width    = 5;
		cloud_in->height   = 1;
		cloud_in->is_dense = false;
		cloud_in->points.resize (cloud_in->width * cloud_in->height);
		for (size_t i = 0; i < cloud_in->points.size (); ++i)
		{
		  cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
		  cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
		  cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
		}
 		std::cout << "Saved " << cloud_in->points.size () <<
		" data points to input:" << std::endl;
  		for (size_t i = 0; i < cloud_in->points.size (); ++i)
		  std::cout <<"    " << cloud_in->points[i].x << " " <<
		  cloud_in->points[i].y << " " << cloud_in->points[i].z <<
		  std::endl;

   fills in the PointCloud structure with random point values, and sets the appropriate parameters (width, height, is_dense).  Also, outputs the number of points saved, and their actual data values.
   
   Then:

   .. code-block:: cpp

		for (size_t i = 0; i < cloud_in->points.size (); ++i)
		  cloud_out->points[i].x = cloud_in->points[i].x + .7;
		std::cout << "Transformed " << cloud_in->points.size () <<
		  " datapoints:"<< std::endl;
		for (size_t i = 0; i < cloud_out->points.size (); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " <<
		  cloud_out->points[i].y << " " << cloud_out->points[i].z <<
		  std::endl;

   performs a simple rigid transform on the pointcloud and then again outputs the data values.

   .. code-block:: cpp

		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputCloud(cloud_in);
		icp.setInputTarget(cloud_out);

   This creates an instance of an IterativeClosestPoint and gives it some useful information.  "icp.setInputCloud(cloud_in);" sets cloud_in as the PointCloud to begin from and "icp.setInputTarget(cloud_out);" sets cloud_out as the PointCloud which we want cloud_in to look like.

   Next,

   .. code-block:: cpp

		pcl::PointCloud<pcl::PointXYZ> Final;
		icp.align(Final);
		std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;

   Creates a pcl::PointCloud<pcl::PointXYZ> to which the IterativeClosestPoint can save the resultant cloud after applying the algorithm.  If the two PointClouds align correctly (meaning they are both the same cloud merely with some kind of rigid transformation applied to one of them) then icp.hasConverged() = 1 (true).  It then outputs the fitness score of the final transformation and some information about it.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/iterative_closest_point/CMakeFiles.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./iterative_closest_point

You will see something similar to::

  Saved 5 data points to input:
   0.352222 -0.151883 -0.106395
  -0.397406 -0.473106 0.292602
  -0.731898 0.667105 0.441304
  -0.734766 0.854581 -0.0361733
  -0.4607 -0.277468 -0.916762
  size:5
  Transformed 5 data points:
   1.05222 -0.151883 -0.106395
   0.302594 -0.473106 0.292602
  -0.0318983 0.667105 0.441304
  -0.0347655 0.854581 -0.0361733
	0.2393 -0.277468 -0.916762
  [pcl::SampleConsensusModelRegistration::setInputCloud] Estimated a sample
  selection distance threshold of: 0.200928
  [pcl::IterativeClosestPoint::computeTransformation] Number of
  correspondences 4 [80.000000%] out of 5 points [100.0%], RANSAC rejected:
  1 [20.000000%].
  [pcl::IterativeClosestPoint::computeTransformation] Convergence reached.
  Number of iterations: 1 out of 0. Transformation difference: 0.700001
  has converged:1 score: 1.95122e-14
            1  4.47035e-08 -3.25963e-09          0.7
  2.98023e-08            1 -1.08499e-07 -2.98023e-08
  1.30385e-08 -1.67638e-08            1  1.86265e-08
            0            0            0            1
