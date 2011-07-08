.. _radius_outlier_removal:

Removing outliers using a RadiusOutlierRemoval filter
-----------------------------------------------------

This document demonstrates how to create and use a RadiusOutlierRemoval object that can be used to remove points from a PointCloud that do not have a given number of neighbors within a specific radius from their location.

The code
--------

First, create a file, let's say, radius_outlier_removal.cpp in your favorite editor, and place the following inside it:

.. literalinclude:: sources/radius_outlier_removal/radius_outlier_removal.cpp
   :language: cpp
   :linenos:

	.. code-block:: cpp

		#include <iostream>
		#include <pcl/point_types.h>
		#include <pcl/filters/radius_outlier_removal.h>

		int
		 main (int argc, char** argv)
		{
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

		  // Fill in the cloud data
		  cloud->width  = 5;
		  cloud->height = 1;
		  cloud->points.resize (cloud->width * cloud->height);
		
		  for (size_t i = 0; i < cloud->points.size (); ++i)
		  {
		    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
		    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
		    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
		  }

		  std::cerr << "Cloud before filtering: " << std::endl;
		  for (size_t i = 0; i < cloud->points.size (); ++i)
		    std::cerr << "    " << cloud->points[i].x << " "
		                        << cloud->points[i].y << " "
		                        << cloud->points[i].z << std::endl;

		  // build the filter
		  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		  outrem.setInputCloud(cloud);
		  outrem.setRadiusSearch(0.8);
		  outrem.setMinNeighborsInRadius (2);

		  // apply filter
		  outrem.filter (*cloud_filtered);

		  // display pointcloud after filtering
		  std::cerr << "Cloud after filtering: " << std::endl;
		  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		    std::cerr << "    " << cloud_filtered->points[i].x << " "
		                        << cloud_filtered->points[i].y << " "
		                        << cloud_filtered->points[i].z << std::endl;
		  return (0);
		}


The explanation
---------------

Now, let's break down the code piece by piece.

In the following lines, we define the PointCloud structures, fill in the input cloud and display it's contents to the screen.

   .. code-block:: cpp

		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

		  // Fill in the cloud data
		  cloud->width  = 5;
		  cloud->height = 1;
		  cloud->points.resize (cloud->width * cloud->height);
		
		  for (size_t i = 0; i < cloud->points.size (); ++i)
		  {
		    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
		    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
		    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
		  }

		  std::cerr << "Cloud before filtering: " << std::endl;
		  for (size_t i = 0; i < cloud->points.size (); ++i)
		    std::cerr << "    " << cloud->points[i].x << " "
		                        << cloud->points[i].y << " "
		                        << cloud->points[i].z << std::endl;

   Then, we create the RadiusOutlierRemoval filter object, set it's parameters and apply it to our input cloud.  The radius of search is set to 0.8, and a point must have a minimum of 2 neighbors in that radius to be kept as part of the PointCloud.

   .. code-block:: cpp

		  // build the filter
		  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		  outrem.setInputCloud(cloud);
		  outrem.setRadiusSearch(0.8);
		  outrem.setMinNeighborsInRadius (2);

		  // apply filter
		  outrem.filter (*cloud_filtered);

   This final block of code just displays the contents of the resulting PointCloud to the screen.

   .. code-block:: cpp

		  // display pointcloud after filtering
		  std::cerr << "Cloud after filtering: " << std::endl;
		  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		    std::cerr << "    " << cloud_filtered->points[i].x << " "
		                        << cloud_filtered->points[i].y << " "
		                        << cloud_filtered->points[i].z << std::endl;

Compiling and running the program
---------------------------------

   Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/radius_outlier_removal/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./conditioinal_removal

You will see something similar to::

  Cloud before filtering: 
       0.352222 -0.151883 -0.106395
	   -0.397406 -0.473106 0.292602
	   -0.731898 0.667105 0.441304
	   -0.734766 0.854581 -0.0361733
	   -0.4607 -0.277468 -0.916762
  Cloud after filtering: 
	   -0.731898 0.667105 0.441304
	   -0.734766 0.854581 -0.0361733

