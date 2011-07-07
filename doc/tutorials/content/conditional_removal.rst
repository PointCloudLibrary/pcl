.. _conditional_removal:

How to use conditional removal
------------------------------

This document demonstrates how to use the ConditionalRemoval filter to remove points from a PointCloud that do no statisfy a specific or multiple conditions.

The code
--------

First, create a file, let's say, conditinoal_removal.cpp in you favorite editor, and place the following inside it:

.. literalinclude:: sources/conditional_removal/conditional_removal.cpp
   :language: cpp
   :linenos:

	.. code-block:: cpp

		#include <iostream>
		#include <pcl/point_types.h>
		#include <pcl/filters/conditional_removal.h>

		int
		 main (int argc, char** argv)
		{
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new
		    pcl::PointCloud<pcl::PointXYZ>);
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new
		    pcl::PointCloud<pcl::PointXYZ>);

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

		  // build the condition
		  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
		                  pcl::ConditionAnd<pcl::PointXYZ> ());
		  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
		      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
		  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
		      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));

		  // build the filter
		  pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
		  condrem.setInputCloud (cloud);
		  condrem.setKeepOrganized(true);
		
		  // apply the filter
		  condrem.filter (*cloud_filtered);
		
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

In the following Lines, we define the PointCloud structures, fill in the input cloud, and display it's content to screen.

	.. code-block:: cpp

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

   Then, we create the condition which a given point must satisfy so that it remains in our PointCloud.  To do this we must add two comparisons to the conditon, greater than 0.0, and less than 0.8.  This condition is then used to build the filter. 

	.. code-block:: cpp

		  // build the condition
		  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
		                  pcl::ConditionAnd<pcl::PointXYZ> ());
		  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
		      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
		  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
		      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));

		  // build the filter
		  pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
		  condrem.setInputCloud (cloud);
		  condrem.setKeepOrganized(true);

   This last bit of code just applies the filter to our original PointCloud, and removes all of the points that do not satisfy the conditions we specified.  Then it outputs all of the points remaining in the PointCloud.

	.. code-block:: cpp

		  // apply the filter
		  condrem.filter (*cloud_filtered);
		
		  std::cerr << "Cloud after filtering: " << std::endl;
		  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		    std::cerr << "    " << cloud_filtered->points[i].x << " "
		                        << cloud_filtered->points[i].y << " "
		                        << cloud_filtered->points[i].z << std::endl;


Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/conditional_removal/CMakeLists.txt
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
		-0.397406 -0.473106 0.292602
		-0.731898 0.667105 0.441304

