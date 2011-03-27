.. _passthrough:

Filtering a PointCloud using a PassThrough filter
--------------------------------------------------

In this tutorial we will learn how to perform a simple filtering along a
specified dimension -- that is, cut off values that are either inside or
outside a given user range.

The code
--------

First, create a file, let's say, ``passthrough.cpp`` in your favorite
editor, and place the following inside it:

.. code-block:: cpp
   :linenos:

   #include <iostream>
   #include "pcl/io/pcd_io.h"
   #include "pcl/point_types.h"
   #include "pcl/filters/passthrough.h"
  
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
  
     // Create the filtering object
     pcl::PassThrough<pcl::PointXYZ> pass;
     pass.setInputCloud (cloud);
     pass.setFilterFieldName ("z");
     pass.setFilterLimits (0.0, 1.0);
     //pass.setFilterLimitsNegative (true);
     pass.filter (*cloud_filtered);
  
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

In the following lines, we define the Point Clouds structures, fill in the
input cloud, and display its content to screen.

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


Then, we create the PassThrough filter object, and set its parameters. The
filter field name is set to the z coordinate, and the accepted interval values
are set to (0.0;1.0).

.. code-block:: cpp

     pcl::PassThrough<pcl::PointXYZ> pass;
     pass.setInputCloud (cloud);
     pass.setFilterFieldName ("z");
     pass.setFilterLimits (0.0, 1.0);
     //pass.setFilterLimitsNegative (true);
     pass.filter (*cloud_filtered);

Finally we show the content of the filtered cloud.

.. code-block:: cpp

     std::cerr << "Cloud after filtering: " << std::endl;
     for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
       std::cerr << "    " << cloud_filtered->points[i].x << " " 
                           << cloud_filtered->points[i].y << " " 
                           << cloud_filtered->points[i].z << std::endl;

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. code-block:: cmake
   
   add_executable (passthrough passthrough.cpp)
   target_link_libraries (passthrough pcl_io pcl_filters)

After you have made the executable, you can run it. Simply do::

  $ ./passthrough

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

A graphical display of the filtering process is shown below. 

.. image:: images/passthrough_2.png

Note that the coordinate axis are represented as red (x), green (y), and blue
(z). The five points are represented with green as the points remaining after
filtering and red as the points that have been removed by the filter.

As an exercise, try uncommenting line::

  //pass.setFilterLimitsNegative (true);

and run the program again.

