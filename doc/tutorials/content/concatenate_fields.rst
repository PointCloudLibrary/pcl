.. _concatenate_fields:

Concatenate the fields of two Point Clouds
------------------------------------------

In this tutorial we will learn how to concatenating the fields (e.g.,
dimensions) of two different point clouds. The constraint imposed here is that
the number of points in the two datasets has to be equal.

The code
--------

First, create a file, let's say, ``concatenate_fields.cpp`` in your favorite
editor, and place the following code inside it:

.. code-block:: cpp
   :linenos:

   #include <iostream>
   #include "pcl/io/pcd_io.h"
   #include "pcl/point_types.h"
   
   int
     main (int argc, char** argv)
   {
     pcl::PointCloud<pcl::PointXYZ> cloud_a;
     pcl::PointCloud<pcl::Normal> cloud_b;
     pcl::PointCloud<pcl::PointNormal> cloud_c;
   
     // Fill in the cloud data
     cloud_a.width  = cloud_b.width  = 5;
     cloud_a.height = cloud_b.height = 1;
     cloud_a.points.resize (cloud_a.width * cloud_a.height);
     cloud_b.points.resize (cloud_b.width * cloud_b.height);
   
     for (size_t i = 0; i < cloud_a.points.size (); ++i)
     {
       cloud_a.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
       cloud_a.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
       cloud_a.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
     }
   
     for (size_t i = 0; i < cloud_b.points.size (); ++i)
     {
       cloud_b.points[i].normal[0] = 1024 * rand () / (RAND_MAX + 1.0);
       cloud_b.points[i].normal[1] = 1024 * rand () / (RAND_MAX + 1.0);
       cloud_b.points[i].normal[2] = 1024 * rand () / (RAND_MAX + 1.0);
     }
   
     std::cerr << "Cloud A: " << std::endl;
     for (size_t i = 0; i < cloud_a.points.size (); ++i)
       std::cerr << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;
   
     std::cerr << "Cloud B: " << std::endl;
     for (size_t i = 0; i < cloud_b.points.size (); ++i)
       std::cerr << "    " << cloud_b.points[i].normal[0] << " " << cloud_b.points[i].normal[1] << " " << cloud_b.points[i].normal[2] << std::endl;
   
     pcl::concatenateFields (cloud_a, cloud_b, cloud_c);
     std::cerr << "Cloud C: " << std::endl;
     for (size_t i = 0; i < cloud_c.points.size (); ++i)
       std::cerr << "    " <<
         cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " <<
         cloud_c.points[i].normal[0] << " " << cloud_c.points[i].normal[1] << " " << cloud_c.points[i].normal[2] << std::endl;
   
     return (0);
   }

The explanation
---------------

Now, let's break down the code piece by piece.


.. code-block:: cpp

   pcl::PointCloud<pcl::PointXYZ> cloud_a;
   pcl::PointCloud<pcl::Normal> cloud_b;
   pcl::PointCloud<pcl::PointNormal> cloud_c;

define the three Point Clouds: two inputs (cloud_a and cloud_b), one output
(cloud_c).

The lines:

.. code-block:: cpp

   // Fill in the cloud data
   cloud_a.width  = cloud_b.width  = 5;
   cloud_a.height = cloud_b.height = 1;
   cloud_a.points.resize (cloud_a.width * cloud_a.height);
   cloud_b.points.resize (cloud_b.width * cloud_b.height);
   
   for (size_t i = 0; i < cloud_a.points.size (); ++i)
   {
     cloud_a.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
     cloud_a.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
     cloud_a.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
   }
   
   for (size_t i = 0; i < cloud_b.points.size (); ++i)
   {
     cloud_b.points[i].normal[0] = 1024 * rand () / (RAND_MAX + 1.0);
     cloud_b.points[i].normal[1] = 1024 * rand () / (RAND_MAX + 1.0);
     cloud_b.points[i].normal[2] = 1024 * rand () / (RAND_MAX + 1.0);
   }

fill in the data for the two input point clouds.

Then, lines:

.. code-block:: cpp

   std::cerr << "Cloud A: " << std::endl;
   for (size_t i = 0; i < cloud_a.points.size (); ++i)
     std::cerr << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;
  
   std::cerr << "Cloud B: " << std::endl;
   for (size_t i = 0; i < cloud_b.points.size (); ++i)
     std::cerr << "    " << cloud_b.points[i].normal[0] << " " << cloud_b.points[i].normal[1] << " " << cloud_b.points[i].normal[2] << std::endl;

display the content of cloud_a and cloud_b to screen.

In line:

.. code-block:: cpp

   pcl::concatenateFields (cloud_a, cloud_b, cloud_c);

we create cloud_c by concatenating the fields of cloud_a and cloud_b together.

Finally:

.. code-block:: cpp

   std::cerr << "Cloud C: " << std::endl;
   for (size_t i = 0; i < cloud_c.points.size (); ++i)
     std::cerr << "    " <<
       cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " <<
       cloud_c.points[i].normal[0] << " " << cloud_c.points[i].normal[1] << " " << cloud_c.points[i].normal[2] << std::endl;

is used to show the content of cloud_c.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. code-block:: cmake
   
   add_executable (concatenate_fields concatenate_fields.cpp)
   target_link_libraries (concatenate_fields pcl_io)

After you have made the executable, you can run it. Simply do::

  $ ./concatenate_fields

You will see something similar to::

  Cloud A: 
      0.352222 -0.151883 -0.106395
      -0.397406 -0.473106 0.292602
      -0.731898 0.667105 0.441304
      -0.734766 0.854581 -0.0361733
      -0.4607 -0.277468 -0.916762
  Cloud B: 
      0.183749 0.968809 0.512055
      -0.998983 -0.463871 0.691785
      0.716053 0.525135 -0.523004
      0.439387 0.56706 0.905417
      -0.579787 0.898706 -0.504929
  Cloud C: 
      0.352222 -0.151883 -0.106395 0.183749 0.968809 0.512055
      -0.397406 -0.473106 0.292602 -0.998983 -0.463871 0.691785
      -0.731898 0.667105 0.441304 0.716053 0.525135 -0.523004
      -0.734766 0.854581 -0.0361733 0.439387 0.56706 0.905417
      -0.4607 -0.277468 -0.916762 -0.579787 0.898706 -0.504929

