.. _planar_segmentation:

Planar model segmentation
-------------------------

In this tutorial we will learn how do a simple planar segmentation of a set of
points, that is find all the points within a point cloud that support a planar
model. This tutorial supports the :ref:`extract_indices` tutorial, presented in
the **filtering** section.

.. raw:: html

   <iframe title="Extracting indices from a PointCloud" width="480" height="390" src="http://www.youtube.com/embed/ZTK7NR1Xx4c?rel=0" frameborder="0" allowfullscreen></iframe>

The code
--------

First, create a file, let's say, ``planar_segmentation.cpp`` in your favorite
editor, and place the following inside it:

.. code-block:: cpp
   :linenos:

   #include "pcl/ModelCoefficients.h"
   #include "pcl/io/pcd_io.h"
   #include "pcl/point_types.h"
   #include "pcl/sample_consensus/method_types.h"
   #include "pcl/sample_consensus/model_types.h"
   #include "pcl/segmentation/sac_segmentation.h"
   
   int
     main (int argc, char** argv)
   {
     pcl::PointCloud<pcl::PointXYZ> cloud;
   
     // Fill in the cloud data
     cloud.width  = 15;
     cloud.height = 1;
     cloud.points.resize (cloud.width * cloud.height);
   
     // Generate the data
     for (size_t i = 0; i < cloud.points.size (); ++i)
     {
       cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
       cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
       cloud.points[i].z = 1.0;
     }
   
     // Set a few outliers
     cloud.points[0].z = 2.0;
     cloud.points[3].z = -2.0;
     cloud.points[6].z = 4.0;
   
     std::cerr << "Point cloud data: " << cloud.points.size () << " points" << std::endl;
     for (size_t i = 0; i < cloud.points.size (); ++i)
       std::cerr << "    " << cloud.points[i].x << " " 
                           << cloud.points[i].y << " " 
                           << cloud.points[i].z << std::endl;
   
     pcl::ModelCoefficients coefficients;
     pcl::PointIndices inliers;
     // Create the segmentation object
     pcl::SACSegmentation<pcl::PointXYZ> seg;
     // Optional
     seg.setOptimizeCoefficients (true);
     // Mandatory
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setDistanceThreshold (0.01);
   
     seg.setInputCloud (cloud.makeShared ());
     seg.segment (inliers, coefficients);
   
     if (inliers.indices.size () == 0)
     {
       ROS_ERROR ("Could not estimate a planar model for the given dataset.");
       return (-1);
     }
   
     std::cerr << "Model coefficients: " << coefficients.values[0] << " " 
                                         << coefficients.values[1] << " "
                                         << coefficients.values[2] << " " 
                                         << coefficients.values[3] << std::endl;
   
     std::cerr << "Model inliers: " << inliers.indices.size () << std::endl;
     for (size_t i = 0; i < inliers.indices.size (); ++i)
       std::cerr << inliers.indices[i] << "    " << cloud.points[inliers.indices[i]].x << " "
                                                 << cloud.points[inliers.indices[i]].y << " "
                                                 << cloud.points[inliers.indices[i]].z << std::endl;
   
     return (0);
   }

The explanation
---------------

Now, let's break down the code piece by piece.

Lines:

.. code-block:: cpp

   #include "pcl/sample_consensus/method_types.h"
   #include "pcl/sample_consensus/model_types.h"
   #include "pcl/segmentation/sac_segmentation.h"

import the Sample Consensus model fitting object, and the model and method
types. As of PCL 0.1.3 (04/15/2010), the following models are supported:

.. code-block:: cpp

   const static int SACMODEL_PLANE          = 0;
   const static int SACMODEL_LINE           = 1;
   const static int SACMODEL_CIRCLE2D       = 2;
   const static int SACMODEL_CIRCLE3D       = 3;
   const static int SACMODEL_SPHERE         = 4;
   const static int SACMODEL_CYLINDER       = 5;
   const static int SACMODEL_CONE           = 6;
   const static int SACMODEL_TORUS          = 7;
   const static int SACMODEL_ORIENTED_LINE  = 8;
   const static int SACMODEL_ORIENTED_PLANE = 9;
   const static int SACMODEL_PARALLEL_LINES = 10;
   const static int SACMODEL_NORMAL_PLANE   = 11;

and the following robust estimator methods:

.. code-block:: cpp

   const static int SAC_RANSAC  = 0;
   const static int SAC_LMEDS   = 1;
   const static int SAC_MSAC    = 2;
   const static int SAC_RRANSAC = 3;
   const static int SAC_RMSAC   = 4;
   const static int SAC_MLESAC  = 5;

Lines:

.. code-block:: cpp

  pcl::PointCloud<pcl::PointXYZ> cloud;
  
  // Fill in the cloud data
  cloud.width  = 15;
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);
  
  // Generate the data
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
    cloud.points[i].z = 1.0;
  }
  
  // Set a few outliers
  cloud.points[0].z = 2.0;
  cloud.points[3].z = -2.0;
  cloud.points[6].z = 4.0;
  
  std::cerr << "Point cloud data: " << cloud.points.size () << " points" << std::endl;
  for (size_t i = 0; i < cloud.points.size (); ++i)
     std::cerr << "    " << cloud.points[i].x << " " 
                         << cloud.points[i].y << " " 
                         << cloud.points[i].z << std::endl;
   
create the point cloud structure, fill in the respective values, and display
the content on screen. Note that for the purpose of this tutorial, we manually
added a few outliers in the data, by setting their z values different from 0.

Then, lines:

.. code-block:: cpp

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (inliers, coefficients);

create the SACSegmentation object and set the model and method type, together
with the desired distance to the model threshold. In this tutorial, we will use
the RANSAC method (pcl::SAC_RANSAC) as the robust estimator of choice.

Finally:

.. code-block:: cpp

     std::cerr << "Model coefficients: " << coefficients.values[0] << " " 
                                         << coefficients.values[1] << " "
                                         << coefficients.values[2] << " " 
                                         << coefficients.values[3] << std::endl;

are used to show the contents of the inlier set, together with the estimated
model parameters.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. code-block:: cmake
   
   add_executable (planar_segmentation planar_segmentation.cpp)
   target_link_libraries (planar_segmentation pcl_io pcl_filters)

After you have made the executable, you can run it. Simply do::

  $ ./planar_segmentation

You will see something similar to::

  Point cloud data: 15 points
      0.352222 -0.151883 2
      -0.106395 -0.397406 1
      -0.473106 0.292602 1
      -0.731898 0.667105 -2
      0.441304 -0.734766 1
      0.854581 -0.0361733 1
      -0.4607 -0.277468 4
      -0.916762 0.183749 1
      0.968809 0.512055 1
      -0.998983 -0.463871 1
      0.691785 0.716053 1
      0.525135 -0.523004 1
      0.439387 0.56706 1
      0.905417 -0.579787 1
      0.898706 -0.504929 1
  [pcl::SACSegmentation::initSAC] Setting the maximum number of iterations to 50
  Model coefficients: 0 0 1 -1
  Model inliers: 12
  1    -0.106395 -0.397406 1
  2    -0.473106 0.292602 1
  4    0.441304 -0.734766 1
  5    0.854581 -0.0361733 1
  7    -0.916762 0.183749 1
  8    0.968809 0.512055 1
  9    -0.998983 -0.463871 1
  10    0.691785 0.716053 1
  11    0.525135 -0.523004 1
  12    0.439387 0.56706 1
  13    0.905417 -0.579787 1
  14    0.898706 -0.504929 1

A graphical display of the segmentation process is shown below.

.. image:: images/planar_segmentation_2.png

Note that the coordinate axis are represented as red (x), green (y), and blue
(z). The points are represented with red as the outliers, and green as the
inliers of the planar model found.

