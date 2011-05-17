.. _normal_estimation_using_integral_images:

Normal Estimation Using Integral Images
---------------------------------------

In this tutorial we will learn how to compute normals for an organized point
cloud using integral images. 


The code
--------

First, create a file, let's say, ``normal_estimation.cpp`` in your favorite
editor, and place the following inside it:

.. code-block:: cpp
   :linenos:

    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <pcl/features/integral_image_normal.h>

    int
    main (int argc, char** argv)
    {
      pcl::PointCloud<pcl::PointXYZ> cloud;

      // ... fill point cloud...

      cloud.width = 640;
      cloud.height = 480;
      cloud.points.resize (cloud.width * cloud.height);

      for (int ri = 0; ri < cloud.height; ++ri)
      {
        for (int ci = 0; ci < cloud.width; ++ci)
        {
          const float depth = 0.2f*static_cast<float> (rand ()) / static_cast<float>(RAND_MAX) + 1.0f;
          cloud.points (ri, ci).x = (ci - 320) * depth;
          cloud.points (ri, ci).y = (ri - 240) * depth;
          cloud.points (ri, ci).z = depth;
         }
       }

      // Estimate normals
      pcl::IntegralImageNormalEstimation ne;

      pcl::PointCloud<pcl::Normal> normals;
      ne.compute (cloud, normals, 0.02f, 10.0f, ne.AVERAGE_DEPTH_CHANGE);

      return (0);
    }

The explanation
---------------

Now, let's break down the code piece by piece. In the first part we create a
random point cloud for which we estimate the normals:

.. code-block:: cpp

    pcl::PointCloud<pcl::PointXYZ> cloud;
    // ... fill point cloud...

    cloud.width = 640;
    cloud.height = 480;
    cloud.points.resize (cloud.width*cloud.height);

    for (int ri = 0; ri < cloud.height; ++ri)
    {
      for (int ci = 0; ci < cloud.width; ++ci)
      {
        const float depth = 0.2f*static_cast<float> (rand ()) / static_cast<float>(RAND_MAX) + 1.0f;
        cloud.points (ri, ci).x = (ci - 320) * depth;
        cloud.points (ri, ci).y = (ri - 240) * depth;
        cloud.points (ri, ci).z = depth;
       }
     }

In the second part we create an object for the normal estimation and compute
the normals:

.. code-block:: cpp

    // Estimate normals
    pcl::IntegralImageNormalEstimation ne;

    pcl::PointCloud<pcl::Normal> normals;
    ne.compute (cloud, normals, 0.02f, 10.0f, ne.AVERAGE_DEPTH_CHANGE);

The following normal estimation methods are available:

.. code-block:: cpp

     enum NormalEstimationMethod
     {
       COVARIANCE_MATRIX,
       AVERAGE_3D_GRADIENT,
       AVERAGE_DEPTH_CHANGE
     };
	 
The COVARIANCE_MATRIX mode creates 9 integral images to compute the normal for
a specific point from the covariance matrix of its local neighborhood. The
AVERAGE_3D_GRADIENT mode creates 6 integral images to compute smoothed versions
of horizontal and vertical 3D gradients and computes the normals using the
cross-product between these two gradients. The AVERAGE_DEPTH_CHANGE mode
creates only a single integral image and computes the normals from the average
depth changes.

