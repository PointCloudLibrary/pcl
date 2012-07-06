.. _normal_estimation_using_integral_images:

Normal Estimation Using Integral Images
---------------------------------------

In this tutorial we will learn how to compute normals for an organized point
cloud using integral images. 


The code
--------

First, create a file, let's say, ``normal_estimation_using_integral_images.cpp`` in your favorite
editor, and place the following inside it:

.. code-block:: cpp
   :linenos:

	#include <pcl/io/io.h>
	#include <pcl/io/pcd_io.h>
	#include <pcl/features/integral_image_normal.h>
	#include <pcl/visualization/cloud_viewer.h>
		
	int 
	main ()
	{
		// load point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);
		
		// estimate normals
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
		ne.setMaxDepthChangeFactor(0.02f);
		ne.setNormalSmoothingSize(10.0f);
		ne.setInputCloud(cloud);
		ne.compute(*normals);

		// visualize normals
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		viewer.setBackgroundColor (0.0, 0.0, 0.5);
		viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
		
		while (!viewer.wasStopped ())
		{
		  viewer.spinOnce ();
		}
		return 0;
	}

The explanation
---------------

Now, let's break down the code piece by piece. In the first part we load a
point cloud from a file:

.. code-block:: cpp

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);

In the second part we create an object for the normal estimation and compute
the normals:

.. code-block:: cpp

	// estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud);
	ne.compute(*normals);

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

In the last part we visualize the point cloud and the corresponding normals:

.. code-block:: cpp

	// visualize normals
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor (0.0, 0.0, 0.5);
	viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
	
	while (!viewer.wasStopped ())
	{
	  viewer.spinOnce ();
	}


