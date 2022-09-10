.. _normal_estimation_using_integral_images:

Normal Estimation Using Integral Images
---------------------------------------

In this tutorial we will learn how to compute normals for an organized point
cloud using integral images. 


The code
--------

First, download the dataset `table_scene_mug_stereo_textured.pcd
<https://github.com/PointCloudLibrary/pcl/raw/master/test/table_scene_mug_stereo_textured.pcd>`_
and save it somewhere to disk.

Then, create a file, let's say, ``normal_estimation_using_integral_images.cpp`` in your favorite
editor, and place the following inside it:

.. literalinclude:: sources/normal_estimation_using_integral_images/normal_estimation_using_integral_images.cpp
   :language: cpp
   :linenos:
   
The explanation
---------------

Now, let's break down the code piece by piece. In the first part we load a
point cloud from a file:

.. literalinclude:: sources/normal_estimation_using_integral_images/normal_estimation_using_integral_images.cpp
   :language: cpp
   :lines: 11-12

In the second part we create an object for the normal estimation and compute
the normals:

.. literalinclude:: sources/normal_estimation_using_integral_images/normal_estimation_using_integral_images.cpp
   :language: cpp
   :lines: 14-22

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

.. literalinclude:: sources/normal_estimation_using_integral_images/normal_estimation_using_integral_images.cpp
   :language: cpp
   :lines: 24-32

