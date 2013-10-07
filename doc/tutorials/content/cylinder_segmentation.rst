.. _cylinder_segmentation:

Cylinder model segmentation
---------------------------

This tutorial exemplifies how to run a Sample Consensus segmentation for
cylindrical models. To make the example a bit more practical, the following
operations are applied to the input dataset (in order):

* data points further away than 1.5 meters are filtered
* surface normals at each point are estimated
* a plane model (describing the table in our demo dataset) is segmented and saved to disk
* a cylindrical model (describing the mug in our demo dataset) is segmented and saved to disk

.. raw:: html

   <iframe title="Cylinder model segmentation" width="480" height="390" src="http://www.youtube.com/embed/SjbEDEGAeTk?rel=0" frameborder="0" allowfullscreen></iframe>

.. note:: 
   The cylindrical model is not perfect due to the presence of noise in the data.

The code
--------

First, download the dataset `table_scene_mug_stereo_textured.pcd
<https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_mug_stereo_textured.pcd>`_
and save it somewhere to disk.

Then, create a file, let's say, ``cylinder_segmentation.cpp`` in your favorite
editor, and place the following inside it:

.. literalinclude:: sources/cylinder_segmentation/cylinder_segmentation.cpp
   :language: cpp
   :linenos:

The explanation
---------------

The only relevant lines are the lines below, as the other operations are
already described in the other tutorials.

.. literalinclude:: sources/cylinder_segmentation/cylinder_segmentation.cpp
   :language: cpp
   :lines: 83-91

   
As seen, we're using a RANSAC robust estimator to obtain the cylinder
coefficients, and we're imposing a distance threshold from each inlier point to
the model no greater than 5cm. In addition, we set the surface normals
influence to a weight of 0.1, and we limit the radius of the cylindrical model
to be smaller than 10cm.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/cylinder_segmentation/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./cylinder_segmentation

You will see something similar to::

  PointCloud has: 307200 data points.
  PointCloud after filtering has: 139897 data points.
  [pcl::SACSegmentationFromNormals::initSACModel] Using a model of type: SACMODEL_NORMAL_PLANE
  [pcl::SACSegmentationFromNormals::initSACModel] Setting normal distance weight to 0.100000
  [pcl::SACSegmentationFromNormals::initSAC] Using a method of type: SAC_RANSAC with a model threshold of 0.030000
  [pcl::SACSegmentationFromNormals::initSAC] Setting the maximum number of iterations to 100
  Plane coefficients: header: 
    seq: 0
    stamp: 0.000000000
    frame_id: 
  values[]
    values[0]: -0.0161854
    values[1]: 0.837724
    values[2]: 0.545855
    values[3]: -0.528787

  PointCloud representing the planar component: 117410 data points.
  [pcl::SACSegmentationFromNormals::initSACModel] Using a model of type: SACMODEL_CYLINDER
  [pcl::SACSegmentationFromNormals::initSACModel] Setting radius limits to 0.000000/0.100000
  [pcl::SACSegmentationFromNormals::initSACModel] Setting normal distance weight to 0.100000
  [pcl::SACSegmentationFromNormals::initSAC] Using a method of type: SAC_RANSAC with a model threshold of 0.050000
  [pcl::SampleConsensusModelCylinder::optimizeModelCoefficients] LM solver finished with exit code 2, having a residual norm of 0.322616. 
  Initial solution: 0.0452105 0.0924601 0.790215 0.20495 -0.721649 -0.661225 0.0422902 
  Final solution: 0.0452105 0.0924601 0.790215 0.20495 -0.721649 -0.661225 0.0396354
  Cylinder coefficients: header: 
    seq: 0
    stamp: 0.000000000
    frame_id: 
  values[]
    values[0]: 0.0452105
    values[1]: 0.0924601
    values[2]: 0.790215
    values[3]: 0.20495
    values[4]: -0.721649
    values[5]: -0.661225
    values[6]: 0.0396354

  PointCloud representing the cylindrical component: 8625 data points.

