.. _rops_feature:

RoPs (Rotational Projection Statistics) feature
---------------------------

In this tutorial we will learn how to use the `pcl::ROPSEstimation` class in order to extract points features.
The feature extraction method implemented in this class was proposed by Yulan Guo, Ferdous Sohel, Mohammed Bennamoun, Min Lu and
Jianwei Wanalso in their article "Rotational Projection Statistics for 3D Local Surface Description and Object Recognition"

Theoretical Primer
---------------------------

The idea of the feature extraction method is as follows.
Having a mesh and a set of points for which feature must be computed we perform some simple steps. First of all for a given point of interest
the local surface is cropped. Local surface consists of the points and triangles that are within the given support radius.
For the given local surface LRF (Local Reference Frame) is computed. LRF is simply a triplet of vectors,
the comprehensive information about how these vectors are computed you can find in the article.
What is really important is that using these vectors we can provide the invariance to the rotation of the cloud. To do that, we simply
translate points of the local surface in such way that point of interest became the origin, after that we rotate local surface so that the
LRF vectors were aligned with the Ox, Oy and Oz axes. Having this done, we then start the feature extraction.
For every axis Ox, Oy and Oz the following steps are performed, we will refer to these axes as current axis:

  * local surface is rotated around the current axis by a given angle;
  * points of the rotated local surface are projected onto three planes XY, XZ and YZ;
  * for each projection distribution matrix is built, this matrix simply shows how much points fall onto each bin. Number of bins represents the matrix dimension and is the parameter of the algorithm, as well as the support radius;
  * for each distribution matrix central moments are calculated: M11, M12, M21, M22, E. Here E is the Shannon entropy;
  * calculated values are then concatenated to form the sub-feature.
We iterate through these steps several times. Number of iterations depends on the given number of rotations.
Sub-features for different axes are concatenated to form the final RoPS descriptor.

The code
--------

For this tutorial we will use the model from the Queen's Dataset. You can choose any other point cloud, but in order to make the
code work you will need to use the triangulation algorithm in order to obtain polygons. You can find the proposed model here:

  * `points <http://svn.pointclouds.org/data/tutorials/min_cut_segmentation_tutorial.pcd>`_ - contains the point cloud
  * `indices <http://svn.pointclouds.org/data/tutorials/min_cut_segmentation_tutorial.pcd>`_ - contains indices of the points for which RoPs must be computed
  * `triangles <http://svn.pointclouds.org/data/tutorials/min_cut_segmentation_tutorial.pcd>`_ - contains the polygons

Next what you need to do is to create a file ``rops_feature.cpp`` in any editor you prefer and copy the following code inside of it:

.. literalinclude:: sources/rops_feature/rops_feature.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now let's study out what is the purpose of this code.

.. literalinclude:: sources/rops_feature/rops_feature.cpp
   :language: cpp
   :lines: 9-11

These lines are simply loading the cloud from the .pcd file.

.. literalinclude:: sources/rops_feature/rops_feature.cpp
   :language: cpp
   :lines: 13-23

Here the indices of points for which RoPS feature must be computed are loaded. You can comment it and compute features for every single point in the cloud.
if you want.

.. literalinclude:: sources/rops_feature/rops_feature.cpp
   :language: cpp
   :lines: 25-40

These lines are loading the information about the polygons. You can replace them with the code for the triangulation if you have only the point cloud
instead of the mesh.

.. literalinclude:: sources/rops_feature/rops_feature.cpp
   :language: cpp
   :lines: 42-44

These code defines important algorithm parameters: support radius for local surface cropping, number of partition bins
used to form the distribution matrix and the number of rotations. The last parameter affects the length of the descriptor.

.. literalinclude:: sources/rops_feature/rops_feature.cpp
   :language: cpp
   :lines: 46-47

These lines set up the search method that will be used by the algorithm.

.. literalinclude:: sources/rops_feature/rops_feature.cpp
   :language: cpp
   :lines: 49-58

Here is the place where the instantiation of the ``pcl::ROPSEstimation`` class takes place. It has two parameters:

  * PointInT - type of the input points;
  * PointOutT - type of the output points.

Immediately after that we set the input all the necessary data neede for the feature computation.

.. literalinclude:: sources/rops_feature/rops_feature.cpp
   :language: cpp
   :lines: 60-61

Here is the place where the computational process is launched.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/rops_feature/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./rops_feature points.pcd indices.txt triangles.txt