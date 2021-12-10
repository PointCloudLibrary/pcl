.. _voxelgrid:

Downsampling a PointCloud using a VoxelGrid filter
--------------------------------------------------

In this tutorial we will learn how to downsample -- that is, reduce the number
of points -- a point cloud dataset, using a voxelized grid approach. 

The ``VoxelGrid`` class that we're about to present creates a *3D voxel grid*
(think about a voxel grid as a set of tiny 3D boxes in space) over the input
point cloud data. Then, in each *voxel* (i.e., 3D box), all the points present
will be approximated (i.e., *downsampled*) with their centroid. This approach
is a bit slower than approximating them with the center of the voxel, but it
represents the underlying surface more accurately.

.. raw:: html
  
  <iframe title="Downsampling a PointCloud using a VoxelGrid filter" width="480" height="390" src="https://www.youtube.com/embed/YHR6_OIxtFI?rel=0" frameborder="0" allowfullscreen></iframe>

The code
--------

First, download the dataset `table_scene_lms400.pcd
<https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd>`_
and save it somewhere to disk.

Then, create a file, let's say, ``voxel_grid.cpp`` in your favorite
editor, and place the following inside it:

.. literalinclude:: sources/voxel_grid/voxel_grid.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

The following lines of code will read the point cloud data from disk.

.. literalinclude:: sources/voxel_grid/voxel_grid.cpp
   :language: cpp
   :lines: 12-15
   
   
Then, a *pcl::VoxelGrid* filter is created with a leaf size of 1cm, the input
data is passed, and the output is computed and stored in *cloud_filtered*.

.. literalinclude:: sources/voxel_grid/voxel_grid.cpp
   :language: cpp
   :lines: 21-24
   
Finally, the data is written to disk for later inspection.

.. literalinclude:: sources/voxel_grid/voxel_grid.cpp
   :language: cpp
   :lines: 29-31


Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/voxel_grid/CMakeLists.txt
   :language: cmake
   :linenos:
   
After you have made the executable, you can run it. Simply do::

  $ ./voxel_grid

You will see something similar to::

  PointCloud before filtering: 460400 data points (x y z intensity distance sid).
  PointCloud after filtering: 41049 data points (x y z intensity distance sid).

