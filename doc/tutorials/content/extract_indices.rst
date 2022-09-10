.. _extract_indices:

Extracting indices from a PointCloud
------------------------------------

In this tutorial we will learn how to use an :pcl:`ExtractIndices <pcl::ExtractIndices>` filter to extract a subset of 
points from a point cloud based on the indices output by a segmentation algorithm. In order to not complicate the
tutorial, the segmentation algorithm is not explained here. Please check
the :ref:`planar_segmentation` tutorial for more information.


.. raw:: html

   <iframe title="Extracting indices from a PointCloud" width="480" height="390" src="https://www.youtube.com/embed/ZTK7NR1Xx4c?rel=0" frameborder="0" allowfullscreen></iframe>

The code
--------

First, download the dataset `table_scene_lms400.pcd
<https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd>`_
and save it somewhere to disk.

Then, create a file, let's say, ``extract_indices.cpp`` in your favorite
editor, and place the following inside it:

.. literalinclude:: sources/extract_indices/extract_indices.cpp
   :language: cpp
   :linenos:


The explanation
---------------

Now, let's break down the code piece by piece, skipping the obvious.

After the data has been loaded from the input .PCD file, we create a
:pcl:`VoxelGrid<pcl::VoxelGrid>` filter, to downsample the data. The rationale behind data
downsampling here is just to speed things up -- less points means less time
needed to spend within the segmentation loop.

.. literalinclude:: sources/extract_indices/extract_indices.cpp
   :language: cpp
   :lines: 24-27


The next block of code deals with the parametric segmentation. To keep the
tutorial simple, its explanation will be skipped for now. Please see the
**segmentation** tutorials (in particular :ref:`planar_segmentation`) for more
information.

.. literalinclude:: sources/extract_indices/extract_indices.cpp
   :language: cpp
   :lines: 38-48

   
The line

.. literalinclude:: sources/extract_indices/extract_indices.cpp
   :language: cpp
   :lines: 51

and

.. literalinclude:: sources/extract_indices/extract_indices.cpp
   :language: cpp
   :lines: 67-70

represent the actual indices :pcl:`extraction filter <pcl::ExtractIndices>`. To process multiple models, we
run the process in a loop, and after each model is extracted, we go back to
obtain the remaining points, and iterate. The *inliers* are obtained from the segmentation process, as follows:

.. literalinclude:: sources/extract_indices/extract_indices.cpp
   :language: cpp
   :lines: 58-59


Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/extract_indices/CMakeLists.txt
   :language: cmake
   :linenos: 

After you have made the executable, you can run it. Simply do::

  $ ./extract_indices

You will see something similar to::

  PointCloud before filtering: 460400 data points.
  PointCloud after filtering: 41049 data points.
  PointCloud representing the planar component: 20164 data points.
  PointCloud representing the planar component: 12129 data points.

