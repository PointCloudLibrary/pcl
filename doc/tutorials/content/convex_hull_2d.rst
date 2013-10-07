.. _convex_hull_2d:

Construct a convex hull polygon for a plane model
-------------------------------------------------

In this tutorial we will learn how to calculate a simple 2D convex hull polygon
for a set of points supported by a plane.

The following video shows a demonstration of the code given below on the test
dataset `table_scene_mug_stereo_textured.pcd 
<https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_mug_stereo_textured.pcd>`_.

.. raw:: html
  
  <iframe title="Acquiring the convex hull of a planar surface" width="480" height="390" src="http://www.youtube.com/embed/J9CjWDgPDTM?rel=0" frameborder="0" allowfullscreen></iframe>

The code
--------

First, download the dataset `table_scene_mug_stereo_textured.pcd
<https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_mug_stereo_textured.pcd>`_
and save it somewhere to disk.

Then, create a file, let's say, ``convex_hull_2d.cpp`` in your favorite
editor, and place the following inside it:

.. literalinclude:: sources/convex_hull_2d/convex_hull_2d.cpp
   :language: cpp
   :linenos:


The explanation
---------------

The only interesting part is in the lines below, where the ConvexHull object
gets created and the reconstruction is performed:

.. literalinclude:: sources/convex_hull_2d/convex_hull_2d.cpp
   :language: cpp
   :lines: 48-51


Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/convex_hull_2d/CMakeLists.txt
   :language: cmake
   :linenos:


After you have made the executable, you can run it. Simply do::

  $ ./convex_hull_2d

You will see *something similar* to::

  PointCloud after filtering has: 139656 data points.
  Convex hull has: 30 data points.

