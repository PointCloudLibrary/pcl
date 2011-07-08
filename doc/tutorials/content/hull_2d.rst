.. _hull_2d:

Construct a concave or convex hull polygon for a plane model
--------------------------------------------------

In this tutorial we will learn how to calculate a simple 2D hull polygon (concave or convex) for a set of points supported by a plane.  


The code
--------

First, download the dataset `table_scene_mug_stereo_textured.pcd <http://dev.pointclouds.org/attachments/download/158/table_scene_mug_stereo_textured.pcd>`_ and save it somewhere to disk.

Then, create a file, let's say, ``concave_hull_2d.cpp`` or ``convex_hull2d.cpp`` in your favorite editor and place the following inside:

.. literalinclude:: sources/concave_hull_2d/concave_hull_2d.cpp
   :language: cpp
   :linenos:

**NOTE**: This tutorial is written for assuming you are looking for the **CONCAVE** hull.  If you would like the **CONVEX** hull for a plane model, just replace concave with convex at EVERY point in this tutorial, including the source file, file names and the CMakeLists.txt file.

The explanation
---------------

The only interesting part is in the lines below, where the ConcaveHull object
gets created and the reconstruction is performed:

.. literalinclude:: sources/concave_hull_2d/concave_hull_2d.cpp
   :language: cpp
   :lines: 51-56

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/concave_hull_2d/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./concave_hull_2d

You will see something similar to::

  PointCloud after filtering has: 139656 data points.
  Concave hull has: 30 data points.

