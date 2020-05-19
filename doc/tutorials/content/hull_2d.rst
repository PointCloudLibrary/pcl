.. _hull_2d:

Construct a concave or convex hull polygon for a plane model
------------------------------------------------------------

In this tutorial we will learn how to calculate a simple 2D hull polygon
(concave or convex) for a set of points supported by a plane.  


The code
--------

First, download the dataset `table_scene_mug_stereo_textured.pcd
<https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_mug_stereo_textured.pcd>`_
and save it somewhere to disk.

Then, create a file, let's say, ``concave_hull_2d.cpp`` or
``convex_hull_2d.cpp`` in your favorite editor and place the following inside:

.. literalinclude:: sources/concave_hull_2d/concave_hull_2d.cpp
   :language: cpp
   :linenos:

.. note::

   This tutorial is written for assuming you are looking for the **CONCAVE** hull.
   If you would like the **CONVEX** hull for a plane model, just replace concave
   with convex at EVERY point in this tutorial, including the source file, file
   names and the CMakeLists.txt file. You will also need to comment out
   setAlpha(), as this is not applicable to convex hulls.

The explanation
---------------

In the following lines of code, a segmentation object is created and some
parameters are set.  We use the SACMODEL_PLANE to segment this PointCloud, and
the method used to find this model is SAC_RANSAC.  The actual segmentation
takes place when `seg.segment (*inliers, *coefficients);` is called.  This
function stores all of the inlying points (on the plane) to `inliers`, and it
stores the coefficients to the plane `(a * x + b * y + c * z = d)` in
`coefficients`.

.. literalinclude:: sources/concave_hull_2d/concave_hull_2d.cpp
   :language: cpp
   :lines: 30-41

The next bit of code projects the inliers onto the plane model and creates
another cloud.  One way that we could do this is by just extracting the inliers
that we found before, but in this case we are going to use the coefficients we
found before.  We set the model type we are looking for and then set the
coefficients, and from that the object knows which points to project from
cloud_filtered to cloud_projected. 

.. literalinclude:: sources/concave_hull_2d/concave_hull_2d.cpp
   :language: cpp
   :lines: 46-51

The real interesting part is in the lines below, where the ConcaveHull object
gets created and the reconstruction is performed:

.. literalinclude:: sources/concave_hull_2d/concave_hull_2d.cpp
   :language: cpp
   :lines: 56-60

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
  PointCloud after segmentation has: 123727 inliers.
  PointCloud after projection has: 139656 data points.
  Concave hull has: 457 data points.

