.. _writing_pcd:

Writing Point Cloud data to PCD files
-------------------------------------

In this tutorial we will learn how to write point cloud data to a PCD file.

The code
--------

First, create a file called, let's say, ``pcd_write.cpp`` in your favorite
editor, and place the following code inside it:

.. literalinclude:: sources/pcd_write/pcd_write.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

.. literalinclude:: sources/pcd_write/pcd_write.cpp
   :language: cpp
   :lines: 2-3

pcl/io/pcd_io.h is the header that contains the definitions for PCD I/O
operations, and pcl/point_types.h contains definitions for several PointT type
structures (pcl::PointXYZ in our case).

.. literalinclude:: sources/pcd_write/pcd_write.cpp
   :language: cpp
   :lines: 8

describes the templated PointCloud structure that we will create. The type of
each point is set to pcl::PointXYZ, which is:

.. code-block:: cpp

   // \brief A point structure representing Euclidean xyz coordinates.
   struct PointXYZ
   {
     float x;
     float y;
     float z;
   };

The lines:

.. literalinclude:: sources/pcd_write/pcd_write.cpp
   :language: cpp
   :lines: 10-21

fill in the PointCloud structure with random point values, and set the
appropriate parameters (width, height, is_dense).

Then:

.. literalinclude:: sources/pcd_write/pcd_write.cpp
   :language: cpp
   :lines: 23

saves the PointCloud data to disk into a file called test_pcd.pcd

Finally:

.. literalinclude:: sources/pcd_write/pcd_write.cpp
   :language: cpp
   :lines: 24-27

is used to show the data that was generated.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/pcd_write/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./pcd_write

You will see something similar to::

  Saved 5 data points to test_pcd.pcd.
    0.352222 -0.151883 -0.106395
    -0.397406 -0.473106 0.292602
    -0.731898 0.667105 0.441304
    -0.734766 0.854581 -0.0361733
    -0.4607 -0.277468 -0.916762

You can check the content of the file test_pcd.pcd, using::

  $ cat test_pcd.pcd
  # .PCD v.5 - Point Cloud Data file format
  FIELDS x y z
  SIZE 4 4 4
  TYPE F F F
  WIDTH 5
  HEIGHT 1
  POINTS 5
  DATA ascii
  0.35222 -0.15188 -0.1064
  -0.39741 -0.47311 0.2926
  -0.7319 0.6671 0.4413
  -0.73477 0.85458 -0.036173
  -0.4607 -0.27747 -0.91676

