.. _reading_pcd:

Reading Point Cloud data from PCD files
---------------------------------------

In this tutorial we will learn how to read point cloud data from a PCD file.

The code
--------

First, create a file called, let's say, ``pcd_read.cpp`` in your favorite
editor, and place the following code inside it:

.. literalinclude:: sources/pcd_read/pcd_read.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

.. literalinclude:: sources/pcd_read/pcd_read.cpp
   :language: cpp
   :lines: 8
   
creates a PointCloud<PointXYZ> boost shared pointer and initializes it.

.. literalinclude:: sources/pcd_read/pcd_read.cpp
   :language: cpp
   :lines: 10-14
   
loads the PointCloud data from disk (we assume that test_pcd.pcd has already
been created from the previous tutorial) into the binary blob.

Alternatively, you can read a PCLPointCloud2 blob (available only in PCL 1.x). Due
to the dynamic nature of point clouds, we prefer to read them as binary blobs,
and then convert to the actual representation that we want to use.

.. code-block:: cpp

   pcl::PCLPointCloud2 cloud_blob;
   pcl::io::loadPCDFile ("test_pcd.pcd", cloud_blob);
   pcl::fromPCLPointCloud2 (cloud_blob, *cloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>

reads and converts the binary blob into the templated PointCloud format, here
using pcl::PointXYZ as the underlying point type.

Finally:

.. literalinclude:: sources/pcd_read/pcd_read.cpp
   :language: cpp
   :lines: 19-22
   
is used to show the data that was loaded from file.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/pcd_read/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do::

  $ ./pcd_read 

You will see something similar to::

  Loaded 5 data points from test_pcd.pcd with the following fields: x y z
    0.35222 -0.15188 -0.1064
    -0.39741 -0.47311 0.2926
    -0.7319 0.6671 0.4413
    -0.73477 0.85458 -0.036173
    -0.4607 -0.27747 -0.91676

Note that if the file test_pcd.pcd does not exist (either it hasn't been
created or it has been erased), you should get an error message such as::

  Couldn't read file test_pcd.pcd

