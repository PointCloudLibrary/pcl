.. _moving_least_squares:

Smoothing and normal estimation based on polynomial reconstruction
------------------------------------------------------------------

This tutorial explains how a Moving Least Squares (MLS) surface reconstruction
method can be used to smooth and resample noisy data. Please see an example in
the video below:

.. raw:: html

  <iframe title="Smoothing and normal estimation based on polynomial reconstruction" width="480" height="390" src="http://www.youtube.com/embed/FqHroDuo_I8?rel=0" frameborder="0" allowfullscreen></iframe>


Some of the data irregularities (caused by small distance measurement errors)
are very hard to remove using statistical analysis. To create complete models,
glossy surfaces as well as occlusions in the data must be accounted for. In
situations where additional scans are impossible to acquire, a solution is to
use a resampling algorithm, which attempts to recreate the missing parts of the
surface by higher order polynomial interpolations between the surrounding data
points. By performing resampling, these small errors can be corrected and the
"double walls" artifacts resulted from registering multiple scans together can
be smoothed. 

.. image:: images/resampling_1.png

On the left side of the figure above, we see the effect or estimating surface
normals in a dataset comprised of two registered point clouds together. Due to
alignment errors, the resultant normals are noisy. On the right side we see the
effects of surface normal estimation in the same dataset after it has been
smoothed with a Moving Least Squares algorithm. Plotting the curvatures at each
point as a measure of the eigenvalue relationship before and after resampling,
we obtain:

.. image:: images/resampling_2.png

To approximate the surface defined by a local neighborhood of points **p1, p2
...  pk** at a point **q** we use a bivariate polynomial height function
defined on a on a robustly computed reference plane. 

.. raw:: html

  <iframe title="Removing noisy data through resampling" width="480" height="390" src="http://www.youtube.com/embed/N5AgC0KEcw0?rel=0" frameborder="0" allowfullscreen></iframe>

The code
--------

First, create a file, let's say, ``resampling.cpp`` in your favorite
editor, and place the following inside it:

.. code-block:: cpp
   :linenos:

   #include <pcl/point_types.h>
   #include <pcl/io/pcd_io.h>
   #include <pcl/kdtree/kdtree_flann.h>
   #include <pcl/surface/mls.h>

   using namespace pcl;
   using namespace pcl::io;
   using namespace std;

   int
     main (int argc, char** argv)
   {
     // Load input file into a PointCloud<T> with an appropriate type
     PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
     sensor_msgs::PointCloud2 cloud_blob;
     // Load bun0.pcd -- should be available with the PCL archive in test 
     loadPCDFile ("bun0.pcd", cloud_blob);
     fromROSMsg (cloud_blob, *cloud);

     // Create a KD-Tree
     KdTree<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ>);
     tree->setInputCloud (cloud);

     // Output has the same type as the input one, it will be only smoothed
     PointCloud<PointXYZ> mls_points;

     // Init object (second point type is for the normals, even if unused)
     MovingLeastSquares<PointXYZ, Normal> mls;

     // Optionally, a pointer to a cloud can be provided, to be set by MLS
     PointCloud<Normal>::Ptr mls_normals (new PointCloud<Normal> ());
     mls.setOutputNormals (mls_normals);

     // Set parameters
     mls.setInputCloud (cloud);
     mls.setPolynomialFit (true);
     mls.setSearchMethod (tree);
     mls.setSearchRadius (0.03);

     // Reconstruct
     mls.reconstruct (mls_points);
     
     // Concatenate fields for saving
     PointCloud<PointNormal> mls_cloud;
     pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);

     // Save output
     savePCDFile ("bun0-mls.pcd", mls_cloud);
   }

You should be able to find the input file at *pcl/test/bun0.pcd*.

The explanation
---------------

Now, let's break down the code piece by piece.

.. code-block:: cpp

   KdTree<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ>);
   tree->setInputCloud (cloud);

as the example PCD has only XYZ coordinates, we load it into a
PointCloud<PointXYZ>. These fields are mandatory for the method, other ones are
allowed and will be preserved.

.. code-block:: cpp

  // Init object (second point type is for the normals, even if unused)
  MovingLeastSquares<PointXYZ, Normal> mls;

if normal estimation is not required, this step can be skipped.

.. code-block:: cpp

   mls.setInputCloud (cloud);
   mls.setPolynomialFit (true);

the first template type is used for the input and output cloud. Only the XYZ
dimensions of the input are smoothed in the output.

.. code-block:: cpp

    mls.setSearchRadius (0.03);

    // Reconstruct
    mls.reconstruct (mls_points);

polynomial fitting could be disabled for speeding up smoothing. Please consult
the code API (constructor and setter) for default values and additional
parameters to control the smoothing process.

.. code-block:: cpp

    // Save output
    savePCDFile ("bun0-mls.pcd", mls_cloud);

if the normals and the original dimensions need to be in the same cloud, the
fields have to be concatenated.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. code-block:: cmake
   
   add_executable (resampling resampling.cpp)
   target_link_libraries (resampling ${PCL_IO_LIBRARY} ${PCL_SURFACE_LIBRARY})

After you have made the executable, you can run it. Simply do::

  $ ./resampling

You can view the smoothed cloud for example by executing::

  $ pcd_viewer bun0-mls.pcd

