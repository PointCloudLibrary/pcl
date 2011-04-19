.. _greedy_triangulation:

Fast triangulation of unordered point clouds
--------------------------------------------

This tutorial explains how to run a greedy surface triangulation algorithm on a
PointCloud with normals, to obtain a triangle mesh based on projections of the
local neighborhoods. An example of the method's output can be seen here:

.. raw:: html

  <iframe title="Surface Triangulation and Point Cloud Classification" width="480" height="390" src="http://www.youtube.com/embed/VALTnZCyWc0?rel=0" frameborder="0" allowfullscreen></iframe>

Background: algorithm and parameters
------------------------------------

The method works by maintaining a list of points from which the mesh can be
grown ("fringe" points) and extending it until all possible points are
connected. It can deal with unorganized points, coming from one or multiple
scans, and having multiple connected parts. It works best if the surface is
locally smooth and there are smooth transitions between areas with different
point densities.

Triangulation is performed locally, by projecting the local neighborhood of a
point along the point's normal, and connecting unconnected points. Thus, the
following parameters can be set:

* *setMaximumNearestNeighbors(unsigned)* and *setMu(double)* control the size of
  the neighborhood. The former defines how many neighbors are searched for,
  while the latter specifies the maximum acceptable distance for a point to be
  considered, relative to the distance of the nearest point (in order to adjust
  to changing densities). Typical values are 50-100 and 2.5-3 (or 1.5 for
  grids).

* *setSearchRadius(double)* is practically the maximum edge length for every
  triangle. This has to be set by the user such that to allow for the biggest
  triangles that should be possible.

* *setMinimumAngle(double)* and *setMaximumAngle(double)* are the minimum and
  maximum angles in each triangle. While the first is not guaranteed, the
  second is. Typical values are 10 and 120 degrees (in radians).

* *setMaximumSurfaceAgle(double)* and *setNormalConsistency(bool)* are meant to
  deal with the cases where there are sharp edges or corners and where two
  sides of a surface run very close to each other. To achieve this, points are
  not connected to the current point if their normals deviate more than the
  specified angle (note that most surface normal estimation methods produce
  smooth transitions between normal angles even at sharp edges). This angle is
  computed as the angle between the lines defined by the normals (disregarding
  the normal's direction) if the normal-consistency-flag is not set, as not all
  normal estimation methods can guarantee consistently oriented normals.
  Typically, 45 degrees (in radians) and false works on most datasets.

Please see the example below, and you can consult the following paper and its
references for more details::

  @InProceedings{Marton09ICRA,
    author    = {Zoltan Csaba Marton and Radu Bogdan Rusu and Michael Beetz},
    title     = {{On Fast Surface Reconstruction Methods for Large and Noisy Datasets}},
    booktitle = {Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)},
    month     = {May 12-17},
    year      = {2009},
    address   = {Kobe, Japan},
  }


The code
--------

First, create a file, let's say, ``greedy_projection.cpp`` in your favorite
editor, and place the following code inside it:

.. code-block:: cpp
   :linenos:

   #include <boost/make_shared.hpp>
   #include <pcl/point_types.h>
   #include <pcl/io/pcd_io.h>
   #include <pcl/kdtree/kdtree_flann.h>
   #include <pcl/features/normal_3d.h>
   #include <pcl/surface/gp3.h>

   using namespace pcl;
   using namespace pcl::io;
   using namespace std;

   int
     main (int argc, char** argv)
   {
     // Load input file into a PointCloud<T> with an appropriate type
     PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
     sensor_msgs::PointCloud2 cloud_blob;
     loadPCDFile ("bun0.pcd", cloud_blob);
     fromROSMsg (cloud_blob, *cloud);

     // Normal estimation
     NormalEstimation<PointXYZ, Normal> n;
     PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
     KdTree<PointXYZ>::Ptr tree = boost::make_shared<KdTreeFLANN<PointXYZ> > ();
     tree->setInputCloud (cloud);
     n.setInputCloud (cloud);
     n.setSearchMethod (tree);
     n.setKSearch (20);
     n.compute (*normals);

     // Concatenate the XYZ and normal fields
     PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
     pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

     // Create search tree
     KdTree<PointNormal>::Ptr tree2 = boost::make_shared<KdTreeFLANN<PointNormal> > ();
     tree2->setInputCloud (cloud_with_normals);

     // Initialize objects
     GreedyProjectionTriangulation<PointNormal> gp3;
     PolygonMesh triangles;

     // Set the maximum distance between connected points (maximum edge length)
     gp3.setSearchRadius (0.025);

     // Set typical values for the parameters
     gp3.setMu (2.5);
     gp3.setMaximumNearestNeighbors (100);
     gp3.setMaximumSurfaceAgle(M_PI/4); // 45 degrees
     gp3.setMinimumAngle(M_PI/18); // 10 degrees
     gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
     gp3.setNormalConsistency(false);

     // Get result
     gp3.setInputCloud (cloud_with_normals);
     gp3.setSearchMethod (tree2);
     gp3.reconstruct (triangles);

     // Additional vertex information
     std::vector<int> parts = gp3.getPartIDs();
     std::vector<int> states = gp3.getPointStates();

     // Finish
     return (0);
   }

The input file you can find at pcl/test/bun0.pcd

The explanation
---------------
Now, let's break down the code piece by piece.

.. code-block:: cpp

    // Load input file into a PointCloud<T> with an appropriate type
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
    sensor_msgs::PointCloud2 cloud_blob;
    loadPCDFile ("bun0.pcd", cloud_blob);
    fromROSMsg (cloud_blob, *cloud);

as the example PCD has only XYZ coordinates, we load it into a
PointCloud<PointXYZ>.

.. code-block:: cpp

    // Normal estimation
    NormalEstimation<PointXYZ, Normal> n;
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
    KdTree<PointXYZ>::Ptr tree = boost::make_shared<KdTreeFLANN<PointXYZ> > ();
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);

the method requires normals, so they are estimated using the standard method
from PCL.

.. code-block:: cpp

    // Concatenate the XYZ and normal fields
    PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

Since coordinates and normals need to be in the same PointCloud, we create a PointNormal type point cloud.

.. code-block:: cpp

    // Create search tree
    KdTree<PointNormal>::Ptr tree2 = boost::make_shared<KdTreeFLANN<PointNormal> > ();
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    GreedyProjectionTriangulation<PointNormal> gp3;
    PolygonMesh triangles;

The above lines deal with the initialization of the required objects.

.. code-block:: cpp

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.025);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAgle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

The above lines set the parameters, as explained above.

.. code-block:: cpp

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

The lines above set the input objects and perform the actual triangulation.


.. code-block:: cpp

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    
for each point, the ID of the containing connected component and its "state"
(i.e. gp3.FREE, gp3.BOUNDARY or gp3.COMPLETED) can be retrieved.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. code-block:: cmake
   
   add_executable (greedy_projection greedy_projection.cpp)
   target_link_libraries (greedy_projection pcl_io pcl_surface)

After you have made the executable, you can run it. Simply do::

  $ ./greedy_projection

Saving and viewing the result
-----------------------------

You can view the smoothed cloud for example by saving into a VTK file by::

    #include <pcl/io/vtk_io.h>
    ...
    saveVTKFile ("mesh.vtk", triangles);

