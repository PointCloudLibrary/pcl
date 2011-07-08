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

.. literalinclude:: sources/greedy_projection/greedy_projection.cpp
   :language: cpp
   :linenos:

The input file you can find at pcl/test/bun0.pcd

The explanation
---------------
Now, let's break down the code piece by piece.

.. literalinclude:: sources/greedy_projection/greedy_projection.cpp
   :language: cpp
   :lines: 10-15

as the example PCD has only XYZ coordinates, we load it into a
PointCloud<PointXYZ>.

.. literalinclude:: sources/greedy_projection/greedy_projection.cpp
   :language: cpp
   :lines: 17-26


the method requires normals, so they are estimated using the standard method
from PCL.

.. literalinclude:: sources/greedy_projection/greedy_projection.cpp
   :language: cpp
   :lines: 28-31

Since coordinates and normals need to be in the same PointCloud, we create a PointNormal type point cloud.

.. literalinclude:: sources/greedy_projection/greedy_projection.cpp
   :language: cpp
   :lines: 33-39

The above lines deal with the initialization of the required objects.

.. literalinclude:: sources/greedy_projection/greedy_projection.cpp
   :language: cpp
   :lines: 41-50

The above lines set the parameters, as explained above.

.. literalinclude:: sources/greedy_projection/greedy_projection.cpp
   :language: cpp
   :lines: 52-55

The lines above set the input objects and perform the actual triangulation.

.. literalinclude:: sources/greedy_projection/greedy_projection.cpp
   :language: cpp
   :lines: 56-59
   

for each point, the ID of the containing connected component and its "state"
(i.e. gp3.FREE, gp3.BOUNDARY or gp3.COMPLETED) can be retrieved.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/greedy_projection/CMakeLists.txt
   :language: cmake
   :linenos:
   

After you have made the executable, you can run it. Simply do::

  $ ./greedy_projection

Saving and viewing the result
-----------------------------

You can view the smoothed cloud for example by saving into a VTK file by::

    #include <pcl/io/vtk_io.h>
    ...
    saveVTKFile ("mesh.vtk", triangles);

