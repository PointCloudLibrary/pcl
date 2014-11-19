.. _bspline_fitting:

Fitting trimmed B-splines to unordered point clouds
---------------------------------------------------

This tutorial explains how to run a B-spline fitting algorithm on a
point-cloud, to obtain a smooth, parametric surface representation.
The algorithm consists of the following steps:

* Initialization of the B-spline surface by using the Principal Component Analysis (PCA). This
  assumes that the point-cloud has two main orientations, i.e. that it is roughly planar.

* Refinement and fitting of the B-spline surface.

* Circular initialization of the B-spline curve. Here we assume that the point-cloud is
  compact, i.e. no separated clusters.

* Fitting of the B-spline curve.

* Triangulation of the trimmed B-spline surface.

In this video, the algorithm is applied to the frontal scan of the stanford bunny (204800 points):

.. raw:: html

  <iframe title="Trimmed B-spline surface fitting" width="480" height="390" src="http://www.youtube.com/embed/trH2kWELvyw?rel=0" frameborder="0" allowfullscreen></iframe>


Theoretical background
----------------------

Theoretical information on the algorithm can be found in this `report
<http://pointclouds.org/blog/trcs/moerwald/index.php>`_ and in my `PhD thesis
<http://users.acin.tuwien.ac.at/tmoerwald/?site=3>`_.


PCL installation settings
-------------------------

Please note that the modules for NURBS and B-splines are not enabled by default.
Make sure you enable "BUILD_surface_on_nurbs" in your ccmake configuration, by setting it to ON.

If your license permits, also enable "USE_UMFPACK" for sparse linear solving.
This requires SuiteSparse (libsuitesparse-dev in Ubuntu) which is faster, 
allows more degrees of freedom (i.e. control points) and more data points.

The program created during this tutorial is available in 
*pcl/examples/surface/example_nurbs_fitting_surface.cpp* and is built when
"BUILD_examples" is set to ON. This will create the binary called *pcl_example_nurbs_fitting_surface*
in your *bin* folder.


The code
--------

The cpp file used in this tutorial can be found in *pcl/doc/tutorials/content/sources/bspline_fitting/bspline_fitting.cpp*.
You can find the input file at *pcl/test/bunny.pcd*.

.. literalinclude:: sources/bspline_fitting/bspline_fitting.cpp
   :language: cpp
   :linenos:
   :lines: 1-220


The explanation
---------------
Now, let's break down the code piece by piece.
Lets start with the choice of the parameters for B-spline surface fitting:

.. literalinclude:: sources/bspline_fitting/bspline_fitting.cpp
   :language: cpp
   :linenos:
   :lines: 56-66

* *order* is the polynomial order of the B-spline surface.

* *refinement* is the number of refinement iterations, where for each iteration control-points
  are inserted, approximately doubling the control points in each parametric direction 
  of the B-spline surface.

* *iterations* is the number of iterations that are performed after refinement is completed.

* *mesh_resolution* the number of vertices in each parametric direction,
  used for triangulation of the B-spline surface.

Fitting:

* *interior_smoothness* is the smoothness of the surface interior.

* *interior_weight* is the weight for optimization for the surface interior.

* *boundary_smoothness* is the smoothness of the surface boundary.

* *boundary_weight* is the weight for optimization for the surface boundary.

Note, that the boundary in this case is not the trimming curve used later on.
The boundary can be used when a point-set exists that defines the boundary. Those points
can be declared in *pcl::on_nurbs::NurbsDataSurface::boundary*. In that case, when the
*boundary_weight* is greater than 0.0, the algorithm tries to align the domain boundaries
to these points. In our example we are trimming the surface anyway, so there is no need
for aligning the boundary.   

Initialization of the B-spline surface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: sources/bspline_fitting/bspline_fitting.cpp
   :language: cpp
   :lines: 68-72

The command *initNurbsPCABoundingBox* uses PCA to create a coordinate systems, where the principal
eigenvectors point into the direction of the maximum, middle and minimum extension of the point-cloud.
The center of the coordinate system is located at the mean of the points.
To estimate the extension of the B-spline surface domain, a bounding box is computed in the plane formed
by the maximum and middle eigenvectors. That bounding box is used to initialize the B-spline surface with
its minimum number of control points, according to the polynomial degree chosen.

The surface fitting class *pcl::on_nurbs::FittingSurface* is initialized with the point data and the initial
B-spline.

.. literalinclude:: sources/bspline_fitting/bspline_fitting.cpp
   :language: cpp
   :lines: 74-80

The *on_nurbs::Triangulation* class allows easy conversion between the *ON_NurbsSurface* and the *PolygonMesh* class,
for visualization of the B-spline surfaces. Note that NURBS are a generalization of B-splines,
and are therefore a valid container for B-splines, with all control-point weights = 1.

.. literalinclude:: sources/bspline_fitting/bspline_fitting.cpp
   :language: cpp
   :lines: 82-92

Refinement and fitting of the B-spline surface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

At this point of the code we have a B-spline surface with minimal number of control points.
Typically they are not enough to represent finer details of the underlying geometry 
of the point-cloud. However, if we increase the control-points to our desired level of detail and
subsequently fit the refined B-spline, we run into problems. For robust fitting B-spline surfaces
the rule is: 
"The higher the degree of freedom of the B-spline surface, the closer we have to be to the points to be approximated".

This is the reason why we iteratively increase the degree of freedom by refinement in both directions (line 85-86),
and fit the B-spline surface to the point-cloud, getting closer to the final solution.

.. literalinclude:: sources/bspline_fitting/bspline_fitting.cpp
   :language: cpp
   :lines: 94-102

After we reached the final level of refinement, the surface is further fitted to the point-cloud
for a pleasing end result.

Initialization of the B-spline curve
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Now that we have the surface fitted to the point-cloud, we want to cut off the overlapping regions of the surface.
To achieve this we project the point-cloud into the parametric domain using the closest points to the B-spline surface.
In this domain of R^2 we perform the weighted B-spline curve fitting, that creates a closed trimming curve that approximately
contains all the points.

.. literalinclude:: sources/bspline_fitting/bspline_fitting.cpp
   :language: cpp
   :lines: 107-120

The topic of curve fitting goes a bit deeper into the thematics of B-splines. Here we assume that you are
familiar with the concept of B-splines, knot vectors, control-points, and so forth.
Please consider the curve being split into supporting regions which is bound by consecutive knots.
Also note that points that are inside and outside the curve are distinguished.

* *addCPsAccuracy* the distance of the supporting region of the curve to the closest data points has to be below
  this value, otherwise a control point is inserted.

* *addCPsIteration* inner iterations without inserting control points.

* *maxCPs* the maximum total number of control-points.

* *accuracy* the average fitting accuracy of the curve, w.r.t. the supporting regions.

* *iterations* maximum number of iterations performed.

* *closest_point_resolution* number of control points that must lie within each supporting region. (0 turns this constraint off)

* *closest_point_weight* weight for fitting the curve to its closest points.

* *closest_point_sigma2* threshold for closest points (disregard points that are further away from the curve).

* *interior_sigma2* threshold for interior points (disregard points that are further away from and lie within the curve).

* *smooth_concavity* value that leads to inward bending of the curve (0 = no bending; <0 inward bending; >0 outward bending).

* *smoothness* weight of smoothness term.


.. literalinclude:: sources/bspline_fitting/bspline_fitting.cpp
   :language: cpp
   :lines: 122-127

The curve is initialized using a minimum number of control points to represent a circle, with the center located
at the mean of the point-cloud and the radius of the maximum distance of a point to the center.
Please note that interior weighting is enabled for all points with the command *curve_data.interior_weight_function.push_back (true)*.

Fitting of the B-spline curve
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: sources/bspline_fitting/bspline_fitting.cpp
   :language: cpp
   :lines: 129-133

Similar to the surface fitting approach, the curve is iteratively fitted and refined, as shown in the video.
Note how the curve tends to bend inwards at regions where it is not supported by any points.

Triangulation of the trimmed B-spline surface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: sources/bspline_fitting/bspline_fitting.cpp
   :language: cpp
   :lines: 136-142

After the curve fitting terminated, our geometric representation consists of a B-spline surface and a closed
B-spline curved, defined within the parametric domain of the B-spline surface. This is called trimmed B-spline surface.
In line 140 we can use the trimmed B-spline to create a triangular mesh. The triangulation algorithm first triangulates
the whole domain and afterwards removes triangles that lie outside of the trimming curve. Vertices of triangles
that intersect the trimming curve are clamped to the curve.

When running this example and switch to wire-frame mode (w), you will notice that the triangles are ordered in
a rectangular way, which is a result of the rectangular domain of the surface.

Some hints
----------
Please bear in mind that the robustness of this algorithm heavily depends on the underlying data.
The parameters for B-spline fitting are designed to model the characteristics of this data.

* If you have holes or steps in your data, you might want to work with lower refinement levels and lower accuracy to 
  prevent the B-spline from folding and twisting. Moderately increasing of the smoothness might also work.

* Try to introduce as much pre-conditioning and constraints to the parameters. E.g. if you know, that
  the trimming curve is rather simple, then limit the number of maximum control points.

* Start simple! Before giving up on gaining control over twisting and bending B-splines, I highly recommend
  to start your fitting trials with a small number of control points (low refinement), 
  low accuracy but also low smoothness (B-splines have implicit smoothing property).

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/bspline_fitting/CMakeLists.txt
   :language: cmake
   :linenos:   

After you have made the executable, you can run it. Simply do:

  $ ./bspline_fitting ${PCL_ROOT}/test/bunny.pcd


Saving and viewing the result
-----------------------------

* Saving as OpenNURBS (3dm) file

You can save the B-spline surface by using the commands provided by OpenNurbs:

.. literalinclude:: sources/bspline_fitting/bspline_fitting.cpp
   :language: cpp
   :lines: 145-163

The files generated can be viewed with the pcl/examples/surface/example_nurbs_viewer_surface.cpp.

* Saving as triangle mesh into a vtk file

You can save the triangle mesh for example by saving into a VTK file by:

    #include <pcl/io/vtk_io.h>
    ...
    pcl::io::saveVTKFile ("mesh.vtk", mesh);

PCL also provides vtk conversion into other formats (PLY, OBJ).

