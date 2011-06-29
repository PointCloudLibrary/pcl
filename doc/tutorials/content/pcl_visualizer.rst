.. _pcl_visualizer:

PCLVisualizer
=============

PCLVisualizer is PCL's full-featured visualisation class. While more
complex to use than the CloudViewer, it is also more powerful, offering
features such as displaying normals, drawing shapes and multiple
viewports.

This tutorial will use several code samples to illustrate some of the
features of PCLVisualizer, beginning with displaying a single point
cloud.

Visualising a single cloud
==========================

This code sample uses PCLVisualizer to display a single PointXYZ cloud.
It also illustrates changing the background colour and displaying the
axes.

.. literalinclude:: sources/pcl_visualizer/pcl_visualizer_simple.cpp
    :language: cpp
    :linenos:

Explanation
-----------

Most of the sample code is boilerplate to set up the point cloud that
will be displayed. Let's take a look at the relevant part, line-by-line.

.. code-block:: cpp

    ...
    PCLVisualizer viewer ("3D Viewer");
    ...

This creates the viewer object, giving it a nice name to display in the
title bar.

.. code-block:: cpp

    ...
    viewer.setBackgroundColor (0, 0, 0);
    ...

The background colour of the viewer can be set to any RGB colour you
like. In this case, we are setting it to black.

.. code-block:: cpp

    ...
    viewer.addPointCloud (point_cloud_ptr, "sample cloud");
    ...

This is the most important line. We add the point cloud to the viewer,
giving it an ID string that can be used to identify the cloud in other
methods. Multiple point clouds can be added with multiple calls to
``addPointCloud()``, supplying a new ID each time. If you want to update
a point cloud that is already displayed, you must first call
``removePointCloud()`` and provide the ID of the cloud that is to be
updated. (Note: versions 1.1 and up of PCL provide a new API method,
``updatePointCloud()``, that allows a cloud to be updated without
manually calling ``removePointCloud()`` first.)

This is the most basic of ``addPointCloud()``'s many
variations. Others are used to handle different point types, display
normals, and so on. We will illustrate some others during this tutorial,
or you can see the `PCLVisualizer documentation`_ for more details.

.. _`PCLVisualizer documentation`:
   http://docs.pointclouds.org/1.0.0/classpcl_1_1visualization_1_1_p_c_l_visualizer.html

.. code-block:: cpp

    ...
    viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    ...

This next line changes the size of the rendered points. You can control
the way any point cloud is rendered in the viewer using this method.

.. code-block:: cpp

    ...
    viewer.addCoordinateSystem (1.0);
    ...

Viewing complex point clouds can often be disorientating. To keep
yourself aligned in the world, axes can be displayed. These will appear
as three cylinders along the X, Y and Z axes. The size of the cylinders
can be controlled using the ``scale`` parameter. In this case, we have
set it to 1.0 (which also happens to be the default if no value is
given). An alternative version of this method can be used to place the
axes at any point in the world.

.. code-block:: cpp

    ...
    viewer.initCameraParameters ();
    ...

This final call sets up some handy camera parameters to make things look
nice.

Compiling and running the program
---------------------------------

Create a `CMakeLists.txt` file with the following contents:

.. literalinclude:: sources/pcl_visualizer/CMakeLists.txt.simple
   :language: cmake
   :linenos:

After you have made the executable, you can run it like so::

  $ ./pcl_visualizer_simple


Adding some colour
==================

Often, a point cloud will not use the simple PointXYZ type. One common
point type is PointXYZRGB, which also contains colour data. Aside from
that, you may wish to colour specific point clouds to make them
distinguishable in the viewer. PCLVizualizer provides methods for
displaying point clouds with the colour data stored within them, or for
assigning colours to point clouds. This code sample illustrates these
features.


Normals and other information
=============================

Displaying normals is an important step in understanding a point cloud.
The PCLVisualizer class has the ability to draw normals.


Drawing Shapes
==============

PCLVisualizer allows you to draw various primitive shapes in the view.
This is often used to visualise the results of point cloud processing
algorithms, for example, visualising which clusters of points have been
recognised as landmarks by drawing transparent spheres around them.


Multiple viewports
==================

You will often want to compare multiple point clouds side-by-side. While
you could draw them in the same view port, this can get confusing.
PCLVisualizer allows you to draw multiple point clouds in separate
viewports, making comparison easy.

