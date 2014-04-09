.. _cloud_viewer:

The CloudViewer
---------------

The CloudViewer is a straight forward, simple point cloud visualization, meant
to get you up and viewing clouds in as little code as possible.  

.. note::
   
   The CloudViewer class is **NOT** meant to be used in multi-threaded
   applications! Please check the documentation on
   :pcl:`PCLVisualizer<pcl::visualization::PCLVisualizer>` or read the :ref:`pcl_visualizer` tutorial
   for thread safe visualization.

Simple Cloud Visualization
--------------------------

If you just want to visualize something in your app with a few lines of code,
use a snippet like the following one:

.. code-block:: cpp
   :linenos:

    #include <pcl/visualization/cloud_viewer.h>
    //...
    void 
    foo ()
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
      //... populate cloud
      pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
      viewer.showCloud (cloud);
      while (!viewer.wasStopped ())
      {
      }
    }

A more complete sample:
-----------------------

The following shows how to run code on the visualization thread.  The PCLVisualizer is
the back end of the CloudViewer, but its running in its own thread.  To access it you
must use callback functions, to avoid the visualization concurrency issues.  However
care must be taken to avoid race conditions in your code, as the callbacks will be
called from the visualization thread.

.. literalinclude:: sources/cloud_viewer/cloud_viewer.cpp
   :language: cpp
   :linenos:

Compiling and running the program
---------------------------------

Add the following lines to your `CMakeLists.txt` file:

.. literalinclude:: sources/cloud_viewer/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it like so::

  $ ./cloud_viewer
