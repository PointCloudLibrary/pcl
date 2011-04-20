.. _cloud_viewer:

The CloudViewer
------------------------------------------

The CloudViewer is a straight forward, simple point cloud visualization, meant
to get you up and viewing clouds in as little code as possible.  

--------------------------
Simple Cloud Visualization
--------------------------

If you just want to visualize something in your app with a few lines of code,
use a snippet like the following one:

.. code-block:: cpp
   :linenos:

    #include <pcl_visualization/cloud_viewer.h>
    //...
    void 
    foo ()
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
      //... populate cloud
      pcl_visualization::CloudViewer viewer("Simple Cloud Viewer");
      viewer.showCloud(cloud);
      while (!viewer.wasStopped())
      {
      }
    }

---------------------
A more complete sample:
---------------------

The following shows how to run code on the visualization thread.  The PCLVisualizer is
the back end of the CloudViewer, but its running in its own thread.  To access it you
must use callback functions, to avoid the visualization concurrency issues.  However
care must be taken to avoid race conditions in your code, as the callbacks will be
called from the visualization thread.

.. code-block:: cpp
    :linenos:

    #include <pcl/visualization/cloud_viewer.h>
    #include <iostream>
    #include <pcl/io/io.h>
    #include <pcl/io/pcd_io.h>
    
    int user_data;
    
    void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
    {
      viewer.setBackgroundColor(1.0, 0.5, 1.0);
      pcl::PointXYZ o;
      o.x = 1.0;
      o.y = 0;
      o.z = 0;
      viewer.addSphere(o, 0.25, "sphere", 0);
      std::cout << "i only run once" << std::endl;
    
    }
    
    void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
    {
      static unsigned count = 0;
      std::stringstream ss;
      ss << "Once per viewer loop: " << count++;
      viewer.removeShape("text", 0);
      viewer.addText(ss.str(), 200, 300, "text", 0);
    
      //FIXME: possible race condition here:
      user_data++;
    }
    
    int main()
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::io::loadPCDFile("my_point_cloud.pcd", *cloud);
    
      pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
      //blocks until the cloud is actually rendered
      viewer.showCloud(cloud);
    
      //use the following functions to get access to the underlying more advanced/powerful
      //PCLVisualizer
    
      //This will only get called once
      viewer.runOnVisualizationThreadOnce(viewerOneOff);
    
      //This will get called once per visualization iteration
      viewer.runOnVisualizationThread(viewerPsycho);
      while (!viewer.wasStopped())
      {
        //you can also do cool processing here
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        user_data++;
      }
      return 0;
    }
