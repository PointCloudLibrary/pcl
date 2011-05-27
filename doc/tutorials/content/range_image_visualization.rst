.. _range_image_visualization:

How to visualize a range image
------------------------------

This tutorial demonstrates how to visualize a range image with two different means. As a point cloud (since RangeImage is derived from PointCloud) in a 3D viewer and as a picture visualizing the range values as colors.

The code
--------

First, create a file called, let's say, ``range_image_visualization.cpp`` in your favorite
editor, and place the following code inside it:

.. code-block:: cpp
   :linenos:

    #include <iostream>
    using namespace std;

    #include "pcl/common/common_headers.h"
    #include "pcl/range_image/range_image.h"
    #include "pcl/io/pcd_io.h"
    #include "pcl/visualization/range_image_visualizer.h"
    #include "pcl/visualization/pcl_visualizer.h"

    using namespace pcl;
    using namespace pcl::visualization;
    typedef PointXYZ PointType;

    // --------------------
    // -----Parameters-----
    // --------------------
    float angular_resolution = 0.5f;
    RangeImage::CoordinateFrame coordinate_frame = RangeImage::CAMERA_FRAME;

    // --------------
    // -----Help-----
    // --------------
    void printUsage (const char* progName)
    {
      cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
           << "Options:\n"
           << "-------------------------------------------\n"
           << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
           << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
           << "-h           this help\n"
           << "\n\n";
    }

    // --------------
    // -----Main-----
    // --------------
    int main (int argc, char** argv)
    {
      // --------------------------------------
      // -----Parse Command Line Arguments-----
      // --------------------------------------
      for (char c; (c = getopt (argc, argv, "r:c:h")) != -1; ) {
        switch (c) {
          case 'r':
          {
            angular_resolution = strtod (optarg, NULL);
            cout << "Setting angular resolution to "<<angular_resolution<<".\n";
            break;
          }
          case 'c':
          {
            coordinate_frame = (RangeImage::CoordinateFrame)strtol (optarg, NULL, 0);
            cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
            break;
          }
          case 'h':
            printUsage (argv[0]);
            exit (0);
        }
      }
      angular_resolution = deg2rad (angular_resolution);
      
      // ------------------------------------------------------------------
      // -----Read pcd file or create example point cloud if not given-----
      // ------------------------------------------------------------------
      // Read/Generate point cloud
      pcl::PointCloud<PointType> point_cloud;
      PointCloud<PointWithViewpoint> far_ranges;
      Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
      if (optind < argc)
      {
        sensor_msgs::PointCloud2 point_cloud_data;
        if (pcl::io::loadPCDFile (argv[optind], point_cloud_data) == -1)
        {
          cerr << "Was not able to open file \""<<argv[optind]<<"\".\n";
          printUsage (argv[0]);
          exit (0);
        }
        fromROSMsg (point_cloud_data, point_cloud);
        RangeImage::extractFarRanges (point_cloud_data, far_ranges);
        if (pcl::getFieldIndex (point_cloud_data, "vp_x")>=0)
        {
          cout << "Scene point cloud has viewpoint information.\n";
          PointCloud<PointWithViewpoint> tmp_pc;  fromROSMsg (point_cloud_data, tmp_pc);
          Eigen::Vector3f average_viewpoint = RangeImage::getAverageViewPoint (tmp_pc);
          scene_sensor_pose = Eigen::Translation3f (average_viewpoint) * scene_sensor_pose;
        }
      }
      else
      {
        cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
        for (float x=-0.5f; x<=0.5f; x+=0.01f)
        {
          for (float y=-0.5f; y<=0.5f; y+=0.01f)
          {
            PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
            point_cloud.points.push_back (point);
          }
        }
        point_cloud.width = point_cloud.points.size ();  point_cloud.height = 1;
      }

      
      // -----------------------------------------------
      // -----Create RangeImage from the PointCloud-----
      // -----------------------------------------------
      float noise_level = 0.0;
      float min_range = 0.0f;
      int border_size = 1;
      RangeImage range_image;
      range_image.createFromPointCloud (point_cloud, angular_resolution, deg2rad (360.0f), deg2rad (180.0f),
                                       scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
      range_image.integrateFarRanges (far_ranges);

      // --------------------------------------------
      // -----Open 3D viewer and add point cloud-----
      // --------------------------------------------
      PCLVisualizer viewer ("3D Viewer");
      viewer.addCoordinateSystem (1.0f);
      viewer.addPointCloud (point_cloud.makeShared (), "original point cloud");
      //viewer.addPointCloud (range_image, "range image");
      
      // --------------------------
      // -----Show range image-----
      // --------------------------
      RangeImageVisualizer range_image_widget ("Range image");
      range_image_widget.setRangeImage (range_image);
      
      //--------------------
      // -----Main loop-----
      //--------------------
      while (!viewer.wasStopped () || range_image_widget.isShown ())
      {
        ImageWidgetWX::spinOnce ();  // process GUI events
        viewer.spinOnce (100);
        usleep (100000);
      }
    }


Explanation
-----------

In the beginning we do command line parsing, read a point cloud from disc (or create it if not provided) and create a range image. All of these steps are already covered in the 'How to create a range image from a point cloud' tutorial.

The interesting part begins here:

.. code-block:: cpp

  ...
  PCLVisualizer viewer ("3D Viewer");
  viewer.addCoordinateSystem (1.0f);
  viewer.addPointCloud (point_cloud.makeShared (), "original point cloud");
  //viewer.addPointCloud (range_image, "range image");
  ...

This creates the 3D viewer object, adds a coordinate system and the original point cloud. You can also uncomment the last line to get the 3D points from the range image.

.. code-block:: cpp

  ...
  RangeImageVisualizer range_image_widget ("Range image");
  range_image_widget.setRangeImage (range_image);
  ...

This creates the widget that visualizes the range image (based on a wxWidget). 

.. code-block:: cpp

  ...
  while (!viewer.wasStopped () || range_image_widget.isShown ())
  {
    ImageWidgetWX::spinOnce ();  // process GUI events
    viewer.spinOnce (100);
    usleep (100000);
  }
  ...

This loop keeps the visualization alive, until both windows are closed. ImageWidgetWX::spinOnce() handles the current events of the RangeImageVisualizer and viewer.spinOnce(100) does the same for the 3D viewer (giving it 100ms time).


Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. code-block:: cmake

    add_executable (range_image_visualization range_image_visualization.cpp)
    target_link_libraries (range_image_visualization ${PCL_RANGE_IMAGE_LIBRARY})
    target_link_libraries (range_image_visualization ${PCL_VISUALIZATION_LIBRARY})
    target_link_libraries (range_image_visualization $ENV{PCL_FLANN_HOME}/lib/libflann_cpp.so)

After you have made the executable, you can run it. Simply do::

  $ ./range_image_visualization

This will use an autogenerated point cloud of a rectangle floating in space. It opens two windows, one a 3D viewer of the point cloud and one a visual version of the range image, where the range values are color coded.

You can also try it with an actual 3D scan:

  $ ./range_image_visualization some_pointcloud.pcd

The output should look similar to this:

.. image:: images/range_image_visualization.png
  :width: 500

Unseen areas (range -INFINITY) are shown in pale green and far ranges (range INFINITY - if available in the scan) in pale blue.


