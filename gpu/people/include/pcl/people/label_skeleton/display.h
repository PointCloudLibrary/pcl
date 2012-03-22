/**
 * @copyright Copyright (2011) Willow Garage
 * @author Koen Buys
 * @file display.h
 * @brief This file contains the function prototypes for display functions
 **/
#ifndef PCL_PEOPLE_LABEL_SKELETON_DISPLAY_H_
#define PCL_PEOPLE_LABEL_SKELETON_DISPLAY_H_
// our headers
#include "pcl/people/label_skeleton/blob2.h"
#include "pcl/people/label_skeleton/common.h"

// std
#include <vector>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <math.h>

// opencv drawing stuff
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS specific includes
//#include "pcl_ros/point_cloud.h"
//#include <sensor_msgs/PointCloud2.h>
//#include <geometry_msgs/Point.h>

//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

//#include <image_geometry/pinhole_camera_model.h>

namespace pcl
{
  namespace people
  {
    namespace label_skeleton 
    {

      typedef std::pair<int,int> two_labels_t;
      typedef std::vector<two_labels_t> two_labels_list_t;

//      void drawMarkerTree(  std::vector<std::vector<Blob2> >& sorted,
//                            int                               part_label,
//                            int                               part_lid,
//                            visualization_msgs::MarkerArray&  markerArray);
//
//      void drawPrettyMarkerTree( std::vector<std::vector<Blob2> >& sorted,
//      			   int                               part_label,
//      			   int                               part_lid,
//      			   visualization_msgs::MarkerArray&  markerArray);
//
//      void drawPrettyMarkerTreeRecurse(  std::vector<std::vector<Blob2> >& sorted,
//      				   int                               part_label,
//      				   int                               part_lid,
//      				   two_labels_list_t& omit_list,
//      				   visualization_msgs::MarkerArray&  markerArray,
//      				   std::vector<geometry_msgs::Point>& part_label_locations
//      				   );

      int makeTreePointIndices( std::vector<std::vector<Blob2> >&   sorted,
                                int                                 part_label,
                                int                                 part_lid,
                                pcl::PointCloud<pcl::PointXYZRGB>&  cloud_in,
                                pcl::PointIndices&                  cloud_list);

      int makeTreePointCloud( std::vector<std::vector<Blob2> >&   sorted,
                              int                                 part_label,
                              int                                 part_lid,
                              pcl::PointCloud<pcl::PointXYZRGB>&  cloud_in,
                              pcl::PointCloud<pcl::PointXYZRGB>&  cloud_out);

      int makeBlobPointCloud( std::vector<std::vector<Blob2> >&   sorted,
                              cv::Mat&                            cmap,
                              pcl::PointCloud<pcl::PointXYZRGB>&  cloud_in,
                              pcl::PointCloud<pcl::PointXYZRGB>&  cloud_out);

      // Helper functions
//       bool isValidPart(std::vector<geometry_msgs::Point> part_label_locations, int pid);
//       void addLine(geometry_msgs::Point p0, geometry_msgs::Point p1, int id, int plabel, int clabel, visualization_msgs::Marker& marker);

    } // end namespace LabelSkel
  } // end namespace people
} // end namespace pcl
#endif //#ifndef LABELSKEL_DISPLAY_H

