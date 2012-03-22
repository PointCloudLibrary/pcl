/**
 * @copyright Copyright (2011) Willow Garage
 * @author Koen Buys
 * @file conversion.h
 * @brief This file contains the function prototypes for the conversion functions
**/
#ifndef PCL_PEOPLE_LABELSKELETON_CONVERSION_H_
#define PCL_PEOPLE_LABELSKELETON_CONVERSION_H_
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

namespace pcl
{
  namespace people
  {
    namespace label_skeleton
    {
      int makeColoredPointCloud(  cv::Mat&                            cmap,
                                  pcl::PointCloud<pcl::PointXYZRGB>&  cloud_in,
                                  pcl::PointCloud<pcl::PointXYZRGB>&  cloud_out);

      int makeColoredPointCloud(  cv::Mat&                            cmap,
                                  pcl::PointCloud<pcl::PointXYZ>&     cloud_in,
                                  pcl::PointCloud<pcl::PointXYZRGB>&  cloud_out);

      int makeImageFromPointCloud(cv::Mat&                            image,
                                  pcl::PointCloud<pcl::PointXYZRGB>&  cloud);

      int makeImageFromPointCloud(cv::Mat&                            image,
                                  pcl::PointIndices                   indices,
                                  pcl::PointCloud<pcl::PointXYZRGB>&  cloud);

      int makeFGMaskFromPointCloud(cv::Mat&                            image,
                                   pcl::PointIndices                   indices,
                                   pcl::PointCloud<pcl::PointXYZRGB>&  cloud);

      int makeDepthImage16FromPointCloud(cv::Mat&                            image,
                                         pcl::PointCloud<pcl::PointXYZRGB>&  cloud);

      int makeDepthImage16FromPointCloud(cv::Mat&                         image,
                                         pcl::PointCloud<pcl::PointXYZ>&  cloud);

    } // end namespace LabelSkel
  } // end namespace people
} // end namespace pcl
#endif
