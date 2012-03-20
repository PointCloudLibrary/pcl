/**
 * @copyright Copyright (2011) Willow Garage
 * @author Koen Buys
 * @file conversion.h
 * @brief This file contains the function prototypes for the conversion functions
**/
#ifndef LABELSKEL_CONVERSION_H
#define LABELSKEL_CONVERSION_H
// our headers
#include "libLabelSkel/blob2.h"
#include "libLabelSkel/common.h"

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

namespace LabelSkel 
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
#endif
