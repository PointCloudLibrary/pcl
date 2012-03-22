/**
 * @copyright Copyright (2011) Willow Garage
 * @author Koen Buys
 * @file segment.h
 * @brief This file contains the function prototypes for the segmentation functions
**/

#ifndef LABELSKEL_SEGMENT_H
#define LABELSKEL_SEGMENT_H

// our headers
#include "pcl/people/label_skeleton/blob2.h"
#include "pcl/people/label_skeleton/common.h"

// std
#include <vector>

// opencv drawing stuff
#include <opencv2/highgui/highgui.hpp>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include "pcl_ros/point_cloud.h"
#include "pcl/ros/conversions.h"
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/PointIndices.h>


namespace LabelSkel 
{

void smoothLabelImage ( cv::Mat&  lmap_in,
                        cv::Mat&  dmap,
                        cv::Mat&  lmap_out);

void smoothLabelImage2 ( cv::Mat&  lmap_in,
                        cv::Mat&  dmap,
                        cv::Mat&  lmap_out);

void sortIndicesToBlob2 ( pcl::PointCloud<pcl::PointXYZRGBL>              cloud_in,
                          unsigned int                                    sizeThres,
                          std::vector< std::vector<Blob2> >&              sorted,
                          std::vector< std::vector<pcl::PointIndices> >&  indices);

void sortIndicesToBlob3 ( pcl::PointCloud<pcl::PointXYZRGBL>              cloud_in,
                          unsigned int                                    sizeThres,
                          std::vector< std::vector<Blob2> >&              sorted,
                          std::vector< std::vector<pcl::PointIndices> >&  indices);

int giveSortedBlobsInfo ( std::vector<std::vector<Blob2> >& sorted);

}


#endif //#ifndef LABELSKEL_SEGMENT_H
