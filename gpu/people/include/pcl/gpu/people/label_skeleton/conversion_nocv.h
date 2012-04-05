/**
 * @copyright Copyright (2011) Willow Garage
 * @author Koen Buys
 * @file conversion.h
 * @brief This file contains the conversion functions
**/
#ifndef PCL_GPU_PEOPLE_LABEL_SKELETON_CONVERSION_H_
#define PCL_GPU_PEOPLE_LABEL_SKELETON_CONVERSION_H_

// our headers
#include <pcl/gpu/people/label_skeleton/blob2.h>
#include <pcl/gpu/people/label_skeleton/common.h>

// std
#include <vector>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <math.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace label_skeleton
      {
        /**
         * @brief This function takes a pointcloudRGB in and a pointcloudXYZ and assumes they are both registered
         * It colors the output pointcloud with the colors of the input cmap
         * @param[in] cmap the RGB matrix to take the rgb info from
         * @param[in] cloud_in the input pointcloud to take xyz from
         * @param[out] cloud_out the input pointcloud colored with the rgb info from the cmap
         * @return this is zero when everything went well
         */
        int makeColoredPointCloud ( const pcl::PointCloud<pcl::RGB>&          cmap,
                                    const pcl::PointCloud<pcl::PointXYZRGB>&  cloud_in,
                                    pcl::PointCloud<pcl::PointXYZRGB>&        cloud_out)
        {
          // Test if input image and cloud are same size, otherwise this won't work
          assert(cloud_in.width == cmap.width);
          assert(cloud_in.height == cmap.height);
          // Set output size of the cloud_out
          cloud_out.width = cmap.width;
          cloud_out.height = cmap.height;
        
          for(size_t i = 0; i < cmap.points.size (); i++)
          {
            pcl::PointXYZRGB point;
            point.r = cmap.points[i].r;
            point.g = cmap.points[i].g;
            point.b = cmap.points[i].b;
            point.x = cloud_in.points[i].x;
            point.y = cloud_in.points[i].y;
            point.z = cloud_in.points[i].z;
            cloud_out.points.push_back(point);
          }
          return 0;
        }
        /**
         * @brief This function takes a pointcloudRGB in and a pointcloudXYZ and assumes they are both registered
         * It colors the output pointcloud with the colors of the input cmap
         * @param[in] cmap the RGB matrix to take the rgb info from
         * @param[in] cloud_in the input pointcloud to take xyz from
         * @param[out] cloud_out the input pointcloud colored with the rgb info from the cmap
         * @return this is zero when everything went well
         */
        int makeColoredPointCloud ( const pcl::PointCloud<pcl::RGB>&          cmap,
                                    const pcl::PointCloud<pcl::PointXYZ>&     cloud_in,
                                    pcl::PointCloud<pcl::PointXYZRGB>&        cloud_out)
        {
          // Test if input image and cloud are same size, otherwise this won't work
          assert(cloud_in.width == cmap.width);
          assert(cloud_in.height == cmap.height);
          // Set output size of the cloud_out
          cloud_out.width = cmap.width;
          cloud_out.height = cmap.height;
        
          for(size_t i = 0; i < cmap.points.size (); i++)
          {
            pcl::PointXYZRGB point;
            point.r = cmap.points[i].r;
            point.g = cmap.points[i].g;
            point.b = cmap.points[i].b;
            point.x = cloud_in.points[i].x;
            point.y = cloud_in.points[i].y;
            point.z = cloud_in.points[i].z;
            cloud_out.points.push_back(point);
          }
          return 0;
        }
        /**
         * @brief This function generates a pointcloud rgb from pointcloud xyzrgb
         * @param[out] image the image to which the data will be written
         * @param[in] cloud the pointcloud from which the RGB data will be taken
         * @return zero if everything went well
         **/
        int makeImageFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>&  cloud, 
                                    pcl::PointCloud<pcl::RGB>&                image)
        {
          for(size_t i = 0; i < cloud.points.size (); i++)
          {
            pcl::PointRGB point;
            point.r = cloud.points[i].r;
            point.g = cloud.points[i].g;
            point.b = cloud.points[i].b;
            image.points.push_back(point);
          }
          image.width = cloud.width;
          image.height = cloud.height;
          return 0;
        }

      } // end namespace label_skeleton
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl
#endif
