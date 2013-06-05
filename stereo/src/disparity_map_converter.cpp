/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <pcl/stereo/disparity_map_converter.h>

#include <fstream>
#include <limits>

#pragma warning(disable : 4996 4512)
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/stereo/digital_elevation_map.h>
#pragma warning(default : 4996 4512)

template class PCL_EXPORTS pcl::DisparityMapConverter<pcl::PointXYZI>;
template class PCL_EXPORTS pcl::DisparityMapConverter<pcl::PointDEM>;

template <typename PointT>
pcl::DisparityMapConverter<typename PointT>::DisparityMapConverter ()
{
  center_x_ = 0;
  center_y_ = 0;
  focal_length_ = 0;
  baseline_ = 0;

  disparity_map_width_ = 640;
  disparity_map_height_ = 480;

  disparity_threshold_min_ = 0.0f;
  disparity_threshold_max_ = std::numeric_limits<float>::max();
}

template <typename PointT>
pcl::DisparityMapConverter<typename PointT>::~DisparityMapConverter ()
{
  
}

template <typename PointT> void
pcl::DisparityMapConverter<typename PointT>::setImageCenterX (const float center_x)
{
  center_x_ = center_x;
}

template <typename PointT> float
pcl::DisparityMapConverter<typename PointT>::getImageCenterX () const
{
  return center_x_;
}

template <typename PointT> void
pcl::DisparityMapConverter<typename PointT>::setImageCenterY (const float center_y)
{
  center_y_ = center_y;
}

template <typename PointT> float
pcl::DisparityMapConverter<typename PointT>::getImageCenterY () const
{
  return center_y_;
}

template <typename PointT> void
pcl::DisparityMapConverter<typename PointT>::setFocalLength (const float focal_length)
{
  focal_length_ = focal_length;
}

template <typename PointT> float
pcl::DisparityMapConverter<typename PointT>::getFocalLength () const
{
  return focal_length_;
}

template <typename PointT> void
pcl::DisparityMapConverter<typename PointT>::setBaseline (const float baseline)
{
  baseline_ = baseline;
}

template <typename PointT> float
pcl::DisparityMapConverter<typename PointT>::getBaseline () const
{
  return baseline_;
}

template <typename PointT> void
pcl::DisparityMapConverter<typename PointT>::setDisparityThresholdMin (const float disparity_threshold_min)
{
  disparity_threshold_min_ = disparity_threshold_min;
}

template <typename PointT> float
pcl::DisparityMapConverter<typename PointT>::getDisparityThresholdMin () const
{
  return disparity_threshold_min_;
}

template <typename PointT> void
pcl::DisparityMapConverter<typename PointT>::setDisparityThresholdMax (const float disparity_threshold_max)
{
  disparity_threshold_max_ = disparity_threshold_max;
}

template <typename PointT> float
pcl::DisparityMapConverter<typename PointT>::getDisparityThresholdMax () const
{
  return disparity_threshold_max_;
}

template <typename PointT> bool
pcl::DisparityMapConverter<typename PointT>::loadImage (const std::string &file_name)
{
  // Allocate memory for the image.
  image_ = pcl::PointCloud<pcl::RGB>::Ptr (new pcl::PointCloud<pcl::RGB>);

  // Load the image.
  pcl::PCDReader pcd;
  if (pcd.read (file_name, *image_) == -1)
  {
    PCL_ERROR ("[pcl::DisparityMapConverter::loadImage] Can't load the file.\n");
    return false;
  }

  // Set disparity map's size same with the image size.
  disparity_map_width_ = image_->width;
  disparity_map_height_ = image_->height;

  return true;
}

template <typename PointT> void
pcl::DisparityMapConverter<typename PointT>::setImage (const pcl::PointCloud<pcl::RGB>::Ptr &image)
{
  image_ = image;

  // Set disparity map's size same with the image size.
  disparity_map_width_ = image_->width;
  disparity_map_height_ = image_->height;
}

template <typename PointT> pcl::PointCloud<pcl::RGB>::Ptr
pcl::DisparityMapConverter<typename PointT>::getImage ()
{
  pcl::PointCloud<pcl::RGB>::Ptr image_pointer (new pcl::PointCloud<pcl::RGB>);
  *image_pointer = *image_;
  return image_pointer;
}

template <typename PointT> bool
pcl::DisparityMapConverter<typename PointT>::loadDisparityMap (const std::string &file_name)
{
  std::fstream disparity_file;

  // Open the disparity file
  disparity_file.open (file_name, std::fstream::in);
  if (!disparity_file.is_open())
  {
    PCL_ERROR ("[pcl::DisparityMapConverter::loadDisparityMap] Can't load the file.\n");
    return false;
  }

  // Allocate memory for the disparity map.
  disparity_map_.resize(disparity_map_width_ * disparity_map_height_);

  // Reading the disparity map.
  for (size_t row = 0; row < disparity_map_height_; ++row)
  {
    for (size_t column = 0; column < disparity_map_width_; ++column)
    {
      float disparity;
      disparity_file >> disparity;

      disparity_map_[column + row * disparity_map_width_] = disparity;
    } // column
  } // row

  return true;
}

template <typename PointT> bool
pcl::DisparityMapConverter<typename PointT>::loadDisparityMap (const std::string &file_name, const size_t width, const size_t height)
{
  // Initialize disparity map's size. 
  disparity_map_width_ = width;
  disparity_map_height_ = height;

  // Load the disparity map.
  return loadDisparityMap(file_name);
}

template <typename PointT> void
pcl::DisparityMapConverter<typename PointT>::setDisparityMap(const std::vector<float> &disparity_map)
{
  disparity_map_ = disparity_map;
}

template <typename PointT> void
pcl::DisparityMapConverter<typename PointT>::setDisparityMap(const std::vector<float> &disparity_map, const size_t width, const size_t height)
{
  disparity_map_width_ = width;
  disparity_map_height_ = height;

  disparity_map_ = disparity_map;
}

template <typename PointT> std::vector<float>
pcl::DisparityMapConverter<typename PointT>::getDisparityMap ()
{
  return disparity_map_;
}

template <typename PointT> void
pcl::DisparityMapConverter<typename PointT>::compute (PointCloudPointer &out_cloud)
{
  // Initialize the output cloud.
  out_cloud = PointCloudPointer (new pcl::PointCloud<PointT>);
  out_cloud->width = disparity_map_width_;
  out_cloud->height = disparity_map_height_;
  out_cloud->resize (out_cloud->width * out_cloud->height);

  if (!image_)
  {
    PCL_ERROR ("[pcl::DisparityMapConverter::compute] Memory for the image was not allocated.\n");
    return;
  }

  for (size_t row = 0; row < disparity_map_height_; ++row)
  {
    for (size_t column = 0; column < disparity_map_width_; ++column)
    {
      // ID of curent disparity point.
      size_t disparity_point = column + row * disparity_map_width_;

      // Disparity value.
      float disparity = disparity_map_[disparity_point];

      // New point for the output cloud.
      PointT new_point;
      
      if (disparity_threshold_min_ < disparity && disparity < disparity_threshold_max_)
      {
        // Initialize the new point.
        PointXYZ point_3D (translateCoordinates (row, column, disparity));
        new_point.x = point_3D.x;
        new_point.y = point_3D.y;
        new_point.z = point_3D.z;
        
        new_point.intensity  = static_cast<float> (
            (image_->points[disparity_point].r +
             image_->points[disparity_point].g +
             image_->points[disparity_point].b) / 3);
        
        // Put the point to the output cloud.
        (*out_cloud)[disparity_point] = new_point;
      } // if
    } // column
  } // row
}

template <typename PointT> pcl::PointXYZ
pcl::DisparityMapConverter<typename PointT>::translateCoordinates(
    size_t row, size_t column, float disparity) const
{
  // Returnung point.
  PointXYZ point_3D;

  // Compute 3D-coordinates based on the image coordinates, the disparity and the camera parametres.
  float z_value = focal_length_ * baseline_ / disparity;
  point_3D.z = z_value;
  point_3D.x = (static_cast<float>(column) - center_x_) * (z_value / focal_length_);
  point_3D.y = (static_cast<float>(row) - center_y_) * (z_value / focal_length_);

  return point_3D;
}