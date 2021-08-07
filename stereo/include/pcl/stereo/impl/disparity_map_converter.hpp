/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#ifndef PCL_DISPARITY_MAP_CONVERTER_IMPL_H_
#define PCL_DISPARITY_MAP_CONVERTER_IMPL_H_

#include <pcl/common/intensity.h>
#include <pcl/console/print.h>
#include <pcl/stereo/disparity_map_converter.h>
#include <pcl/point_types.h>

#include <fstream>
#include <limits>

template <typename PointT>
pcl::DisparityMapConverter<PointT>::DisparityMapConverter()
: center_x_(0.0f)
, center_y_(0.0f)
, focal_length_(0.0f)
, baseline_(0.0f)
, is_color_(false)
, disparity_map_width_(640)
, disparity_map_height_(480)
, disparity_threshold_min_(0.0f)
, disparity_threshold_max_(std::numeric_limits<float>::max())
{}

template <typename PointT>
pcl::DisparityMapConverter<PointT>::~DisparityMapConverter()
{}

template <typename PointT>
inline void
pcl::DisparityMapConverter<PointT>::setImageCenterX(const float center_x)
{
  center_x_ = center_x;
}

template <typename PointT>
inline float
pcl::DisparityMapConverter<PointT>::getImageCenterX() const
{
  return center_x_;
}

template <typename PointT>
inline void
pcl::DisparityMapConverter<PointT>::setImageCenterY(const float center_y)
{
  center_y_ = center_y;
}

template <typename PointT>
inline float
pcl::DisparityMapConverter<PointT>::getImageCenterY() const
{
  return center_y_;
}

template <typename PointT>
inline void
pcl::DisparityMapConverter<PointT>::setFocalLength(const float focal_length)
{
  focal_length_ = focal_length;
}

template <typename PointT>
inline float
pcl::DisparityMapConverter<PointT>::getFocalLength() const
{
  return focal_length_;
}

template <typename PointT>
inline void
pcl::DisparityMapConverter<PointT>::setBaseline(const float baseline)
{
  baseline_ = baseline;
}

template <typename PointT>
inline float
pcl::DisparityMapConverter<PointT>::getBaseline() const
{
  return baseline_;
}

template <typename PointT>
inline void
pcl::DisparityMapConverter<PointT>::setDisparityThresholdMin(
    const float disparity_threshold_min)
{
  disparity_threshold_min_ = disparity_threshold_min;
}

template <typename PointT>
inline float
pcl::DisparityMapConverter<PointT>::getDisparityThresholdMin() const
{
  return disparity_threshold_min_;
}

template <typename PointT>
inline void
pcl::DisparityMapConverter<PointT>::setDisparityThresholdMax(
    const float disparity_threshold_max)
{
  disparity_threshold_max_ = disparity_threshold_max;
}

template <typename PointT>
inline float
pcl::DisparityMapConverter<PointT>::getDisparityThresholdMax() const
{
  return disparity_threshold_max_;
}

template <typename PointT>
void
pcl::DisparityMapConverter<PointT>::setImage(
    const pcl::PointCloud<pcl::RGB>::ConstPtr& image)
{
  image_ = image;

  // Set disparity map's size same with the image size.
  disparity_map_width_ = image_->width;
  disparity_map_height_ = image_->height;

  is_color_ = true;
}

template <typename PointT>
pcl::PointCloud<pcl::RGB>::Ptr
pcl::DisparityMapConverter<PointT>::getImage()
{
  pcl::PointCloud<pcl::RGB>::Ptr image_pointer(new pcl::PointCloud<pcl::RGB>);
  *image_pointer = *image_;
  return image_pointer;
}

template <typename PointT>
bool
pcl::DisparityMapConverter<PointT>::loadDisparityMap(const std::string& file_name)
{
  std::fstream disparity_file;

  // Open the disparity file
  disparity_file.open(file_name.c_str(), std::fstream::in);
  if (!disparity_file.is_open()) {
    PCL_ERROR("[pcl::DisparityMapConverter::loadDisparityMap] Can't load the file.\n");
    return false;
  }

  // Allocate memory for the disparity map.
  disparity_map_.resize(disparity_map_width_ * disparity_map_height_);

  // Reading the disparity map.
  for (std::size_t row = 0; row < disparity_map_height_; ++row) {
    for (std::size_t column = 0; column < disparity_map_width_; ++column) {
      float disparity;
      disparity_file >> disparity;

      disparity_map_[column + row * disparity_map_width_] = disparity;
    } // column
  }   // row

  return true;
}

template <typename PointT>
bool
pcl::DisparityMapConverter<PointT>::loadDisparityMap(const std::string& file_name,
                                                     const std::size_t width,
                                                     const std::size_t height)
{
  // Initialize disparity map's size.
  disparity_map_width_ = width;
  disparity_map_height_ = height;

  // Load the disparity map.
  return loadDisparityMap(file_name);
}

template <typename PointT>
void
pcl::DisparityMapConverter<PointT>::setDisparityMap(
    const std::vector<float>& disparity_map)
{
  disparity_map_ = disparity_map;
}

template <typename PointT>
void
pcl::DisparityMapConverter<PointT>::setDisparityMap(
    const std::vector<float>& disparity_map,
    const std::size_t width,
    const std::size_t height)
{
  disparity_map_width_ = width;
  disparity_map_height_ = height;

  disparity_map_ = disparity_map;
}

template <typename PointT>
std::vector<float>
pcl::DisparityMapConverter<PointT>::getDisparityMap()
{
  return disparity_map_;
}

template <typename PointT>
void
pcl::DisparityMapConverter<PointT>::compute(PointCloud& out_cloud)
{
  // Initialize the output cloud.
  out_cloud.clear();
  out_cloud.width = disparity_map_width_;
  out_cloud.height = disparity_map_height_;
  out_cloud.resize(out_cloud.width * out_cloud.height);

  if (is_color_ && !image_) {
    PCL_ERROR("[pcl::DisparityMapConverter::compute] Memory for the image was not "
              "allocated.\n");
    return;
  }

  for (std::size_t row = 0; row < disparity_map_height_; ++row) {
    for (std::size_t column = 0; column < disparity_map_width_; ++column) {
      // ID of current disparity point.
      std::size_t disparity_point = column + row * disparity_map_width_;

      // Disparity value.
      float disparity = disparity_map_[disparity_point];

      // New point for the output cloud.
      PointT new_point;

      // Init color
      if (is_color_) {
        pcl::common::IntensityFieldAccessor<PointT> intensity_accessor;
        intensity_accessor.set(new_point,
                               static_cast<float>((*image_)[disparity_point].r +
                                                  (*image_)[disparity_point].g +
                                                  (*image_)[disparity_point].b) /
                                   3.0f);
      }

      // Init coordinates.
      if (disparity_threshold_min_ < disparity &&
          disparity < disparity_threshold_max_) {
        // Compute new coordinates.
        PointXYZ point_3D(translateCoordinates(row, column, disparity));
        new_point.x = point_3D.x;
        new_point.y = point_3D.y;
        new_point.z = point_3D.z;
      }
      else {
        new_point.x = std::numeric_limits<float>::quiet_NaN();
        new_point.y = std::numeric_limits<float>::quiet_NaN();
        new_point.z = std::numeric_limits<float>::quiet_NaN();
      }
      // Put the point to the output cloud.
      out_cloud[disparity_point] = new_point;
    } // column
  }   // row
}

template <typename PointT>
pcl::PointXYZ
pcl::DisparityMapConverter<PointT>::translateCoordinates(std::size_t row,
                                                         std::size_t column,
                                                         float disparity) const
{
  // Returning point.
  PointXYZ point_3D;

  if (disparity != 0.0f) {
    // Compute 3D-coordinates based on the image coordinates, the disparity and the
    // camera parameters.
    float z_value = focal_length_ * baseline_ / disparity;
    point_3D.z = z_value;
    point_3D.x = (static_cast<float>(column) - center_x_) * (z_value / focal_length_);
    point_3D.y = (static_cast<float>(row) - center_y_) * (z_value / focal_length_);
  }

  return point_3D;
}

#define PCL_INSTANTIATE_DisparityMapConverter(T)                                       \
  template class PCL_EXPORTS pcl::DisparityMapConverter<T>;

#endif // PCL_DISPARITY_MAP_CONVERTER_IMPL_H_
