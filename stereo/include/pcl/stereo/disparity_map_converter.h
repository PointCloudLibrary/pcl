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

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstring>
#include <vector>

namespace pcl {

/** \brief Compute point cloud from the disparity map.
 *
 * Example of usage:
 *
 * \code
 *  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new
 *    pcl::PointCloud<pcl::PointXYZI>);
 *  pcl::PointCloud<pcl::RGB>::Ptr left_image (new
 *    pcl::PointCloud<pcl::RGB>);
 *  // Fill left image cloud.
 *
 *  pcl::DisparityMapConverter<pcl::PointXYZI> dmc;
 *  dmc.setBaseline (0.8387445f);
 *  dmc.setFocalLength (368.534700f);
 *  dmc.setImageCenterX (318.112200f);
 *  dmc.setImageCenterY (224.334900f);
 *  dmc.setDisparityThresholdMin(15.0f);
 *
 *  // Left view of the scene.
 *  dmc.setImage (left_image);
 *  // Disparity map of the scene.
 *  dmc.loadDisparityMap ("disparity_map.txt", 640, 480);
 *
 *  dmc.compute(*cloud);
 * \endcode
 *
 * \author Timur Ibadov (ibadov.timur@gmail.com)
 * \ingroup stereo
 */
template <typename PointT>
class DisparityMapConverter {
protected:
  using PointCloud = pcl::PointCloud<PointT>;

public:
  /** \brief DisparityMapConverter constructor. */
  DisparityMapConverter();

  /** \brief Empty destructor. */
  virtual ~DisparityMapConverter();

  /** \brief Set x-coordinate of the image center.
   * \param[in] center_x x-coordinate of the image center.
   */
  inline void
  setImageCenterX(const float center_x);

  /** \brief Get x-coordinate of the image center.
   * \return x-coordinate of the image center.
   */
  inline float
  getImageCenterX() const;

  /** \brief Set y-coordinate of the image center.
   * \param[in] center_y y-coordinate of the image center.
   */
  inline void
  setImageCenterY(const float center_y);

  /** \brief Get y-coordinate of the image center.
   * \return y-coordinate of the image center.
   */
  inline float
  getImageCenterY() const;

  /** \brief Set focal length.
   * \param[in] focal_length the focal length.
   */
  inline void
  setFocalLength(const float focal_length);

  /** \brief Get focal length.
   * \return the focal length.
   */
  inline float
  getFocalLength() const;

  /** \brief Set baseline.
   * \param[in] baseline baseline.
   */
  inline void
  setBaseline(const float baseline);

  /** \brief Get baseline.
   * \return the baseline.
   */
  inline float
  getBaseline() const;

  /** \brief Set min disparity threshold.
   * \param[in] disparity_threshold_min min disparity threshold.
   */
  inline void
  setDisparityThresholdMin(const float disparity_threshold_min);

  /** \brief Get min disparity threshold.
   * \return min disparity threshold.
   */
  inline float
  getDisparityThresholdMin() const;

  /** \brief Set max disparity threshold.
   * \param[in] disparity_threshold_max max disparity threshold.
   */
  inline void
  setDisparityThresholdMax(const float disparity_threshold_max);

  /** \brief Get max disparity threshold.
   * \return max disparity threshold.
   */
  inline float
  getDisparityThresholdMax() const;

  /** \brief Set an image, that will be used for coloring of the output cloud.
   * \param[in] image the image.
   */
  void
  setImage(const pcl::PointCloud<pcl::RGB>::ConstPtr& image);

  /** \brief Get the image, that is used for coloring of the output cloud.
   * \return the image.
   */
  pcl::PointCloud<RGB>::Ptr
  getImage();

  /** \brief Load the disparity map.
   * \param[in] file_name the name of the disparity map file.
   * \return "true" if the disparity map was successfully loaded; "false" otherwise
   */
  bool
  loadDisparityMap(const std::string& file_name);

  /** \brief Load the disparity map and initialize it's size.
   * \param[in] file_name the name of the disparity map file.
   * \param[in] width width of the disparity map.
   * \param[in] height height of the disparity map.
   * \return "true" if the disparity map was successfully loaded; "false" otherwise
   */
  bool
  loadDisparityMap(const std::string& file_name,
                   const std::size_t width,
                   const std::size_t height);

  /** \brief Set the disparity map.
   * \param[in] disparity_map the disparity map.
   */
  void
  setDisparityMap(const std::vector<float>& disparity_map);

  /** \brief Set the disparity map and initialize it's size.
   * \param[in] disparity_map the disparity map.
   * \param[in] width width of the disparity map.
   * \param[in] height height of the disparity map.
   * \return "true" if the disparity map was successfully loaded; "false" otherwise
   */
  void
  setDisparityMap(const std::vector<float>& disparity_map,
                  const std::size_t width,
                  const std::size_t height);

  /** \brief Get the disparity map.
   * \return the disparity map.
   */
  std::vector<float>
  getDisparityMap();

  /** \brief Compute the output cloud.
   * \param[out] out_cloud the variable to return the resulting cloud.
   */
  virtual void
  compute(PointCloud& out_cloud);

protected:
  /** \brief Translate point from image coordinates and disparity to 3D-coordinates
   * \param[in] row
   * \param[in] column
   * \param[in] disparity
   * \return the 3D point, that corresponds to the input parametres and the camera
   * calibration.
   */
  PointXYZ
  translateCoordinates(std::size_t row, std::size_t column, float disparity) const;

  /** \brief X-coordinate of the image center. */
  float center_x_;
  /** \brief Y-coordinate of the image center. */
  float center_y_;
  /** \brief Focal length. */
  float focal_length_;
  /** \brief Baseline. */
  float baseline_;

  /** \brief Is color image is set. */
  bool is_color_;
  /** \brief Color image of the scene. */
  pcl::PointCloud<pcl::RGB>::ConstPtr image_;

  /** \brief Vector for the disparity map. */
  std::vector<float> disparity_map_;
  /** \brief X-size of the disparity map. */
  std::size_t disparity_map_width_;
  /** \brief Y-size of the disparity map. */
  std::size_t disparity_map_height_;

  /** \brief Thresholds of the disparity. */
  float disparity_threshold_min_;
  float disparity_threshold_max_;
};

} // namespace pcl

#include <pcl/stereo/impl/disparity_map_converter.hpp>
