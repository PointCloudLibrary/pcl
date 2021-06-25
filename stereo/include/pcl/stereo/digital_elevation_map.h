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

#include <pcl/stereo/disparity_map_converter.h>
#include <pcl/point_types.h>

namespace pcl {

/** \brief Build a Digital Elevation Map in the column-disparity space from a disparity
 * map and a color image of the scene.
 *
 * Example of usage:
 *
 * \code
 *  pcl::PointCloud<pcl::PointDEM>::Ptr cloud (new
 *      pcl::PointCloud<pcl::PointDEM>);
 *  pcl::PointCloud<pcl::RGB>::Ptr left_image (new
 *    pcl::PointCloud<pcl::RGB>);
 *  // Fill left image cloud.
 *
 *  pcl::DigitalElevationMapBuilder demb;
 *  demb.setBaseline (0.8387445f);
 *  demb.setFocalLength (368.534700f);
 *  demb.setImageCenterX (318.112200f);
 *  demb.setImageCenterY (224.334900f);
 *  demb.setDisparityThresholdMin (15.0f);
 *  demb.setDisparityThresholdMax (80.0f);
 *  demb.setResolution (64, 32);
 *
 *  // Left view of the scene.
 *  demb.loadImage (left_image);
 *  // Disparity map of the scene.
 *  demb.loadDisparityMap ("disparity_map.txt", 640, 480);
 *
 *  demb.compute(*cloud);
 * \endcode
 *
 * \author Timur Ibadov (ibadov.timur@gmail.com)
 * \ingroup stereo
 */
class PCL_EXPORTS DigitalElevationMapBuilder : public DisparityMapConverter<PointDEM> {
public:
  using DisparityMapConverter<PointDEM>::baseline_;
  using DisparityMapConverter<PointDEM>::translateCoordinates;
  using DisparityMapConverter<PointDEM>::image_;
  using DisparityMapConverter<PointDEM>::disparity_map_;
  using DisparityMapConverter<PointDEM>::disparity_map_width_;
  using DisparityMapConverter<PointDEM>::disparity_map_height_;
  using DisparityMapConverter<PointDEM>::disparity_threshold_min_;
  using DisparityMapConverter<PointDEM>::disparity_threshold_max_;

  /** \brief DigitalElevationMapBuilder constructor. */
  DigitalElevationMapBuilder();

  /** \brief Empty destructor. */
  ~DigitalElevationMapBuilder() override;

  /** \brief Set resolution of the DEM.
   * \param[in] resolution_column the column resolution.
   * \param[in] resolution_disparity the disparity resolution.
   */
  void
  setResolution(std::size_t resolution_column, std::size_t resolution_disparity);

  /** \brief Get column resolution of the DEM.
   * \return column resolution of the DEM.
   */
  std::size_t
  getColumnResolution() const;

  /** \brief Get disparity resolution of the DEM.
   * \return disparity resolution of the DEM.
   */
  std::size_t
  getDisparityResolution() const;

  /** \brief Set minimum amount of points in a DEM's cell.
   * \param[in] min_points_in_cell minimum amount of points in a DEM's cell.
   */
  void
  setMinPointsInCell(std::size_t min_points_in_cell);

  /** \brief Get minimum amount of points in a DEM's cell.
   * \return minimum amount of points in a DEM's cell.
   */
  std::size_t
  getMinPointsInCell() const;

  /** \brief Compute the Digital Elevation Map.
   * \param[out] out_cloud the variable to return the resulting cloud.
   */
  void
  compute(pcl::PointCloud<PointDEM>& out_cloud) override;

protected:
  /** \brief Column resolution of the DEM. */
  std::size_t resolution_column_;
  /** \brief disparity resolution of the DEM. */
  std::size_t resolution_disparity_;
  /** \brief Minimum amount of points in a DEM's cell. */
  std::size_t min_points_in_cell_;
};

} // namespace pcl
