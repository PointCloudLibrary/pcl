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

#include <pcl/stereo/digital_elevation_map.h>

#include <pcl/console/print.h>
#include <pcl/common/feature_histogram.h>

pcl::DigitalElevationMapBuilder::DigitalElevationMapBuilder () :
    resolution_column_ (64),
    resolution_disparity_ (32),
    min_points_in_cell_ (1)
{
}

pcl::DigitalElevationMapBuilder::~DigitalElevationMapBuilder ()
{
  
}

void
pcl::DigitalElevationMapBuilder::setResolution (size_t resolution_column, 
    size_t resolution_disparity)
{
  resolution_column_ = resolution_column;
  resolution_disparity_ = resolution_disparity;
}

size_t
pcl::DigitalElevationMapBuilder::getColumnResolution () const
{
  return resolution_column_;
}

size_t
pcl::DigitalElevationMapBuilder::getDisparityResolution () const
{
  return resolution_disparity_;
}

void 
pcl::DigitalElevationMapBuilder::setMinPointsInCell (size_t min_points_in_cell)
{
  min_points_in_cell_ = min_points_in_cell;
}

size_t
pcl::DigitalElevationMapBuilder::getMinPointsInCell () const
{
  return min_points_in_cell_;
}

// Build DEM.
void 
pcl::DigitalElevationMapBuilder::compute (pcl::PointCloud<PointDEM> &out_cloud)
{
  // Initialize.
  // Initialize the output cloud.
  out_cloud.clear ();
  out_cloud.width = resolution_column_;
  out_cloud.height = resolution_disparity_;
  out_cloud.resize (out_cloud.width * out_cloud.height);

  // Initialize steps.
  const size_t kColumnStep = (disparity_map_width_ - 1) / resolution_column_ + 1;
  const float kDisparityStep = (disparity_threshold_max_ -
      disparity_threshold_min_) / resolution_disparity_;

  // Initialize histograms.
  const size_t kNumberOfHistograms = resolution_column_ * resolution_disparity_;

  const float kHeightMin = -0.5f;
  const float kHeightMax = 1.5f;
  const float kHeightResolution = 0.01f;
  const size_t kHeightBins = static_cast<size_t> ((kHeightMax - kHeightMin) / kHeightResolution);
  // Histogram for initializing other height histograms.
  FeatureHistogram height_histogram_example (kHeightBins, kHeightMin, kHeightMax);

  const float kIntensityMin = 0.0f;
  const float kIntensityMax = 255.0f;
  const size_t kIntensityBins = 256;
  // Histogram for initializing other intensity histograms.
  FeatureHistogram intensity_histogram_example (kIntensityBins, kIntensityMin, kIntensityMax);

  std::vector <FeatureHistogram> height_histograms 
      (kNumberOfHistograms, height_histogram_example);
  std::vector <FeatureHistogram> intensity_histograms 
      (kNumberOfHistograms, intensity_histogram_example);

  // Check, if an image was loaded.
  if (!image_)
  {
    PCL_ERROR ("[pcl::DisparityMapConverter::compute] Memory for the image was not allocated.\n");
    return;
  }

  for (size_t column = 0; column < disparity_map_width_; ++column)
  {
    for (size_t row = 0; row < disparity_map_height_; ++row)
    {
      float disparity = disparity_map_[column + row * disparity_map_width_];
      if (disparity_threshold_min_ < disparity && disparity < disparity_threshold_max_)
      {
        // Find a height and an intensity of the point of interest.
        PointXYZ point_3D = translateCoordinates (row, column, disparity);
        float height = point_3D.y;

        RGB point_RGB = image_->points[column + row * disparity_map_width_];
        float intensity = static_cast<float> ((point_RGB.r + point_RGB.g + point_RGB.b) / 3);

        // Calculate index of histograms.
        size_t index_column = column / kColumnStep;
        size_t index_disparity = static_cast<size_t> (
            (disparity - disparity_threshold_min_) / kDisparityStep);

        size_t index = index_column + index_disparity * resolution_column_;

        // Increase the histograms.
        height_histograms[index].addValue (height);
        intensity_histograms[index].addValue (intensity);

      } // if
    } // row
  } // column
  
  // For all histograms.
  for (size_t index_column = 0; index_column < resolution_column_; ++index_column)
  {
    for (size_t index_disparity = 0; index_disparity < resolution_disparity_; ++index_disparity)
    {
      size_t index = index_column + index_disparity * resolution_column_;
      // Compute the corresponding DEM cell.
      size_t column = index_column * kColumnStep;
      float disparity = disparity_threshold_min_ +
          static_cast<float> (index_disparity) * kDisparityStep;
      
      PointXYZ point_3D = translateCoordinates (0, column, disparity);
      PointDEM point_DEM;
      point_DEM.x = point_3D.x;
      point_DEM.z = point_3D.z;

      if (height_histograms[index].getNumberOfElements () >= min_points_in_cell_)
      {
        point_DEM.y = height_histograms[index].getMeanValue ();
        point_DEM.height_variance = 0.5f * baseline_ / disparity;

        point_DEM.intensity = intensity_histograms[index].getMeanValue ();
        point_DEM.intensity_variance = intensity_histograms[index].getVariance (point_DEM.intensity);
      }
      else // height_histograms[index].getNumberOfElements () < min_points_in_cell_
      {
        point_DEM.y = std::numeric_limits<float>::quiet_NaN ();
        point_DEM.intensity = std::numeric_limits<float>::quiet_NaN ();
        point_DEM.height_variance = std::numeric_limits<float>::quiet_NaN ();
        point_DEM.intensity_variance = std::numeric_limits<float>::quiet_NaN ();
      }

      out_cloud.at (index_column, index_disparity) = point_DEM;
      
    } // index_disparity
  } // index_column
}