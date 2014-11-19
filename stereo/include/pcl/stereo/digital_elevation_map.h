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
#ifndef PCL_DIGITAL_ELEVATION_MAP_H_
#define PCL_DIGITAL_ELEVATION_MAP_H_

#pragma warning(disable : 4996)
#include <pcl/point_types.h>
#include <pcl/stereo/disparity_map_converter.h>
#pragma warning(default : 4996)

namespace pcl
{ 
  /** \brief Build a Digital Elevation Map in the column-disparity space from a disparity map and a color image of the scene.
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
    *  pcl::DigitalElevationMapBuilder<pcl::PointDEM> demb;
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
  template <typename PointT>
  class DigitalElevationMapBuilder : public DisparityMapConverter <PointT>
  {
    public:
      using DisparityMapConverter<PointT>::baseline_;
      using DisparityMapConverter<PointT>::translateCoordinates;
      using DisparityMapConverter<PointT>::image_;
      using DisparityMapConverter<PointT>::disparity_map_;
      using DisparityMapConverter<PointT>::disparity_map_width_;
      using DisparityMapConverter<PointT>::disparity_map_height_;
      using DisparityMapConverter<PointT>::disparity_threshold_min_;
      using DisparityMapConverter<PointT>::disparity_threshold_max_;
    
      /** \brief DigitalElevationMapBuilder constructor. */
      DigitalElevationMapBuilder ();
      /** \brief Empty destructor. */
      virtual ~DigitalElevationMapBuilder ();

      /** \brief Set resolution of the DEM.
        * \param[in] resolution_column the column resolution.
        * \param[in] resolution_disparity the disparity resolution.
        */
      void
      setResolution (size_t resolution_column, size_t resolution_disparity);

      /** \brief Get column resolution of the DEM.
        * \return column resolution of the DEM.
        */
      size_t
      getColumnResolution () const;

      /** \brief Get disparity resolution of the DEM.
        * \return disparity resolution of the DEM.
        */
      size_t
      getDisparityResolution () const;

      /** \brief Set minimum amount of points in a DEM's cell.
        * \param[in] min_points_in_cell minimum amount of points in a DEM's cell.
        */
      void
      setMinPointsInCell(size_t min_points_in_cell);

      /** \brief Get minimum amount of points in a DEM's cell.
        * \return minimum amount of points in a DEM's cell.
        */
      size_t
      getMinPointsInCell() const;

      /** \brief Compute the Digital Elevation Map.
        * \param[out] out_cloud the variable to return the resulting cloud.
        */
      void 
      compute (pcl::PointCloud<PointT> &out_cloud);

    protected:
      /** \brief Column resolution of the DEM. */
      size_t resolution_column_;
      /** \brief disparity resolution of the DEM. */
      size_t resolution_disparity_;

      /** \brief Minimum amount of points in a DEM's cell. */
      size_t min_points_in_cell_;
  };

  /** \brief Type for histograms for computing values of the bins in the Digital Elevation Map.
    *
    * \author Timur Ibadov (ibadov.timur@gmail.com)
    * \ingroup stereo
    */
  class FeatureHistogram
  {
    public:
      /** \brief Public constructor. */
      FeatureHistogram (size_t number_of_bins);
      /** \brief Public destructor. */
      virtual ~FeatureHistogram ();

      /** \brief Set min and max thresholds. */
      void
      setThresholds(float min, float max);

      /** \brief Get the lower threshold. */
      float
      getThresholdMin() const;

      /** \brief Get the upper threshold. */
      float
      getThresholdMax() const;

      /** \brief Get the number of elements was added to the histogram. */
      size_t
      getNumberOfElements() const;

      /** \brief Get number of bins in the histogram. */
      size_t
      getNumberOfBins() const;

      /** \brief Increase a bin, that corresponds the value. */
      void
      addValue (float value);

      /** \brief Get value, corresponds to the greatest bin. */
      float
      getMeanValue ();

      /** \brief Get variance of the value. */
      float
      getVariance (float mean);

    protected:
      /** \brief Vector, that contain the histogram. */
      std::vector <unsigned> histogram_;

      /** \brief Min threshold. */
      float threshold_min_;
      /** \brief Max threshold. */
      float threshold_max_;
      /** \brief "Width" of a bin. */
      float step_;

      /** \brief Number of values was added to the histogram. */
      size_t number_of_elements_;

      /** \brief Number of bins. */
      size_t number_of_bins_;
  };
}

//#include <pcl/stereo/impl/digital_elevation_map.hpp>
//class PCL_EXPORTS pcl::FeatureHistogram;

#endif // PCL_DIGITAL_ELEVATION_MAP_H_
