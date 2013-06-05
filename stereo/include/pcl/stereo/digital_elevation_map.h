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
  /** \brief A point structure representing Digital Elevation Map.
    * \ingroup stereo
    */
  struct EIGEN_ALIGN16 _PointDEM
  {
    PCL_ADD_POINT4D;
    float intensity;
    float height_variance;
    float intensity_variance;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct PointDEM : public _PointDEM
  {
    inline PointDEM()
    {
      x = y = z = 0.0f;
      intensity = 0.0f;
      height_variance = intensity_variance = 0.0f;
    }
    inline bool
    isValid()
    {
      return (height_variance >= 0.0f && intensity_variance >= 0.0f);
    }
  };

  /** \brief Build a Digital Elevation Map in the column-disparity space from a disparity map and color image of the scene.
    *
    * Exampe of usage:
    * 
    * \code
    *  pcl::PointCloud<pcl::PointDEM>::Ptr cloud;
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
    *  // Left view of the scene. Must be converted to pcd format with png2pcd.
    *  demb.loadImage ("left_image.pcd");
    *  // Disparity map of the scene.
    *  demb.loadDisparityMap ("disparity_map.txt", 640, 480);
    *
    *  demb.compute(cloud);
    * \endcode
    *
    * \author Timur Ibadov (ibadov.timur@gmail.com)
    * \ingroup stereo
    */
  template <typename PointT>
  class DigitalElevationMapBuilder : public DisparityMapConverter <PointT>
  {
    public:
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

      /** \brief Get column resolution od the DEM.
        * \return column resolution od the DEM.
        */
      size_t
      getColumnResolution () const;

      /** \brief Get disparity resolution od the DEM.
        * \return disparity resolution od the DEM.
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

      /** \brief Compute Digital Elevation Map.
        * \param[out] out_cloud the variable to return the resulting cloud.
        */
      void 
      compute (PointCloudPointer &out_cloud);

    protected:
      /** \brief Column resolution of the DEM. */
      size_t resolution_column_;
      /** \brief disparuty resolution of the DEM. */
      size_t resolution_disparity_;

      /** \brief Minimum amount of points in a DEM's cell. */
      size_t min_points_in_cell_;
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT (_PointDEM,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, height_variance, height_variance)
                                    (float, intensity_variance, intensity_variance)
 )



#endif // PCL_DIGITAL_ELEVATION_MAP_H_