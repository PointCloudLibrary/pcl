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
#ifndef PCL_DISPARITY_MAP_CONVERTER_H_
#define PCL_DISPARITY_MAP_CONVERTER_H_

#include <cstring>
#include <vector>

#pragma warning(disable : 4996 4521)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#pragma warning(default : 4996 4521)

namespace pcl
{
  /** \brief Compute point cloud from the disparity map.
    *
    * Exampe of usage:
    * 
    * \code
    *  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    *   
    *  pcl::DisparityMapConverter<pcl::PointXYZI> dmc;
    *  dmc.setBaseline (0.8387445f);
    *  dmc.setFocalLength (368.534700f);
    *  dmc.setImageCenterX (318.112200f);
    *  dmc.setImageCenterY (224.334900f);
    *  dmc.setDisparityThresholdMin(15.0f);
    *
    *  // Left view of the scene. Must be converted to pcd format with png2pcd.
    *  dmc.loadImage ("left_image.pcd");
    *  // Disparity map of the scene.
    *  dmc.loadDisparityMap ("disparity_map.txt", 640, 480);
    *
    *  dmc.compute(cloud);
    * \endcode
    *
    * \author Timur Ibadov (ibadov.timur@gmail.com)
    * \ingroup stereo
    */
  template <typename PointT>
  class DisparityMapConverter
  {
    protected:
      typedef typename PointCloud<PointT>::Ptr PointCloudPointer;

    public:
      /** \brief DisparityMapConverter constructor. */
      DisparityMapConverter ();
      /** \brief Empty destructor. */
      virtual ~DisparityMapConverter ();

      /** \brief Set x-coordinate of the image center.
        * \param[in] center_x x-coordinate of the image center.
        */
      void
      setImageCenterX (const float center_x);

      /** \brief Get x-coordinate of the image center.
        * \return x-coordinate of the image center.
        */
      float
      getImageCenterX () const;

      /** \brief Set y-coordinate of the image center.
        * \param[in] center_y y-coordinate of the image center.
        */
      void
      setImageCenterY (const float center_y);

      /** \brief Get y-coordinate of the image center.
        * \return y-coordinate of the image center.
        */
      float
      getImageCenterY () const;

      /** \brief Set focal length.
        * \param[in] focal_length the focal length.
        */
      void
      setFocalLength (const float focal_length);

      /** \brief Get focal length.
        * \return the focal length.
        */
      float
      getFocalLength () const;

      /** \brief Set baseline.
        * \param[in] baseline baseline.
        */
      void
      setBaseline (const float baseline);

      /** \brief Get baseline.
        * \return the baseline.
        */
      float
      getBaseline () const;

      /** \brief Set min disparity threshold.
        * \param[in] disparity_threshold_min min disparity threshold.
        */
      void
      setDisparityThresholdMin (const float disparity_threshold_min);

      /** \brief Get min disparity threshold.
        * \return min disparity threshold.
        */
      float
      getDisparityThresholdMin () const;

      /** \brief Set max disparity threshold.
        * \param[in] disparity_threshold_max max disparity threshold.
        */
      void
      setDisparityThresholdMax (const float disparity_threshold_max);

      /** \brief Get max disparity threshold.
        * \return max disparity threshold.
        */
      float
      getDisparityThresholdMax () const;

      /** \brief Load an image, that will be used for coloring of the output cloud.
        *
        * The image must be converted in pcd format. You may use png2pcd tool
        * from pcl (tools/png2pcd.cpp) with parameter "-mode FORCE_COLOR".
        *
        * \param[in] file_name the name of the image file.
        * \return "true" if the image was successfully loaded; "false" otherwise
        */
      bool
      loadImage (const std::string &file_name);

      /** \brief Set an image, that will be used for coloring of the output cloud.
        * \param[in] image the image.
        */
      void
      setImage (const PointCloud<RGB>::Ptr &image);

      /** \brief Get the image, that is used for coloring of the output cloud.
        * \return the image.
        */
      PointCloud<RGB>::Ptr
      getImage ();

      /** \brief Load the disparity map.
        * \param[in] file_name the name of the disparity map file.
        * \return "true" if the disparity map was successfully loaded; "false" otherwise
        */
      bool
      loadDisparityMap (const std::string &file_name);

      /** \brief Load the disparity map and initialize it's size.
        * \param[in] file_name the name of the disparity map file.
        * \param[in] width width of the disparity map.
        * \param[in] height height of the disparity map.
        * \return "true" if the disparity map was successfully loaded; "false" otherwise
        */
      bool
      loadDisparityMap (const std::string &file_name, const size_t width, const size_t height);

      /** \brief Set the disparity map.
        * \param[in] disparity_map the disparity map.
        */
      void
      setDisparityMap(const std::vector<float> &disparity_map);

      /** \brief Set the disparity map and initialize it's size.
        * \param[in] disparity_map the disparity map.
        * \param[in] width width of the disparity map.
        * \param[in] height height of the disparity map.
        * \return "true" if the disparity map was successfully loaded; "false" otherwise
        */
      void
      setDisparityMap(const std::vector<float> &disparity_map, const size_t width, const size_t height);

      /** \brief Get the disparity map.
        * \return the disparity map.
        */
      std::vector<float>
      getDisparityMap ();

      /** \brief Compute the output cloud.
        * \param[out] out_cloud the variable to return the resulting cloud.
        */
      virtual void
      compute (PointCloudPointer &out_cloud);

    protected:
      /** \brief X-coordinate of the image center. */
      float center_x_;
      /** \brief Y-coordinate of the image center. */
      float center_y_;
      /** \brief Focal length. */
      float focal_length_;
      /** \brief Baseline. */
      float baseline_;
      
      /** \brief Color image of the scene. */
      pcl::PointCloud<pcl::RGB>::Ptr image_;

      /** \brief Vector for the disparity map. */
      std::vector<float> disparity_map_;
      /** \brief X-size of the disparity map. */
      size_t disparity_map_width_;
      /** \brief Y-size of the disparity map. */
      size_t disparity_map_height_;

      /** \brief Thresholds of the disparity. */
      float disparity_threshold_min_;
      float disparity_threshold_max_;

      /** \brief Translate point from image coordinates and disparity to 3D-coordinates
        * \param[in] row
        * \param[in] column
        * \param[in] disparity
        * \return the 3D point, that corresponds to the input parametres and the camera calibration.
        */
      PointXYZ 
      translateCoordinates (size_t row, size_t column, float disparity) const;
  };

}
#endif // PCL_DISPARITY_MAP_CONVERTER_H_