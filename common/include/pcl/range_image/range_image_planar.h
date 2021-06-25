/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#pragma once

#include <pcl/range_image/range_image.h>

namespace pcl
{
  /** \brief @b RangeImagePlanar is derived from the original range image and differs from it because it's not a 
    * spherical projection, but using a projection plane (as normal cameras do), therefore being better applicable 
    * for range sensors that already provide a range image by themselves (stereo cameras, ToF-cameras), so that
    * a conversion to point cloud and then to a spherical range image becomes unnecessary.
    * \author Bastian Steder 
    * \ingroup range_image
    */
  class RangeImagePlanar : public RangeImage
  {
    public:
      // =====TYPEDEFS=====
      using BaseClass = RangeImage;
      using Ptr = shared_ptr<RangeImagePlanar>;
      using ConstPtr = shared_ptr<const RangeImagePlanar>;
      
      // =====CONSTRUCTOR & DESTRUCTOR=====
      /** Constructor */
      PCL_EXPORTS RangeImagePlanar ();
      /** Destructor */
      PCL_EXPORTS ~RangeImagePlanar () override;

      /** Return a newly created RangeImagePlanar.
       *  Reimplementation to return an image of the same type. */
      RangeImage* 
      getNew () const override { return new RangeImagePlanar; }

      /** Copy *this to other. Derived version - also copying additional RangeImagePlanar members */
      PCL_EXPORTS void
      copyTo (RangeImage& other) const override;
      
      // =====PUBLIC METHODS=====
      /** \brief Get a boost shared pointer of a copy of this */
      inline Ptr 
      makeShared () { return Ptr (new RangeImagePlanar (*this)); } 
      
      /** \brief Create the image from an existing disparity image.
        * \param disparity_image the input disparity image data
        * \param di_width the disparity image width
        * \param di_height the disparity image height
        * \param focal_length the focal length of the primary camera that generated the disparity image
        * \param base_line the baseline of the stereo pair that generated the disparity image
        * \param desired_angular_resolution If this is set, the system will skip as many pixels as necessary to get as
        *         close to this angular resolution as possible while not going over this value (the density will not be
        *         lower than this value). The value is in radians per pixel. 
        */
      PCL_EXPORTS void
      setDisparityImage (const float* disparity_image, int di_width, int di_height,
                         float focal_length, float base_line, float desired_angular_resolution=-1);
      
      /** Create the image from an existing depth image.
        * \param depth_image the input depth image data as float values
        * \param di_width the disparity image width 
        * \param di_height the disparity image height
        * \param di_center_x the x-coordinate of the camera's center of projection
        * \param di_center_y the y-coordinate of the camera's center of projection
        * \param di_focal_length_x the camera's focal length in the horizontal direction
        * \param di_focal_length_y the camera's focal length in the vertical direction
        * \param desired_angular_resolution If this is set, the system will skip as many pixels as necessary to get as
        *         close to this angular resolution as possible while not going over this value (the density will not be
        *         lower than this value). The value is in radians per pixel.
        */
      PCL_EXPORTS void
      setDepthImage (const float* depth_image, int di_width, int di_height, float di_center_x, float di_center_y,
                     float di_focal_length_x, float di_focal_length_y, float desired_angular_resolution=-1);
      
      /** Create the image from an existing depth image.
        * \param depth_image the input disparity image data as short values describing millimeters
        * \param di_width the disparity image width 
        * \param di_height the disparity image height
        * \param di_center_x the x-coordinate of the camera's center of projection
        * \param di_center_y the y-coordinate of the camera's center of projection
        * \param di_focal_length_x the camera's focal length in the horizontal direction
        * \param di_focal_length_y the camera's focal length in the vertical direction
        * \param desired_angular_resolution If this is set, the system will skip as many pixels as necessary to get as
        *         close to this angular resolution as possible while not going over this value (the density will not be
        *         lower than this value). The value is in radians per pixel.
        */
      PCL_EXPORTS void
      setDepthImage (const unsigned short* depth_image, int di_width, int di_height, float di_center_x, float di_center_y,
                     float di_focal_length_x, float di_focal_length_y, float desired_angular_resolution=-1);
      
      /** Create the image from an existing point cloud.
        * \param point_cloud the source point cloud
        * \param di_width the disparity image width 
        * \param di_height the disparity image height
        * \param di_center_x the x-coordinate of the camera's center of projection
        * \param di_center_y the y-coordinate of the camera's center of projection
        * \param di_focal_length_x the camera's focal length in the horizontal direction
        * \param di_focal_length_y the camera's focal length in the vertical direction
        * \param sensor_pose the pose of the virtual depth camera
        * \param coordinate_frame the used coordinate frame of the point cloud
        * \param noise_level what is the typical noise of the sensor - is used for averaging in the z-buffer
        * \param min_range minimum range to consifder points
        */
      template <typename PointCloudType> void
      createFromPointCloudWithFixedSize (const PointCloudType& point_cloud,
                                         int di_width, int di_height, float di_center_x, float di_center_y,
                                         float di_focal_length_x, float di_focal_length_y,
                                         const Eigen::Affine3f& sensor_pose,
                                         CoordinateFrame coordinate_frame=CAMERA_FRAME, float noise_level=0.0f,
                                         float min_range=0.0f);
      
      // Since we reimplement some of these overloaded functions, we have to do the following:
      using RangeImage::calculate3DPoint;
      using RangeImage::getImagePoint;
      
      /** \brief Calculate the 3D point according to the given image point and range
        * \param image_x the x image position
        * \param image_y the y image position
        * \param range the range
        * \param point the resulting 3D point
        * \note Implementation according to planar range images (compared to spherical as in the original)
        */
      inline void
      calculate3DPoint (float image_x, float image_y, float range, Eigen::Vector3f& point) const override;
      
      /** \brief Calculate the image point and range from the given 3D point
        * \param point the resulting 3D point
        * \param image_x the resulting x image position
        * \param image_y the resulting y image position
        * \param range the resulting range
        * \note Implementation according to planar range images (compared to spherical as in the original)
        */
      inline void 
      getImagePoint (const Eigen::Vector3f& point, float& image_x, float& image_y, float& range) const override;
      
      /** Get a sub part of the complete image as a new range image.
        * \param sub_image_image_offset_x - The x coordinate of the top left pixel of the sub image.
        *                         This is always according to absolute 0,0 meaning -180°,-90°
        *                         and it is already in the system of the new image, so the
        *                         actual pixel used in the original image is
        *                         combine_pixels* (image_offset_x-image_offset_x_)
        * \param sub_image_image_offset_y - Same as image_offset_x for the y coordinate
        * \param sub_image_width - width of the new image
        * \param sub_image_height - height of the new image
        * \param combine_pixels - shrinking factor, meaning the new angular resolution
        *                         is combine_pixels times the old one
        * \param sub_image - the output image
        */
      PCL_EXPORTS void
      getSubImage (int sub_image_image_offset_x, int sub_image_image_offset_y, int sub_image_width,
                   int sub_image_height, int combine_pixels, RangeImage& sub_image) const override;
      
      //! Get a range image with half the resolution
      PCL_EXPORTS void 
      getHalfImage (RangeImage& half_image) const override;
      
      //! Getter for the focal length in X
      inline float
      getFocalLengthX () const { return focal_length_x_; }
      
      //! Getter for the focal length in Y
      inline float
      getFocalLengthY () const { return focal_length_y_; }
      
      //! Getter for the principal point in X
      inline float
      getCenterX () const { return center_x_; }
      
      //! Getter for the principal point in Y
      inline float
      getCenterY () const { return center_y_; }


    protected:
      float focal_length_x_, focal_length_y_; //!< The focal length of the image in pixels
      float focal_length_x_reciprocal_, focal_length_y_reciprocal_;  //!< 1/focal_length -> for internal use
      float center_x_, center_y_;      //!< The principle point of the image
  };
}  // namespace end


#include <pcl/range_image/impl/range_image_planar.hpp>  // Definitions of templated and inline functions
