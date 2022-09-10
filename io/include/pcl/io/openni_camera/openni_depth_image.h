/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011 Willow Garage, Inc.
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
 *
 */

#pragma once
 
#include <pcl/pcl_config.h>
#include <pcl/memory.h>
#ifdef HAVE_OPENNI

#include "openni.h"

//#include <pcl/pcl_macros.h> // <-- because current header is included in NVCC-compiled code and contains <Eigen/Core>. Consider <pcl/pcl_exports.h>
#include <pcl/pcl_exports.h>
#include "openni_exception.h"

namespace openni_wrapper
{
  /** \brief This class provides methods to fill a depth or disparity image.
    * \author Suat Gedikli
    */
  class PCL_EXPORTS DepthImage
  {
    public:
      using Ptr = pcl::shared_ptr<DepthImage>;
      using ConstPtr = pcl::shared_ptr<const DepthImage>;

      /** \brief Constructor
        * \param[in] depth_meta_data the actual data from the OpenNI library
        * \param[in] baseline the baseline of the "stereo" camera, i.e. the distance between the projector and the IR camera for
        *        Primesense like cameras. e.g. 7.5cm for PSDK5 and PSDK6 reference design.
        * \param[in] focal_length focal length of the "stereo" frame.
        * \param[in] shadow_value defines which values in the depth data are indicating shadow (resulting from the parallax between projector and IR camera)
        * \param[in] no_sample_value defines which values in the depth data are indicating that no depth (disparity) could be determined .
        * \attention The focal length may change, depending whether the depth stream is registered/mapped to the RGB stream or not.
        */
      inline DepthImage (pcl::shared_ptr<xn::DepthMetaData> depth_meta_data, float baseline, float focal_length, XnUInt64 shadow_value, XnUInt64 no_sample_value) noexcept;

      /** \brief Destructor. Never throws an exception. */
      inline virtual ~DepthImage () noexcept;

      /** \brief method to access the internal data structure from OpenNI. If the data is accessed just read-only, then this method is faster than a fillXXX method
        * \return the actual depth data of type xn::DepthMetaData.
        */
      inline const xn::DepthMetaData& 
      getDepthMetaData () const throw ();

      /** \brief fills a user given block of memory with the disparity values with additional nearest-neighbor down-scaling.
        * \param[in] width the width of the desired disparity image.
        * \param[in] height the height of the desired disparity image.
        * \param[in,out] disparity_buffer the float pointer to the actual memory buffer to be filled with the disparity values.
        * \param[in] line_step if only a rectangular sub region of the buffer needs to be filled, then line_step is the
        *        width in bytes (not floats) of the original width of the depth buffer.
        */
      void 
      fillDisparityImage (unsigned width, unsigned height, float* disparity_buffer, unsigned line_step = 0) const;

      /** \brief fills a user given block of memory with the disparity values with additional nearest-neighbor down-scaling.
        * \param[in] width width the width of the desired depth image.
        * \param[in] height height the height of the desired depth image.
        * \param[in,out] depth_buffer the float pointer to the actual memory buffer to be filled with the depth values.
        * \param[in] line_step if only a rectangular sub region of the buffer needs to be filled, then line_step is the
        *        width in bytes (not floats) of the original width of the depth buffer.
        */
      void 
      fillDepthImage (unsigned width, unsigned height, float* depth_buffer, unsigned line_step = 0) const;

      /** \brief fills a user given block of memory with the raw values with additional nearest-neighbor down-scaling.
        * \param[in] width width the width of the desired raw image.
        * \param[in] height height the height of the desired raw image.
        * \param[in,out] depth_buffer the unsigned short pointer to the actual memory buffer to be filled with the raw values.
        * \param[in] line_step if only a rectangular sub region of the buffer needs to be filled, then line_step is the
        *        width in bytes (not floats) of the original width of the depth buffer.
        */
      void 
      fillDepthImageRaw (unsigned width, unsigned height, unsigned short* depth_buffer, unsigned line_step = 0) const;

      /** \brief method to access the baseline of the "stereo" frame that was used to retrieve the depth image.
        * \return baseline in meters
        */
      inline float 
      getBaseline () const throw ();

      /** \brief method to access the focal length of the "stereo" frame that was used to retrieve the depth image.
        * \return focal length in pixels
        */
      inline float 
      getFocalLength () const throw ();

      /** \brief method to access the shadow value, that indicates pixels lying in shadow in the depth image.
        * \return shadow value
        */
      inline XnUInt64 
      getShadowValue () const throw ();

      /** \brief method to access the no-sample value, that indicates pixels where no disparity could be determined for the depth image.
        * \return no-sample value
        */
      inline XnUInt64 
      getNoSampleValue () const throw ();

      /** \return the width of the depth image */
      inline unsigned 
      getWidth () const throw ();

      /** \return the height of the depth image */
      inline unsigned 
      getHeight () const throw ();

      /** \return an ascending id for the depth frame
        * \attention not necessarily synchronized with other streams
        */
      inline unsigned 
      getFrameID () const throw ();

      /** \return a ascending timestamp for the depth frame
        * \attention its not the system time, thus can not be used directly to synchronize different sensors.
        *            But definitely synchronized with other streams
        */
      inline unsigned long 
      getTimeStamp () const throw ();

    protected:
      pcl::shared_ptr<xn::DepthMetaData> depth_md_;
      float baseline_;
      float focal_length_;
      XnUInt64 shadow_value_;
      XnUInt64 no_sample_value_;
  } ;

  DepthImage::DepthImage (pcl::shared_ptr<xn::DepthMetaData> depth_meta_data, float baseline, float focal_length, XnUInt64 shadow_value, XnUInt64 no_sample_value) noexcept
  : depth_md_ (std::move(depth_meta_data))
  , baseline_ (baseline)
  , focal_length_ (focal_length)
  , shadow_value_ (shadow_value)
  , no_sample_value_ (no_sample_value) { }

  DepthImage::~DepthImage () noexcept = default;

  const xn::DepthMetaData&
  DepthImage::getDepthMetaData () const throw ()
  {
    return *depth_md_;
  }

  float
  DepthImage::getBaseline () const throw ()
  {
    return baseline_;
  }

  float
  DepthImage::getFocalLength () const throw ()
  {
    return focal_length_;
  }

  XnUInt64
  DepthImage::getShadowValue () const throw ()
  {
    return shadow_value_;
  }

  XnUInt64
  DepthImage::getNoSampleValue () const throw ()
  {
    return no_sample_value_;
  }

  unsigned
  DepthImage::getWidth () const throw ()
  {
    return depth_md_->XRes ();
  }

  unsigned
  DepthImage::getHeight () const throw ()
  {
    return depth_md_->YRes ();
  }

  unsigned
  DepthImage::getFrameID () const throw ()
  {
    return depth_md_->FrameID ();
  }

  unsigned long
  DepthImage::getTimeStamp () const throw ()
  {
    return static_cast<unsigned long> (depth_md_->Timestamp ());
  }
} // namespace
#endif
