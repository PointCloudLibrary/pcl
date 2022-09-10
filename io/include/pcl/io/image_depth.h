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

#include <chrono>

#include <pcl/pcl_config.h>
#include <pcl/pcl_exports.h>

#include<pcl/io/image_metadata_wrapper.h>

namespace pcl
{
  namespace io
  {
    /** \brief This class provides methods to fill a depth or disparity image.
      */
    class PCL_EXPORTS DepthImage
    {
      public:
        using Ptr = shared_ptr<DepthImage>;
        using ConstPtr = shared_ptr<const DepthImage>;

        using Clock = std::chrono::high_resolution_clock;
        using Timestamp = std::chrono::high_resolution_clock::time_point;

        /** \brief Constructor
          * \param[in] depth_metadata the actual data from the OpenNI library
          * \param[in] baseline the baseline of the "stereo" camera, i.e. the distance between the projector and the IR camera for
          *        Primesense like cameras. e.g. 7.5cm for PSDK5 and PSDK6 reference design.
          * \param[in] focal_length focal length of the "stereo" frame.
          * \param[in] shadow_value defines which values in the depth data are indicating shadow (resulting from the parallax between projector and IR camera)
          * \param[in] no_sample_value defines which values in the depth data are indicating that no depth (disparity) could be determined .
          * \attention The focal length may change, depending whether the depth stream is registered/mapped to the RGB stream or not.
          */
        DepthImage (FrameWrapper::Ptr depth_metadata, float baseline, float focal_length, std::uint64_t shadow_value, std::uint64_t no_sample_value);
        DepthImage (FrameWrapper::Ptr depth_metadata, float baseline, float focal_length, std::uint64_t shadow_value, std::uint64_t no_sample_value, Timestamp time);

        /** \brief Destructor. Never throws an exception. */
        ~DepthImage ();

        /** \brief method to access the internal data structure from OpenNI. If the data is accessed just read-only, then this method is faster than a fillXXX method
          * \return the actual depth data of type openni::VideoFrameRef.
          */
        const FrameWrapper::Ptr
        getMetaData () const;

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
        float
        getBaseline () const;

        /** \brief method to access the focal length of the "stereo" frame that was used to retrieve the depth image.
          * \return focal length in pixels
          */
        float
        getFocalLength () const;

        /** \brief method to access the shadow value, that indicates pixels lying in shadow in the depth image.
          * \return shadow value
          */
        std::uint64_t
        getShadowValue () const;

        /** \brief method to access the no-sample value, that indicates pixels where no disparity could be determined for the depth image.
          * \return no-sample value
          */
        std::uint64_t
        getNoSampleValue () const;

        /** \return the width of the depth image */
        unsigned
        getWidth () const;

        /** \return the height of the depth image */
        unsigned
        getHeight () const;

        /** \return an ascending id for the depth frame
          * \attention not necessarily synchronized with other streams
          */
        unsigned
        getFrameID () const;

        /** \return a ascending timestamp for the depth frame
          * \attention its not the system time, thus can not be used directly to synchronize different sensors.
          *            But definitely synchronized with other streams
          */
        std::uint64_t
        getTimestamp () const;

        Timestamp
        getSystemTimestamp () const;

        // Get a const pointer to the raw depth buffer
        const unsigned short*
        getData ();

        // Data buffer size in bytes
        int
        getDataSize () const;

        // Size of each row, including any padding
        inline unsigned
        getStep() const
        {
          return (getDataSize() / getHeight());
        }

      protected:
        pcl::io::FrameWrapper::Ptr wrapper_;

        float baseline_;
        float focal_length_;
        std::uint64_t shadow_value_;
        std::uint64_t no_sample_value_;
        Timestamp timestamp_;
    };

}} // namespace
