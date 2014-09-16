/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
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
#include <pcl/pcl_config.h>
#ifndef PCL_IO_IMAGE_H_
#define PCL_IO_IMAGE_H_

#include <pcl/pcl_exports.h>
#include <pcl/io/boost.h>
#include <boost/chrono.hpp>

#include <pcl/io/image_metadata_wrapper.h>

namespace pcl
{
  namespace io
  { 

    /**
    * @brief Image interface class providing an interface to fill a RGB or Grayscale image buffer.
    * @param[in] image_metadata
    * @ingroup io
    */
    class PCL_EXPORTS Image
    {
      public:
        typedef boost::shared_ptr<Image> Ptr;
        typedef boost::shared_ptr<const Image> ConstPtr;

        typedef boost::chrono::high_resolution_clock Clock;
        typedef boost::chrono::high_resolution_clock::time_point Timestamp;

        typedef enum
        {
          BAYER_GRBG,
          YUV422,
          RGB
        } Encoding;

        Image (FrameWrapper::Ptr image_metadata)
          : wrapper_ (image_metadata)
          , timestamp_ (Clock::now ())
        {}

        Image (FrameWrapper::Ptr image_metadata, Timestamp time)
          : wrapper_ (image_metadata)
          , timestamp_ (time)
        {}

        /**
        * @brief virtual Destructor that never throws an exception.
        */
        inline virtual ~Image ()
        {}

        /**
        * @param[in] input_width width of input image
        * @param[in] input_height height of input image
        * @param[in] output_width width of desired output image
        * @param[in] output_height height of desired output image
        * @return wheter the resizing is supported or not.
        */
        virtual bool
        isResizingSupported (unsigned input_width, unsigned input_height,
          unsigned output_width, unsigned output_height) const = 0;

        /**
        * @brief fills a user given buffer with the RGB values, with an optional nearest-neighbor down sampling and an optional subregion
        * @param[in] width desired width of output image.
        * @param[in] height desired height of output image.
        * @param[in,out] rgb_buffer the output RGB buffer.
        * @param[in] rgb_line_step optional line step in bytes to allow the output in a rectangular subregion of the output buffer.
        */
        virtual void
        fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer, unsigned rgb_line_step = 0) const = 0;

        /**
        * @brief returns the encoding of the native data.
        * @return encoding
        */
        virtual Encoding
        getEncoding () const = 0;

        /**
        * @brief fills a user given buffer with the raw values.
        * @param[in,out] rgb_buffer
        */
        virtual void
        fillRaw (unsigned char* rgb_buffer) const
        {
          memcpy (rgb_buffer, wrapper_->getData (), wrapper_->getDataSize ());
        }

        /**
        * @brief fills a user given buffer with the gray values, with an optional nearest-neighbor down sampling and an optional subregion
        * @param[in] width desired width of output image.
        * @param[in] height desired height of output image.
        * @param[in,out] gray_buffer the output gray buffer.
        * @param[in] gray_line_step optional line step in bytes to allow the output in a rectangular subregion of the output buffer.
        */
        virtual void
        fillGrayscale (unsigned width, unsigned height, unsigned char* gray_buffer,
          unsigned gray_line_step = 0) const = 0;

        /**
        * @return width of the image
        */
        unsigned
        getWidth () const
        {
          return (wrapper_->getWidth ());
        }

        /**
        * @return height of the image
        */
        unsigned
        getHeight () const
        {
          return (wrapper_->getHeight ());
        }

        /**
        * @return frame id of the image.
        * @note frame ids are ascending, but not necessarily synchronized with other streams
        */
        unsigned
        getFrameID () const
        {
          return (wrapper_->getFrameID ());
        }

        /**
        * @return the timestamp of the image
        * @note the time value is not synchronized with the system time
        */
        pcl::uint64_t
        getTimestamp () const
        {
          return (wrapper_->getTimestamp ());
        }


        /**
        * @return the timestamp of the image
        * @note the time value *is* synchronized with the system time.
        */
        Timestamp
        getSystemTimestamp () const
        {
          return (timestamp_);
        }

        // Get a const pointer to the raw depth buffer
        const void*
        getData ()
        {
          return (wrapper_->getData ());
        }

        // Data buffer size in bytes
        int
        getDataSize () const
        {
          return (wrapper_->getDataSize ());
        }

        // Size of each row, including any padding
        inline unsigned
        getStep() const
        {
          return (getDataSize() / getHeight());
        }

      protected:
        FrameWrapper::Ptr wrapper_;
        Timestamp timestamp_;
    };

  } // namespace
}

#endif //PCL_IO_IMAGE_H_
