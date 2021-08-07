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
 
#pragma once

#include <pcl/pcl_config.h>
#include <pcl/pcl_macros.h>

#include <pcl/io/image.h>

namespace pcl 
{
  namespace io
  { 
    /**
      * @brief This class provides methods to fill a RGB or Grayscale image buffer from underlying RGB24 image.
      * @ingroup io
      */
    class PCL_EXPORTS ImageRGB24 : public pcl::io::Image
    {
      public:

        ImageRGB24 (FrameWrapper::Ptr image_metadata);
        ImageRGB24 (FrameWrapper::Ptr image_metadata, Timestamp timestamp);
        ~ImageRGB24 () noexcept;

        inline Encoding
        getEncoding () const override
        {
          return (RGB);
        }

        void
        fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer, unsigned rgb_line_step = 0) const override;
      
        void
        fillGrayscale (unsigned width, unsigned height, unsigned char* gray_buffer, unsigned gray_line_step = 0) const override;
      
        bool
        isResizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const override;

      private:

        // Struct used for type conversion
        struct RGB888Pixel
        {
          std::uint8_t r;
          std::uint8_t g;
          std::uint8_t b;
        };
    };

  } // namespace
}
