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
#ifdef HAVE_OPENNI

#ifndef __OPENNI_IMAGE_YUV422__
#define __OPENNI_IMAGE_YUV422__
#include <pcl/pcl_macros.h>
#include "openni_image.h"

namespace openni_wrapper
{

  /**
   * @brief Concrete implementation of the interface Image for a YUV 422 image used by Primesense devices.
   * @author Suat Gedikli
   * @date 02.january 2011
   * @ingroup io
   */
  class PCL_EXPORTS ImageYUV422 : public Image
  {
  public:
    ImageYUV422 (boost::shared_ptr<xn::ImageMetaData> image_meta_data) throw ();
    virtual ~ImageYUV422 () throw ();

    inline virtual Encoding
    getEncoding () const
    {
      return (YUV422);
    }

    virtual bool isResizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const;
    virtual void fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer, unsigned rgb_line_step = 0) const;
    virtual void fillGrayscale (unsigned width, unsigned height, unsigned char* gray_buffer, unsigned gray_line_step = 0) const;
    inline static bool resizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height);
  } ;

  bool
  ImageYUV422::resizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height)
  {
    return (output_width <= input_width && output_height <= input_height && input_width % output_width == 0 && input_height % output_height == 0);
  }
} // namespace

#endif
#endif // __OPENNI_IMAGE__
