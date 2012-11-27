/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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
#include <pcl/pcl_config.h>
#ifdef HAVE_OPENNI

#ifndef __OPENNI_IMAGE_BAYER_GRBG__
#define __OPENNI_IMAGE_BAYER_GRBG__
#include <pcl/pcl_macros.h>
#include "openni_image.h"

namespace openni_wrapper
{
  /** \brief This class provides methods to fill a RGB or Grayscale image buffer from underlying Bayer pattern image.
    * \author Suat Gedikli <gedikli@willowgarage.com>
    * \ingroup io
    */
  class PCL_EXPORTS ImageBayerGRBG : public Image
  {
    public:
      typedef enum
      {
        Bilinear = 0,
        EdgeAware,
        EdgeAwareWeighted
      } DebayeringMethod;

      ImageBayerGRBG (boost::shared_ptr<xn::ImageMetaData> image_meta_data, DebayeringMethod method) throw ();
      virtual ~ImageBayerGRBG () throw ();

      inline virtual Encoding
      getEncoding () const
      {
        return (BAYER_GRBG);
      }

      virtual void fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer, unsigned rgb_line_step = 0) const;
      virtual void fillGrayscale (unsigned width, unsigned height, unsigned char* gray_buffer, unsigned gray_line_step = 0) const;
      virtual bool isResizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const;
      inline void setDebayeringMethod (const DebayeringMethod& method) throw ();
      inline DebayeringMethod getDebayeringMethod () const throw ();
      inline static bool resizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height);


    protected:
      DebayeringMethod debayering_method_;
  };

  void
  ImageBayerGRBG::setDebayeringMethod (const ImageBayerGRBG::DebayeringMethod& method) throw ()
  {
    debayering_method_ = method;
  }

  ImageBayerGRBG::DebayeringMethod
  ImageBayerGRBG::getDebayeringMethod () const throw ()
  {
    return debayering_method_;
  }

  bool
  ImageBayerGRBG::resizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height)
  {
    return (output_width <= input_width && output_height <= input_height && input_width % output_width == 0 && input_height % output_height == 0 );
  }
} // namespace

#endif
#endif // __OPENNI_IMAGE__
