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

#ifndef __OPENNI_IMAGE__
#define __OPENNI_IMAGE__

#include <pcl/pcl_exports.h>
#include "openni.h"
#include "openni_exception.h"
#include <pcl/io/boost.h>

namespace openni_wrapper
{

  /**
   * @brief Image class containing just a reference to image meta data. Thus this class
   * just provides an interface to fill a RGB or Grayscale image buffer.
   * @author Suat Gedikli
   * @date 02.january 2011
   * @param[in] image_meta_data
   * @ingroup io
   */
  class PCL_EXPORTS Image
  {
  public:
    typedef boost::shared_ptr<Image> Ptr;
    typedef boost::shared_ptr<const Image> ConstPtr;

    typedef enum
    {
      BAYER_GRBG,
      YUV422,
      RGB
    } Encoding;

    /**
     * @author Suat Gedikli
     * @brief Constructor
     * @param[in] image_meta_data the actual image data from the OpenNI driver
     */
    inline Image (boost::shared_ptr<xn::ImageMetaData> image_meta_data) throw ();

    /**
     * @author Suat Gedikli
     * @brief virtual Destructor that never throws an exception.
     */
    inline virtual ~Image () throw ();

    /**
     * @author Suat Gedikli
     * @param[in] input_width width of input image
     * @param[in] input_height height of input image
     * @param[in] output_width width of desired output image
     * @param[in] output_height height of desired output image
     * @return whether the resizing is supported or not.
     */
    virtual bool isResizingSupported (unsigned input_width, unsigned input_height,
                                      unsigned output_width, unsigned output_height) const = 0;

    /**
     * @author Suat Gedikli
     * @brief fills a user given buffer with the RGB values, with an optional nearest-neighbor down sampling and an optional subregion
     * @param[in] width desired width of output image.
     * @param[in] height desired height of output image.
     * @param[in,out] rgb_buffer the output RGB buffer.
     * @param[in] rgb_line_step optional line step in bytes to allow the output in a rectangular subregion of the output buffer.
     */
    virtual void fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer,
                          unsigned rgb_line_step = 0) const = 0;

    /**
     * @author Suat Gedikli
     * @brief returns the encoding of the native data.
     * @return encoding
     */
    virtual Encoding getEncoding () const = 0;

    /**
     * @author Suat Gedikli
     * @brief fills a user given buffer with the raw values.
     * @param[in,out] rgb_buffer
     */
    inline void
    fillRaw (unsigned char* rgb_buffer) const throw ()
    {
      memcpy (rgb_buffer, image_md_->Data (), image_md_->DataSize ());
    }

    /**
     * @author Suat Gedikli
     * @brief fills a user given buffer with the gray values, with an optional nearest-neighbor down sampling and an optional subregion
     * @param[in] width desired width of output image.
     * @param[in] height desired height of output image.
     * @param[in,out] gray_buffer the output gray buffer.
     * @param[in] gray_line_step optional line step in bytes to allow the output in a rectangular subregion of the output buffer.
     */
    virtual void fillGrayscale (unsigned width, unsigned height, unsigned char* gray_buffer,
                                unsigned gray_line_step = 0) const = 0;

    /**
     * @author Suat Gedikli
     * @return width of the image
     */
    inline unsigned getWidth () const throw ();

    /**
     * @author Suat Gedikli
     * @return height of the image
     */
    inline unsigned getHeight () const throw ();

    /**
     * @author Suat Gedikli
     * @return frame id of the image.
     * @note frame ids are ascending, but not necessarily synch'ed with other streams
     */
    inline unsigned getFrameID () const throw ();

    /**
     * @author Suat Gedikli
     * @return the time stamp of the image
     * @note the time value is not synche'ed with the system time
     */
    inline unsigned long getTimeStamp () const throw ();

    /**
     * @author Suat Gedikli
     * @return the actual data in native OpenNI format.
     */
    inline const xn::ImageMetaData& getMetaData () const throw ();

  protected:
    boost::shared_ptr<xn::ImageMetaData> image_md_;
  } ;

  Image::Image (boost::shared_ptr<xn::ImageMetaData> image_meta_data) throw ()
  : image_md_ (image_meta_data)
  {
  }

  Image::~Image () throw () { }

  unsigned
  Image::getWidth () const throw ()
  {
    return image_md_->XRes ();
  }

  unsigned
  Image::getHeight () const throw ()
  {
    return image_md_->YRes ();
  }

  unsigned
  Image::getFrameID () const throw ()
  {
    return image_md_->FrameID ();
  }

  unsigned long
  Image::getTimeStamp () const throw ()
  {
    return static_cast<unsigned long> (image_md_->Timestamp ());
  }

  const xn::ImageMetaData&
  Image::getMetaData () const throw ()
  {
    return *image_md_;
  }
} // namespace
#endif
#endif //__OPENNI_IMAGE__
