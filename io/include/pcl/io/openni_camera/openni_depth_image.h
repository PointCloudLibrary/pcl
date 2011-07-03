/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
 *    Suat Gedikli <gedikli@willowgarage.com>
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef __OPENNI_DEPTH_IMAGE__
#define __OPENNI_DEPTH_IMAGE__

#include <XnCppWrapper.h>

#include "openni_exception.h"
#include <boost/shared_ptr.hpp>

namespace openni_wrapper
{
/**
* @brief This class provides methods to fill a depth or disparity image.
* @author Suat Gedikli
* @date 02.january 2011
*/
class DepthImage
{
public:
  typedef boost::shared_ptr<DepthImage> Ptr;
  typedef boost::shared_ptr<const DepthImage> ConstPtr;

  inline DepthImage (boost::shared_ptr<xn::DepthMetaData> depth_meta_data, float baseline, float focal_length, XnUInt64 shadow_value, XnUInt64 no_sample_value) throw ();
  inline virtual ~DepthImage () throw ();

  inline const xn::DepthMetaData& getDepthMetaData () const throw ();
  void fillDisparityImage (unsigned width, unsigned height, float* disparity_buffer, unsigned line_step = 0) const throw (OpenNIException);
  void fillDepthImage (unsigned width, unsigned height, float* depth_buffer, unsigned line_step = 0) const throw (OpenNIException);
  void fillDepthImageRaw (unsigned width, unsigned height, unsigned short* depth_buffer, unsigned line_step = 0) const throw (OpenNIException);

  inline float getBaseline () const throw ();
  inline float getFocalLength () const throw ();
  inline XnUInt64 getShadowValue () const throw ();
  inline XnUInt64 getNoSampleValue () const throw ();
  inline unsigned getWidth () const throw ();
  inline unsigned getHeight () const throw ();
  inline unsigned getFrameID () const throw ();
  inline unsigned long getTimeStamp () const throw ();
protected:
  boost::shared_ptr<xn::DepthMetaData> depth_md_;
  float baseline_;
  float focal_length_;
  XnUInt64 shadow_value_;
  XnUInt64 no_sample_value_;
};

DepthImage::DepthImage (boost::shared_ptr<xn::DepthMetaData> depth_meta_data, float baseline, float focal_length, XnUInt64 shadow_value, XnUInt64 no_sample_value) throw ()
: depth_md_ (depth_meta_data)
, baseline_ (baseline)
, focal_length_ (focal_length)
, shadow_value_ (shadow_value)
, no_sample_value_ (no_sample_value)
{
}

DepthImage::~DepthImage () throw ()
{
}

const xn::DepthMetaData& DepthImage::getDepthMetaData () const throw ()
{
  return *depth_md_;
}

float DepthImage::getBaseline () const throw ()
{
  return baseline_;
}

float DepthImage::getFocalLength () const throw ()
{
  return focal_length_;
}

XnUInt64 DepthImage::getShadowValue () const throw ()
{
  return shadow_value_;
}

XnUInt64 DepthImage::getNoSampleValue () const throw ()
{
  return no_sample_value_;
}

unsigned DepthImage::getWidth () const throw ()
{
  return depth_md_->XRes ();
}

unsigned DepthImage::getHeight () const throw ()
{
  return depth_md_->YRes ();
}

unsigned DepthImage::getFrameID () const throw ()
{
  return depth_md_->FrameID ();
}

unsigned long DepthImage::getTimeStamp () const throw ()
{
  return (unsigned long) depth_md_->Timestamp ();
}
} // namespace
#endif
#endif //__OPENNI_DEPTH_IMAGE
