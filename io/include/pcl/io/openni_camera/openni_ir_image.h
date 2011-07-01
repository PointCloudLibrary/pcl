/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
 *    Patrick Mihelich <mihelich@willowgarage.com>
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
#ifndef __OPENNI_IR_IMAGE__
#define __OPENNI_IR_IMAGE__

#include <XnCppWrapper.h>
#include "openni_exception.h"
#include <boost/shared_ptr.hpp>

namespace openni_wrapper
{

/**
 * @brief Class containing just a reference to IR meta data.
 */
class IRImage
{
public:
  typedef boost::shared_ptr<IRImage> Ptr;
  typedef boost::shared_ptr<const IRImage> ConstPtr;

  inline IRImage (boost::shared_ptr<xn::IRMetaData> ir_meta_data) throw ();
  inline virtual ~IRImage () throw ();

  void fillRaw (unsigned width, unsigned height, unsigned short* ir_buffer, unsigned line_step = 0) const throw (OpenNIException);

  inline unsigned getWidth () const throw ();
  inline unsigned getHeight () const throw ();
  inline unsigned getFrameID () const throw ();
  inline unsigned long getTimeStamp () const throw ();
  inline const xn::IRMetaData& getMetaData () const throw ();
	
protected:
  boost::shared_ptr<xn::IRMetaData> ir_md_;
};

IRImage::IRImage (boost::shared_ptr<xn::IRMetaData> ir_meta_data) throw ()
: ir_md_ (ir_meta_data)
{
}

IRImage::~IRImage () throw ()
{
}

unsigned IRImage::getWidth () const throw ()
{
  return ir_md_->XRes ();
}

unsigned IRImage::getHeight () const throw ()
{
  return ir_md_->YRes ();
}

unsigned IRImage::getFrameID () const throw ()
{
  return ir_md_->FrameID ();
}

unsigned long IRImage::getTimeStamp () const throw ()
{
  return ir_md_->Timestamp ();
}

const xn::IRMetaData& IRImage::getMetaData () const throw ()
{
	return *ir_md_;
}
} // namespace
#endif //__OPENNI_IR_IMAGE__
