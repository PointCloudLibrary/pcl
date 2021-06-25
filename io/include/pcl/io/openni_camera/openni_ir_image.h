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
#ifndef __OPENNI_IR_IMAGE__
#define __OPENNI_IR_IMAGE__

#include <pcl/pcl_macros.h>
#include <pcl/memory.h>
#include "openni.h"
#include "openni_exception.h"

namespace openni_wrapper
{

/**
 * @brief Class containing just a reference to IR meta data.
 * @author Patrick Mihelich <mihelich@willowgarage.com>, Suat Gedikli <gedikli@willowgarage.com>
 */
class PCL_EXPORTS IRImage
{
public:
  using Ptr = pcl::shared_ptr<IRImage>;
  using ConstPtr = pcl::shared_ptr<const IRImage>;

  inline IRImage (pcl::shared_ptr<xn::IRMetaData> ir_meta_data) noexcept;
  inline virtual ~IRImage () noexcept;

  void fillRaw (unsigned width, unsigned height, unsigned short* ir_buffer, unsigned line_step = 0) const;

  inline unsigned getWidth () const throw ();
  inline unsigned getHeight () const throw ();
  inline unsigned getFrameID () const throw ();
  inline unsigned long getTimeStamp () const throw ();
  inline const xn::IRMetaData& getMetaData () const throw ();

protected:
  pcl::shared_ptr<xn::IRMetaData> ir_md_;
};

IRImage::IRImage (pcl::shared_ptr<xn::IRMetaData> ir_meta_data) noexcept
: ir_md_ (std::move(ir_meta_data))
{
}

IRImage::~IRImage () noexcept = default;

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
  return static_cast<unsigned long> (ir_md_->Timestamp ());
}

const xn::IRMetaData& IRImage::getMetaData () const throw ()
{
	return *ir_md_;
}
} // namespace
#endif //__OPENNI_IR_IMAGE__
