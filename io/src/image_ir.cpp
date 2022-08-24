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
#include <pcl/io/image_ir.h>

#include <pcl/io/io_exception.h>

using pcl::io::FrameWrapper;
using pcl::io::IOException;

pcl::io::IRImage::IRImage (FrameWrapper::Ptr ir_metadata)
  : wrapper_(std::move(ir_metadata))
  , timestamp_(Clock::now ())
{}


pcl::io::IRImage::IRImage (FrameWrapper::Ptr ir_metadata, Timestamp time)
  : wrapper_(std::move(ir_metadata))
  , timestamp_(time)
{}


const unsigned short*
pcl::io::IRImage::getData ()
{
  return ( static_cast<const unsigned short*> (wrapper_->getData ()) );
}


int
pcl::io::IRImage::getDataSize () const
{
  return (wrapper_->getDataSize ());
}


const FrameWrapper::Ptr
pcl::io::IRImage::getMetaData () const
{
  return (wrapper_);
}


unsigned
pcl::io::IRImage::getWidth () const
{
  return (wrapper_->getWidth ());
}


unsigned
pcl::io::IRImage::getHeight () const
{
  return (wrapper_->getHeight ());
}


unsigned
pcl::io::IRImage::getFrameID () const
{
  return (wrapper_->getFrameID ());
}


std::uint64_t
pcl::io::IRImage::getTimestamp () const
{
  return (wrapper_->getTimestamp ());
}


pcl::io::IRImage::Timestamp
pcl::io::IRImage::getSystemTimestamp () const
{
  return (timestamp_);
}


void pcl::io::IRImage::fillRaw (unsigned width, unsigned height, unsigned short* ir_buffer, unsigned line_step) const
{
  if (width > wrapper_->getWidth () || height > wrapper_->getHeight ())
    THROW_IO_EXCEPTION ("upsampling not supported: %d x %d -> %d x %d", wrapper_->getWidth (), wrapper_->getHeight (), width, height);

  if (wrapper_->getWidth () % width != 0 || wrapper_->getHeight () % height != 0)
    THROW_IO_EXCEPTION ("downsampling only supported for integer scale: %d x %d -> %d x %d", wrapper_->getWidth (), wrapper_->getHeight (), width, height);

  if (line_step == 0)
    line_step = width * static_cast<unsigned> (sizeof (unsigned short));

  // special case no sclaing, no padding => memcopy!
  if (width == wrapper_->getWidth () && height == wrapper_->getHeight () && (line_step == width * sizeof (unsigned short)))
  {
    memcpy (ir_buffer, wrapper_->getData (), wrapper_->getDataSize ());
    return;
  }

  // padding skip for destination image
  unsigned bufferSkip = line_step - width * static_cast<unsigned> (sizeof (unsigned short));

  // step and padding skip for source image
  unsigned xStep = wrapper_->getWidth () / width;
  unsigned ySkip = (wrapper_->getHeight () / height - 1) * wrapper_->getWidth ();

  unsigned irIdx = 0;

  const auto* inputBuffer = static_cast<const unsigned short*> (wrapper_->getData ());

  for (unsigned yIdx = 0; yIdx < height; ++yIdx, irIdx += ySkip)
  {
    for (unsigned xIdx = 0; xIdx < width; ++xIdx, irIdx += xStep, ++ir_buffer)
      *ir_buffer = inputBuffer[irIdx];

    // if we have padding
    if (bufferSkip > 0)
    {
      char* cBuffer = reinterpret_cast<char*> (ir_buffer);
      ir_buffer = reinterpret_cast<unsigned short*> (cBuffer + bufferSkip);
    }
  }
}

