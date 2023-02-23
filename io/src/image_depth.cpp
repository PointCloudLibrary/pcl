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
#include <pcl/io/image_depth.h>

#include <limits>

#include <pcl/io/io_exception.h>

using pcl::io::FrameWrapper;
using pcl::io::IOException;

pcl::io::DepthImage::DepthImage (FrameWrapper::Ptr depth_metadata, float baseline, float focal_length, std::uint64_t shadow_value, std::uint64_t no_sample_value)
: wrapper_ (std::move(depth_metadata))
, baseline_ (baseline)
, focal_length_ (focal_length)
, shadow_value_ (shadow_value)
, no_sample_value_ (no_sample_value)
, timestamp_ (Clock::now ())
{}


pcl::io::DepthImage::DepthImage (FrameWrapper::Ptr depth_metadata, float baseline, float focal_length, std::uint64_t shadow_value, std::uint64_t no_sample_value, Timestamp timestamp)
: wrapper_(std::move(depth_metadata))
, baseline_ (baseline)
, focal_length_ (focal_length)
, shadow_value_ (shadow_value)
, no_sample_value_ (no_sample_value)
, timestamp_(timestamp)
{}


pcl::io::DepthImage::~DepthImage () = default;

const unsigned short*
pcl::io::DepthImage::getData ()
{
  return static_cast<const unsigned short*> (wrapper_->getData ());
}


int
pcl::io::DepthImage::getDataSize () const
{
  return (wrapper_->getDataSize ());
}


const FrameWrapper::Ptr
pcl::io::DepthImage::getMetaData () const
{
  return (wrapper_);
}


float
pcl::io::DepthImage::getBaseline () const
{
  return (baseline_);
}


float
pcl::io::DepthImage::getFocalLength () const
{
  return (focal_length_);
}


std::uint64_t
pcl::io::DepthImage::getShadowValue () const
{
  return (shadow_value_);
}


std::uint64_t
pcl::io::DepthImage::getNoSampleValue () const
{
  return (no_sample_value_);
}


unsigned
pcl::io::DepthImage::getWidth () const
{
  return (wrapper_->getWidth ());
}


unsigned
pcl::io::DepthImage::getHeight () const
{
  return (wrapper_->getHeight ());
}


unsigned
pcl::io::DepthImage::getFrameID () const
{
  return (wrapper_->getFrameID ());
}


std::uint64_t
pcl::io::DepthImage::getTimestamp () const
{
  return (wrapper_->getTimestamp ());
}


pcl::io::DepthImage::Timestamp
pcl::io::DepthImage::getSystemTimestamp () const
{
  return (timestamp_);
}

// Fill external buffers ////////////////////////////////////////////////////

void
pcl::io::DepthImage::fillDepthImageRaw (unsigned width, unsigned height, unsigned short* depth_buffer, unsigned line_step) const
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
    memcpy (depth_buffer, wrapper_->getData (), wrapper_->getDataSize ());
    return;
  }

  // padding skip for destination image
  unsigned bufferSkip = line_step - width * static_cast<unsigned> (sizeof (unsigned short));

  // step and padding skip for source image
  unsigned xStep = wrapper_->getWidth () / width;
  unsigned ySkip = (wrapper_->getHeight () / height - 1) * wrapper_->getWidth ();

  // Fill in the depth image data, converting mm to m
  short bad_point = std::numeric_limits<short>::quiet_NaN ();
  unsigned depthIdx = 0;

  const auto* inputBuffer = static_cast<const unsigned short*> (wrapper_->getData ());

  for (unsigned yIdx = 0; yIdx < height; ++yIdx, depthIdx += ySkip)
  {
    for (unsigned xIdx = 0; xIdx < width; ++xIdx, depthIdx += xStep, ++depth_buffer)
    {
      /// @todo Different values for these cases
      unsigned short pixel = inputBuffer[depthIdx];
      if (pixel == 0 || pixel == no_sample_value_ || pixel == shadow_value_)
        *depth_buffer = bad_point;
      else
      {
        *depth_buffer = static_cast<unsigned short>( pixel );
      }
    }
    // if we have padding
    if (bufferSkip > 0)
    {
      char* cBuffer = reinterpret_cast<char*> (depth_buffer);
      depth_buffer = reinterpret_cast<unsigned short*> (cBuffer + bufferSkip);
    }
  }
}

void
pcl::io::DepthImage::fillDepthImage (unsigned width, unsigned height, float* depth_buffer, unsigned line_step) const
{
  if (width > wrapper_->getWidth () || height > wrapper_->getHeight ())
    THROW_IO_EXCEPTION ("upsampling not supported: %d x %d -> %d x %d", wrapper_->getWidth (), wrapper_->getHeight (), width, height);

  if (wrapper_->getWidth () % width != 0 || wrapper_->getHeight () % height != 0)
    THROW_IO_EXCEPTION ("downsampling only supported for integer scale: %d x %d -> %d x %d", wrapper_->getWidth (), wrapper_->getHeight (), width, height);

  if (line_step == 0)
    line_step = width * static_cast<unsigned> (sizeof (float));

  // padding skip for destination image
  unsigned bufferSkip = line_step - width * static_cast<unsigned> (sizeof (float));

  // step and padding skip for source image
  unsigned xStep = wrapper_->getWidth () / width;
  unsigned ySkip = (wrapper_->getHeight () / height - 1) * wrapper_->getWidth ();

  // Fill in the depth image data, converting mm to m
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  unsigned depthIdx = 0;

  const auto* inputBuffer = static_cast<const unsigned short*> (wrapper_->getData ());

  for (unsigned yIdx = 0; yIdx < height; ++yIdx, depthIdx += ySkip)
  {
    for (unsigned xIdx = 0; xIdx < width; ++xIdx, depthIdx += xStep, ++depth_buffer)
    {
      /// @todo Different values for these cases
      unsigned short pixel = inputBuffer[depthIdx];
      if (pixel == 0 || pixel == no_sample_value_ || pixel == shadow_value_)
        *depth_buffer = bad_point;
      else
      {
        *depth_buffer = static_cast<unsigned short>( pixel ) * 0.001f;  // millimeters to meters
      }
    }
    // if we have padding
    if (bufferSkip > 0)
    {
      char* cBuffer = reinterpret_cast<char*> (depth_buffer);
      depth_buffer = reinterpret_cast<float*> (cBuffer + bufferSkip);
    }
  }
}

void
pcl::io::DepthImage::fillDisparityImage (unsigned width, unsigned height, float* disparity_buffer, unsigned line_step) const
{
  if (width > wrapper_->getWidth () || height > wrapper_->getHeight ())
    THROW_IO_EXCEPTION ("upsampling not supported: %d x %d -> %d x %d", wrapper_->getWidth (), wrapper_->getHeight (), width, height);

  if (wrapper_->getWidth () % width != 0 || wrapper_->getHeight () % height != 0)
    THROW_IO_EXCEPTION ("downsampling only supported for integer scale: %d x %d -> %d x %d", wrapper_->getWidth (), wrapper_->getHeight (), width, height);

  if (line_step == 0)
    line_step = width * static_cast<unsigned> (sizeof (float));

  unsigned xStep = wrapper_->getWidth () / width;
  unsigned ySkip = (wrapper_->getHeight () / height - 1) * wrapper_->getWidth ();

  unsigned bufferSkip = line_step - width * static_cast<unsigned> (sizeof (float));

  // Fill in the depth image data
  // iterate over all elements and fill disparity matrix: disp[x,y] = f * b / z_distance[x,y];
  // focal length is for the native image resolution -> focal_length = focal_length_ / xStep;
  float constant = focal_length_ * baseline_ * 1000.0f / static_cast<float> (xStep);

  const auto* inputBuffer = static_cast<const unsigned short*> (wrapper_->getData ());

  for (unsigned yIdx = 0, depthIdx = 0; yIdx < height; ++yIdx, depthIdx += ySkip)
  {
    for (unsigned xIdx = 0; xIdx < width; ++xIdx, depthIdx += xStep, ++disparity_buffer)
    {
      unsigned short pixel = inputBuffer[depthIdx];
      if (pixel == 0 || pixel == no_sample_value_ || pixel == shadow_value_)
        *disparity_buffer = 0.0;
      else
        *disparity_buffer = constant / static_cast<float> (pixel);
    }

    // if we have padding
    if (bufferSkip > 0)
    {
      char* cBuffer = reinterpret_cast<char*> (disparity_buffer);
      disparity_buffer = reinterpret_cast<float*> (cBuffer + bufferSkip);
    }
  }
}
