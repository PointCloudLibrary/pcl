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

#include <pcl/io/openni_camera/openni_ir_image.h>

namespace openni_wrapper
{
void IRImage::fillRaw (unsigned width, unsigned height, unsigned short* ir_buffer, unsigned line_step) const
{
  if (width > ir_md_->XRes () || height > ir_md_->YRes ())
    THROW_OPENNI_EXCEPTION ("upsampling not supported: %d x %d -> %d x %d", ir_md_->XRes (), ir_md_->YRes (), width, height);

  if (ir_md_->XRes () % width != 0 || ir_md_->YRes () % height != 0)
    THROW_OPENNI_EXCEPTION ("downsampling only supported for integer scale: %d x %d -> %d x %d", ir_md_->XRes (), ir_md_->YRes (), width, height);

  if (line_step == 0)
    line_step = width * static_cast<unsigned> (sizeof (unsigned short));

  // special case no sclaing, no padding => memcopy!
  if (width == ir_md_->XRes () && height == ir_md_->YRes () && (line_step == width * sizeof (unsigned short)))
  {
    memcpy (ir_buffer, ir_md_->Data(), ir_md_->DataSize ());
    return;
  }

  // padding skip for destination image
  unsigned bufferSkip = line_step - width * static_cast<unsigned> (sizeof (unsigned short));

  // step and padding skip for source image
  unsigned xStep = ir_md_->XRes () / width;
  unsigned ySkip = (ir_md_->YRes () / height - 1) * ir_md_->XRes ();

  unsigned irIdx = 0;

  for (unsigned yIdx = 0; yIdx < height; ++yIdx, irIdx += ySkip)
  {
    for (unsigned xIdx = 0; xIdx < width; ++xIdx, irIdx += xStep, ++ir_buffer)
      *ir_buffer = static_cast<unsigned short> (ir_md_->Data()[irIdx]);

    // if we have padding
    if (bufferSkip > 0)
    {
      char* cBuffer = reinterpret_cast<char*> (ir_buffer);
      ir_buffer = reinterpret_cast<unsigned short*> (cBuffer + bufferSkip);
    }
  }
}
} // namespace openni_wrapper

#endif //HAVE_OPENNI

