/*
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 * Copyright (c) 2014, respective authors.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once
#ifndef PCL_IO_OPENNI2_METADATA_WRAPPER_H_
#define PCL_IO_OPENNI2_METADATA_WRAPPER_H_

#include <pcl/pcl_config.h>

#if defined(HAVE_OPENNI2)

#include <pcl/io/image_metadata_wrapper.h>
#include <pcl/io/openni2/openni.h>

namespace pcl
{
  namespace io
  {
    namespace openni2
    {
      class Openni2FrameWrapper : public pcl::io::FrameWrapper
      {
        public:
          Openni2FrameWrapper (openni::VideoFrameRef metadata)
            : metadata_(metadata)
          {}

          virtual inline const void*
          getData () const
          {
            return (metadata_.getData ());
          }

          virtual inline unsigned
          getDataSize () const
          {
            return (metadata_.getDataSize ());
          }

          virtual inline unsigned
          getWidth () const
          {
            return (metadata_.getWidth ());
          }

          virtual inline unsigned
          getHeight () const
          {
            return (metadata_.getHeight ());
          }

          virtual inline unsigned
          getFrameID () const
          {
            return (metadata_.getFrameIndex ());
          }

          virtual inline uint64_t
          getTimestamp () const
          {
            return (metadata_.getTimestamp ());
          }


          const inline openni::VideoFrameRef&
          getMetaData () const
          {
            return (metadata_);
          }

        private:
          openni::VideoFrameRef metadata_; // Internally reference counted
      };

    } // namespace
  }
}
#endif // HAVE_OPENNI2

#endif // PCL_IO_OPENNI2_METADATA_WRAPPER_H_
