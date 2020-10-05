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

#pragma once

#include <chrono>

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>

#include <pcl/io/image_metadata_wrapper.h>

namespace pcl
{
  namespace io
  {

    /**
    * @brief Class containing just a reference to IR meta data.
    */
    class PCL_EXPORTS IRImage
    {
      public:
        using Ptr = shared_ptr<IRImage>;
        using ConstPtr = shared_ptr<const IRImage>;

        using Clock = std::chrono::high_resolution_clock;
        using Timestamp = std::chrono::high_resolution_clock::time_point;

        IRImage (FrameWrapper::Ptr ir_metadata);
        IRImage (FrameWrapper::Ptr ir_metadata, Timestamp time);

        ~IRImage () noexcept
        {}

        void
        fillRaw (unsigned width, unsigned height, unsigned short* ir_buffer, unsigned line_step = 0) const;

        unsigned
        getWidth () const;

        unsigned
        getHeight () const;

        unsigned
        getFrameID () const;

        std::uint64_t
        getTimestamp () const;

        Timestamp
        getSystemTimestamp () const;

        // Get a const pointer to the raw depth buffer.  If the data is accessed just read-only, then this method is faster than a fillXXX method
        const unsigned short*
        getData ();

        // Data buffer size in bytes
        int
        getDataSize () const;

        // Size of each row, including any padding
        inline unsigned
        getStep() const
        {
          return (getDataSize() / getHeight());
        }

        /** \brief method to access the internal data structure wrapper, which needs to be cast to an 
        * approperate subclass before the getMetadata(..) function is available to access the native data type.
        */
        const FrameWrapper::Ptr
        getMetaData () const;

      protected:
        FrameWrapper::Ptr wrapper_;
        Timestamp timestamp_;
    };

  } // namespace
}
