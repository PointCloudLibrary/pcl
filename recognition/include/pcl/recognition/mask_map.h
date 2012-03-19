/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#ifndef PCL_FEATURES_MASK_MAP
#define PCL_FEATURES_MASK_MAP

#include <vector>
#include <pcl/pcl_macros.h>
#include <cstring>

namespace pcl
{
  class PCL_EXPORTS MaskMap
  {
    public:
      MaskMap ();
      MaskMap (size_t width, size_t height);
      virtual ~MaskMap ();

      void
      resize (size_t width, size_t height);

      inline size_t 
      getWidth () const { return (width_); }
      
      inline size_t
      getHeight () const { return (height_); }
      
      inline unsigned char* 
      getData () { return (&data_[0]); }

      inline const unsigned char* 
      getData () const { return (&data_[0]); }

      static void
      getDifferenceMask (const MaskMap & mask0,
                         const MaskMap & mask1,
                         MaskMap & diff_mask);

      inline void
      set (const size_t x, const size_t y)
      {
        data_[y*width_+x] = 255;
      }

      inline void
      unset (const size_t x, const size_t y)
      {
        data_[y*width_+x] = 0;
      }

      inline bool
      isSet (const size_t x, const size_t y) const
      {
        return (data_[y*width_+x] != 0);
      }

      inline void
      reset ()
      {
        memset (&data_[0], 0, width_*height_);
      }

      inline unsigned char & 
      operator() (const size_t x, const size_t y) 
      { 
        return (data_[y*width_+x]); 
      }

      inline const unsigned char & 
      operator() (const size_t x, const size_t y) const
      { 
        return (data_[y*width_+x]); 
      }

      void
      erode (MaskMap & eroded_mask) const;

    private:
      //unsigned char * data_;
      std::vector<unsigned char> data_;
      size_t width_;
      size_t height_;  
  };

}

#endif  
