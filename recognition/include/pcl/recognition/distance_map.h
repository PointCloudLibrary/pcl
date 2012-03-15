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

#ifndef PCL_RECOGNITION_DISTANCE_MAP
#define PCL_RECOGNITION_DISTANCE_MAP

namespace pcl
{

  class DistanceMap
  {
    public:
      DistanceMap () : data_ (0), width_ (0), height_ (0) {}
      virtual ~DistanceMap () {}

      inline size_t 
      getWidth () const
      {
        return (width_); 
      }

      inline size_t 
      getHeight () const
      { 
        return (height_); 
      }
    
      inline float * 
      getData () 
      { 
        return (&data_[0]); 
      }

      void 
      resize (const size_t width, const size_t height)
      {
        data_.resize (width*height);
        width_ = width;
        height_ = height;
      }

      inline float & 
      operator() (const size_t col_index, const size_t row_index)
      {
        return (data_[row_index*width_ + col_index]);
      }

      inline const float & 
      operator() (const size_t col_index, const size_t row_index) const
      {
        return (data_[row_index*width_ + col_index]);
      }

    protected:
      std::vector<float> data_;
      size_t width_;
      size_t height_;
  };

}


#endif 
