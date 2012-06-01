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

#ifndef PCL_FEATURES_QUANTIZED_MAP
#define PCL_FEATURES_QUANTIZED_MAP

#include <vector>
#include <pcl/pcl_macros.h>

namespace pcl
{
  class PCL_EXPORTS QuantizedMap
  {
    public:

      QuantizedMap ();
      QuantizedMap (size_t width, size_t height);
      QuantizedMap (const QuantizedMap & copy_me);

      virtual ~QuantizedMap ();

      inline size_t
      getWidth () const { return (width_); }
      
      inline size_t
      getHeight () const { return (height_); }
      
      inline unsigned char*
      getData () { return (&data_[0]); }

      inline const unsigned char*
      getData () const { return (&data_[0]); }

      inline QuantizedMap
      getSubMap (size_t x,
                 size_t y,
                 size_t width,
                 size_t height)
      {
        QuantizedMap subMap(width, height);

        for (size_t row_index = 0; row_index < height; ++row_index)
        {
          for (size_t col_index = 0; col_index < width; ++col_index)
          {
            //const size_t index = (row_index+y)*width_ + (col_index+x);
            //const unsigned char value = data_[index];
            //subMap.data_[row_index*width + col_index] = value;//data_[(row_index+y)*width_ + (col_index+x)];
            subMap (col_index, row_index) = (*this) (col_index + x, row_index + y);
          }
        }

        return subMap;
      }

      void 
      resize (size_t width, size_t height);

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

      static void
      spreadQuantizedMap (const QuantizedMap & input_map, QuantizedMap & output_map, size_t spreading_size);

      void 
      serialize (std::ostream & stream) const
      {
        const int width = static_cast<int> (width_);
        const int height = static_cast<int> (height_);
        
        stream.write (reinterpret_cast<const char*> (&width), sizeof (width));
        stream.write (reinterpret_cast<const char*> (&height), sizeof (height));

        const int num_of_elements = static_cast<int> (data_.size ());
        stream.write (reinterpret_cast<const char*> (&num_of_elements), sizeof (num_of_elements));
        for (int element_index = 0; element_index < num_of_elements; ++element_index)
        {
          stream.write (reinterpret_cast<const char*> (&(data_[element_index])), sizeof (data_[element_index]));
        }
      }

      void 
      deserialize (std::istream & stream)
      {
        int width;
        int height;

        stream.read (reinterpret_cast<char*> (&width), sizeof (width));
        stream.read (reinterpret_cast<char*> (&height), sizeof (height));

        width_ = static_cast<size_t> (width);
        height_ = static_cast<size_t> (height);

        int num_of_elements;
        stream.read (reinterpret_cast<char*> (&num_of_elements), sizeof (num_of_elements));
        data_.resize (num_of_elements);
        for (int element_index = 0; element_index < num_of_elements; ++element_index)
        {
          stream.read (reinterpret_cast<char*> (&(data_[element_index])), sizeof (data_[element_index]));
        }
      }


    //private:
      std::vector<unsigned char> data_;
      size_t width_;
      size_t height_;  
    
  };
}

#endif   
