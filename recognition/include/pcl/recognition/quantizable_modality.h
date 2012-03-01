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

#ifndef PCL_FEATURES_QUANTIZABLE_MODALITY
#define PCL_FEATURES_QUANTIZABLE_MODALITY

#include <vector>
#include <pcl/pcl_macros.h>
#include "pcl/recognition/region_xy.h"
#include "pcl/recognition/sparse_quantized_multi_mod_template.h"

namespace pcl
{
  class PCL_EXPORTS MaskMap
  {
    public:
      MaskMap ();
      MaskMap (const size_t width, const size_t height);
      virtual ~MaskMap ();

      void
      resize (const size_t width, const size_t height);

      inline size_t 
      getWidth () const { return (width_); }
      
      inline size_t
      getHeight () const { return (height_); }
      
      inline unsigned char* 
      getData () { return (&data_[0]); }

      inline const unsigned char* 
      getData () const { return (&data_[0]); }

      inline unsigned char & 
      operator() (int x, int y) 
      { 
        return (data_[y*width_+x]); 
      }

      inline const unsigned char & 
      operator() (int x, int y) const
      { 
        return (data_[y*width_+x]); 
      }

    private:
      //unsigned char * data_;
      std::vector<unsigned char> data_;
      size_t width_;
      size_t height_;  
  };

  class PCL_EXPORTS QuantizedMap
  {
    public:

      QuantizedMap ();
      QuantizedMap (const size_t width, const size_t height);

      virtual ~QuantizedMap ();

      inline size_t
      getWidth () const { return (width_); }
      
      inline size_t
      getHeight () const { return (height_); }
      
      inline unsigned char*
      getData () { return (&data_[0]); }

      inline const unsigned char*
      getData () const { return (&data_[0]); }

      void 
      resize (const size_t width, const size_t height);

      inline unsigned char & 
      operator() (const int x, const int y) 
      { 
        return (data_[y*width_+x]); 
      }

      inline const unsigned char & 
      operator() (const int x, const int y) const
      { 
        return (data_[y*width_+x]); 
      }

      static void
      spreadQuantizedMap (const QuantizedMap & input_map, QuantizedMap & output_map, const int spreading_size);

    private:
      std::vector<unsigned char> data_;
      size_t width_;
      size_t height_;  
    
  };

  class PCL_EXPORTS QuantizableModality
  {
    public:
      QuantizableModality ();
      virtual ~QuantizableModality ();

      //inline int getWidth () const { return width; }
      //inline int getHeight () const { return height; }

      //inline unsigned char & operator() (int x, int y) { return data[y*width+x]; }
      //inline const unsigned char & operator() (int x, int y) const { return data[y*width+x]; }

      virtual QuantizedMap &
      getQuantizedMap () = 0;

      virtual QuantizedMap &
      getSpreadedQuantizedMap () = 0;

      virtual void 
      extractFeatures (const MaskMap & mask, size_t nr_features, size_t modality_index, 
                       std::vector<QuantizedMultiModFeature> & features) const = 0;

    private:
      //unsigned char * data_;
      //int width_;
      //int height_;  
  };
}

#endif    // PCL_FEATURES_QUANTIZABLE_MODALITY
