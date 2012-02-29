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

#ifndef PCL_FEATURES_SPARSE_QUANTIZED_MULTI_MOD_TEMPLATE
#define PCL_FEATURES_SPARSE_QUANTIZED_MULTI_MOD_TEMPLATE

#include <vector>

#include "pcl/recognition/region_xy.h"

namespace pcl
{

  struct QuantizedMultiModFeature
  {
    int x;
    int y;
    size_t modality_index;
    unsigned char quantized_value;

    void 
    serialize (std::ostream & stream) const
    {
      write (stream, x);
      write (stream, y);
      write (stream, modality_index);
      write (stream, quantized_value);
    }

    void 
    deserialize (std::istream & stream)
    {
      read (stream, x);
      read (stream, y);
      read (stream, modality_index);
      read (stream, quantized_value);
    }
  };

  struct SparseQuantizedMultiModTemplate
  {
    std::vector<QuantizedMultiModFeature> features;

    RegionXY region;

    void 
    serialize (std::ostream & stream) const
    {
      const int num_of_features = static_cast<int> (features.size ());
      write (stream, num_of_features);
      for (int feature_index = 0; feature_index < num_of_features; ++feature_index)
      {
        features[feature_index].serialize (stream);
      }

      region.serialize (stream);
    }

    void 
    deserialize (::std::istream & stream)
    {
      features.clear ();

      int num_of_features;
      read (stream, num_of_features);
      features.resize (num_of_features);
      for (int feature_index = 0; feature_index < num_of_features; ++feature_index)
      {
        features[feature_index].deserialize (stream);
      }

      region.deserialize (stream);
    }
  };

}

#endif    // PCL_FEATURES_SPARSE_QUANTIZED_MULTI_MOD_TEMPLATE
