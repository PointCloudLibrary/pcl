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

#ifndef PCL_FEATURES_DENSE_QUANTIZED_MULTI_MOD_TEMPLATE
#define PCL_FEATURES_DENSE_QUANTIZED_MULTI_MOD_TEMPLATE

#include <vector>

#include <pcl/recognition/region_xy.h>

namespace pcl
{

  struct DenseQuantizedSingleModTemplate
  {
    std::vector<unsigned char> features;

    void 
    serialize (std::ostream & stream) const
    {
      const size_t num_of_features = static_cast<size_t> (features.size ());
      write (stream, num_of_features);
      for (size_t feature_index = 0; feature_index < num_of_features; ++feature_index)
      {
        write (stream, features[feature_index]);
      }
    }

    void 
    deserialize (std::istream & stream)
    {
      features.clear ();

      size_t num_of_features;
      read (stream, num_of_features);
      features.resize (num_of_features);
      for (size_t feature_index = 0; feature_index < num_of_features; ++feature_index)
      {
        read (stream, features[feature_index]);
      }
    }
  };

  struct DenseQuantizedMultiModTemplate
  {
    std::vector<DenseQuantizedSingleModTemplate> modalities;
    float response_factor;

    RegionXY region;

    void 
    serialize (std::ostream & stream) const
    {
      const size_t num_of_modalities = static_cast<size_t> (modalities.size ());
      write (stream, num_of_modalities);
      for (size_t modality_index = 0; modality_index < num_of_modalities; ++modality_index)
      {
        modalities[modality_index].serialize (stream);
      }

      region.serialize (stream);
    }

    void 
    deserialize (std::istream & stream)
    {
      modalities.clear ();

      size_t num_of_modalities;
      read (stream, num_of_modalities);
      modalities.resize (num_of_modalities);
      for (size_t modality_index = 0; modality_index < num_of_modalities; ++modality_index)
      {
        modalities[modality_index].deserialize (stream);
      }

      region.deserialize (stream);
    }
  };

}

#endif 