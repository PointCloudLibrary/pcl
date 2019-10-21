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

#pragma once

#include <vector>
#include <pcl/pcl_macros.h>
#include <pcl/recognition/mask_map.h>
#include <pcl/recognition/quantized_map.h>
#include <pcl/recognition/region_xy.h>
#include <pcl/recognition/sparse_quantized_multi_mod_template.h>

namespace pcl
{
  /** \brief Interface for a quantizable modality. 
    * \author Stefan Holzer
    */
  class PCL_EXPORTS QuantizableModality
  {
    public:
      /** \brief Constructor. */
      QuantizableModality ();
      /** \brief Destructor. */
      virtual ~QuantizableModality ();

      /** \brief Returns a reference to the internally computed quantized map. */
      virtual QuantizedMap &
      getQuantizedMap () = 0;

      /** \brief Returns a reference to the internally computed spread quantized map. */
      virtual QuantizedMap &
      getSpreadedQuantizedMap () = 0;

      /** \brief Extracts features from this modality within the specified mask.
        * \param[in] mask defines the areas where features are searched in. 
        * \param[in] nr_features defines the number of features to be extracted 
        *            (might be less if not sufficient information is present in the modality).
        * \param[in] modality_index the index which is stored in the extracted features.
        * \param[out] features the destination for the extracted features.
        */
      virtual void 
      extractFeatures (const MaskMap & mask, std::size_t nr_features, std::size_t modality_index, 
                       std::vector<QuantizedMultiModFeature> & features) const = 0;

      /** \brief Extracts all possible features from the modality within the specified mask.
        * \param[in] mask defines the areas where features are searched in. 
        * \param[in] nr_features IGNORED (TODO: remove this parameter).
        * \param[in] modality_index the index which is stored in the extracted features.
        * \param[out] features the destination for the extracted features.
        */
      virtual void 
      extractAllFeatures (const MaskMap & mask, std::size_t nr_features, std::size_t modality_index, 
                       std::vector<QuantizedMultiModFeature> & features) const = 0;
  };
}
