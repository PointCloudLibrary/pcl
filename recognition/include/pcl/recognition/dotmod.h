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
#include <cstddef>
#include <pcl/pcl_macros.h>
#include <pcl/recognition/dot_modality.h>
#include <pcl/recognition/dense_quantized_multi_mod_template.h>
#include <pcl/recognition/mask_map.h>
#include <pcl/recognition/region_xy.h>

namespace pcl
{

  struct DOTMODDetection
  {
    std::size_t bin_x;
    std::size_t bin_y;
    std::size_t template_id;
    float score;
  };

  /**
    * \brief Template matching using the DOTMOD approach.
    * \author Stefan Holzer, Stefan Hinterstoisser
    */
  class PCL_EXPORTS DOTMOD
  {
    public:
      /** \brief Constructor */
      DOTMOD (std::size_t template_width,
              std::size_t template_height);

      /** \brief Destructor */
      virtual ~DOTMOD ();

      /** \brief Creates a template from the specified data and adds it to the matching queue. 
        * \param modalities
        * \param masks
        * \param template_anker_x
        * \param template_anker_y
        * \param region
        */
      std::size_t 
      createAndAddTemplate (const std::vector<DOTModality*> & modalities,
                            const std::vector<MaskMap*> & masks,
                            std::size_t template_anker_x,
                            std::size_t template_anker_y,
                            const RegionXY & region);

      void
      detectTemplates (const std::vector<DOTModality*> & modalities,
                       float template_response_threshold,
                       std::vector<DOTMODDetection> & detections,
                       const std::size_t bin_size) const;

      inline const DenseQuantizedMultiModTemplate &
      getTemplate (std::size_t template_id) const
      { 
        return (templates_[template_id]);
      }

      inline std::size_t
      getNumOfTemplates ()
      {
        return (templates_.size ());
      }

      void
      saveTemplates (const char * file_name) const;

      void
      loadTemplates (const char * file_name);

      void 
      serialize (std::ostream & stream) const;

      void 
      deserialize (std::istream & stream);


    private:
      /** template width */
      std::size_t template_width_;
      /** template height */
      std::size_t template_height_;
      /** template storage */
      std::vector<DenseQuantizedMultiModTemplate> templates_;
  };

}
