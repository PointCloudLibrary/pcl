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

#ifndef PCL_RECOGNITION_LINEMOD
#define PCL_RECOGNITION_LINEMOD

#include <vector>
#include <cstddef>
#include <string.h>
#include <pcl/pcl_macros.h>
#include "pcl/recognition/quantizable_modality.h"
#include "pcl/recognition/region_xy.h"
#include "pcl/recognition/sparse_quantized_multi_mod_template.h"

namespace pcl
{
  class EnergyMaps
  {
    public:
      EnergyMaps () : width_ (-1), height_ (-1), nr_bins_ (-1), maps_ () 
      {
      }

      virtual ~EnergyMaps () 
      {
      }

      inline int 
      getWidth () const 
      { 
        return (width_); 
      }
      
      inline int 
      getHeight () const 
      { 
        return (height_); 
      }
      
      inline int 
      getNumOfBins () const
      { 
        return (nr_bins_);
      }

      void 
      initialize (const int width, const int height, const int nr_bins)
      {
        maps_.resize(nr_bins, NULL);
        width_ = width;
        height_ = height;
        nr_bins_ = nr_bins;

        const int mapsSize = width*height;

        for (size_t map_index = 0; map_index < maps_.size (); ++map_index)
        {
          maps_[map_index] = new unsigned char[mapsSize];
          memset (maps_[map_index], 0, mapsSize);
        }
      }

      void 
      releaseAll ()
      {
        for (size_t map_index = 0; map_index < maps_.size (); ++map_index)
          if (maps_[map_index] != NULL) delete[] maps_[map_index];

        maps_.clear ();
        width_ = -1;
        height_ = -1;
        nr_bins_ = -1;
      }

      inline unsigned char & 
      operator() (const int bin_index, const int col_index, const int row_index)
      {
        return (maps_[bin_index][row_index*width_ + col_index]);
      }

      inline unsigned char & 
      operator() (const int bin_index, const int index)
      {
        return (maps_[bin_index][index]);
      }

      inline unsigned char * 
      operator() (const int bin_index)
      {
        return (maps_[bin_index]);
      }

    private:
      int width_;
      int height_;
      int nr_bins_;
      std::vector<unsigned char*> maps_;
  };

  class LinearizedMaps
  {
    public:
      LinearizedMaps () : width_ (-1), height_ (-1), mem_width_ (-1), mem_height_ (-1), step_size_ (-1), maps_ ()
      {
      }
      
      virtual ~LinearizedMaps () 
      {
      }

      inline int 
      getWidth () const { return (width_); }
      
      inline int 
      getHeight () const { return (height_); }
      
      inline int 
      getStepSize () const { return (step_size_); }
      
      inline int 
      getMapMemorySize () const { return (mem_width_ * mem_height_); }

      void 
      initialize (const int width, const int height, const int step_size)
      {
        maps_.resize(step_size*step_size, NULL);
        width_ = width;
        height_ = height;
        mem_width_ = width / step_size;
        mem_height_ = height / step_size;
        step_size_ = step_size;

        const int mapsSize = mem_width_ * mem_height_;

        for (size_t map_index = 0; map_index < maps_.size (); ++map_index)
        {
          maps_[map_index] = new unsigned char[2*mapsSize];
          memset (maps_[map_index], 0, 2*mapsSize);
        }
      }

      void 
      releaseAll ()
      {
        for (size_t map_index = 0; map_index < maps_.size (); ++map_index)
          if (maps_[map_index] != NULL) delete[] maps_[map_index];

        maps_.clear ();
        width_ = -1;
        height_ = -1;
        mem_width_ = -1;
        mem_height_ = -1;
        step_size_ = -1;
      }

      inline unsigned char* 
      operator() (const int col_index, const int row_index)
      {
        return (maps_[row_index*step_size_ + col_index]);
      }

      inline unsigned char* 
      getOffsetMap (const int col_index, const int row_index)
      {
        const int map_col = col_index % step_size_;
        const int map_row = row_index % step_size_;

        const int map_mem_col_index = col_index / step_size_;
        const int map_mem_row_index = row_index / step_size_;

        return (maps_[map_row*step_size_ + map_col] + map_mem_row_index*mem_width_ + map_mem_col_index);
      }

    private:
      int width_;
      int height_;
      int mem_width_;
      int mem_height_;
      int step_size_;
      std::vector<unsigned char*> maps_;
  };

  struct LINEMODDetection
  {
    int x;
    int y;
    int template_id;
    float score;
  };

  /**
    * \brief Template matching using the LINEMOD approach.
    * \author Stefan Holzer, Stefan Hinterstoisser
    */
  class PCL_EXPORTS LINEMOD
  {
    public:
      /** \brief Constructor */
      LINEMOD ();

      /** \brief Destructor */
      virtual ~LINEMOD ();

      /** \brief Creates a template from the specified data and adds it to the matching queue. 
        * \param 
        * \param 
        * \param 
        */
      int 
      createAndAddTemplate (const std::vector<QuantizableModality*> & modalities,
                            const std::vector<MaskMap*> & masks,
                            const RegionXY & region);

      void
      detectTemplates (std::vector<QuantizableModality*> & modalities,
                       std::vector<LINEMODDetection> & detections);

      inline SparseQuantizedMultiModTemplate &
      getTemplate (int template_id)
      { 
        return (templates_[template_id]);
      }

      void
      saveTemplates (const char* file_name);

      void
      loadTemplates (const char* file_name);

      void 
      serialize (std::ostream & stream);

      void 
      deserialize (std::istream & stream);


    private:
      /** template storage */
      std::vector<SparseQuantizedMultiModTemplate> templates_;
  };

}

#endif    // PCL_RECOGNITION_LINEMOD
