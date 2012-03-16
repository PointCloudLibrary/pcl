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
#include <pcl/recognition/quantizable_modality.h>
#include <pcl/recognition/region_xy.h>
#include <pcl/recognition/sparse_quantized_multi_mod_template.h>

namespace pcl
{
  class EnergyMaps
  {
    public:
      EnergyMaps () : width_ (0), height_ (0), nr_bins_ (0), maps_ () 
      {
      }

      virtual ~EnergyMaps () 
      {
      }

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
      
      inline size_t 
      getNumOfBins () const
      { 
        return (nr_bins_);
      }

      void 
      initialize (const size_t width, const size_t height, const size_t nr_bins)
      {
        maps_.resize(nr_bins, NULL);
        width_ = width;
        height_ = height;
        nr_bins_ = nr_bins;

        const size_t mapsSize = width*height;

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
        width_ = 0;
        height_ = 0;
        nr_bins_ = 0;
      }

      inline unsigned char & 
      operator() (const size_t bin_index, const size_t col_index, const size_t row_index)
      {
        return (maps_[bin_index][row_index*width_ + col_index]);
      }

      inline unsigned char & 
      operator() (const size_t bin_index, const size_t index)
      {
        return (maps_[bin_index][index]);
      }

      inline unsigned char * 
      operator() (const size_t bin_index)
      {
        return (maps_[bin_index]);
      }

    private:
      size_t width_;
      size_t height_;
      size_t nr_bins_;
      std::vector<unsigned char*> maps_;
  };

  class LinearizedMaps
  {
    public:
      LinearizedMaps () : width_ (0), height_ (0), mem_width_ (0), mem_height_ (0), step_size_ (0), maps_ ()
      {
      }
      
      virtual ~LinearizedMaps () 
      {
      }

      inline size_t 
      getWidth () const { return (width_); }
      
      inline size_t 
      getHeight () const { return (height_); }
      
      inline size_t 
      getStepSize () const { return (step_size_); }
      
      inline size_t 
      getMapMemorySize () const { return (mem_width_ * mem_height_); }

      void 
      initialize (const size_t width, const size_t height, const size_t step_size)
      {
        maps_.resize(step_size*step_size, NULL);
        width_ = width;
        height_ = height;
        mem_width_ = width / step_size;
        mem_height_ = height / step_size;
        step_size_ = step_size;

        const size_t mapsSize = mem_width_ * mem_height_;

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
        width_ = 0;
        height_ = 0;
        mem_width_ = 0;
        mem_height_ = 0;
        step_size_ = 0;
      }

      inline unsigned char * 
      operator() (const size_t col_index, const size_t row_index)
      {
        return (maps_[row_index*step_size_ + col_index]);
      }

      inline unsigned char * 
      getOffsetMap (const size_t col_index, const size_t row_index)
      {
        const size_t map_col = col_index % step_size_;
        const size_t map_row = row_index % step_size_;

        const size_t map_mem_col_index = col_index / step_size_;
        const size_t map_mem_row_index = row_index / step_size_;

        return (maps_[map_row*step_size_ + map_col] + map_mem_row_index*mem_width_ + map_mem_col_index);
      }

    private:
      size_t width_;
      size_t height_;
      size_t mem_width_;
      size_t mem_height_;
      size_t step_size_;
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
      detectTemplates (const std::vector<QuantizableModality*> & modalities,
                       std::vector<LINEMODDetection> & detections) const;

      void
      matchTemplates (const std::vector<QuantizableModality*> & modalities,
                      std::vector<LINEMODDetection> & detections) const;

      inline void
      setDetectionThreshold (float threshold)
      {
        template_threshold_ = threshold;
      }

      inline const SparseQuantizedMultiModTemplate &
      getTemplate (int template_id) const
      { 
        return (templates_[template_id]);
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
      /** template response threshold */
      float template_threshold_;
      /** template storage */
      std::vector<SparseQuantizedMultiModTemplate> templates_;
  };

}

#endif    // PCL_RECOGNITION_LINEMOD
