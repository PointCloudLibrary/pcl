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
#include <cstring>
#include <pcl/pcl_macros.h>
#include <pcl/recognition/quantizable_modality.h>
#include <pcl/recognition/region_xy.h>
#include <pcl/recognition/sparse_quantized_multi_mod_template.h>

namespace pcl
{

  /** \brief Stores a set of energy maps.
    * \author Stefan Holzer
    */
  class PCL_EXPORTS EnergyMaps
  {
    public:
      /** \brief Constructor. */
      EnergyMaps () : width_ (0), height_ (0), nr_bins_ (0)
      {
      }

      /** \brief Destructor. */
      virtual ~EnergyMaps () = default;

      /** \brief Returns the width of the energy maps. */
      inline std::size_t 
      getWidth () const 
      { 
        return (width_); 
      }
      
      /** \brief Returns the height of the energy maps. */
      inline std::size_t 
      getHeight () const 
      { 
        return (height_); 
      }
      
      /** \brief Returns the number of bins used for quantization (which is equal to the number of energy maps). */
      inline std::size_t 
      getNumOfBins () const
      { 
        return (nr_bins_);
      }

      /** \brief Initializes the set of energy maps.
        * \param[in] width the width of the energy maps.
        * \param[in] height the height of the energy maps.
        * \param[in] nr_bins the number of bins used for quantization.
        */
      void 
      initialize (const std::size_t width, const std::size_t height, const std::size_t nr_bins)
      {
        maps_.resize(nr_bins, nullptr);
        width_ = width;
        height_ = height;
        nr_bins_ = nr_bins;

        const std::size_t mapsSize = width*height;

        for (auto &map : maps_)
        {
          //maps_[map_index] = new unsigned char[mapsSize];
          map = reinterpret_cast<unsigned char*> (aligned_malloc (mapsSize));
          std::fill_n(map, mapsSize, 0);
        }
      }

      /** \brief Releases the internal data. */
      void 
      releaseAll ()
      {
        for (auto &map : maps_)
          //if (maps_[map_index] != NULL) delete[] maps_[map_index];
          if (map != nullptr) aligned_free (map);

        maps_.clear ();
        width_ = 0;
        height_ = 0;
        nr_bins_ = 0;
      }

      /** \brief Operator for accessing a specific element in the set of energy maps.
        * \param[in] bin_index the quantization bin (states which of the energy maps to access).
        * \param[in] col_index the column index within the specified energy map.
        * \param[in] row_index the row index within the specified energy map.
        */
      inline unsigned char & 
      operator() (const std::size_t bin_index, const std::size_t col_index, const std::size_t row_index)
      {
        return (maps_[bin_index][row_index*width_ + col_index]);
      }

      /** \brief Operator for accessing a specific element in the set of energy maps.
        * \param[in] bin_index the quantization bin (states which of the energy maps to access).
        * \param[in] index the element index within the specified energy map.
        */
      inline unsigned char & 
      operator() (const std::size_t bin_index, const std::size_t index)
      {
        return (maps_[bin_index][index]);
      }

      /** \brief Returns a pointer to the data of the specified energy map.
        * \param[in] bin_index the index of the energy map to return (== the quantization bin).
        */
      inline unsigned char * 
      operator() (const std::size_t bin_index)
      {
        return (maps_[bin_index]);
      }

      /** \brief Operator for accessing a specific element in the set of energy maps.
        * \param[in] bin_index the quantization bin (states which of the energy maps to access).
        * \param[in] col_index the column index within the specified energy map.
        * \param[in] row_index the row index within the specified energy map.
        */
      inline const unsigned char & 
      operator() (const std::size_t bin_index, const std::size_t col_index, const std::size_t row_index) const
      {
        return (maps_[bin_index][row_index*width_ + col_index]);
      }

      /** \brief Operator for accessing a specific element in the set of energy maps.
        * \param[in] bin_index the quantization bin (states which of the energy maps to access).
        * \param[in] index the element index within the specified energy map.
        */
      inline const unsigned char & 
      operator() (const std::size_t bin_index, const std::size_t index) const
      {
        return (maps_[bin_index][index]);
      }

      /** \brief Returns a pointer to the data of the specified energy map.
        * \param[in] bin_index the index of the energy map to return (== the quantization bin).
        */
      inline const unsigned char * 
      operator() (const std::size_t bin_index) const
      {
        return (maps_[bin_index]);
      }

    private:
      /** \brief The width of the energy maps. */
      std::size_t width_;
      /** \brief The height of the energy maps. */
      std::size_t height_;
      /** \brief The number of quantization bins (== the number of internally stored energy maps). */
      std::size_t nr_bins_;
      /** \brief Storage for the energy maps. */
      std::vector<unsigned char*> maps_;
  };

  /** \brief Stores a set of linearized maps.
    * \author Stefan Holzer
    */
  class PCL_EXPORTS LinearizedMaps
  {
    public:
      /** \brief Constructor. */
      LinearizedMaps () : width_ (0), height_ (0), mem_width_ (0), mem_height_ (0), step_size_ (0)
      {
      }
      
      /** \brief Destructor. */
      virtual ~LinearizedMaps () = default;

      /** \brief Returns the width of the linearized map. */
      inline std::size_t 
      getWidth () const { return (width_); }
      
      /** \brief Returns the height of the linearized map. */
      inline std::size_t 
      getHeight () const { return (height_); }
      
      /** \brief Returns the step-size used to construct the linearized map. */
      inline std::size_t 
      getStepSize () const { return (step_size_); }
      
      /** \brief Returns the size of the memory map. */
      inline std::size_t 
      getMapMemorySize () const { return (mem_width_ * mem_height_); }

      /** \brief Initializes the linearized map.
        * \param[in] width the width of the source map.
        * \param[in] height the height of the source map.
        * \param[in] step_size the step-size used to sample the source map.
        */
      void 
      initialize (const std::size_t width, const std::size_t height, const std::size_t step_size)
      {
        maps_.resize(step_size*step_size, nullptr);
        width_ = width;
        height_ = height;
        mem_width_ = width / step_size;
        mem_height_ = height / step_size;
        step_size_ = step_size;

        const std::size_t mapsSize = mem_width_ * mem_height_;

        for (auto &map : maps_)
        {
          //maps_[map_index] = new unsigned char[2*mapsSize];
          map = reinterpret_cast<unsigned char*> (aligned_malloc (2*mapsSize));
          std::fill_n(map, 2*mapsSize, 0);
        }
      }

      /** \brief Releases the internal memory. */
      void 
      releaseAll ()
      {
        for (auto &map : maps_)
          //if (maps_[map_index] != NULL) delete[] maps_[map_index];
          if (map != nullptr) aligned_free (map);

        maps_.clear ();
        width_ = 0;
        height_ = 0;
        mem_width_ = 0;
        mem_height_ = 0;
        step_size_ = 0;
      }

      /** \brief Operator to access elements of the linearized map by column and row index.
        * \param[in] col_index the column index.
        * \param[in] row_index the row index.
        */
      inline unsigned char * 
      operator() (const std::size_t col_index, const std::size_t row_index)
      {
        return (maps_[row_index*step_size_ + col_index]);
      }

      /** \brief Returns a linearized map starting at the specified position.
        * \param[in] col_index the column index at which the returned map starts.
        * \param[in] row_index the row index at which the returned map starts.
        */
      inline unsigned char * 
      getOffsetMap (const std::size_t col_index, const std::size_t row_index)
      {
        const std::size_t map_col = col_index % step_size_;
        const std::size_t map_row = row_index % step_size_;

        const std::size_t map_mem_col_index = col_index / step_size_;
        const std::size_t map_mem_row_index = row_index / step_size_;

        return (maps_[map_row*step_size_ + map_col] + map_mem_row_index*mem_width_ + map_mem_col_index);
      }

    private:
      /** \brief the original width of the data represented by the map. */
      std::size_t width_;
      /** \brief the original height of the data represented by the map. */
      std::size_t height_;
      /** \brief the actual width of the linearized map. */
      std::size_t mem_width_;
      /** \brief the actual height of the linearized map. */
      std::size_t mem_height_;
      /** \brief the step-size used for sampling the original data. */
      std::size_t step_size_;
      /** \brief a vector containing all the linearized maps. */
      std::vector<unsigned char*> maps_;
  };

  /** \brief Represents a detection of a template using the LINEMOD approach.
    * \author Stefan Holzer
    */
  struct PCL_EXPORTS LINEMODDetection
  {
    /** \brief Constructor. */
    LINEMODDetection () : x (0), y (0), template_id (0), score (0.0f), scale (1.0f) {}

    /** \brief x-position of the detection. */
    int x;
    /** \brief y-position of the detection. */
    int y;
    /** \brief ID of the detected template. */
    int template_id;
    /** \brief score of the detection. */
    float score;
    /** \brief scale at which the template was detected. */
    float scale;
  };

  /**
    * \brief Template matching using the LINEMOD approach.
    * \author Stefan Holzer, Stefan Hinterstoisser
    * \ingroup recognition
    */
  class PCL_EXPORTS LINEMOD
  {
    public:
      /** \brief Constructor */
      LINEMOD ();

      /** \brief Destructor */
      virtual ~LINEMOD ();

      /** \brief Creates a template from the specified data and adds it to the matching queue. 
        * \param[in] modalities the modalities used to create the template.
        * \param[in] masks the masks that determine which parts of the modalities are used for creating the template.
        * \param[in] region the region which will be associated with the template (can be larger than the actual modality-maps).
        */
      int 
      createAndAddTemplate (const std::vector<QuantizableModality*> & modalities,
                            const std::vector<MaskMap*> & masks,
                            const RegionXY & region);

      /** \brief Adds the specified template to the matching queue.
        * \param[in] linemod_template the template to add.
        */
      int
      addTemplate (const SparseQuantizedMultiModTemplate & linemod_template);

      /** \brief Detects the stored templates in the supplied modality data.
        * \param[in] modalities the modalities that will be used for detection.
        * \param[out] detections the destination for the detections.
        */
      void
      detectTemplates (const std::vector<QuantizableModality*> & modalities,
                       std::vector<LINEMODDetection> & detections) const;

      /** \brief Detects the stored templates in a semi scale invariant manner 
        *        by applying the detection to multiple scaled versions of the input data.
        * \param[in] modalities the modalities that will be used for detection.
        * \param[out] detections the destination for the detections.
        * \param[in] min_scale the minimum scale.
        * \param[in] max_scale the maximum scale.
        * \param[in] scale_multiplier the multiplier for getting from one scale to the next.
        */
      void
      detectTemplatesSemiScaleInvariant (const std::vector<QuantizableModality*> & modalities,
                                         std::vector<LINEMODDetection> & detections,
                                         float min_scale = 0.6944444f,
                                         float max_scale = 1.44f,
                                         float scale_multiplier = 1.2f) const;

      /** \brief Matches the stored templates to the supplied modality data.
        * \param[in] modalities the modalities that will be used for matching.
        * \param[out] matches the found matches.
        */
      void
      matchTemplates (const std::vector<QuantizableModality*> & modalities,
                      std::vector<LINEMODDetection> & matches) const;

      /** \brief Sets the detection threshold. 
        * \param[in] threshold the detection threshold.
        */
      inline void
      setDetectionThreshold (float threshold)
      {
        template_threshold_ = threshold;
      }

      /** \brief Enables/disables non-maximum suppression.
        * \param[in] use_non_max_suppression determines whether to use non-maximum suppression or not.
        */
      inline void
      setNonMaxSuppression (bool use_non_max_suppression)
      {
        use_non_max_suppression_ = use_non_max_suppression;
      }

      /** \brief Enables/disables averaging of close detections.
        * \param[in] average_detections determines whether to average close detections or not.
        */
      inline void
      setDetectionAveraging (bool average_detections)
      {
        average_detections_ = average_detections;
      }

      /** \brief Returns the template with the specified ID.
        * \param[in] template_id the ID of the template to return.
        */
      inline const SparseQuantizedMultiModTemplate &
      getTemplate (int template_id) const
      { 
        return (templates_[template_id]);
      }

      /** \brief Returns the number of stored/trained templates. */
      inline std::size_t
      getNumOfTemplates () const
      {
        return (templates_.size ());
      }

      /** \brief Saves the stored templates to the specified file.
        * \param[in] file_name the name of the file to save the templates to.
        */
      void
      saveTemplates (const char * file_name) const;

      /** \brief Loads templates from the specified file.
        * \param[in] file_name the name of the file to load the template from.
        */
      void
      loadTemplates (const char * file_name);

      /** \brief Loads templates from the specified files.
        * \param[in] file_names vector of files to load the templates from.
        */

      void
      loadTemplates (std::vector<std::string> & file_names);

      /** \brief Serializes the stored templates to the specified stream.
        * \param[in] stream the stream the templates will be written to.
        */
      void 
      serialize (std::ostream & stream) const;

      /** \brief Deserializes templates from the specified stream.
        * \param[in] stream the stream the templates will be read from.
        */
      void 
      deserialize (std::istream & stream);


    private:
      /** template response threshold */
      float template_threshold_;
      /** states whether non-max-suppression on detections is enabled or not */
      bool use_non_max_suppression_;
      /** states whether to return an averaged detection */
      bool average_detections_;
      /** template storage */
      std::vector<SparseQuantizedMultiModTemplate> templates_;
  };

}
