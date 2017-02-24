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

//#define __SSE2__

#include <pcl/recognition/linemod.h>

#ifdef __SSE2__
#include <emmintrin.h>
#endif

#include <fstream>

//#define LINEMOD_USE_SEPARATE_ENERGY_MAPS

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::LINEMOD::LINEMOD () 
  : template_threshold_ (0.75f)
  , use_non_max_suppression_ (false)
  , average_detections_ (false)
  , templates_ ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::LINEMOD::~LINEMOD()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
int 
pcl::LINEMOD::createAndAddTemplate (const std::vector<pcl::QuantizableModality*> & modalities,
                      const std::vector<pcl::MaskMap*> & masks,
                      const pcl::RegionXY & region)
{
  // assuming width and height is same for all modalities; should we check this??
  //const int width = modalities[0]->getQuantizedMap().getWidth ();
  //const int height = modalities[0]->getQuantizedMap().getHeight ();

  SparseQuantizedMultiModTemplate linemod_template;

  // select N features from every modality (N = 50, hardcoded; CHANGE this to a parameter!!!)
  const size_t nr_features_per_modality = 63;
  const size_t nr_modalities = modalities.size();
  for (size_t modality_index = 0; modality_index < nr_modalities; ++modality_index)
  {
    const MaskMap & mask = *(masks[modality_index]);
    modalities[modality_index]->extractFeatures(mask, nr_features_per_modality, modality_index,
                                                linemod_template.features);
  }

  // up to now all features are relative to the input frame; make them relative to the region center
  //const int centerX = region.x+region.width/2;
  //const int centerY = region.y+region.height/2;

  const size_t nr_features = linemod_template.features.size();
  for (size_t feature_index = 0; feature_index < nr_features; ++feature_index)
  {
    //linemod_template.features[feature_index].x -= centerX;
    //linemod_template.features[feature_index].y -= centerY;
    linemod_template.features[feature_index].x -= region.x;
    linemod_template.features[feature_index].y -= region.y;
  }

  // set region relative to the center
  linemod_template.region.x = 0;
  linemod_template.region.y = 0;
  //linemod_template.region.x = region.x - centerX;
  //linemod_template.region.y = region.y - centerY;
  linemod_template.region.width = region.width;
  linemod_template.region.height = region.height;

  // add template to template storage
  templates_.push_back(linemod_template);

  return static_cast<int> (templates_.size () - 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
int 
pcl::LINEMOD::addTemplate (const SparseQuantizedMultiModTemplate & linemod_template)
{
  // add template to template storage
  templates_.push_back(linemod_template);

  return static_cast<int> (templates_.size () - 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::matchTemplates (const std::vector<QuantizableModality*> & modalities, std::vector<LINEMODDetection> & detections) const
{
  // create energy maps
  std::vector<EnergyMaps> modality_energy_maps;
  const size_t nr_modalities = modalities.size();
  for (size_t modality_index = 0; modality_index < nr_modalities; ++modality_index)
  {
    const QuantizedMap & quantized_map = modalities[modality_index]->getSpreadedQuantizedMap ();

    const size_t width = quantized_map.getWidth ();
    const size_t height = quantized_map.getHeight ();

    const unsigned char * quantized_data = quantized_map.getData ();

    const unsigned char nr_bins = 8;
    EnergyMaps energy_maps;
    energy_maps.initialize (width, height, nr_bins);
    //std::vector< unsigned char* > energy_maps(nr_bins);
    for (unsigned char bin_index = 0; bin_index < nr_bins; ++bin_index)
    {
      //energy_maps[bin_index] = new unsigned char[width*height];
      //memset (energy_maps[bin_index], 0, width*height);

      const unsigned char base_bit = static_cast<unsigned char> (0x1);
      unsigned char val0 = static_cast<unsigned char> (base_bit << bin_index); // e.g. 00100000
      unsigned char val1 = static_cast<unsigned char> (val0 | (base_bit << (bin_index+1)&7) | (base_bit << (bin_index+7)&7)); // e.g. 01110000
      unsigned char val2 = static_cast<unsigned char> (val1 | (base_bit << (bin_index+2)&7) | (base_bit << (bin_index+6)&7)); // e.g. 11111000
      unsigned char val3 = static_cast<unsigned char> (val2 | (base_bit << (bin_index+3)&7) | (base_bit << (bin_index+5)&7)); // e.g. 11111101
      for (size_t index = 0; index < width*height; ++index)
      {
        if ((val0 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
        if ((val1 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
        if ((val2 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
        if ((val3 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
      }
    }

    modality_energy_maps.push_back (energy_maps);
  }

  // create linearized maps
  const size_t step_size = 8;
  std::vector<std::vector<LinearizedMaps> > modality_linearized_maps;
  for (size_t modality_index = 0; modality_index < nr_modalities; ++modality_index)
  {
    const size_t width = modality_energy_maps[modality_index].getWidth ();
    const size_t height = modality_energy_maps[modality_index].getHeight ();

    std::vector<LinearizedMaps> linearized_maps;
    const size_t nr_bins = modality_energy_maps[modality_index].getNumOfBins ();
    for (size_t bin_index = 0; bin_index < nr_bins; ++bin_index)
    {
      unsigned char * energy_map = modality_energy_maps[modality_index] (bin_index);

      LinearizedMaps maps;
      maps.initialize (width, height, step_size);
      for (size_t map_row = 0; map_row < step_size; ++map_row)
      {
        for (size_t map_col = 0; map_col < step_size; ++map_col)
        {
          unsigned char * linearized_map = maps (map_col, map_row);

          // copy data from energy maps
          const size_t lin_width = width/step_size;
          const size_t lin_height = height/step_size;
          for (size_t row_index = 0; row_index < lin_height; ++row_index)
          {
            for (size_t col_index = 0; col_index < lin_width; ++col_index)
            {
              const size_t tmp_col_index = col_index*step_size + map_col;
              const size_t tmp_row_index = row_index*step_size + map_row;

              linearized_map[row_index*lin_width + col_index] = energy_map[tmp_row_index*width + tmp_col_index];
            }
          }
        }
      }

      linearized_maps.push_back (maps);
    }

    modality_linearized_maps.push_back (linearized_maps);
  }

  // compute scores for templates
  const size_t width = modality_energy_maps[0].getWidth ();
  const size_t height = modality_energy_maps[0].getHeight ();
  for (size_t template_index = 0; template_index < templates_.size (); ++template_index)
  {
    const size_t mem_width = width / step_size;
    const size_t mem_height = height / step_size;
    const size_t mem_size = mem_width * mem_height;

#ifdef __SSE2__
    unsigned short * score_sums = reinterpret_cast<unsigned short*> (aligned_malloc (mem_size*sizeof(unsigned short)));
    unsigned char * tmp_score_sums = reinterpret_cast<unsigned char*> (aligned_malloc (mem_size*sizeof(unsigned char)));
    memset (score_sums, 0, mem_size*sizeof (score_sums[0]));
    memset (tmp_score_sums, 0, mem_size*sizeof (tmp_score_sums[0]));

    //__m128i * score_sums_m128i = reinterpret_cast<__m128i*> (score_sums);
    __m128i * tmp_score_sums_m128i = reinterpret_cast<__m128i*> (tmp_score_sums);

    const size_t mem_size_16 = mem_size / 16;
    //const size_t mem_size_mod_16 = mem_size & 15;
    const size_t mem_size_mod_16_base = mem_size_16 * 16;

    size_t max_score = 0;
    size_t copy_back_counter = 0;
    for (size_t feature_index = 0; feature_index < templates_[template_index].features.size (); ++feature_index)
    {
      const QuantizedMultiModFeature & feature = templates_[template_index].features[feature_index];

      for (size_t bin_index = 0; bin_index < 8; ++bin_index)
      {
        if ((feature.quantized_value & (0x1<<bin_index)) != 0)
        {
          max_score += 4;

          unsigned char * data = modality_linearized_maps[feature.modality_index][bin_index].getOffsetMap (feature.x, feature.y);
          __m128i * data_m128i = reinterpret_cast<__m128i*> (data);

          for (size_t mem_index = 0; mem_index < mem_size_16; ++mem_index)
          {
            __m128i aligned_data_m128i = _mm_loadu_si128 (reinterpret_cast<const __m128i*> (data_m128i + mem_index)); // SSE2
            //__m128i aligned_data_m128i = _mm_lddqu_si128 (reinterpret_cast<const __m128i*> (data_m128i + mem_index)); // SSE3
            tmp_score_sums_m128i[mem_index] = _mm_add_epi8 (tmp_score_sums_m128i[mem_index], aligned_data_m128i);
          }
          for (size_t mem_index = mem_size_mod_16_base; mem_index < mem_size; ++mem_index)
          {
            tmp_score_sums[mem_index] = static_cast<unsigned char> (tmp_score_sums[mem_index] + data[mem_index]);
          }
        }
      }

      ++copy_back_counter;

      //if ((feature_index & 7) == 7)
      //if ((feature_index & 63) == 63)
      if (copy_back_counter > 63) // only valid if each feature has only one bit set..
      {
        copy_back_counter = 0;

        for (size_t mem_index = 0; mem_index < mem_size_mod_16_base; mem_index += 16)
        {
          score_sums[mem_index+0]  = static_cast<unsigned short> (score_sums[mem_index+0]  + tmp_score_sums[mem_index+0]);
          score_sums[mem_index+1]  = static_cast<unsigned short> (score_sums[mem_index+1]  + tmp_score_sums[mem_index+1]);
          score_sums[mem_index+2]  = static_cast<unsigned short> (score_sums[mem_index+2]  + tmp_score_sums[mem_index+2]);
          score_sums[mem_index+3]  = static_cast<unsigned short> (score_sums[mem_index+3]  + tmp_score_sums[mem_index+3]);
          score_sums[mem_index+4]  = static_cast<unsigned short> (score_sums[mem_index+4]  + tmp_score_sums[mem_index+4]);
          score_sums[mem_index+5]  = static_cast<unsigned short> (score_sums[mem_index+5]  + tmp_score_sums[mem_index+5]);
          score_sums[mem_index+6]  = static_cast<unsigned short> (score_sums[mem_index+6]  + tmp_score_sums[mem_index+6]);
          score_sums[mem_index+7]  = static_cast<unsigned short> (score_sums[mem_index+7]  + tmp_score_sums[mem_index+7]);
          score_sums[mem_index+8]  = static_cast<unsigned short> (score_sums[mem_index+8]  + tmp_score_sums[mem_index+8]);
          score_sums[mem_index+9]  = static_cast<unsigned short> (score_sums[mem_index+9]  + tmp_score_sums[mem_index+9]);
          score_sums[mem_index+10] = static_cast<unsigned short> (score_sums[mem_index+10] + tmp_score_sums[mem_index+10]);
          score_sums[mem_index+11] = static_cast<unsigned short> (score_sums[mem_index+11] + tmp_score_sums[mem_index+11]);
          score_sums[mem_index+12] = static_cast<unsigned short> (score_sums[mem_index+12] + tmp_score_sums[mem_index+12]);
          score_sums[mem_index+13] = static_cast<unsigned short> (score_sums[mem_index+13] + tmp_score_sums[mem_index+13]);
          score_sums[mem_index+14] = static_cast<unsigned short> (score_sums[mem_index+14] + tmp_score_sums[mem_index+14]);
          score_sums[mem_index+15] = static_cast<unsigned short> (score_sums[mem_index+15] + tmp_score_sums[mem_index+15]);
        }
        for (size_t mem_index = mem_size_mod_16_base; mem_index < mem_size; ++mem_index)
        {
          score_sums[mem_index] = static_cast<unsigned short> (score_sums[mem_index] + tmp_score_sums[mem_index]);
        }

        memset (tmp_score_sums, 0, mem_size*sizeof (tmp_score_sums[0]));
      }
    }
    {
      for (size_t mem_index = 0; mem_index < mem_size; ++mem_index)
      {
        score_sums[mem_index] = static_cast<unsigned short> (score_sums[mem_index] + tmp_score_sums[mem_index]);
      }
        
      memset (tmp_score_sums, 0, mem_size*sizeof (tmp_score_sums[0]));
    }
#else
    unsigned short * score_sums = new unsigned short[mem_size];
    //unsigned char * score_sums = new unsigned char[mem_size];
    memset (score_sums, 0, mem_size*sizeof (score_sums[0]));

    size_t max_score = 0;
    for (size_t feature_index = 0; feature_index < templates_[template_index].features.size (); ++feature_index)
    {
      const QuantizedMultiModFeature & feature = templates_[template_index].features[feature_index];

      //feature.modality_index;
      for (size_t bin_index = 0; bin_index < 8; ++bin_index)
      {
        if ((feature.quantized_value & (0x1<<bin_index)) != 0)
        {
          max_score += 4;

          unsigned char * data = modality_linearized_maps[feature.modality_index][bin_index].getOffsetMap (feature.x, feature.y);
          for (size_t mem_index = 0; mem_index < mem_size; ++mem_index)
          {
            score_sums[mem_index] += data[mem_index];
          }
        }
      }
    }
#endif

    const float inv_max_score = 1.0f / float (max_score);
    
    size_t max_value = 0;
    size_t max_index = 0;
    for (size_t mem_index = 0; mem_index < mem_size; ++mem_index)
    {
      if (score_sums[mem_index] > max_value) 
      {
        max_value = score_sums[mem_index];
        max_index = mem_index;
      }
    }

    const size_t max_col_index = (max_index % mem_width) * step_size;
    const size_t max_row_index = (max_index / mem_width) * step_size;

    LINEMODDetection detection;
    detection.x = static_cast<int> (max_col_index);
    detection.y = static_cast<int> (max_row_index);
    detection.template_id = static_cast<int> (template_index);
    detection.score = static_cast<float> (max_value) * inv_max_score;

    detections.push_back (detection);

#ifdef __SSE2__
    aligned_free (score_sums);
    aligned_free (tmp_score_sums);
#else
    delete[] score_sums;
#endif
  }

  // release data
  for (size_t modality_index = 0; modality_index < modality_linearized_maps.size (); ++modality_index)
  {
    modality_energy_maps[modality_index].releaseAll ();
    for (size_t bin_index = 0; bin_index < modality_linearized_maps[modality_index].size (); ++bin_index)
      modality_linearized_maps[modality_index][bin_index].releaseAll ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::detectTemplates (const std::vector<QuantizableModality*> & modalities, std::vector<LINEMODDetection> & detections) const
{
  // create energy maps
  std::vector<EnergyMaps> modality_energy_maps;
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
  std::vector<EnergyMaps> modality_energy_maps_1;
  std::vector<EnergyMaps> modality_energy_maps_2;
  std::vector<EnergyMaps> modality_energy_maps_3;
#endif
  const size_t nr_modalities = modalities.size();
  for (size_t modality_index = 0; modality_index < nr_modalities; ++modality_index)
  {
    const QuantizedMap & quantized_map = modalities[modality_index]->getSpreadedQuantizedMap ();

    const size_t width = quantized_map.getWidth ();
    const size_t height = quantized_map.getHeight ();

    const unsigned char * quantized_data = quantized_map.getData ();

    const int nr_bins = 8;
    EnergyMaps energy_maps;
    energy_maps.initialize (width, height, nr_bins);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    EnergyMaps energy_maps_1;
    EnergyMaps energy_maps_2;
    EnergyMaps energy_maps_3;
    energy_maps_1.initialize (width, height, nr_bins);
    energy_maps_2.initialize (width, height, nr_bins);
    energy_maps_3.initialize (width, height, nr_bins);
#endif
    //std::vector< unsigned char* > energy_maps(nr_bins);
    for (int bin_index = 0; bin_index < nr_bins; ++bin_index)
    {
      //energy_maps[bin_index] = new unsigned char[width*height];
      //memset (energy_maps[bin_index], 0, width*height);

      const unsigned char base_bit = static_cast<unsigned char> (0x1);
      unsigned char val0 = static_cast<unsigned char> (base_bit << bin_index); // e.g. 00100000
      unsigned char val1 = static_cast<unsigned char> (val0 | (base_bit << ((bin_index+1)%8)) | (base_bit << ((bin_index+7)%8))); // e.g. 01110000
      unsigned char val2 = static_cast<unsigned char> (val1 | (base_bit << ((bin_index+2)%8)) | (base_bit << ((bin_index+6)%8))); // e.g. 11111000
      unsigned char val3 = static_cast<unsigned char> (val2 | (base_bit << ((bin_index+3)%8)) | (base_bit << ((bin_index+5)%8))); // e.g. 11111101
      for (size_t index = 0; index < width*height; ++index)
      {
        if ((val0 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
        if ((val1 & quantized_data[index]) != 0)
          ++energy_maps_1 (bin_index, index);
        if ((val2 & quantized_data[index]) != 0)
          ++energy_maps_2 (bin_index, index);
        if ((val3 & quantized_data[index]) != 0)
          ++energy_maps_3 (bin_index, index);
#else
        if ((val1 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
        if ((val2 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
        if ((val3 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
#endif
      }
    }

    modality_energy_maps.push_back (energy_maps);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    modality_energy_maps_1.push_back (energy_maps_1);
    modality_energy_maps_2.push_back (energy_maps_2);
    modality_energy_maps_3.push_back (energy_maps_3);
#endif
  }

  // create linearized maps
  const size_t step_size = 8;
  std::vector<std::vector<LinearizedMaps> > modality_linearized_maps;
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
  std::vector<std::vector<LinearizedMaps> > modality_linearized_maps_1;
  std::vector<std::vector<LinearizedMaps> > modality_linearized_maps_2;
  std::vector<std::vector<LinearizedMaps> > modality_linearized_maps_3;
#endif
  for (size_t modality_index = 0; modality_index < nr_modalities; ++modality_index)
  {
    const size_t width = modality_energy_maps[modality_index].getWidth ();
    const size_t height = modality_energy_maps[modality_index].getHeight ();

    std::vector<LinearizedMaps> linearized_maps;
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    std::vector<LinearizedMaps> linearized_maps_1;
    std::vector<LinearizedMaps> linearized_maps_2;
    std::vector<LinearizedMaps> linearized_maps_3;
#endif
    const size_t nr_bins = modality_energy_maps[modality_index].getNumOfBins ();
    for (size_t bin_index = 0; bin_index < nr_bins; ++bin_index)
    {
      unsigned char * energy_map = modality_energy_maps[modality_index] (bin_index);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      unsigned char * energy_map_1 = modality_energy_maps_1[modality_index] (bin_index);
      unsigned char * energy_map_2 = modality_energy_maps_2[modality_index] (bin_index);
      unsigned char * energy_map_3 = modality_energy_maps_3[modality_index] (bin_index);
#endif

      LinearizedMaps maps;
      maps.initialize (width, height, step_size);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      LinearizedMaps maps_1;
      LinearizedMaps maps_2;
      LinearizedMaps maps_3;
      maps_1.initialize (width, height, step_size);
      maps_2.initialize (width, height, step_size);
      maps_3.initialize (width, height, step_size);
#endif
      for (size_t map_row = 0; map_row < step_size; ++map_row)
      {
        for (size_t map_col = 0; map_col < step_size; ++map_col)
        {
          unsigned char * linearized_map = maps (map_col, map_row);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
          unsigned char * linearized_map_1 = maps_1 (map_col, map_row);
          unsigned char * linearized_map_2 = maps_2 (map_col, map_row);
          unsigned char * linearized_map_3 = maps_3 (map_col, map_row);
#endif

          // copy data from energy maps
          const size_t lin_width = width/step_size;
          const size_t lin_height = height/step_size;
          for (size_t row_index = 0; row_index < lin_height; ++row_index)
          {
            for (size_t col_index = 0; col_index < lin_width; ++col_index)
            {
              const size_t tmp_col_index = col_index*step_size + map_col;
              const size_t tmp_row_index = row_index*step_size + map_row;

              linearized_map[row_index*lin_width + col_index] = energy_map[tmp_row_index*width + tmp_col_index];
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
              linearized_map_1[row_index*lin_width + col_index] = energy_map_1[tmp_row_index*width + tmp_col_index];
              linearized_map_2[row_index*lin_width + col_index] = energy_map_2[tmp_row_index*width + tmp_col_index];
              linearized_map_3[row_index*lin_width + col_index] = energy_map_3[tmp_row_index*width + tmp_col_index];
#endif
            }
          }
        }
      }

      linearized_maps.push_back (maps);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      linearized_maps_1.push_back (maps_1);
      linearized_maps_2.push_back (maps_2);
      linearized_maps_3.push_back (maps_3);
#endif
    }

    modality_linearized_maps.push_back (linearized_maps);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    modality_linearized_maps_1.push_back (linearized_maps_1);
    modality_linearized_maps_2.push_back (linearized_maps_2);
    modality_linearized_maps_3.push_back (linearized_maps_3);
#endif
  }

  // compute scores for templates
  const size_t width = modality_energy_maps[0].getWidth ();
  const size_t height = modality_energy_maps[0].getHeight ();
  for (size_t template_index = 0; template_index < templates_.size (); ++template_index)
  {
    const size_t mem_width = width / step_size;
    const size_t mem_height = height / step_size;
    const size_t mem_size = mem_width * mem_height;

#ifdef __SSE2__
    unsigned short * score_sums = reinterpret_cast<unsigned short*> (aligned_malloc (mem_size*sizeof(unsigned short)));
    unsigned char * tmp_score_sums = reinterpret_cast<unsigned char*> (aligned_malloc (mem_size*sizeof(unsigned char)));
    memset (score_sums, 0, mem_size*sizeof (score_sums[0]));
    memset (tmp_score_sums, 0, mem_size*sizeof (tmp_score_sums[0]));

    //__m128i * score_sums_m128i = reinterpret_cast<__m128i*> (score_sums);
    __m128i * tmp_score_sums_m128i = reinterpret_cast<__m128i*> (tmp_score_sums);

    const size_t mem_size_16 = mem_size / 16;
    //const size_t mem_size_mod_16 = mem_size & 15;
    const size_t mem_size_mod_16_base = mem_size_16 * 16;

    int max_score = 0;
    size_t copy_back_counter = 0;
    for (size_t feature_index = 0; feature_index < templates_[template_index].features.size (); ++feature_index)
    {
      const QuantizedMultiModFeature & feature = templates_[template_index].features[feature_index];

      for (size_t bin_index = 0; bin_index < 8; ++bin_index)
      {
        if ((feature.quantized_value & (0x1<<bin_index)) != 0)
        {
          max_score += 4;

          unsigned char * data = modality_linearized_maps[feature.modality_index][bin_index].getOffsetMap (feature.x, feature.y);
          __m128i * data_m128i = reinterpret_cast<__m128i*> (data);

          for (size_t mem_index = 0; mem_index < mem_size_16; ++mem_index)
          {
            __m128i aligned_data_m128i = _mm_loadu_si128 (reinterpret_cast<const __m128i*> (data_m128i + mem_index)); // SSE2
            //__m128i aligned_data_m128i = _mm_lddqu_si128 (reinterpret_cast<const __m128i*> (data_m128i + mem_index)); // SSE3
            tmp_score_sums_m128i[mem_index] = _mm_add_epi8 (tmp_score_sums_m128i[mem_index], aligned_data_m128i);
          }
          for (size_t mem_index = mem_size_mod_16_base; mem_index < mem_size; ++mem_index)
          {
            tmp_score_sums[mem_index] = static_cast<unsigned char> (tmp_score_sums[mem_index] + data[mem_index]);
          }
        }
      }

      ++copy_back_counter;

      //if ((feature_index & 7) == 7)
      //if ((feature_index & 63) == 63)
      if (copy_back_counter > 63) // only valid if each feature has only one bit set..
      {
        copy_back_counter = 0;

        for (size_t mem_index = 0; mem_index < mem_size_mod_16_base; mem_index += 16)
        {
          score_sums[mem_index+0]  = static_cast<unsigned short> (score_sums[mem_index+0]  + tmp_score_sums[mem_index+0]);
          score_sums[mem_index+1]  = static_cast<unsigned short> (score_sums[mem_index+1]  + tmp_score_sums[mem_index+1]);
          score_sums[mem_index+2]  = static_cast<unsigned short> (score_sums[mem_index+2]  + tmp_score_sums[mem_index+2]);
          score_sums[mem_index+3]  = static_cast<unsigned short> (score_sums[mem_index+3]  + tmp_score_sums[mem_index+3]);
          score_sums[mem_index+4]  = static_cast<unsigned short> (score_sums[mem_index+4]  + tmp_score_sums[mem_index+4]);
          score_sums[mem_index+5]  = static_cast<unsigned short> (score_sums[mem_index+5]  + tmp_score_sums[mem_index+5]);
          score_sums[mem_index+6]  = static_cast<unsigned short> (score_sums[mem_index+6]  + tmp_score_sums[mem_index+6]);
          score_sums[mem_index+7]  = static_cast<unsigned short> (score_sums[mem_index+7]  + tmp_score_sums[mem_index+7]);
          score_sums[mem_index+8]  = static_cast<unsigned short> (score_sums[mem_index+8]  + tmp_score_sums[mem_index+8]);
          score_sums[mem_index+9]  = static_cast<unsigned short> (score_sums[mem_index+9]  + tmp_score_sums[mem_index+9]);
          score_sums[mem_index+10] = static_cast<unsigned short> (score_sums[mem_index+10] + tmp_score_sums[mem_index+10]);
          score_sums[mem_index+11] = static_cast<unsigned short> (score_sums[mem_index+11] + tmp_score_sums[mem_index+11]);
          score_sums[mem_index+12] = static_cast<unsigned short> (score_sums[mem_index+12] + tmp_score_sums[mem_index+12]);
          score_sums[mem_index+13] = static_cast<unsigned short> (score_sums[mem_index+13] + tmp_score_sums[mem_index+13]);
          score_sums[mem_index+14] = static_cast<unsigned short> (score_sums[mem_index+14] + tmp_score_sums[mem_index+14]);
          score_sums[mem_index+15] = static_cast<unsigned short> (score_sums[mem_index+15] + tmp_score_sums[mem_index+15]);
        }
        for (size_t mem_index = mem_size_mod_16_base; mem_index < mem_size; ++mem_index)
        {
          score_sums[mem_index] = static_cast<unsigned short> (score_sums[mem_index] + tmp_score_sums[mem_index]);
        }

        memset (tmp_score_sums, 0, mem_size*sizeof (tmp_score_sums[0]));
      }
    }
    {
      for (size_t mem_index = 0; mem_index < mem_size; ++mem_index)
      {
        score_sums[mem_index] = static_cast<unsigned short> (score_sums[mem_index] + tmp_score_sums[mem_index]);
      }
        
      memset (tmp_score_sums, 0, mem_size*sizeof (tmp_score_sums[0]));
    }
#else
    unsigned short * score_sums = new unsigned short[mem_size];
    //unsigned char * score_sums = new unsigned char[mem_size];
    memset (score_sums, 0, mem_size*sizeof (score_sums[0]));

#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    unsigned short * score_sums_1 = new unsigned short[mem_size];
    unsigned short * score_sums_2 = new unsigned short[mem_size];
    unsigned short * score_sums_3 = new unsigned short[mem_size];
    memset (score_sums_1, 0, mem_size*sizeof (score_sums_1[0]));
    memset (score_sums_2, 0, mem_size*sizeof (score_sums_2[0]));
    memset (score_sums_3, 0, mem_size*sizeof (score_sums_3[0]));
#endif

    int max_score = 0;
    for (size_t feature_index = 0; feature_index < templates_[template_index].features.size (); ++feature_index)
    {
      const QuantizedMultiModFeature & feature = templates_[template_index].features[feature_index];

      //feature.modality_index;
      for (size_t bin_index = 0; bin_index < 8; ++bin_index)
      {
        if ((feature.quantized_value & (0x1<<bin_index)) != 0)
        {
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
          ++max_score;

          unsigned char * data = modality_linearized_maps[feature.modality_index][bin_index].getOffsetMap (feature.x, feature.y);
          unsigned char * data_1 = modality_linearized_maps_1[feature.modality_index][bin_index].getOffsetMap (feature.x, feature.y);
          unsigned char * data_2 = modality_linearized_maps_2[feature.modality_index][bin_index].getOffsetMap (feature.x, feature.y);
          unsigned char * data_3 = modality_linearized_maps_3[feature.modality_index][bin_index].getOffsetMap (feature.x, feature.y);
          for (size_t mem_index = 0; mem_index < mem_size; ++mem_index)
          {
            score_sums[mem_index] += data[mem_index];
            score_sums_1[mem_index] += data_1[mem_index];
            score_sums_2[mem_index] += data_2[mem_index];
            score_sums_3[mem_index] += data_3[mem_index];
          }
#else
          max_score += 4;

          unsigned char * data = modality_linearized_maps[feature.modality_index][bin_index].getOffsetMap (feature.x, feature.y);
          for (size_t mem_index = 0; mem_index < mem_size; ++mem_index)
          {
            score_sums[mem_index] += data[mem_index];
          }
#endif
        }
      }
    }
#endif

    const float inv_max_score = 1.0f / float (max_score);

    // we compute a new threshold based on the threshold supplied by the user;
    // this is due to the use of the cosine approx. in the response computation;
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    const float raw_threshold = (4.0f * float (max_score) / 2.0f + template_threshold_ * (4.0f * float (max_score) / 2.0f));
#else
    const float raw_threshold = (float (max_score) / 2.0f + template_threshold_ * (float (max_score) / 2.0f));
#endif

    //int max_value = 0;
    //size_t max_index = 0;
    for (size_t mem_index = 0; mem_index < mem_size; ++mem_index)
    {
      //const float score = score_sums[mem_index] * inv_max_score;

#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      const float raw_score = score_sums[mem_index] 
        + score_sums_1[mem_index]
        + score_sums_2[mem_index]
        + score_sums_3[mem_index];

      const float score = 2.0f * static_cast<float> (raw_score) * 0.25f * inv_max_score - 1.0f;
#else
      const float raw_score = score_sums[mem_index];

      const float score = 2.0f * static_cast<float> (raw_score) * inv_max_score - 1.0f;
#endif


      //if (score > template_threshold_) 
      if (raw_score > raw_threshold) /// \todo Ask Stefan why this line was used instead of the one above
      {
        const size_t mem_col_index = (mem_index % mem_width);
        const size_t mem_row_index = (mem_index / mem_width);

        if (use_non_max_suppression_)
        {
          bool is_local_max = true;
          for (size_t sup_row_index = mem_row_index-1; sup_row_index <= mem_row_index+1 && is_local_max; ++sup_row_index)
          {
            if (sup_row_index >= mem_height)
              continue;

            for (size_t sup_col_index = mem_col_index-1; sup_col_index <= mem_col_index+1; ++sup_col_index)
            {
              if (sup_col_index >= mem_width)
                continue;

              if (score_sums[mem_index] < score_sums[sup_row_index*mem_width + sup_col_index])
              {
                is_local_max = false;
                break;
              }
            } 
          }

          if (!is_local_max)
            continue;
        }

        LINEMODDetection detection;

        if (average_detections_)
        {
          size_t average_col = 0;
          size_t average_row = 0;
          size_t sum = 0;

          for (size_t sup_row_index = mem_row_index-1; sup_row_index <= mem_row_index+1; ++sup_row_index)
          {
            if (sup_row_index >= mem_height)
              continue;

            for (size_t sup_col_index = mem_col_index-1; sup_col_index <= mem_col_index+1; ++sup_col_index)
            {
              if (sup_col_index >= mem_width)
                continue;

              const size_t weight = static_cast<size_t> (score_sums[sup_row_index*mem_width + sup_col_index]);
              average_col += sup_col_index * weight;
              average_row += sup_row_index * weight;
              sum += weight;
            } 
          }

          average_col *= step_size;
          average_row *= step_size;

          average_col /= sum;
          average_row /= sum;

          //std::cerr << mem_col_index << ", " << mem_row_index << " - " << average_col << ", " << average_row << std::endl;
          std::cerr << mem_col_index*step_size << ", " << mem_row_index*step_size << " - " << average_col << ", " << average_row << std::endl;

          const size_t detection_col_index = average_col;// * step_size;
          const size_t detection_row_index = average_row;// * step_size;

          detection.x = static_cast<int> (detection_col_index);
          detection.y = static_cast<int> (detection_row_index);
        }
        else
        {
          const size_t detection_col_index = mem_col_index * step_size;
          const size_t detection_row_index = mem_row_index * step_size;

          detection.x = static_cast<int> (detection_col_index);
          detection.y = static_cast<int> (detection_row_index);
        }

        detection.template_id = static_cast<int> (template_index);
        detection.score = score;

#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
        std::cerr << "score: " << static_cast<float> (raw_score) * inv_max_score * 0.25f << ", " << (2.0f * static_cast<float> (raw_score) * inv_max_score - 1.0f) << std::endl;
        std::cerr << "score0: " << static_cast<float> (score_sums[mem_index]) * inv_max_score << ", " << (2.0f * static_cast<float> (score_sums[mem_index]) * inv_max_score - 1.0f) << std::endl;
        std::cerr << "score1: " << static_cast<float> (score_sums_1[mem_index]) * inv_max_score << ", " << (2.0f * static_cast<float> (score_sums_1[mem_index]) * inv_max_score - 1.0f) << std::endl;
        std::cerr << "score2: " << static_cast<float> (score_sums_2[mem_index]) * inv_max_score << ", " << (2.0f * static_cast<float> (score_sums_2[mem_index]) * inv_max_score - 1.0f) << std::endl;
        std::cerr << "score3: " << static_cast<float> (score_sums_3[mem_index]) * inv_max_score << ", " << (2.0f * static_cast<float> (score_sums_3[mem_index]) * inv_max_score - 1.0f) << std::endl;
#endif


        detections.push_back (detection);
      }
    }

#ifdef __SSE2__
    aligned_free (score_sums);
    aligned_free (tmp_score_sums);
#else
    delete[] score_sums;
#endif

#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    delete[] score_sums_1;
    delete[] score_sums_2;
    delete[] score_sums_3;
#endif
  }

  // release data
  for (size_t modality_index = 0; modality_index < modality_linearized_maps.size (); ++modality_index)
  {
    modality_energy_maps[modality_index].releaseAll ();
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    modality_energy_maps_1[modality_index].releaseAll ();
    modality_energy_maps_2[modality_index].releaseAll ();
    modality_energy_maps_3[modality_index].releaseAll ();
#endif
    for (size_t bin_index = 0; bin_index < modality_linearized_maps[modality_index].size (); ++bin_index)
    {
      modality_linearized_maps[modality_index][bin_index].releaseAll ();
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      modality_linearized_maps_1[modality_index][bin_index].releaseAll ();
      modality_linearized_maps_2[modality_index][bin_index].releaseAll ();
      modality_linearized_maps_3[modality_index][bin_index].releaseAll ();
#endif
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::detectTemplatesSemiScaleInvariant (
    const std::vector<QuantizableModality*> & modalities,
    std::vector<LINEMODDetection> & detections,
    const float min_scale,
    const float max_scale,
    const float scale_multiplier) const
{
  // create energy maps
  std::vector<EnergyMaps> modality_energy_maps;
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
  std::vector<EnergyMaps> modality_energy_maps_1;
  std::vector<EnergyMaps> modality_energy_maps_2;
  std::vector<EnergyMaps> modality_energy_maps_3;
#endif
  const size_t nr_modalities = modalities.size();
  for (size_t modality_index = 0; modality_index < nr_modalities; ++modality_index)
  {
    const QuantizedMap & quantized_map = modalities[modality_index]->getSpreadedQuantizedMap ();

    const size_t width = quantized_map.getWidth ();
    const size_t height = quantized_map.getHeight ();

    const unsigned char * quantized_data = quantized_map.getData ();

    const int nr_bins = 8;
    EnergyMaps energy_maps;
    energy_maps.initialize (width, height, nr_bins);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    EnergyMaps energy_maps_1;
    EnergyMaps energy_maps_2;
    EnergyMaps energy_maps_3;
    energy_maps_1.initialize (width, height, nr_bins);
    energy_maps_2.initialize (width, height, nr_bins);
    energy_maps_3.initialize (width, height, nr_bins);
#endif
    //std::vector< unsigned char* > energy_maps(nr_bins);
    for (int bin_index = 0; bin_index < nr_bins; ++bin_index)
    {
      //energy_maps[bin_index] = new unsigned char[width*height];
      //memset (energy_maps[bin_index], 0, width*height);

      const unsigned char base_bit = static_cast<unsigned char> (0x1);
      unsigned char val0 = static_cast<unsigned char> (base_bit << bin_index); // e.g. 00100000
      unsigned char val1 = static_cast<unsigned char> (val0 | (base_bit << ((bin_index+1)%8)) | (base_bit << ((bin_index+7)%8))); // e.g. 01110000
      unsigned char val2 = static_cast<unsigned char> (val1 | (base_bit << ((bin_index+2)%8)) | (base_bit << ((bin_index+6)%8))); // e.g. 11111000
      unsigned char val3 = static_cast<unsigned char> (val2 | (base_bit << ((bin_index+3)%8)) | (base_bit << ((bin_index+5)%8))); // e.g. 11111101
      for (size_t index = 0; index < width*height; ++index)
      {
        if ((val0 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
        if ((val1 & quantized_data[index]) != 0)
          ++energy_maps_1 (bin_index, index);
        if ((val2 & quantized_data[index]) != 0)
          ++energy_maps_2 (bin_index, index);
        if ((val3 & quantized_data[index]) != 0)
          ++energy_maps_3 (bin_index, index);
#else
        if ((val1 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
        if ((val2 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
        if ((val3 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
#endif
      }
    }

    modality_energy_maps.push_back (energy_maps);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    modality_energy_maps_1.push_back (energy_maps_1);
    modality_energy_maps_2.push_back (energy_maps_2);
    modality_energy_maps_3.push_back (energy_maps_3);
#endif
  }

  // create linearized maps
  const size_t step_size = 8;
  std::vector<std::vector<LinearizedMaps> > modality_linearized_maps;
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
  std::vector<std::vector<LinearizedMaps> > modality_linearized_maps_1;
  std::vector<std::vector<LinearizedMaps> > modality_linearized_maps_2;
  std::vector<std::vector<LinearizedMaps> > modality_linearized_maps_3;
#endif
  for (size_t modality_index = 0; modality_index < nr_modalities; ++modality_index)
  {
    const size_t width = modality_energy_maps[modality_index].getWidth ();
    const size_t height = modality_energy_maps[modality_index].getHeight ();

    std::vector<LinearizedMaps> linearized_maps;
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    std::vector<LinearizedMaps> linearized_maps_1;
    std::vector<LinearizedMaps> linearized_maps_2;
    std::vector<LinearizedMaps> linearized_maps_3;
#endif
    const size_t nr_bins = modality_energy_maps[modality_index].getNumOfBins ();
    for (size_t bin_index = 0; bin_index < nr_bins; ++bin_index)
    {
      unsigned char * energy_map = modality_energy_maps[modality_index] (bin_index);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      unsigned char * energy_map_1 = modality_energy_maps_1[modality_index] (bin_index);
      unsigned char * energy_map_2 = modality_energy_maps_2[modality_index] (bin_index);
      unsigned char * energy_map_3 = modality_energy_maps_3[modality_index] (bin_index);
#endif

      LinearizedMaps maps;
      maps.initialize (width, height, step_size);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      LinearizedMaps maps_1;
      LinearizedMaps maps_2;
      LinearizedMaps maps_3;
      maps_1.initialize (width, height, step_size);
      maps_2.initialize (width, height, step_size);
      maps_3.initialize (width, height, step_size);
#endif
      for (size_t map_row = 0; map_row < step_size; ++map_row)
      {
        for (size_t map_col = 0; map_col < step_size; ++map_col)
        {
          unsigned char * linearized_map = maps (map_col, map_row);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
          unsigned char * linearized_map_1 = maps_1 (map_col, map_row);
          unsigned char * linearized_map_2 = maps_2 (map_col, map_row);
          unsigned char * linearized_map_3 = maps_3 (map_col, map_row);
#endif

          // copy data from energy maps
          const size_t lin_width = width/step_size;
          const size_t lin_height = height/step_size;
          for (size_t row_index = 0; row_index < lin_height; ++row_index)
          {
            for (size_t col_index = 0; col_index < lin_width; ++col_index)
            {
              const size_t tmp_col_index = col_index*step_size + map_col;
              const size_t tmp_row_index = row_index*step_size + map_row;

              linearized_map[row_index*lin_width + col_index] = energy_map[tmp_row_index*width + tmp_col_index];
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
              linearized_map_1[row_index*lin_width + col_index] = energy_map_1[tmp_row_index*width + tmp_col_index];
              linearized_map_2[row_index*lin_width + col_index] = energy_map_2[tmp_row_index*width + tmp_col_index];
              linearized_map_3[row_index*lin_width + col_index] = energy_map_3[tmp_row_index*width + tmp_col_index];
#endif
            }
          }
        }
      }

      linearized_maps.push_back (maps);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      linearized_maps_1.push_back (maps_1);
      linearized_maps_2.push_back (maps_2);
      linearized_maps_3.push_back (maps_3);
#endif
    }

    modality_linearized_maps.push_back (linearized_maps);
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    modality_linearized_maps_1.push_back (linearized_maps_1);
    modality_linearized_maps_2.push_back (linearized_maps_2);
    modality_linearized_maps_3.push_back (linearized_maps_3);
#endif
  }

  // compute scores for templates
  const size_t width = modality_energy_maps[0].getWidth ();
  const size_t height = modality_energy_maps[0].getHeight ();
  for (size_t template_index = 0; template_index < templates_.size (); ++template_index)
  {
    const size_t mem_width = width / step_size;
    const size_t mem_height = height / step_size;
    const size_t mem_size = mem_width * mem_height;

    for (float scale = min_scale; scale <= max_scale; scale *= scale_multiplier)
    {
#ifdef __SSE2__
      unsigned short * score_sums = reinterpret_cast<unsigned short*> (aligned_malloc (mem_size*sizeof(unsigned short)));
      unsigned char * tmp_score_sums = reinterpret_cast<unsigned char*> (aligned_malloc (mem_size*sizeof(unsigned char)));
      memset (score_sums, 0, mem_size*sizeof (score_sums[0]));
      memset (tmp_score_sums, 0, mem_size*sizeof (tmp_score_sums[0]));

      //__m128i * score_sums_m128i = reinterpret_cast<__m128i*> (score_sums);
      __m128i * tmp_score_sums_m128i = reinterpret_cast<__m128i*> (tmp_score_sums);

      const size_t mem_size_16 = mem_size / 16;
      //const size_t mem_size_mod_16 = mem_size & 15;
      const size_t mem_size_mod_16_base = mem_size_16 * 16;

      int max_score = 0;
      size_t copy_back_counter = 0;
      for (size_t feature_index = 0; feature_index < templates_[template_index].features.size (); ++feature_index)
      {
        const QuantizedMultiModFeature & feature = templates_[template_index].features[feature_index];

        for (size_t bin_index = 0; bin_index < 8; ++bin_index)
        {
          if ((feature.quantized_value & (0x1<<bin_index)) != 0)
          {
            max_score += 4;

            unsigned char *data = modality_linearized_maps[feature.modality_index][bin_index].getOffsetMap (
                size_t (float (feature.x) * scale), size_t (float (feature.y) * scale));
            __m128i * data_m128i = reinterpret_cast<__m128i*> (data);

            for (size_t mem_index = 0; mem_index < mem_size_16; ++mem_index)
            {
              __m128i aligned_data_m128i = _mm_loadu_si128 (reinterpret_cast<const __m128i*> (data_m128i + mem_index)); // SSE2
              //__m128i aligned_data_m128i = _mm_lddqu_si128 (reinterpret_cast<const __m128i*> (data_m128i + mem_index)); // SSE3
              tmp_score_sums_m128i[mem_index] = _mm_add_epi8 (tmp_score_sums_m128i[mem_index], aligned_data_m128i);
            }
            for (size_t mem_index = mem_size_mod_16_base; mem_index < mem_size; ++mem_index)
            {
              tmp_score_sums[mem_index] = static_cast<unsigned char> (tmp_score_sums[mem_index] + data[mem_index]);
            }
          }
        }

        ++copy_back_counter;

        //if ((feature_index & 7) == 7)
        //if ((feature_index & 63) == 63)
        if (copy_back_counter > 63) // only valid if each feature has only one bit set..
        {
          copy_back_counter = 0;

          for (size_t mem_index = 0; mem_index < mem_size_mod_16_base; mem_index += 16)
          {
            score_sums[mem_index+0]  = static_cast<unsigned short> (score_sums[mem_index+0]  + tmp_score_sums[mem_index+0]);
            score_sums[mem_index+1]  = static_cast<unsigned short> (score_sums[mem_index+1]  + tmp_score_sums[mem_index+1]);
            score_sums[mem_index+2]  = static_cast<unsigned short> (score_sums[mem_index+2]  + tmp_score_sums[mem_index+2]);
            score_sums[mem_index+3]  = static_cast<unsigned short> (score_sums[mem_index+3]  + tmp_score_sums[mem_index+3]);
            score_sums[mem_index+4]  = static_cast<unsigned short> (score_sums[mem_index+4]  + tmp_score_sums[mem_index+4]);
            score_sums[mem_index+5]  = static_cast<unsigned short> (score_sums[mem_index+5]  + tmp_score_sums[mem_index+5]);
            score_sums[mem_index+6]  = static_cast<unsigned short> (score_sums[mem_index+6]  + tmp_score_sums[mem_index+6]);
            score_sums[mem_index+7]  = static_cast<unsigned short> (score_sums[mem_index+7]  + tmp_score_sums[mem_index+7]);
            score_sums[mem_index+8]  = static_cast<unsigned short> (score_sums[mem_index+8]  + tmp_score_sums[mem_index+8]);
            score_sums[mem_index+9]  = static_cast<unsigned short> (score_sums[mem_index+9]  + tmp_score_sums[mem_index+9]);
            score_sums[mem_index+10] = static_cast<unsigned short> (score_sums[mem_index+10] + tmp_score_sums[mem_index+10]);
            score_sums[mem_index+11] = static_cast<unsigned short> (score_sums[mem_index+11] + tmp_score_sums[mem_index+11]);
            score_sums[mem_index+12] = static_cast<unsigned short> (score_sums[mem_index+12] + tmp_score_sums[mem_index+12]);
            score_sums[mem_index+13] = static_cast<unsigned short> (score_sums[mem_index+13] + tmp_score_sums[mem_index+13]);
            score_sums[mem_index+14] = static_cast<unsigned short> (score_sums[mem_index+14] + tmp_score_sums[mem_index+14]);
            score_sums[mem_index+15] = static_cast<unsigned short> (score_sums[mem_index+15] + tmp_score_sums[mem_index+15]);
          }
          for (size_t mem_index = mem_size_mod_16_base; mem_index < mem_size; ++mem_index)
          {
            score_sums[mem_index] = static_cast<unsigned short> (score_sums[mem_index] + tmp_score_sums[mem_index]);
          }

          memset (tmp_score_sums, 0, mem_size*sizeof (tmp_score_sums[0]));
        }
      }
      {
        for (size_t mem_index = 0; mem_index < mem_size; ++mem_index)
        {
          score_sums[mem_index] = static_cast<unsigned short> (score_sums[mem_index] + tmp_score_sums[mem_index]);
        }
        
        memset (tmp_score_sums, 0, mem_size*sizeof (tmp_score_sums[0]));
      }
#else
      unsigned short * score_sums = new unsigned short[mem_size];
      //unsigned char * score_sums = new unsigned char[mem_size];
      memset (score_sums, 0, mem_size*sizeof (score_sums[0]));

#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      unsigned short * score_sums_1 = new unsigned short[mem_size];
      unsigned short * score_sums_2 = new unsigned short[mem_size];
      unsigned short * score_sums_3 = new unsigned short[mem_size];
      memset (score_sums_1, 0, mem_size*sizeof (score_sums_1[0]));
      memset (score_sums_2, 0, mem_size*sizeof (score_sums_2[0]));
      memset (score_sums_3, 0, mem_size*sizeof (score_sums_3[0]));
#endif

      int max_score = 0;
      for (size_t feature_index = 0; feature_index < templates_[template_index].features.size (); ++feature_index)
      {
        const QuantizedMultiModFeature & feature = templates_[template_index].features[feature_index];

        //feature.modality_index;
        for (size_t bin_index = 0; bin_index < 8; ++bin_index)
        {
          if ((feature.quantized_value & (0x1<<bin_index)) != 0)
          {
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
            ++max_score;

            unsigned char * data = modality_linearized_maps[feature.modality_index][bin_index].getOffsetMap (feature.x*scale, feature.y*scale);
            unsigned char * data_1 = modality_linearized_maps_1[feature.modality_index][bin_index].getOffsetMap (feature.x*scale, feature.y*scale);
            unsigned char * data_2 = modality_linearized_maps_2[feature.modality_index][bin_index].getOffsetMap (feature.x*scale, feature.y*scale);
            unsigned char * data_3 = modality_linearized_maps_3[feature.modality_index][bin_index].getOffsetMap (feature.x*scale, feature.y*scale);
            for (size_t mem_index = 0; mem_index < mem_size; ++mem_index)
            {
              score_sums[mem_index] += data[mem_index];
              score_sums_1[mem_index] += data_1[mem_index];
              score_sums_2[mem_index] += data_2[mem_index];
              score_sums_3[mem_index] += data_3[mem_index];
            }
#else
            max_score += 4;

            unsigned char * data = modality_linearized_maps[feature.modality_index][bin_index].getOffsetMap (static_cast<size_t> (feature.x*scale), static_cast<size_t> (feature.y*scale));
            for (size_t mem_index = 0; mem_index < mem_size; ++mem_index)
            {
              score_sums[mem_index] += data[mem_index];
            }
#endif
          }
        }
      }
#endif

      const float inv_max_score = 1.0f / float (max_score);

      // we compute a new threshold based on the threshold supplied by the user;
      // this is due to the use of the cosine approx. in the response computation;
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      const float raw_threshold = (4.0f * float (max_score) / 2.0f + template_threshold_ * (4.0f * float (max_score) / 2.0f));
#else
      const float raw_threshold = (float (max_score) / 2.0f + template_threshold_ * (float (max_score) / 2.0f));
#endif

      //int max_value = 0;
      //size_t max_index = 0;
      for (size_t mem_index = 0; mem_index < mem_size; ++mem_index)
      {
        //const float score = score_sums[mem_index] * inv_max_score;

#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
        const float raw_score = score_sums[mem_index] 
          + score_sums_1[mem_index]
          + score_sums_2[mem_index]
          + score_sums_3[mem_index];

        const float score = 2.0f * static_cast<float> (raw_score) * 0.25f * inv_max_score - 1.0f;
#else
        const float raw_score = score_sums[mem_index];

        const float score = 2.0f * static_cast<float> (raw_score) * inv_max_score - 1.0f;
#endif


        //if (score > template_threshold_) 
        if (raw_score > raw_threshold) /// \todo Ask Stefan why this line was used instead of the one above
        {
          const size_t mem_col_index = (mem_index % mem_width);
          const size_t mem_row_index = (mem_index / mem_width);

          if (use_non_max_suppression_)
          {
            bool is_local_max = true;
            for (size_t sup_row_index = mem_row_index-1; sup_row_index <= mem_row_index+1 && is_local_max; ++sup_row_index)
            {
              if (sup_row_index >= mem_height)
                continue;

              for (size_t sup_col_index = mem_col_index-1; sup_col_index <= mem_col_index+1; ++sup_col_index)
              {
                if (sup_col_index >= mem_width)
                  continue;

                if (score_sums[mem_index] < score_sums[sup_row_index*mem_width + sup_col_index])
                {
                  is_local_max = false;
                  break;
                }
              } 
            }

            if (!is_local_max)
              continue;
          }

          LINEMODDetection detection;

          if (average_detections_)
          {
            size_t average_col = 0;
            size_t average_row = 0;
            size_t sum = 0;

            for (size_t sup_row_index = mem_row_index-1; sup_row_index <= mem_row_index+1; ++sup_row_index)
            {
              if (sup_row_index >= mem_height)
                continue;

              for (size_t sup_col_index = mem_col_index-1; sup_col_index <= mem_col_index+1; ++sup_col_index)
              {
                if (sup_col_index >= mem_width)
                  continue;

                const size_t weight = static_cast<size_t> (score_sums[sup_row_index*mem_width + sup_col_index]);
                average_col += sup_col_index * weight;
                average_row += sup_row_index * weight;
                sum += weight;
              } 
            }

            average_col *= step_size;
            average_row *= step_size;

            average_col /= sum;
            average_row /= sum;

            //std::cerr << mem_col_index << ", " << mem_row_index << " - " << average_col << ", " << average_row << std::endl;
            std::cerr << mem_col_index*step_size << ", " << mem_row_index*step_size << " - " << average_col << ", " << average_row << std::endl;

            const size_t detection_col_index = average_col;// * step_size;
            const size_t detection_row_index = average_row;// * step_size;

            detection.x = static_cast<int> (detection_col_index);
            detection.y = static_cast<int> (detection_row_index);
          }
          else
          {
            const size_t detection_col_index = mem_col_index * step_size;
            const size_t detection_row_index = mem_row_index * step_size;

            detection.x = static_cast<int> (detection_col_index);
            detection.y = static_cast<int> (detection_row_index);
          }

          detection.template_id = static_cast<int> (template_index);
          detection.score = score;
          detection.scale = scale;

#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
          std::cerr << "score: " << static_cast<float> (raw_score) * inv_max_score * 0.25f << ", " << (2.0f * static_cast<float> (raw_score) * inv_max_score - 1.0f) << std::endl;
          std::cerr << "score0: " << static_cast<float> (score_sums[mem_index]) * inv_max_score << ", " << (2.0f * static_cast<float> (score_sums[mem_index]) * inv_max_score - 1.0f) << std::endl;
          std::cerr << "score1: " << static_cast<float> (score_sums_1[mem_index]) * inv_max_score << ", " << (2.0f * static_cast<float> (score_sums_1[mem_index]) * inv_max_score - 1.0f) << std::endl;
          std::cerr << "score2: " << static_cast<float> (score_sums_2[mem_index]) * inv_max_score << ", " << (2.0f * static_cast<float> (score_sums_2[mem_index]) * inv_max_score - 1.0f) << std::endl;
          std::cerr << "score3: " << static_cast<float> (score_sums_3[mem_index]) * inv_max_score << ", " << (2.0f * static_cast<float> (score_sums_3[mem_index]) * inv_max_score - 1.0f) << std::endl;
#endif


          detections.push_back (detection);
        }
      }

#ifdef __SSE2__
      aligned_free (score_sums);
      aligned_free (tmp_score_sums);
#else
      delete[] score_sums;
#endif

#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      delete[] score_sums_1;
      delete[] score_sums_2;
      delete[] score_sums_3;
#endif
    }
  }

  // release data
  for (size_t modality_index = 0; modality_index < modality_linearized_maps.size (); ++modality_index)
  {
    modality_energy_maps[modality_index].releaseAll ();
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    modality_energy_maps_1[modality_index].releaseAll ();
    modality_energy_maps_2[modality_index].releaseAll ();
    modality_energy_maps_3[modality_index].releaseAll ();
#endif
    for (size_t bin_index = 0; bin_index < modality_linearized_maps[modality_index].size (); ++bin_index)
    {
      modality_linearized_maps[modality_index][bin_index].releaseAll ();
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      modality_linearized_maps_1[modality_index][bin_index].releaseAll ();
      modality_linearized_maps_2[modality_index][bin_index].releaseAll ();
      modality_linearized_maps_3[modality_index][bin_index].releaseAll ();
#endif
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::saveTemplates (const char * file_name) const
{
  std::ofstream file_stream;
  file_stream.open (file_name, std::ofstream::out | std::ofstream::binary);

  serialize (file_stream);

  file_stream.close ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::loadTemplates (const char * file_name)
{
  std::ifstream file_stream;
  file_stream.open (file_name, std::ofstream::in | std::ofstream::binary);

  deserialize (file_stream);

  file_stream.close ();
}

void
pcl::LINEMOD::loadTemplates (std::vector<std::string> & file_names)
{
  templates_.clear ();

  for(size_t i=0; i < file_names.size (); i++)
  {
    std::ifstream file_stream;
    file_stream.open (file_names[i].c_str (), std::ofstream::in | std::ofstream::binary);

    int nr_templates;
    read (file_stream, nr_templates);
    SparseQuantizedMultiModTemplate sqmm_template;

    for (int template_index = 0; template_index < nr_templates; ++template_index)
    {
      sqmm_template.deserialize (file_stream);
      templates_.push_back (sqmm_template);
    }

    file_stream.close ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::serialize (std::ostream & stream) const
{
  const int nr_templates = static_cast<int> (templates_.size ());
  write (stream, nr_templates);
  for (int template_index = 0; template_index < nr_templates; ++template_index)
    templates_[template_index].serialize (stream);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::LINEMOD::deserialize (std::istream & stream)
{
  templates_.clear ();

  int nr_templates;
  read (stream, nr_templates);
  templates_.resize (nr_templates);
  for (int template_index = 0; template_index < nr_templates; ++template_index)
    templates_[template_index].deserialize (stream);
}
