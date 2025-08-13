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

#include <pcl/console/print.h>
#include <pcl/recognition/linemod.h>

#if __AVX2__
#include <immintrin.h>
#endif
#ifdef __SSE2__
#include <emmintrin.h>
#endif

#include <fstream>
#include <map>
#include <algorithm>

//#define LINEMOD_USE_SEPARATE_ENERGY_MAPS

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::LINEMOD::LINEMOD () 
  : template_threshold_ (0.75f)
  , translation_clustering_threshold_ (0)  // 0 for disabled
  , rotation_clustering_threshold_ (0)  // 0 for disabled
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
                      const pcl::RegionXY & region,
                      size_t nr_features_per_modality)
{
  // assuming width and height is same for all modalities; should we check this??
  //const int width = modalities[0]->getQuantizedMap().getWidth ();
  //const int height = modalities[0]->getQuantizedMap().getHeight ();

  SparseQuantizedMultiModTemplate linemod_template;

  createTemplate(modalities, masks, region, linemod_template, nr_features_per_modality);

  // add template to template storage
  templates_.push_back(linemod_template);

  return static_cast<int> (templates_.size () - 1);
}

void
pcl::LINEMOD::createTemplate (const std::vector<QuantizableModality*> & modalities,
                              const std::vector<MaskMap*> & masks,
                              const RegionXY & region,
                              SparseQuantizedMultiModTemplate & linemod_template,
                              size_t nr_features_per_modality) const
{
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
pcl::LINEMOD::groupAndSortOverlappingDetections (
    const std::vector<LINEMODDetection> & detections,
    std::vector<std::vector<LINEMODDetection>>& grouped_detections,
    const size_t grouping_threshold) const
{
  typedef std::tuple<size_t, size_t> GroupingKey;
  std::map<GroupingKey, std::vector<LINEMODDetection>> groups;
  const size_t nr_detections = detections.size ();
  // compute overlap between each detection
  for (size_t detection_id = 0; detection_id < nr_detections; ++detection_id)
  {
    const GroupingKey key = {
      detections[detection_id].x / grouping_threshold,
      detections[detection_id].y / grouping_threshold,
    };

    groups[key].push_back(detections[detection_id]);
  }
  grouped_detections.resize(groups.size());

  size_t group_id;
  std::map<GroupingKey, std::vector<LINEMODDetection>>::iterator it;
  for (group_id = 0, it = groups.begin(); it != groups.end(); ++group_id, ++it)
  {
    std::vector<LINEMODDetection>& detections_group = it->second;
    sortDetections(detections_group);
    grouped_detections[group_id] = detections_group;
  }
  std::sort(grouped_detections.begin(), grouped_detections.end(), [](const std::vector<LINEMODDetection> & a,
                                                                     const std::vector<LINEMODDetection> & b) -> bool { return a[0].score > b[0].score; });
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::removeOverlappingDetections (
    std::vector<LINEMODDetection> & detections,
    size_t translation_clustering_threshold,
    float rotation_clustering_threshold) const
{
  // check if clustering is disabled
  if (translation_clustering_threshold == 0 && rotation_clustering_threshold == 0.f) {
    return;
  }

  if (translation_clustering_threshold == 0) {
    translation_clustering_threshold = 1;
  }
  if (rotation_clustering_threshold == 0.f) {
    rotation_clustering_threshold = std::numeric_limits<float>::epsilon();
  }

  typedef std::tuple<int, int, int> TemplatesClusteringKey;
  std::map<TemplatesClusteringKey, std::vector<size_t>> templateClusters;
  const size_t n_templates = templates_.size ();
  for (size_t template_index = 0; template_index < n_templates; ++template_index)
  {
    const pcl::SparseQuantizedMultiModTemplate& template_ = templates_[template_index];
    const TemplatesClusteringKey key = {
      static_cast<int>(template_.rx / rotation_clustering_threshold),
      static_cast<int>(template_.ry / rotation_clustering_threshold),
      static_cast<int>(template_.rz / rotation_clustering_threshold),
    };

    templateClusters[key].push_back(template_index);
  }

  std::vector<size_t> clusteredTemplates(n_templates, std::numeric_limits<size_t>::max()); // index is template_index
  if (templateClusters.size() <= 1) {
    PCL_ERROR ("[removeOverlappingDetections] All %u templates got grouped into %u cluster(s). Either rotation threshold=%.4f is too large, or template rotations are not set properly.\n"
               "Making each template in its own cluster for now...\n", n_templates, templateClusters.size(), rotation_clustering_threshold);
    for (size_t template_index = 0; template_index < n_templates; ++template_index)
    {
      clusteredTemplates[template_index] = template_index;
    }
  }
  else
  {
    PCL_INFO ("[removeOverlappingDetections] Grouping %u templates into %u clusters\n", n_templates, templateClusters.size());
    size_t cluster_id;
    std::map<TemplatesClusteringKey, std::vector<size_t>>::iterator tempIt;
    for (cluster_id = 0, tempIt = templateClusters.begin(); tempIt != templateClusters.end(); ++cluster_id, ++tempIt)
    {
      const std::vector<size_t>& cluster = tempIt->second;
      const size_t elements_in_cluster = cluster.size ();
      for (size_t cluster_index = 0; cluster_index < elements_in_cluster; ++cluster_index)
      {
        const size_t template_index = cluster[cluster_index];
        clusteredTemplates[template_index] = cluster_id;
      }
    }
  }

  // compute overlap between each detection
  const size_t nr_detections = detections.size ();

  typedef std::tuple<size_t, size_t, size_t> ClusteringKey;
  std::map<ClusteringKey, std::vector<size_t>> clusters;
  for (size_t detection_id = 0; detection_id < nr_detections; ++detection_id)
  {
    const LINEMODDetection& d = detections[detection_id];
    const ClusteringKey key = {
      d.x / translation_clustering_threshold,
      d.y / translation_clustering_threshold,
      clusteredTemplates[d.template_id],
    };

    clusters[key].push_back(detection_id);
  }

  // compute detection representatives for every cluster
  std::vector<LINEMODDetection> clustered_detections;
  size_t cluster_id;
  std::map<ClusteringKey, std::vector<size_t>>::iterator it;
  for (cluster_id = 0, it = clusters.begin(); it != clusters.end(); ++cluster_id, ++it)
  {
    const std::vector<size_t>& cluster = it->second;
    float weight_sum = 0.0f;

    float average_score = 0.0f;
    float average_scale = 0.0f;
    float average_rx = 0.0f;
    float average_ry = 0.0f;
    float average_rz = 0.0f;
    float average_region_x = 0.0f;
    float average_region_y = 0.0f;

    const size_t elements_in_cluster = cluster.size ();
    for (size_t cluster_index = 0; cluster_index < elements_in_cluster; ++cluster_index)
    {
      const size_t detection_id = cluster[cluster_index];
      const LINEMODDetection& d = detections[detection_id];
      const pcl::SparseQuantizedMultiModTemplate& template_ = templates_[d.template_id];

      const float weight = d.score * d.score;

      weight_sum += weight;

      average_score += d.score * weight;
      average_scale += d.scale * weight;
      average_rx += template_.rx * weight;
      average_ry += template_.ry * weight;
      average_rz += template_.rz * weight;
      average_region_x += static_cast<float>(d.x) * weight;
      average_region_y += static_cast<float>(d.y) * weight;
    }

    const float inv_weight_sum = 1.0f / weight_sum;

    average_rx *= inv_weight_sum;
    average_ry *= inv_weight_sum;
    average_rz *= inv_weight_sum;

    float min_dist2 = std::numeric_limits<float>::max ();
    size_t best_template_id = detections[cluster[0]].template_id;
    for (size_t template_index = 0; template_index < n_templates; ++template_index)
    {
      // Skip templates that does not belong to the same cluster
      // This is also important to protect wrong ID assignment in case all rotations are not set, thus ended up to have the same distance
      if (clusteredTemplates[best_template_id] != clusteredTemplates[template_index]) {
        continue;
      }

      const pcl::SparseQuantizedMultiModTemplate& template_ = templates_[template_index];
      const float dist2 = std::pow(template_.rx - average_rx, 2) + std::pow(template_.ry - average_ry, 2) + std::pow(template_.rz - average_rz, 2);
      if (dist2 < min_dist2)
      {
        min_dist2 = dist2;
        best_template_id = template_index;
      }
    }

    LINEMODDetection detection;
    detection.template_id = best_template_id;
    detection.score = average_score * inv_weight_sum * std::exp(-0.5f / elements_in_cluster);
    detection.scale = average_scale * inv_weight_sum;
    detection.x = int (average_region_x * inv_weight_sum);
    detection.y = int (average_region_y * inv_weight_sum);

    clustered_detections.push_back (detection);
  }
  detections = clustered_detections;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::sortDetections (
    std::vector<LINEMODDetection> & detections) const
{
  std::sort(detections.begin(), detections.end(), [](const LINEMODDetection & a,
                                                     const LINEMODDetection & b) -> bool { return a.score > b.score; });
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

#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
    unsigned short * score_sums_1 = new unsigned short[mem_size];
    unsigned short * score_sums_2 = new unsigned short[mem_size];
    unsigned short * score_sums_3 = new unsigned short[mem_size];
    memset (score_sums_1, 0, mem_size*sizeof (score_sums_1[0]));
    memset (score_sums_2, 0, mem_size*sizeof (score_sums_2[0]));
    memset (score_sums_3, 0, mem_size*sizeof (score_sums_3[0]));
#endif

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
#else  // #ifdef __SSE2__
    unsigned short * score_sums = new unsigned short[mem_size];
    //unsigned char * score_sums = new unsigned char[mem_size];
    memset (score_sums, 0, mem_size*sizeof (score_sums[0]));

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
#endif  // #ifdef __SSE2__

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

#include <time.h>
static size_t getTickCount() {
struct timespec tp;
clock_gettime(CLOCK_MONOTONIC, &tp);
return (size_t)tp.tv_sec*1000000000 + tp.tv_nsec;
}
//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::detectTemplatesSemiScaleInvariant (
    const std::vector<QuantizableModality*> & modalities,
    std::vector<LINEMODDetection> & detections,
    const float min_scale,
    const float max_scale,
    const float scale_multiplier,
    const float importanceOfDepthModality) const
{
  // create energy maps
  std::vector<EnergyMaps> modality_energy_maps;
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
  std::vector<EnergyMaps> modality_energy_maps_1;
  std::vector<EnergyMaps> modality_energy_maps_2;
  std::vector<EnergyMaps> modality_energy_maps_3;
#endif
  double start = getTickCount();

  const size_t nr_modalities = modalities.size();
  modality_energy_maps.reserve(nr_modalities);
  const float weightMoveToDepthModality = importanceOfDepthModality;
  const size_t indexModalityDepth = 1;
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
    //#pragma omp parallel for
    for (int bin_index = 0; bin_index < nr_bins; ++bin_index)
    {
      //energy_maps[bin_index] = new unsigned char[width*height];
      //memset (energy_maps[bin_index], 0, width*height);

      static const unsigned char base_bit = static_cast<unsigned char> (0x1);
      const unsigned char val0 = static_cast<unsigned char> (base_bit << bin_index); // e.g. 00100000
      const unsigned char val1 = static_cast<unsigned char> (val0 | (base_bit << ((bin_index+1)%8)) | (base_bit << ((bin_index+7)%8))); // e.g. 01110000
      const unsigned char val2 = static_cast<unsigned char> (val1 | (base_bit << ((bin_index+2)%8)) | (base_bit << ((bin_index+6)%8))); // e.g. 11111000
      const unsigned char val3 = static_cast<unsigned char> (val2 | (base_bit << ((bin_index+3)%8)) | (base_bit << ((bin_index+5)%8))); // e.g. 11111101
      unsigned char *energy_map_bin = energy_maps (bin_index);

      size_t index = 0;
#if defined(__AVX2__) && !defined(LINEMOD_USE_SEPARATE_ENERGY_MAPS)
      const __m256i __val0 = _mm256_set1_epi8(val0);
      const __m256i __val1 = _mm256_set1_epi8(val1);
      const __m256i __val2 = _mm256_set1_epi8(val2);
      const __m256i __val3 = _mm256_set1_epi8(val3);
      const __m256i __one  = _mm256_set1_epi8(1);
      const __m256i __zero  = _mm256_set1_epi8(0);
      for (; index <= width*height - 32; index += 32)
      {
        const __m256i __quantized_data = _mm256_loadu_si256((const __m256i*)&quantized_data[index]);

        const __m256i __sum =
          _mm256_add_epi8(_mm256_add_epi8(_mm256_add_epi8(
            _mm256_andnot_si256(_mm256_cmpeq_epi8(_mm256_and_si256(__val0, __quantized_data), __zero), __one),
            _mm256_andnot_si256(_mm256_cmpeq_epi8(_mm256_and_si256(__val1, __quantized_data), __zero), __one)),
            _mm256_andnot_si256(_mm256_cmpeq_epi8(_mm256_and_si256(__val2, __quantized_data), __zero), __one)),
            _mm256_andnot_si256(_mm256_cmpeq_epi8(_mm256_and_si256(__val3, __quantized_data), __zero), __one));

        _mm256_store_si256((__m256i*) &energy_map_bin[index], __sum);
      }
#endif
      for (; index < width*height; ++index)
      {
#ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
        if ((val0 & quantized_data[index]) != 0)
          ++energy_map_bin[index];
        if ((val1 & quantized_data[index]) != 0)
          ++energy_maps_1 (bin_index, index);
        if ((val2 & quantized_data[index]) != 0)
          ++energy_maps_2 (bin_index, index);
        if ((val3 & quantized_data[index]) != 0)
          ++energy_maps_3 (bin_index, index);
#else
        const uint8_t byte = quantized_data[index];
        energy_map_bin[index] = ((val0 & byte) != 0) +
                                ((val1 & byte) != 0) +
                                ((val2 & byte) != 0) +
                                ((val3 & byte) != 0);
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

  // printf("1 %f\n", 1000.0*(getTickCount()-start)/1e9);
  start = getTickCount();
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
    float weightOnThisModality = (modality_index == indexModalityDepth)?(weightMoveToDepthModality + 1.0f):(1.0f - weightMoveToDepthModality);
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
            const size_t tmp_row_index = row_index*step_size + map_row;
            for (size_t col_index = 0; col_index < lin_width; ++col_index)
            {
              const size_t tmp_col_index = col_index*step_size + map_col;
              
              unsigned char energy = energy_map[tmp_row_index*width + tmp_col_index];
              float energyWeighted = static_cast<float>(energy) * weightOnThisModality;
              linearized_map[row_index*lin_width + col_index] = static_cast<unsigned char>(energyWeighted);
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

  // printf("2 %f\n", 1000.0*(getTickCount()-start)/1e9);
  start = getTickCount();

  // compute scores for templates
  const size_t width = modality_energy_maps[0].getWidth ();
  const size_t height = modality_energy_maps[0].getHeight ();
  const size_t mem_width = width / step_size;
  const size_t mem_height = height / step_size;
  const size_t mem_size = mem_width * mem_height;

  #pragma omp parallel
  {
    // Thread local storage
    #ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
      unsigned short * score_sums_1 = new unsigned short[mem_size];
      unsigned short * score_sums_2 = new unsigned short[mem_size];
      unsigned short * score_sums_3 = new unsigned short[mem_size];
    #endif

    #if defined(__AVX2__)
      unsigned short * score_sums;
      unsigned char * tmp_score_sums;
      posix_memalign((void**) &score_sums, 32, mem_size*sizeof(unsigned short));
      posix_memalign((void**) &tmp_score_sums, 32, mem_size*sizeof(unsigned char));
    #elif defined(__SSE2__)
      unsigned short * score_sums = reinterpret_cast<unsigned short*> (aligned_malloc (mem_size*sizeof(unsigned short)));
      unsigned char * tmp_score_sums = reinterpret_cast<unsigned char*> (aligned_malloc (mem_size*sizeof(unsigned char)));
    #else
      unsigned short * score_sums = new unsigned short[mem_size];
    #endif

    #pragma omp for
    for (size_t template_index = 0; template_index < templates_.size (); ++template_index)
    {
      for (float scale = min_scale; scale <= max_scale; scale *= scale_multiplier)
      {
  #ifdef __AVX2__
        memset (score_sums, 0, mem_size*sizeof (score_sums[0]));
        memset (tmp_score_sums, 0, mem_size*sizeof (tmp_score_sums[0]));

        int max_score = 0;
        size_t copy_back_counter = 0;
        for (size_t feature_index = 0; feature_index < templates_[template_index].features.size (); ++feature_index)
        {
          const QuantizedMultiModFeature & feature = templates_[template_index].features[feature_index];

          std::vector<LinearizedMaps> &linearized_map_modality = modality_linearized_maps[feature.modality_index];
          const size_t map_x = size_t (float (feature.x) * scale);
          const size_t map_y = size_t (float (feature.y) * scale);

          for (size_t bin_index = 0; bin_index < 8; ++bin_index)
          {
            if ((feature.quantized_value & (0x1<<bin_index)) != 0)
            {
              max_score += 4;

              const unsigned char *data = linearized_map_modality[bin_index].getOffsetMap(map_x, map_y);

              size_t mem_index = 0;
              for (; mem_index <= mem_size - 32; mem_index+=32)
              {
                __m256i __tmp_score_sums = _mm256_load_si256(reinterpret_cast<const __m256i*>(tmp_score_sums + mem_index));
                __tmp_score_sums = _mm256_add_epi8(__tmp_score_sums, _mm256_loadu_si256(reinterpret_cast<const __m256i*>(data + mem_index)));
                _mm256_store_si256(reinterpret_cast<__m256i*>(tmp_score_sums + mem_index), __tmp_score_sums);
              }

              for (; mem_index < mem_size; ++mem_index)
              {
                tmp_score_sums[mem_index] = static_cast<unsigned char> (tmp_score_sums[mem_index] + data[mem_index]);
              }
            }
          }

          ++copy_back_counter;

          if (copy_back_counter > 31) // only valid if each feature has only one bit set..
          {
            copy_back_counter = 0;

            size_t mem_index = 0;
            for (; mem_index <= mem_size - 32; mem_index+=32)
            {
              __m256i __tmp_score_sums = _mm256_load_si256(reinterpret_cast<const __m256i*>(tmp_score_sums + mem_index));
              // First half
              __m256i __score_sums = _mm256_load_si256(reinterpret_cast<const __m256i*>(score_sums + mem_index));
              __score_sums = _mm256_add_epi16(__score_sums, _mm256_cvtepu8_epi16(_mm256_extractf128_si256(__tmp_score_sums, 0)));
              _mm256_store_si256(reinterpret_cast<__m256i*>(score_sums + mem_index), __score_sums);
              // Second half
              __score_sums = _mm256_load_si256(reinterpret_cast<const __m256i*>(score_sums + mem_index + 16));
              __score_sums = _mm256_add_epi16(__score_sums, _mm256_cvtepu8_epi16(_mm256_extractf128_si256(__tmp_score_sums, 1)));
              _mm256_store_si256(reinterpret_cast<__m256i*>(score_sums + mem_index + 16), __score_sums);
            }
            for (; mem_index < mem_size; ++mem_index)
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

  #elif defined(__SSE2__)
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
  #else // NO AVX2 and SSE
        memset (score_sums, 0, mem_size*sizeof (score_sums[0]));

  #ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
        memset (score_sums_1, 0, mem_size*sizeof (score_sums_1[0]));
        memset (score_sums_2, 0, mem_size*sizeof (score_sums_2[0]));
        memset (score_sums_3, 0, mem_size*sizeof (score_sums_3[0]));
  #endif

        int max_score = 0;
        for (size_t feature_index = 0; feature_index < templates_[template_index].features.size (); ++feature_index)
        {
          const QuantizedMultiModFeature & feature = templates_[template_index].features[feature_index];

          std::vector<LinearizedMaps> &linearized_map_modality = modality_linearized_maps[feature.modality_index];
  #ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
          std::vector<LinearizedMaps> &linearized_map_modality1 = modality_linearized_maps_1[feature.modality_index];
          std::vector<LinearizedMaps> &linearized_map_modality2 = modality_linearized_maps_2[feature.modality_index];
          std::vector<LinearizedMaps> &linearized_map_modality3 = modality_linearized_maps_3[feature.modality_index];
  #endif
          const size_t map_x = size_t (float (feature.x) * scale);
          const size_t map_y = size_t (float (feature.y) * scale);

          //feature.modality_index;
          for (size_t bin_index = 0; bin_index < 8; ++bin_index)
          {
            if ((feature.quantized_value & (0x1<<bin_index)) != 0)
            {
  #ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
              ++max_score;

              unsigned char * data = linearized_map_modality[bin_index].getOffsetMap(map_x, map_y);
              unsigned char * data_1 = linearized_map_modality1[bin_index].getOffsetMap(map_x, map_y);
              unsigned char * data_2 = linearized_map_modality2[bin_index].getOffsetMap(map_x, map_y);
              unsigned char * data_3 = linearized_map_modality3[bin_index].getOffsetMap(map_x, map_y);
              for (size_t mem_index = 0; mem_index < mem_size; ++mem_index)
              {
                score_sums[mem_index] += data[mem_index];
                score_sums_1[mem_index] += data_1[mem_index];
                score_sums_2[mem_index] += data_2[mem_index];
                score_sums_3[mem_index] += data_3[mem_index];
              }
  #else
              max_score += 4;

              unsigned char * data = linearized_map_modality[bin_index].getOffsetMap(map_x, map_y);
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
          // const float score = 2.0f * static_cast<float> (raw_score) * 0.25f * inv_max_score - 1.0f;
  #else
          const float raw_score = score_sums[mem_index];
          // const float score = 2.0f * static_cast<float> (raw_score) * inv_max_score - 1.0f;  // Note: switch to use raw_score_rebased
  #endif
          float raw_score_rebased = (raw_score - (float (max_score) / 2.0f)) / (float (max_score) / 2.0f);


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
            detection.score = raw_score_rebased;//score;
            detection.scale = scale;

  #ifdef LINEMOD_USE_SEPARATE_ENERGY_MAPS
            std::cerr << "score: " << static_cast<float> (raw_score) * inv_max_score * 0.25f << ", " << (2.0f * static_cast<float> (raw_score) * inv_max_score - 1.0f) << std::endl;
            std::cerr << "score0: " << static_cast<float> (score_sums[mem_index]) * inv_max_score << ", " << (2.0f * static_cast<float> (score_sums[mem_index]) * inv_max_score - 1.0f) << std::endl;
            std::cerr << "score1: " << static_cast<float> (score_sums_1[mem_index]) * inv_max_score << ", " << (2.0f * static_cast<float> (score_sums_1[mem_index]) * inv_max_score - 1.0f) << std::endl;
            std::cerr << "score2: " << static_cast<float> (score_sums_2[mem_index]) * inv_max_score << ", " << (2.0f * static_cast<float> (score_sums_2[mem_index]) * inv_max_score - 1.0f) << std::endl;
            std::cerr << "score3: " << static_cast<float> (score_sums_3[mem_index]) * inv_max_score << ", " << (2.0f * static_cast<float> (score_sums_3[mem_index]) * inv_max_score - 1.0f) << std::endl;
  #endif

            #pragma omp critical
            {
              detections.push_back (detection);
            }
          }
        }
      }
    }
    // Free thread local storage
    #if defined(__AVX2__) || defined(__SSE2__)
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
  } // #pragma omp parallel

  // printf("3 %f\n", 1000.0*(getTickCount()-start)/1e9);
  start = getTickCount();

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
pcl::LINEMOD::evaluateDetections(
  const std::vector<QuantizableModality*>& modalities,
  const std::vector<LINEMODDetection>& inputDetections,
  const std::vector<SparseQuantizedMultiModTemplate>& inputTemplates,
  const float importanceOfDepthModality,
  std::vector<float>& evaluationScores
) const
{
  // create energy maps
  std::vector<EnergyMaps> modality_energy_maps;
  const size_t nr_modalities = modalities.size();
  modality_energy_maps.reserve(nr_modalities);
  const float weightMoveToDepthModality = importanceOfDepthModality;
  const size_t indexModalityDepth = 1;
  for (size_t modality_index = 0; modality_index < nr_modalities; ++modality_index)
  {
    const QuantizedMap & quantized_map = modalities[modality_index]->getSpreadedQuantizedMap ();

    const size_t width = quantized_map.getWidth ();
    const size_t height = quantized_map.getHeight ();
    const unsigned char * quantized_data = quantized_map.getData ();

    const int nr_bins = 8;
    EnergyMaps energy_maps;
    energy_maps.initialize (width, height, nr_bins);

    #pragma omp parallel for
    for (int bin_index = 0; bin_index < nr_bins; ++bin_index)
    {
      static const unsigned char base_bit = static_cast<unsigned char> (0x1);
      const unsigned char val0 = static_cast<unsigned char> (base_bit << bin_index); // e.g. 00100000
      const unsigned char val1 = static_cast<unsigned char> (val0 | (base_bit << ((bin_index+1)%8)) | (base_bit << ((bin_index+7)%8))); // e.g. 01110000
      const unsigned char val2 = static_cast<unsigned char> (val1 | (base_bit << ((bin_index+2)%8)) | (base_bit << ((bin_index+6)%8))); // e.g. 11111000
      const unsigned char val3 = static_cast<unsigned char> (val2 | (base_bit << ((bin_index+3)%8)) | (base_bit << ((bin_index+5)%8))); // e.g. 11111101
      unsigned char *energy_map_bin = energy_maps (bin_index);

      size_t index = 0;
      for (; index < width*height; ++index)
      {
        const uint8_t byte = quantized_data[index];
        energy_map_bin[index] = ((val0 & byte) != 0) +
                                ((val1 & byte) != 0) +
                                ((val2 & byte) != 0) +
                                ((val3 & byte) != 0);
      }
    }

    modality_energy_maps.push_back (energy_maps);
  }

  // compute scores for templates
  const size_t width = modality_energy_maps[0].getWidth ();
  const size_t height = modality_energy_maps[0].getHeight ();

  std::vector<size_t> indicesOfDetections(inputDetections.size(), 0);
  std::vector<float> scoresTemp(inputDetections.size(), 0.0);
  evaluationScores.resize(inputDetections.size(), 0.0);
  #pragma omp for
  for (size_t detection_index = 0; detection_index < inputDetections.size(); ++detection_index)
  {
    const LINEMODDetection& detection = inputDetections[detection_index];
    const SparseQuantizedMultiModTemplate& inputTemplate = inputTemplates[detection_index];
    int max_score = 0;
    int raw_score = 0;
    for (size_t feature_index = 0; feature_index < inputTemplate.features.size (); ++feature_index)
    {
      const QuantizedMultiModFeature & feature = inputTemplate.features[feature_index];
      float weightOnThisModality = (feature.modality_index == indexModalityDepth)?(weightMoveToDepthModality + 1.0f):(1.0f - weightMoveToDepthModality);

      EnergyMaps& energymap = modality_energy_maps[feature.modality_index];
      const size_t map_x = size_t (float (feature.x * detection.scale + detection.x));
      const size_t map_y = size_t (float (feature.y * detection.scale + detection.y));

      //feature.modality_index;
      for (size_t bin_index = 0; bin_index < 8; ++bin_index)
      {
        if ((feature.quantized_value & (0x1<<bin_index)) != 0)
        {
          max_score += 4;

          unsigned char energy = energymap(bin_index)[map_y * width + map_x];
          float energyWeighted = static_cast<float> (energy) * weightOnThisModality;
          raw_score += static_cast<int> (energyWeighted);
        }
      }
    }
    float raw_score_rebased = ((float)raw_score - (float (max_score) / 2.0f)) / (float (max_score) / 2.0f);
    // std::cout << "raw_score_rebased: " << raw_score_rebased << ", max_score: " << max_score << std::endl;

    #pragma omp critical
    {
      scoresTemp.push_back(raw_score_rebased);
      indicesOfDetections.push_back(detection_index);
    }
  }

  // Note: sort scores as the same order as inputDetections
  for (size_t indexTemp = 0; indexTemp < indicesOfDetections.size(); indexTemp++)
  {
    evaluationScores[indicesOfDetections[indexTemp]] = scoresTemp[indexTemp];
  }

  // release data
  for (size_t modality_index = 0; modality_index < modality_energy_maps.size (); ++modality_index)
  {
    modality_energy_maps[modality_index].releaseAll ();
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
