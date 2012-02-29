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

#include "pcl/recognition/linemod.h"

#include <fstream>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::LINEMOD::
LINEMOD() :
  templates_ ()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::LINEMOD::
~LINEMOD()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
int 
pcl::LINEMOD::
createAndAddTemplate (const std::vector<pcl::QuantizableModality*> & modalities,
                      const std::vector<pcl::MaskMap*> & masks,
                      const pcl::RegionXY & region)
{
  // assuming width and height is same for all modalities; should we check this??
  //const int width = modalities[0]->getQuantizedMap().getWidth ();
  //const int height = modalities[0]->getQuantizedMap().getHeight ();

  SparseQuantizedMultiModTemplate linemod_template;

  // select N features from every modality (N = 50, hardcoded; CHANGE this to a parameter!!!)
  const int nr_features_per_modality = 50;
  const size_t nr_modalities = modalities.size();
  for (size_t modality_index = 0; modality_index < nr_modalities; ++modality_index)
  {
    modalities[modality_index]->extractFeatures(*(masks[modality_index]), nr_features_per_modality, 
                                                modality_index, linemod_template.features);
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
  linemod_template.region.x = region.x;
  linemod_template.region.y = region.y;
  //linemod_template.region.x = region.x - centerX;
  //linemod_template.region.y = region.y - centerY;
  linemod_template.region.width = region.width;
  linemod_template.region.height = region.height;

  // add template to template storage
  templates_.push_back(linemod_template);

  return static_cast<int> (templates_.size () - 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::
detectTemplates (std::vector<QuantizableModality*> & modalities, std::vector<LINEMODDetection> & detections)
{
  // create energy maps
  std::vector<EnergyMaps> modality_energy_maps;
  const size_t nr_modalities = modalities.size();
  for (size_t modality_index = 0; modality_index < nr_modalities; ++modality_index)
  {
    const QuantizedMap & quantized_map = modalities[modality_index]->getSpreadedQuantizedMap ();

    const int width = quantized_map.getWidth ();
    const int height = quantized_map.getHeight ();

    const unsigned char * quantized_data = quantized_map.getData ();

    const int nr_bins = 8;
    EnergyMaps energy_maps;
    energy_maps.initialize (width, height, nr_bins);
    //std::vector< unsigned char* > energy_maps(nr_bins);
    for (int bin_index = 0; bin_index < nr_bins; ++bin_index)
    {
      //energy_maps[bin_index] = new unsigned char[width*height];
      //memset (energy_maps[bin_index], 0, width*height);

      unsigned char val0 = 0x1 << bin_index; // e.g. 00100000
      unsigned char val1 = (0x1 << (bin_index+1)%8) | (0x1 << (bin_index+9)%8); // e.g. 01010000
      for (int index = 0; index < width*height; ++index)
      {
        if ((val0 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
        if ((val1 & quantized_data[index]) != 0)
          ++energy_maps (bin_index, index);
      }
    }

    modality_energy_maps.push_back (energy_maps);
  }

  // create linearized maps
  const int step_size = 8;
  std::vector<std::vector<LinearizedMaps> > modality_linearized_maps;
  for (size_t modality_index = 0; modality_index < nr_modalities; ++modality_index)
  {
    const int width = modality_energy_maps[modality_index].getWidth ();
    const int height = modality_energy_maps[modality_index].getHeight ();

    std::vector< LinearizedMaps > linearized_maps;
    const int nr_bins = modality_energy_maps[modality_index].getNumOfBins ();
    for (int bin_index = 0; bin_index < nr_bins; ++bin_index)
    {
      unsigned char * energy_map = modality_energy_maps[modality_index] (bin_index);

      LinearizedMaps maps;
      maps.initialize (width, height, step_size);
      for (int map_row = 0; map_row < step_size; ++map_row)
      {
        for (int map_col = 0; map_col < step_size; ++map_col)
        {
          unsigned char * linearized_map = maps (map_col, map_row);

          // copy data from energy maps
          const int lin_width = width/step_size;
          const int lin_height = height/step_size;
          for (int row_index = 0; row_index < lin_height; ++row_index)
          {
            for (int col_index = 0; col_index < lin_width; ++col_index)
            {
              const int tmp_col_index = col_index*step_size + map_col;
              const int tmp_row_index = row_index*step_size + map_row;

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
  const int width = modality_energy_maps[0].getWidth ();
  const int height = modality_energy_maps[0].getHeight ();
  for (size_t template_index = 0; template_index < templates_.size (); ++template_index)
  {
    const int mem_width = width / step_size;
    const int mem_height = height / step_size;
    const int mem_size = mem_width * mem_height;

    unsigned char * score_sums = new unsigned char[mem_size];
    memset (score_sums, 0, mem_size);

    int max_score = 0;
    for (size_t feature_index = 0; feature_index < templates_[template_index].features.size (); ++feature_index)
    {
      const QuantizedMultiModFeature & feature = templates_[template_index].features[feature_index];

      //feature.modality_index;
      for (int bin_index = 0; bin_index < 8; ++bin_index)
      {
        if ((feature.quantized_value & (0x1<<bin_index)) != 0)
        {
          max_score += 2;

          unsigned char * data = modality_linearized_maps[feature.modality_index][bin_index].getOffsetMap (feature.x, feature.y);
          for (int mem_index = 0; mem_index < mem_size; ++mem_index)
          {
            score_sums[mem_index] += data[mem_index];
          }
        }
      }
    }

    const float inv_max_score = 1.0f / max_score;
    
    int max_value = 0;
    int max_index = 0;
    for (int mem_index = 0; mem_index < mem_size; ++mem_index)
    {
      if (score_sums[mem_index] > max_value) 
      {
        max_value = score_sums[mem_index];
        max_index = mem_index;
      }
    }

    const int max_col_index = (max_index % mem_width) * step_size;
    const int max_row_index = (max_index / mem_width) * step_size;

    LINEMODDetection detection;
    detection.x = max_col_index;
    detection.y = max_row_index;
    detection.template_id = static_cast<int> (template_index);
    detection.score = max_value * inv_max_score;

    detections.push_back (detection);

    delete[] score_sums;
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
pcl::LINEMOD::
saveTemplates (const char* file_name)
{
  std::ofstream file_stream;
  file_stream.open (file_name, std::ofstream::out | std::ofstream::binary);

  serialize (file_stream);

  file_stream.close ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::
loadTemplates (const char* file_name)
{
  std::ifstream file_stream;
  file_stream.open (file_name, std::ofstream::in | std::ofstream::binary);

  deserialize (file_stream);

  file_stream.close ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::
serialize (std::ostream & stream)
{
  const int num_of_templates = static_cast<int> (templates_.size ());
  write (stream, num_of_templates);
  for (int template_index = 0; template_index < num_of_templates; ++template_index)
  {
    templates_[template_index].serialize (stream);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::LINEMOD::
deserialize (std::istream & stream)
{
  templates_.clear ();

  int num_of_templates;
  read (stream, num_of_templates);
  templates_.resize (num_of_templates);
  for (int template_index = 0; template_index < num_of_templates; ++template_index)
  {
    templates_[template_index].deserialize (stream);
  }
}
