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

#include <pcl/recognition/quantizable_modality.h>
#include <pcl/recognition/distance_map.h>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/recognition/point_types.h>

#include <algorithm>
#include <cstddef>
#include <list>
#include <vector>

namespace pcl
{

  // --------------------------------------------------------------------------

  template <typename PointInT>
  class ColorModality
    : public QuantizableModality, public PCLBase<PointInT>
  {
    protected:
      using PCLBase<PointInT>::input_;

      struct Candidate
      {
        float distance;

        unsigned char bin_index;
    
        std::size_t x;
        std::size_t y;	

        bool 
        operator< (const Candidate & rhs)
        {
          return (distance > rhs.distance);
        }
      };

    public:
      using PointCloudIn = pcl::PointCloud<PointInT>;

      ColorModality ();
  
      virtual ~ColorModality () = default;
  
      inline QuantizedMap &
      getQuantizedMap () 
      { 
        return (filtered_quantized_colors_);
      }
  
      inline QuantizedMap &
      getSpreadedQuantizedMap () 
      { 
        return (spreaded_filtered_quantized_colors_);
      }
  
      void
      extractFeatures (const MaskMap & mask, std::size_t nr_features, std::size_t modalityIndex,
                       std::vector<QuantizedMultiModFeature> & features) const;
  
      /** \brief Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method)
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      virtual void 
      setInputCloud (const typename PointCloudIn::ConstPtr & cloud) 
      { 
        input_ = cloud;
      }

      virtual void
      processInputData ();

    protected:

      void
      quantizeColors ();
  
      void
      filterQuantizedColors ();

      static inline int
      quantizeColorOnRGBExtrema (const float r,
                                 const float g,
                                 const float b);
  
      void
      computeDistanceMap (const MaskMap & input, DistanceMap & output) const;

    private:
      float feature_distance_threshold_;
      
      pcl::QuantizedMap quantized_colors_;
      pcl::QuantizedMap filtered_quantized_colors_;
      pcl::QuantizedMap spreaded_filtered_quantized_colors_;
  
  };

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::ColorModality<PointInT>::ColorModality ()
  : feature_distance_threshold_ (1.0f), quantized_colors_ (), filtered_quantized_colors_ (), spreaded_filtered_quantized_colors_ ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorModality<PointInT>::processInputData ()
{
  // quantize gradients
  quantizeColors ();

  // filter quantized gradients to get only dominants one + thresholding
  filterQuantizedColors ();

  // spread filtered quantized gradients
  //spreadFilteredQunatizedColorGradients ();
  const int spreading_size = 8;
  pcl::QuantizedMap::spreadQuantizedMap (filtered_quantized_colors_,
                                         spreaded_filtered_quantized_colors_, spreading_size);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void pcl::ColorModality<PointInT>::extractFeatures (const MaskMap & mask, 
                                                    const std::size_t nr_features, 
                                                    const std::size_t modality_index,
                                                    std::vector<QuantizedMultiModFeature> & features) const
{
  const std::size_t width = mask.getWidth ();
  const std::size_t height = mask.getHeight ();

  MaskMap mask_maps[8];
  for (std::size_t map_index = 0; map_index < 8; ++map_index)
    mask_maps[map_index].resize (width, height);

  unsigned char map[255]{};

  map[0x1<<0] = 0;
  map[0x1<<1] = 1;
  map[0x1<<2] = 2;
  map[0x1<<3] = 3;
  map[0x1<<4] = 4;
  map[0x1<<5] = 5;
  map[0x1<<6] = 6;
  map[0x1<<7] = 7;

  QuantizedMap distance_map_indices (width, height);
  //memset (distance_map_indices.data, 0, sizeof (distance_map_indices.data[0])*width*height);

  for (std::size_t row_index = 0; row_index < height; ++row_index)
  {
    for (std::size_t col_index = 0; col_index < width; ++col_index)
    {
      if (mask (col_index, row_index) != 0)
      {
        //const unsigned char quantized_value = quantized_surface_normals_ (row_index, col_index);
        const unsigned char quantized_value = filtered_quantized_colors_ (col_index, row_index);

        if (quantized_value == 0) 
          continue;
        const int dist_map_index = map[quantized_value];

        distance_map_indices (col_index, row_index) = dist_map_index;
        //distance_maps[dist_map_index].at<unsigned char>(row_index, col_index) = 255;
        mask_maps[dist_map_index] (col_index, row_index) = 255;
      }
    }
  }

  DistanceMap distance_maps[8];
  for (int map_index = 0; map_index < 8; ++map_index)
    computeDistanceMap (mask_maps[map_index], distance_maps[map_index]);

  std::list<Candidate> list1;
  std::list<Candidate> list2;

  float weights[8] = {0,0,0,0,0,0,0,0};

  const std::size_t off = 4;
  for (std::size_t row_index = off; row_index < height-off; ++row_index)
  {
    for (std::size_t col_index = off; col_index < width-off; ++col_index)
    {
      if (mask (col_index, row_index) != 0)
      {
        //const unsigned char quantized_value = quantized_surface_normals_ (row_index, col_index);
        const unsigned char quantized_value = filtered_quantized_colors_ (col_index, row_index);

        //const float nx = surface_normals_ (col_index, row_index).normal_x;
        //const float ny = surface_normals_ (col_index, row_index).normal_y;
        //const float nz = surface_normals_ (col_index, row_index).normal_z;

        if (quantized_value != 0)
        {
          const int distance_map_index = map[quantized_value];

          //const float distance = distance_maps[distance_map_index].at<float> (row_index, col_index);
          const float distance = distance_maps[distance_map_index] (col_index, row_index);

          if (distance >= feature_distance_threshold_)
          {
            Candidate candidate;

            candidate.distance = distance;
            candidate.x = col_index;
            candidate.y = row_index;
            candidate.bin_index = distance_map_index;

            list1.push_back (candidate);

            ++weights[distance_map_index];
          }
        }
      }
    }
  }

  for (typename std::list<Candidate>::iterator iter = list1.begin (); iter != list1.end (); ++iter)
    iter->distance *= 1.0f / weights[iter->bin_index];

  list1.sort ();

  if (list1.size () <= nr_features)
  {
    features.reserve (list1.size ());
    for (typename std::list<Candidate>::iterator iter = list1.begin (); iter != list1.end (); ++iter)
    {
      QuantizedMultiModFeature feature;

      feature.x = static_cast<int> (iter->x);
      feature.y = static_cast<int> (iter->y);
      feature.modality_index = modality_index;
      feature.quantized_value = filtered_quantized_colors_ (iter->x, iter->y);

      features.push_back (feature);
    }

    return;
  }

  int distance = static_cast<int> (list1.size () / nr_features + 1); // ???  @todo:!:!:!:!:!:!
  while (list2.size () != nr_features)
  {
    const int sqr_distance = distance*distance;
    for (typename std::list<Candidate>::iterator iter1 = list1.begin (); iter1 != list1.end (); ++iter1)
    {
      bool candidate_accepted = true;

      for (typename std::list<Candidate>::iterator iter2 = list2.begin (); iter2 != list2.end (); ++iter2)
      {
        const int dx = static_cast<int> (iter1->x) - static_cast<int> (iter2->x);
        const int dy = static_cast<int> (iter1->y) - static_cast<int> (iter2->y);
        const int tmp_distance = dx*dx + dy*dy;

        if (tmp_distance < sqr_distance)
        {
          candidate_accepted = false;
          break;
        }
      }

      if (candidate_accepted)
        list2.push_back (*iter1);

      if (list2.size () == nr_features) break;
    }
    --distance;
  }

  for (typename std::list<Candidate>::iterator iter2 = list2.begin (); iter2 != list2.end (); ++iter2)
  {
    QuantizedMultiModFeature feature;

    feature.x = static_cast<int> (iter2->x);
    feature.y = static_cast<int> (iter2->y);
    feature.modality_index = modality_index;
    feature.quantized_value = filtered_quantized_colors_ (iter2->x, iter2->y);

    features.push_back (feature);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorModality<PointInT>::quantizeColors ()
{
  const std::size_t width = input_->width;
  const std::size_t height = input_->height;

  quantized_colors_.resize (width, height);

  for (std::size_t row_index = 0; row_index < height; ++row_index)
  {
    for (std::size_t col_index = 0; col_index < width; ++col_index)
    {
      const float r = static_cast<float> ((*input_) (col_index, row_index).r);
      const float g = static_cast<float> ((*input_) (col_index, row_index).g);
      const float b = static_cast<float> ((*input_) (col_index, row_index).b);

      quantized_colors_ (col_index, row_index) = quantizeColorOnRGBExtrema (r, g, b);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorModality<PointInT>::filterQuantizedColors ()
{
  const std::size_t width = input_->width;
  const std::size_t height = input_->height;

  filtered_quantized_colors_.resize (width, height);

  // filter data
  for (std::size_t row_index = 1; row_index < height-1; ++row_index)
  {
    for (std::size_t col_index = 1; col_index < width-1; ++col_index)
    {
      unsigned char histogram[8] = {0,0,0,0,0,0,0,0};

      {
        const unsigned char * data_ptr = quantized_colors_.getData () + (row_index-1)*width+col_index-1;
        assert (0 <= data_ptr[0] && data_ptr[0] < 9 && 
                0 <= data_ptr[1] && data_ptr[1] < 9 && 
                0 <= data_ptr[2] && data_ptr[2] < 9);
        ++histogram[data_ptr[0]];
        ++histogram[data_ptr[1]];
        ++histogram[data_ptr[2]];
      }
      {
        const unsigned char * data_ptr = quantized_colors_.getData () + row_index*width+col_index-1;
        assert (0 <= data_ptr[0] && data_ptr[0] < 9 && 
                0 <= data_ptr[1] && data_ptr[1] < 9 && 
                0 <= data_ptr[2] && data_ptr[2] < 9);
        ++histogram[data_ptr[0]];
        ++histogram[data_ptr[1]];
        ++histogram[data_ptr[2]];
      }
      {
        const unsigned char * data_ptr = quantized_colors_.getData () + (row_index+1)*width+col_index-1;
        assert (0 <= data_ptr[0] && data_ptr[0] < 9 && 
                0 <= data_ptr[1] && data_ptr[1] < 9 && 
                0 <= data_ptr[2] && data_ptr[2] < 9);
        ++histogram[data_ptr[0]];
        ++histogram[data_ptr[1]];
        ++histogram[data_ptr[2]];
      }

      unsigned char max_hist_value = 0;
      int max_hist_index = -1;

      // for (int i = 0; i < 8; ++i)
      // {
      //   if (max_hist_value < histogram[i+1])
      //   {
      //     max_hist_index = i;
      //     max_hist_value = histogram[i+1]
      //   }
      // }
      // Unrolled for performance optimization:
      if (max_hist_value < histogram[0]) {max_hist_index = 0; max_hist_value = histogram[0];}
      if (max_hist_value < histogram[1]) {max_hist_index = 1; max_hist_value = histogram[1];}
      if (max_hist_value < histogram[2]) {max_hist_index = 2; max_hist_value = histogram[2];}
      if (max_hist_value < histogram[3]) {max_hist_index = 3; max_hist_value = histogram[3];}
      if (max_hist_value < histogram[4]) {max_hist_index = 4; max_hist_value = histogram[4];}
      if (max_hist_value < histogram[5]) {max_hist_index = 5; max_hist_value = histogram[5];}
      if (max_hist_value < histogram[6]) {max_hist_index = 6; max_hist_value = histogram[6];}
      if (max_hist_value < histogram[7]) {max_hist_index = 7; max_hist_value = histogram[7];}

      //if (max_hist_index != -1 && max_hist_value >= 5)
        filtered_quantized_colors_ (col_index, row_index) = 0x1 << max_hist_index;
      //else
      //  filtered_quantized_color_gradients_ (col_index, row_index) = 0;

    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
int
pcl::ColorModality<PointInT>::quantizeColorOnRGBExtrema (const float r,
                                                         const float g,
                                                         const float b)
{
  const float r_inv = 255.0f-r;
  const float g_inv = 255.0f-g;
  const float b_inv = 255.0f-b;

  const float dist_0 = (r*r + g*g + b*b)*2.0f;
  const float dist_1 = r*r + g*g + b_inv*b_inv;
  const float dist_2 = r*r + g_inv*g_inv+ b*b;
  const float dist_3 = r*r + g_inv*g_inv + b_inv*b_inv;
  const float dist_4 = r_inv*r_inv + g*g + b*b;
  const float dist_5 = r_inv*r_inv + g*g + b_inv*b_inv;
  const float dist_6 = r_inv*r_inv + g_inv*g_inv+ b*b;
  const float dist_7 = (r_inv*r_inv + g_inv*g_inv + b_inv*b_inv)*1.5f;

  const float min_dist = std::min (std::min (std::min (dist_0, dist_1), std::min (dist_2, dist_3)), std::min (std::min (dist_4, dist_5), std::min (dist_6, dist_7)));

  if (min_dist == dist_0)
  {
    return 0;
  }
  if (min_dist == dist_1)
  {
    return 1;
  }
  if (min_dist == dist_2)
  {
    return 2;
  }
  if (min_dist == dist_3)
  {
    return 3;
  }
  if (min_dist == dist_4)
  {
    return 4;
  }
  if (min_dist == dist_5)
  {
    return 5;
  }
  if (min_dist == dist_6)
  {
    return 6;
  }
  return 7;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ColorModality<PointInT>::computeDistanceMap (const MaskMap & input, 
                                                  DistanceMap & output) const
{
  const std::size_t width = input.getWidth ();
  const std::size_t height = input.getHeight ();

  output.resize (width, height);

  // compute distance map
  //float *distance_map = new float[input_->size ()];
  const unsigned char * mask_map = input.getData ();
  float * distance_map = output.getData ();
  for (std::size_t index = 0; index < width*height; ++index)
  {
    if (mask_map[index] == 0)
      distance_map[index] = 0.0f;
    else
      distance_map[index] = static_cast<float> (width + height);
  }

  // first pass
  float * previous_row = distance_map;
  float * current_row = previous_row + width;
  for (std::size_t ri = 1; ri < height; ++ri)
  {
    for (std::size_t ci = 1; ci < width; ++ci)
    {
      const float up_left  = previous_row [ci - 1] + 1.4f; //distance_map[(ri-1)*input_->width + ci-1] + 1.4f;
      const float up       = previous_row [ci]     + 1.0f; //distance_map[(ri-1)*input_->width + ci] + 1.0f;
      const float up_right = previous_row [ci + 1] + 1.4f; //distance_map[(ri-1)*input_->width + ci+1] + 1.4f;
      const float left     = current_row  [ci - 1] + 1.0f; //distance_map[ri*input_->width + ci-1] + 1.0f;
      const float center   = current_row  [ci];            //distance_map[ri*input_->width + ci];

      const float min_value = std::min (std::min (up_left, up), std::min (left, up_right));

      if (min_value < center)
        current_row[ci] = min_value; //distance_map[ri * input_->width + ci] = min_value;
    }
    previous_row = current_row;
    current_row += width;
  }

  // second pass
  float * next_row = distance_map + width * (height - 1);
  current_row = next_row - width;
  for (int ri = static_cast<int> (height)-2; ri >= 0; --ri)
  {
    for (int ci = static_cast<int> (width)-2; ci >= 0; --ci)
    {
      const float lower_left  = next_row    [ci - 1] + 1.4f; //distance_map[(ri+1)*input_->width + ci-1] + 1.4f;
      const float lower       = next_row    [ci]     + 1.0f; //distance_map[(ri+1)*input_->width + ci] + 1.0f;
      const float lower_right = next_row    [ci + 1] + 1.4f; //distance_map[(ri+1)*input_->width + ci+1] + 1.4f;
      const float right       = current_row [ci + 1] + 1.0f; //distance_map[ri*input_->width + ci+1] + 1.0f;
      const float center      = current_row [ci];            //distance_map[ri*input_->width + ci];

      const float min_value = std::min (std::min (lower_left, lower), std::min (right, lower_right));

      if (min_value < center)
        current_row[ci] = min_value; //distance_map[ri*input_->width + ci] = min_value;
    }
    next_row = current_row;
    current_row -= width;
  }
}
