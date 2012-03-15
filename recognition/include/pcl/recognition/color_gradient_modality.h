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

#ifndef PCL_RECOGNITION_COLOR_GRADIENT_MODALITY
#define PCL_RECOGNITION_COLOR_GRADIENT_MODALITY

#include <pcl/recognition/quantizable_modality.h>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/recognition/point_types.h>


namespace pcl
{

  // --------------------------------------------------------------------------

  template <typename PointInT>
  class ColorGradientModality
    : public QuantizableModality, public PCLBase<PointInT>
  {
    protected:
      using PCLBase<PointInT>::input_;

    struct Candidate
    {
      GradientXY gradient;
    
      int x;
      int y;	
    
      bool operator< (const Candidate & rhs)
      {
        return (gradient.magnitude > rhs.gradient.magnitude);
      }
    };

    public:
      typedef typename pcl::PointCloud<PointInT> PointCloudIn;

      ColorGradientModality ();
  
      virtual ~ColorGradientModality ();
  
      inline void
      setGradientMagnitudeThreshold (const float threshold)
      {
        gradient_magnitude_threshold_ = threshold;
      }
  
      inline QuantizedMap &
      getQuantizedMap () 
      { 
        return (filtered_quantized_color_gradients_);
      }
  
      inline QuantizedMap &
      getSpreadedQuantizedMap () 
      { 
        return (spreaded_filtered_quantized_color_gradients_);
      }
  
      void
      extractFeatures (const MaskMap & mask, size_t nr_features, size_t modalityIndex,
                       std::vector<QuantizedMultiModFeature> & features) const;
  
      /** \brief Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method)
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      virtual void 
      setInputCloud (const typename PointCloudIn::ConstPtr & cloud) 
      { 
        input_ = cloud;
      }

    protected:
      virtual void
      processInputData ();

      void
      computeMaxColorGradients ();
  
      void
      quantizeColorGradients ();
  
      void
      filterQuantizedColorGradients ();
  
    private:
      float gradient_magnitude_threshold_;
      pcl::PointCloud<pcl::GradientXY> color_gradients_;
  
      pcl::QuantizedMap quantized_color_gradients_;
      pcl::QuantizedMap filtered_quantized_color_gradients_;
      pcl::QuantizedMap spreaded_filtered_quantized_color_gradients_;
  
  };

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::ColorGradientModality<PointInT>::
ColorGradientModality ()
  : gradient_magnitude_threshold_ (80.0f)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::ColorGradientModality<PointInT>::
~ColorGradientModality ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorGradientModality<PointInT>::
processInputData ()
{
  // extract color gradients
  computeMaxColorGradients ();

  // quantize gradients
  quantizeColorGradients ();

  // filter quantized gradients to get only dominants one + thresholding
  filterQuantizedColorGradients ();

  // spread filtered quantized gradients
  //spreadFilteredQunatizedColorGradients ();
  const int spreading_size = 8;
  pcl::QuantizedMap::spreadQuantizedMap (filtered_quantized_color_gradients_,
                                         spreaded_filtered_quantized_color_gradients_, spreading_size);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void pcl::ColorGradientModality<PointInT>::
extractFeatures (const MaskMap & mask, const size_t nr_features, const size_t modality_index,
                 std::vector<QuantizedMultiModFeature> & features) const
{
  const size_t width = mask.getWidth ();
  const size_t height = mask.getHeight ();
  
  std::list<Candidate> list1;
  std::list<Candidate> list2;

  for (size_t row_index = 0; row_index < height; ++row_index)
  {
    for (size_t col_index = 0; col_index < width; ++col_index)
    {
      if (mask (col_index, row_index) != 0)
      {
        const GradientXY & gradient = color_gradients_ (col_index, row_index);
        if (gradient.magnitude > gradient_magnitude_threshold_)
        {
          Candidate candidate;
          candidate.gradient = gradient;
          candidate.x = static_cast<int> (col_index);
          candidate.y = static_cast<int> (row_index);

          list1.push_back (candidate);
        }
      }
    }
  }

  list1.sort();

  if (list1.size () <= nr_features)
  {
    for (typename std::list<Candidate>::iterator iter1 = list1.begin (); iter1 != list1.end (); ++iter1)
    {
      QuantizedMultiModFeature feature;
          
      feature.x = iter1->x;
      feature.y = iter1->y;
      feature.modality_index = modality_index;
      feature.quantized_value = filtered_quantized_color_gradients_ (iter1->x, iter1->y);

      features.push_back (feature);
    }
    return;
  }

  size_t distance = list1.size () / nr_features + 1; // ??? 
  while (list2.size () != nr_features)
  {
    const int sqr_distance = distance*distance;
    for (typename std::list<Candidate>::iterator iter1 = list1.begin (); iter1 != list1.end (); ++iter1)
    {
      bool candidate_accepted = true;

      for (typename std::list<Candidate>::iterator iter2 = list2.begin (); iter2 != list2.end (); ++iter2)
      {
        const int dx = iter1->x - iter2->x;
        const int dy = iter1->y - iter2->y;
        const int tmp_distance = dx*dx + dy*dy;

        //if (tmp_distance < distance) 
        if (tmp_distance < sqr_distance) /// \todo Ask Stefan if this fix is correct
        {
          candidate_accepted = false;
          break;
        }
      }

      if (candidate_accepted)
        list2.push_back (*iter1);

      if (list2.size () == nr_features)
        break;
    }
    --distance;
  }

  for (typename std::list<Candidate>::iterator iter2 = list2.begin (); iter2 != list2.end (); ++iter2)
  {
    QuantizedMultiModFeature feature;
    
    feature.x = iter2->x;
    feature.y = iter2->y;
    feature.modality_index = modality_index;
    feature.quantized_value = filtered_quantized_color_gradients_ (iter2->x, iter2->y);

    features.push_back (feature);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorGradientModality<PointInT>::
computeMaxColorGradients ()
{
  const int width = input_->width;
  const int height = input_->height;

  color_gradients_.points.resize (width*height);
  color_gradients_.width = width;
  color_gradients_.height = height;

  const float pi = tan(1.0f)*4;
  for (int row_index = 0; row_index < height-2; ++row_index)
  {
    for (int col_index = 0; col_index < width-2; ++col_index)
    {
      const int index0 = row_index*width+col_index;
      const int index_c = row_index*width+col_index+2;
      const int index_r = (row_index+2)*width+col_index;

      //const int index_d = (row_index+1)*width+col_index+1;

      const unsigned char r0 = input_->points[index0].r;
      const unsigned char g0 = input_->points[index0].g;
      const unsigned char b0 = input_->points[index0].b;

      const unsigned char r_c = input_->points[index_c].r;
      const unsigned char g_c = input_->points[index_c].g;
      const unsigned char b_c = input_->points[index_c].b;

      const unsigned char r_r = input_->points[index_r].r;
      const unsigned char g_r = input_->points[index_r].g;
      const unsigned char b_r = input_->points[index_r].b;

      const float r_dx = static_cast<float> (r_c) - static_cast<float> (r0);
      const float g_dx = static_cast<float> (g_c) - static_cast<float> (g0);
      const float b_dx = static_cast<float> (b_c) - static_cast<float> (b0);

      const float r_dy = static_cast<float> (r_r) - static_cast<float> (r0);
      const float g_dy = static_cast<float> (g_r) - static_cast<float> (g0);
      const float b_dy = static_cast<float> (b_r) - static_cast<float> (b0);

      const float sqr_mag_r = r_dx*r_dx + r_dy*r_dy;
      const float sqr_mag_g = g_dx*g_dx + g_dy*g_dy;
      const float sqr_mag_b = b_dx*b_dx + b_dy*b_dy;

      if (sqr_mag_r > sqr_mag_g && sqr_mag_r > sqr_mag_b)
      {
        GradientXY gradient;
        gradient.magnitude = sqrt (sqr_mag_r);
        gradient.angle = atan2 (r_dy, r_dx) * 180.0f / pi;
        gradient.x = static_cast<float> (col_index);
        gradient.y = static_cast<float> (row_index);

        color_gradients_ (col_index+1, row_index+1) = gradient;
      }
      else if (sqr_mag_g > sqr_mag_b)
      {
        GradientXY gradient;
        gradient.magnitude = sqrt (sqr_mag_g);
        gradient.angle = atan2 (g_dy, g_dx) * 180.0f / pi;
        gradient.x = static_cast<float> (col_index);
        gradient.y = static_cast<float> (row_index);

        color_gradients_ (col_index+1, row_index+1) = gradient;
      }
      else
      {
        GradientXY gradient;
        gradient.magnitude = sqrt (sqr_mag_b);
        gradient.angle = atan2 (b_dy, b_dx) * 180.0f / pi;
        gradient.x = static_cast<float> (col_index);
        gradient.y = static_cast<float> (row_index);

        color_gradients_ (col_index+1, row_index+1) = gradient;
      }

      assert (color_gradients_ (col_index+1, row_index+1).angle >= -180 &&
              color_gradients_ (col_index+1, row_index+1).angle <=  180);
    }
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorGradientModality<PointInT>::
quantizeColorGradients ()
{
  const size_t width = input_->width;
  const size_t height = input_->height;

  quantized_color_gradients_.resize (width, height);

  //unsigned char quantization_map[16] = {0,1,2,3,4,5,6,7,0,1,2,3,4,5,6,7};
  unsigned char quantization_map[16] = {1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8};

  const float angleScale = 1.0f/22.6f;
  for (size_t row_index = 0; row_index < height; ++row_index)
  {
    for (size_t col_index = 0; col_index < width; ++col_index)
    {
      if (color_gradients_ (col_index, row_index).magnitude < gradient_magnitude_threshold_) 
      {
        quantized_color_gradients_ (col_index, row_index) = 0;
        continue;
      }

      const int quantized_value = static_cast<int> (color_gradients_ (col_index, row_index).angle * angleScale) + 8;
      assert (0 <= quantized_value && quantized_value < 16);
      quantized_color_gradients_ (col_index, row_index) = quantization_map[quantized_value];
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorGradientModality<PointInT>::
filterQuantizedColorGradients ()
{
  const size_t width = input_->width;
  const size_t height = input_->height;

  filtered_quantized_color_gradients_.resize (width, height);

  // filter data
  for (size_t row_index = 1; row_index < height-1; ++row_index)
  {
    for (size_t col_index = 1; col_index < width-1; ++col_index)
    {
      unsigned char histogram[9] = {0,0,0,0,0,0,0,0,0};

      {
        const unsigned char * data_ptr = quantized_color_gradients_.getData () + (row_index-1)*width+col_index-1;
        assert (0 <= data_ptr[0] && data_ptr[0] < 9 && 
                0 <= data_ptr[1] && data_ptr[1] < 9 && 
                0 <= data_ptr[2] && data_ptr[2] < 9);
        ++histogram[data_ptr[0]];
        ++histogram[data_ptr[1]];
        ++histogram[data_ptr[2]];
      }
      {
        const unsigned char * data_ptr = quantized_color_gradients_.getData () + row_index*width+col_index-1;
        assert (0 <= data_ptr[0] && data_ptr[0] < 9 && 
                0 <= data_ptr[1] && data_ptr[1] < 9 && 
                0 <= data_ptr[2] && data_ptr[2] < 9);
        ++histogram[data_ptr[0]];
        ++histogram[data_ptr[1]];
        ++histogram[data_ptr[2]];
      }
      {
        const unsigned char * data_ptr = quantized_color_gradients_.getData () + (row_index+1)*width+col_index-1;
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
      if (max_hist_value < histogram[1]) {max_hist_index = 0; max_hist_value = histogram[1];}
      if (max_hist_value < histogram[2]) {max_hist_index = 1; max_hist_value = histogram[2];}
      if (max_hist_value < histogram[3]) {max_hist_index = 2; max_hist_value = histogram[3];}
      if (max_hist_value < histogram[4]) {max_hist_index = 3; max_hist_value = histogram[4];}
      if (max_hist_value < histogram[5]) {max_hist_index = 4; max_hist_value = histogram[5];}
      if (max_hist_value < histogram[6]) {max_hist_index = 5; max_hist_value = histogram[6];}
      if (max_hist_value < histogram[7]) {max_hist_index = 6; max_hist_value = histogram[7];}
      if (max_hist_value < histogram[8]) {max_hist_index = 7; max_hist_value = histogram[8];}

      if (max_hist_index != -1 && max_hist_value >= 5)
        filtered_quantized_color_gradients_ (col_index, row_index) = 0x1 << max_hist_index;
      else
        filtered_quantized_color_gradients_ (col_index, row_index) = 0;

    }
  }
}

#endif 
