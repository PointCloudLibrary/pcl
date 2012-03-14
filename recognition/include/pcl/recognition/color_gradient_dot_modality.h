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

#ifndef PCL_FEATURES_COLOR_GRADIENT_DOT_MODALITY
#define PCL_FEATURES_COLOR_GRADIENT_DOT_MODALITY

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/recognition/dot_modality.h>
#include <pcl/recognition/quantized_map.h>


namespace pcl
{

  /** \brief A point structure for representing RGB color
    * \ingroup common
    */
  struct EIGEN_ALIGN16 PointRGB
  {
    union
    {
      union
      {
        struct
        {
          uint8_t b;
          uint8_t g;
          uint8_t r;
          uint8_t _unused;
        };
        float rgb;
      };
      uint32_t rgba;
    };

    inline PointRGB ()
    {}

    inline PointRGB (const uint8_t b, const uint8_t g, const uint8_t r)
      : b (b), g (g), r (r), _unused (0)
    {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };


  /** \brief A point structure representing Euclidean xyz coordinates, and the intensity value.
    * \ingroup common
    */
  struct EIGEN_ALIGN16 GradientXY
  {
    union
    {
      struct
      {
        float x;
        float y;
        float angle;
        float magnitude;
      };
      float data[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline bool operator< (const GradientXY & rhs)
    {
      return (magnitude > rhs.magnitude);
    }
  };
  inline std::ostream & operator << (std::ostream & os, const GradientXY & p)
  {
    os << "(" << p.x << "," << p.y << " - " << p.magnitude << ")";
    return (os);
  }

  // --------------------------------------------------------------------------

  template <typename PointInT>
  class ColorGradientDOTModality
    : public DOTModality, public PCLBase<PointInT>
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

      ColorGradientDOTModality (size_t bin_size);
  
      virtual ~ColorGradientDOTModality ();
  
      inline void
      setGradientMagnitudeThreshold (const float threshold)
      {
        gradient_magnitude_threshold_ = threshold;
      }
  
      //inline QuantizedMap &
      //getDominantQuantizedMap () 
      //{ 
      //  return (dominant_quantized_color_gradients_);
      //}
  
      inline QuantizedMap &
      getDominantQuantizedMap () 
      { 
        return (dominant_quantized_color_gradients_);
      }

      QuantizedMap
      computeInvariantQuantizedMap (const MaskMap & mask,
                                   const RegionXY & region);
  
      /** \brief Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method)
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      virtual void 
      setInputCloud (const typename PointCloudIn::ConstPtr & cloud) 
      { 
        input_ = cloud;
        //processInputData ();
      }

      virtual void
      processInputData ();

    protected:

      void
      computeMaxColorGradients ();
  
      void
      computeDominantQuantizedGradients ();
  
      //void
      //computeInvariantQuantizedGradients ();
  
    private:
      size_t bin_size_;

      float gradient_magnitude_threshold_;
      pcl::PointCloud<pcl::GradientXY> color_gradients_;
  
      pcl::QuantizedMap dominant_quantized_color_gradients_;
      //pcl::QuantizedMap invariant_quantized_color_gradients_;
  
  };

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::ColorGradientDOTModality<PointInT>::
ColorGradientDOTModality (const size_t bin_size)
  : bin_size_ (bin_size), gradient_magnitude_threshold_ (80.0f), color_gradients_ (), dominant_quantized_color_gradients_ ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::ColorGradientDOTModality<PointInT>::
~ColorGradientDOTModality ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorGradientDOTModality<PointInT>::
processInputData ()
{
  // extract color gradients
  computeMaxColorGradients ();

  // compute dominant quantized gradient map
  computeDominantQuantizedGradients ();

  // compute invariant quantized gradient map
  //computeInvariantQuantizedGradients ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorGradientDOTModality<PointInT>::
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

      GradientXY gradient;
      gradient.x = col_index;
      gradient.y = row_index;
      if (sqr_mag_r > sqr_mag_g && sqr_mag_r > sqr_mag_b)
      {
        gradient.magnitude = sqrt (sqr_mag_r);
        gradient.angle = atan2 (r_dy, r_dx) * 180.0f / pi;
      }
      else if (sqr_mag_g > sqr_mag_b)
      {
        //GradientXY gradient;
        gradient.magnitude = sqrt (sqr_mag_g);
        gradient.angle = atan2 (g_dy, g_dx) * 180.0f / pi;
        //gradient.x = col_index;
        //gradient.y = row_index;

        //color_gradients_ (col_index+1, row_index+1) = gradient;
      }
      else
      {
        //GradientXY gradient;
        gradient.magnitude = sqrt (sqr_mag_b);
        gradient.angle = atan2 (b_dy, b_dx) * 180.0f / pi;
        //gradient.x = col_index;
        //gradient.y = row_index;

        //color_gradients_ (col_index+1, row_index+1) = gradient;
      }

      assert (color_gradients_ (col_index+1, row_index+1).angle >= -180 &&
              color_gradients_ (col_index+1, row_index+1).angle <=  180);

      color_gradients_ (col_index+1, row_index+1) = gradient;
    }
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointInT>
//void
//pcl::ColorGradientDOTModality<PointInT>::
//computeInvariantQuantizedGradients ()
//{
//  const size_t input_width = input_->width;
//  const size_t input_height = input_->height;
//
//  const size_t output_width = input_width / bin_size;
//  const size_t output_height = input_height / bin_size;
//
//  invariant_quantized_color_gradients_.resize (output_width, output_height);
//
//  size_t offset_x = 0;
//  size_t offset_y = 0;
//  
//  const size_t num_gradient_bins = 7;
//  const size_t max_num_of_gradients = 7;
//  
//  const float divisor = 180.0f / (num_gradient_bins - 1.0f);
//  
//  float global_max_gradient = 0.0f;
//  float local_max_gradient = 0.0f;
//  
//  unsigned char * peak_pointer = dominant_quantized_color_gradients_.getData ();
//  
//  //int tmpCounter = 0;
//  for (size_t row_bin_index = 0; row_bin_index < output_height; ++row_bin_index)
//  {
//    for (size_t col_bin_index = 0; col_bin_index < output_width; ++col_bin_index)
//    {
//      std::vector<int> x_coordinates;
//      std::vector<int> y_coordinates;
//      std::vector<float> values;
//      
//      for (int row_pixel_index = -static_cast<int> (bin_size)/2; 
//           row_pixel_index <= static_cast<int> (bin_size)/2; 
//           row_pixel_index += static_cast<int> (bin_size)/2)
//      {
//        const size_t y_position = offset_y + row_pixel_index;
//
//        if (y_position < 0 || y_position >= input_height) continue;
//
//        for (int col_pixel_index = -static_cast<int> (bin_size)/2; 
//             col_pixel_index <= static_cast<int> (bin_size)/2; 
//             col_pixel_index += static_cast<int> (bin_size)/2)
//        {
//          const size_t x_position = offset_x + col_pixel_index;
//          size_t counter = 0;
//          
//          if (x_position < 0 || x_position >= input_width) continue;
//
//          // find maximum gradient magnitude in current bin
//          {
//            local_max_gradient = 0.0f;
//            for (size_t row_sub_index = 0; row_sub_index < bin_size; ++row_sub_index)
//            {
//              for (size_t col_sub_index = 0; col_sub_index < bin_size; ++col_sub_index)
//              {
//                const float magnitude = color_gradients_ (col_sub_index + x_position, row_sub_index + y_position).magnitude;
//
//                if (magnitude > local_max_gradient)
//                  local_max_gradient = magnitude;
//              }
//            }
//          }
//          
//          //*stringPointer += localMaxGradient;
//          
//          if (local_max_gradient > global_max_gradient)
//          {
//            global_max_gradient = local_max_gradient;
//          }
//          
//          // iteratively search for the largest gradients, set it to -1, search the next largest ... etc.
//          while (true)
//          {
//            float max_gradient;
//            size_t max_gradient_pos_x;
//            size_t max_gradient_pos_y;
//            
//            // find next location and value of maximum gradient magnitude in current region
//            {
//              max_gradient = 0.0f;
//              for (size_t row_sub_index = 0; row_sub_index < bin_size; ++row_sub_index)
//              {
//                for (size_t col_sub_index = 0; col_sub_index < bin_size; ++col_sub_index)
//                {
//                  const float magnitude = color_gradients_ (col_sub_index + x_position, row_sub_index + y_position).magnitude;
//
//                  if (magnitude > max_gradient)
//                  {
//                    max_gradient = magnitude;
//                    max_gradient_pos_x = col_sub_index;
//                    max_gradient_pos_y = row_sub_index;
//                  }
//                }
//              }
//            }
//            
//            // TODO: really localMaxGradient and not maxGradient???
//            if (local_max_gradient < gradient_magnitude_threshold_)
//            {
//              //*peakPointer |= 1 << (numOfGradientBins-1);
//              break;
//            }
//            
//            // TODO: replace gradient_magnitude_threshold_ here by a fixed ratio?
//            if (max_gradient < (local_max_gradient * gradient_magnitude_threshold_) ||
//                counter >= max_num_of_gradients)
//            {
//              break;
//            }
//            
//            ++counter;
//            
//            const size_t angle = static_cast<size_t> (180 + color_gradients_ (max_gradient_pos_x + x_position, max_gradient_pos_y + y_position).angle + 0.5f);
//            const size_t bin_index = static_cast<size_t> ((angle >= 180 ? angle-180 : angle)/divisor);
//            
//            *peak_pointer |= 1 << bin_index;
//            
//            x_coordinates.push_back (max_gradient_pos_x + x_position);
//            y_coordinates.push_back (max_gradient_pos_y + y_position);
//            values.push_back (max_gradient);
//            
//            color_gradients_ (max_gradient_pos_x + x_position, max_gradient_pos_y + y_position).magnitude = -1.0f;
//          }
//          
//          // reset values which have been set to -1
//          for (size_t value_index = 0; value_index < values.size (); ++value_index)
//          {
//            color_gradients_ (x_coordinates[value_index], y_coordinates[value_index]).magnitude = values[value_index];
//          }
//          
//          x_coordinates.clear ();
//          y_coordinates.clear ();
//          values.clear ();
//        }
//      }
//
//      if (*peak_pointer == 0)
//      {
//        *peak_pointer |= 1 << 7;
//      }
//
//      //if (*peakPointer != 0)
//      //{
//      //  ++tmpCounter;
//      //}
//      
//      //++stringPointer;
//      ++peak_pointer;
//      
//      offset_x += bin_size;
//    }
//    
//    offset_y += bin_size;
//    offset_x = bin_size/2+1;
//  }
//}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::ColorGradientDOTModality<PointInT>::
computeDominantQuantizedGradients ()
{
  const size_t input_width = input_->width;
  const size_t input_height = input_->height;

  const size_t output_width = input_width / bin_size_;
  const size_t output_height = input_height / bin_size_;

  dominant_quantized_color_gradients_.resize (output_width, output_height);

  //size_t offset_x = 0;
  //size_t offset_y = 0;
  
  const size_t num_gradient_bins = 7;
  const size_t max_num_of_gradients = 1;
  
  const float divisor = 180.0f / (num_gradient_bins - 1.0f);
  
  float global_max_gradient = 0.0f;
  float local_max_gradient = 0.0f;
  
  unsigned char * peak_pointer = dominant_quantized_color_gradients_.getData ();
  memset (peak_pointer, 0, output_width*output_height);
  
  //int tmpCounter = 0;
  for (size_t row_bin_index = 0; row_bin_index < output_height; ++row_bin_index)
  {
    for (size_t col_bin_index = 0; col_bin_index < output_width; ++col_bin_index)
    {
      const size_t x_position = col_bin_index * bin_size_;
      const size_t y_position = row_bin_index * bin_size_;

      //std::vector<int> x_coordinates;
      //std::vector<int> y_coordinates;
      //std::vector<float> values;
      
      // iteratively search for the largest gradients, set it to -1, search the next largest ... etc.
      //while (counter < max_num_of_gradients)
      {
        float max_gradient;
        size_t max_gradient_pos_x;
        size_t max_gradient_pos_y;
            
        // find next location and value of maximum gradient magnitude in current region
        {
          max_gradient = 0.0f;
          for (size_t row_sub_index = 0; row_sub_index < bin_size_; ++row_sub_index)
          {
            for (size_t col_sub_index = 0; col_sub_index < bin_size_; ++col_sub_index)
            {
              const float magnitude = color_gradients_ (col_sub_index + x_position, row_sub_index + y_position).magnitude;

              if (magnitude > max_gradient)
              {
                max_gradient = magnitude;
                max_gradient_pos_x = col_sub_index;
                max_gradient_pos_y = row_sub_index;
              }
            }
          }
        }
            
        if (max_gradient >= gradient_magnitude_threshold_)
        {
          const size_t angle = static_cast<size_t> (180 + color_gradients_ (max_gradient_pos_x + x_position, max_gradient_pos_y + y_position).angle + 0.5f);
          const size_t bin_index = static_cast<size_t> ((angle >= 180 ? angle-180 : angle)/divisor);
            
          *peak_pointer |= 1 << bin_index;
        }
            
        //++counter;
            
        //x_coordinates.push_back (max_gradient_pos_x + x_position);
        //y_coordinates.push_back (max_gradient_pos_y + y_position);
        //values.push_back (max_gradient);
            
        //color_gradients_ (max_gradient_pos_x + x_position, max_gradient_pos_y + y_position).magnitude = -1.0f;
      }

      //// reset values which have been set to -1
      //for (size_t value_index = 0; value_index < values.size (); ++value_index)
      //{
      //  color_gradients_ (x_coordinates[value_index], y_coordinates[value_index]).magnitude = values[value_index];
      //}


      if (*peak_pointer == 0)
      {
        *peak_pointer |= 1 << 7;
      }

      //if (*peakPointer != 0)
      //{
      //  ++tmpCounter;
      //}
      
      //++stringPointer;
      ++peak_pointer;
      
      //offset_x += bin_size;
    }
    
    //offset_y += bin_size;
    //offset_x = bin_size/2+1;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::QuantizedMap
pcl::ColorGradientDOTModality<PointInT>::
computeInvariantQuantizedMap (const MaskMap & mask,
                              const RegionXY & region)
{
  const size_t input_width = input_->width;
  const size_t input_height = input_->height;

  const size_t output_width = input_width / bin_size_;
  const size_t output_height = input_height / bin_size_;

  const size_t sub_start_x = region.x / bin_size_;
  const size_t sub_start_y = region.y / bin_size_;
  const size_t sub_width = region.width / bin_size_;
  const size_t sub_height = region.height / bin_size_;

  QuantizedMap map;
  map.resize (sub_width, sub_height);

  //size_t offset_x = 0;
  //size_t offset_y = 0;
  
  const size_t num_gradient_bins = 7;
  const size_t max_num_of_gradients = 7;
  
  const float divisor = 180.0f / (num_gradient_bins - 1.0f);
  
  float global_max_gradient = 0.0f;
  float local_max_gradient = 0.0f;
  
  unsigned char * peak_pointer = map.getData ();
  
  //int tmpCounter = 0;
  for (size_t row_bin_index = 0; row_bin_index < sub_height; ++row_bin_index)
  {
    for (size_t col_bin_index = 0; col_bin_index < sub_width; ++col_bin_index)
    {
      std::vector<size_t> x_coordinates;
      std::vector<size_t> y_coordinates;
      std::vector<float> values;
      
      for (int row_pixel_index = -static_cast<int> (bin_size_)/2; 
           row_pixel_index <= static_cast<int> (bin_size_)/2; 
           row_pixel_index += static_cast<int> (bin_size_)/2)
      {
        const size_t y_position = /*offset_y +*/ row_pixel_index + (sub_start_y + row_bin_index)*bin_size_;

        if (y_position < 0 || y_position >= input_height) 
          continue;

        for (int col_pixel_index = -static_cast<int> (bin_size_)/2; 
             col_pixel_index <= static_cast<int> (bin_size_)/2; 
             col_pixel_index += static_cast<int> (bin_size_)/2)
        {
          const size_t x_position = /*offset_x +*/ col_pixel_index + (sub_start_x + col_bin_index)*bin_size_;
          size_t counter = 0;
          
          if (x_position < 0 || x_position >= input_width) 
            continue;

          // find maximum gradient magnitude in current bin
          {
            local_max_gradient = 0.0f;
            for (size_t row_sub_index = 0; row_sub_index < bin_size_; ++row_sub_index)
            {
              for (size_t col_sub_index = 0; col_sub_index < bin_size_; ++col_sub_index)
              {
                const float magnitude = color_gradients_ (col_sub_index + x_position, row_sub_index + y_position).magnitude;

                if (magnitude > local_max_gradient)
                  local_max_gradient = magnitude;
              }
            }
          }
          
          //*stringPointer += localMaxGradient;
          
          if (local_max_gradient > global_max_gradient)
          {
            global_max_gradient = local_max_gradient;
          }
          
          // iteratively search for the largest gradients, set it to -1, search the next largest ... etc.
          while (true)
          {
            float max_gradient;
            size_t max_gradient_pos_x;
            size_t max_gradient_pos_y;
            
            // find next location and value of maximum gradient magnitude in current region
            {
              max_gradient = 0.0f;
              for (size_t row_sub_index = 0; row_sub_index < bin_size_; ++row_sub_index)
              {
                for (size_t col_sub_index = 0; col_sub_index < bin_size_; ++col_sub_index)
                {
                  const float magnitude = color_gradients_ (col_sub_index + x_position, row_sub_index + y_position).magnitude;

                  if (magnitude > max_gradient)
                  {
                    max_gradient = magnitude;
                    max_gradient_pos_x = col_sub_index;
                    max_gradient_pos_y = row_sub_index;
                  }
                }
              }
            }
            
            // TODO: really localMaxGradient and not maxGradient???
            if (local_max_gradient < gradient_magnitude_threshold_)
            {
              //*peakPointer |= 1 << (numOfGradientBins-1);
              break;
            }
            
            // TODO: replace gradient_magnitude_threshold_ here by a fixed ratio?
            if (/*max_gradient < (local_max_gradient * gradient_magnitude_threshold_) ||*/
                counter >= max_num_of_gradients)
            {
              break;
            }
            
            ++counter;
            
            const size_t angle = static_cast<size_t> (180 + color_gradients_ (max_gradient_pos_x + x_position, max_gradient_pos_y + y_position).angle + 0.5f);
            const size_t bin_index = static_cast<size_t> ((angle >= 180 ? angle-180 : angle)/divisor);
            
            *peak_pointer |= 1 << bin_index;
            
            x_coordinates.push_back (max_gradient_pos_x + x_position);
            y_coordinates.push_back (max_gradient_pos_y + y_position);
            values.push_back (max_gradient);
            
            color_gradients_ (max_gradient_pos_x + x_position, max_gradient_pos_y + y_position).magnitude = -1.0f;
          }
          
          // reset values which have been set to -1
          for (size_t value_index = 0; value_index < values.size (); ++value_index)
          {
            color_gradients_ (x_coordinates[value_index], y_coordinates[value_index]).magnitude = values[value_index];
          }
          
          x_coordinates.clear ();
          y_coordinates.clear ();
          values.clear ();
        }
      }

      if (*peak_pointer == 0)
      {
        *peak_pointer |= 1 << 7;
      }

      //if (*peakPointer != 0)
      //{
      //  ++tmpCounter;
      //}
      
      //++stringPointer;
      ++peak_pointer;
      
      //offset_x += bin_size;
    }
    
    //offset_y += bin_size;
    //offset_x = bin_size/2+1;
  }

  return map;
}

#endif
