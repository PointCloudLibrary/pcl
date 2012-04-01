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

#ifndef PCL_RECOGNITION_SURFACE_NORMAL_MODALITY
#define PCL_RECOGNITION_SURFACE_NORMAL_MODALITY

#include <pcl/recognition/quantizable_modality.h>
#include <pcl/recognition/distance_map.h>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/linear_least_squares_normal.h>

namespace pcl
{

  struct QuantizedNormalLookUpTable
  {
    int range_x;
    int range_y;
    int range_z;

    int offset_x;
    int offset_y;
    int offset_z;

    int size_x;
    int size_y;
    int size_z;

    unsigned char * lut;

    QuantizedNormalLookUpTable () : 
      range_x (-1), range_y (-1), range_z (-1), 
      offset_x (-1), offset_y (-1), offset_z (-1), 
      size_x (-1), size_y (-1), size_z (-1), lut (NULL) 
    {}

    //~QuantizedNormalLookUpTable () { if (lut != NULL) free16(lut); }
    ~QuantizedNormalLookUpTable () 
    { 
      if (lut != NULL) 
        delete[] lut; 
    }

    void 
    initializeLUT (const int range_x_arg, const int range_y_arg, const int range_z_arg)
    {
      range_x = range_x_arg;
      range_y = range_y_arg;
      range_z = range_z_arg;
      size_x = range_x_arg+1;
      size_y = range_y_arg+1;
      size_z = range_z_arg+1;
      offset_x = range_x_arg/2;
      offset_y = range_y_arg/2;
      offset_z = range_z_arg;

      //if (lut != NULL) free16(lut);
      //lut = malloc16(size_x*size_y*size_z);

      if (lut != NULL) 
        delete[] lut;
      lut = new unsigned char[size_x*size_y*size_z];

      const int nr_normals = 8;
	  pcl::PointCloud<PointXYZ>::VectorType ref_normals (nr_normals);
      
      const float normal0_angle = 40.0f * 3.14f / 180.0f;
      ref_normals[0].x = cosf (normal0_angle);
      ref_normals[0].y = 0.0f;
      ref_normals[0].z = -sinf (normal0_angle);

      const float inv_nr_normals = 1.0f / static_cast<float> (nr_normals);
      for (int normal_index = 1; normal_index < nr_normals; ++normal_index)
      {
        const float angle = 2.0f * static_cast<float> (M_PI * normal_index * inv_nr_normals);

        ref_normals[normal_index].x = cosf (angle) * ref_normals[0].x - sinf (angle) * ref_normals[0].y;
        ref_normals[normal_index].y = sinf (angle) * ref_normals[0].x + cosf (angle) * ref_normals[0].y;
        ref_normals[normal_index].z = ref_normals[0].z;
      }

      // normalize normals
      for (int normal_index = 0; normal_index < nr_normals; ++normal_index)
      {
        const float length = sqrt (ref_normals[normal_index].x * ref_normals[normal_index].x +
                                   ref_normals[normal_index].y * ref_normals[normal_index].y +
                                   ref_normals[normal_index].z * ref_normals[normal_index].z);

        const float inv_length = 1.0f / length;

        ref_normals[normal_index].x *= inv_length;
        ref_normals[normal_index].y *= inv_length;
        ref_normals[normal_index].z *= inv_length;
      }

      // set LUT
      for (int z_index = 0; z_index < size_z; ++z_index)
      {
        for (int y_index = 0; y_index < size_y; ++y_index)
        {
          for (int x_index = 0; x_index < size_x; ++x_index)
          {
            PointXYZ normal (static_cast<float> (x_index - range_x/2), 
                             static_cast<float> (y_index - range_y/2), 
                             static_cast<float> (z_index - range_z));
            const float length = sqrt (normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
            const float inv_length = 1.0f / (length + 0.00001f);

            normal.x *= inv_length;
            normal.y *= inv_length;
            normal.z *= inv_length;

            float max_response = -1.0f;
            int max_index = -1;

            for (int normal_index = 0; normal_index < nr_normals; ++normal_index)
            {
              const float response = normal.x * ref_normals[normal_index].x +
                                     normal.y * ref_normals[normal_index].y +
                                     normal.z * ref_normals[normal_index].z;

              const float abs_response = fabsf (response);
              if (max_response < abs_response)
              {
                max_response = abs_response;
                max_index = normal_index;
              }

              lut[z_index*size_y*size_x + y_index*size_x + x_index] = static_cast<unsigned char> (0x1 << max_index);
            }
          }
        }
      }
    }

    inline unsigned char 
    operator() (const float x, const float y, const float z) const
    {
      const size_t x_index = static_cast<size_t> (x * static_cast<float> (offset_x) + static_cast<float> (offset_x));
      const size_t y_index = static_cast<size_t> (y * static_cast<float> (offset_y) + static_cast<float> (offset_y));
      const size_t z_index = static_cast<size_t> (z * static_cast<float> (range_z) + static_cast<float> (range_z));

      const size_t index = z_index*size_y*size_x + y_index*size_x + x_index;

      return (lut[index]);
    }

    inline unsigned char 
    operator() (const int index) const
    {
      return (lut[index]);
    }
  };


  template <typename PointInT>
  class SurfaceNormalModality : public QuantizableModality, public PCLBase<PointInT>
  {
    protected:
      using PCLBase<PointInT>::input_;

      struct Candidate
      {
        Candidate () : normal (), distance (0.0f), bin_index (0), x (0), y (0) {}

        Normal normal;
        float distance;

        unsigned char bin_index;
    
        size_t x;
        size_t y;	

        bool 
        operator< (const Candidate & rhs)
        {
          return (distance > rhs.distance);
        }
      };

    public:
      typedef typename pcl::PointCloud<PointInT> PointCloudIn;

      SurfaceNormalModality ();

      virtual ~SurfaceNormalModality ();

      inline void
      setSpreadingSize (const size_t spreading_size)
      {
        spreading_size_ = spreading_size;
      }

      inline QuantizedMap &
      getQuantizedMap () 
      { 
        return (filtered_quantized_surface_normals_); 
      }

      inline QuantizedMap &
      getSpreadedQuantizedMap () 
      { 
        return (spreaded_quantized_surface_normals_); 
      }

      void 
      extractFeatures (const MaskMap & mask, size_t nr_features, size_t modality_index,
                       std::vector<QuantizedMultiModFeature> & features) const;

      /** \brief Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method)
        * \param[in] cloud the const boost shared pointer to a PointCloud message
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
      quantizeSurfaceNormals ();

      void
      filterQuantizedSurfaceNormals ();

      void
      computeDistanceMap (const MaskMap & input, DistanceMap & output) const;

    private:

      float feature_distance_threshold_;

      QuantizedNormalLookUpTable normal_lookup_;

      size_t spreading_size_;

      pcl::PointCloud<pcl::Normal> surface_normals_;
      pcl::QuantizedMap quantized_surface_normals_;
      pcl::QuantizedMap filtered_quantized_surface_normals_;
      pcl::QuantizedMap spreaded_quantized_surface_normals_;

  };

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::SurfaceNormalModality<PointInT>::
SurfaceNormalModality ()
  : feature_distance_threshold_ (1.0f)
  , normal_lookup_ ()
  , spreading_size_ (8)
  , surface_normals_ ()
  , quantized_surface_normals_ ()
  , filtered_quantized_surface_normals_ ()
  , spreaded_quantized_surface_normals_ ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::SurfaceNormalModality<PointInT>::~SurfaceNormalModality ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::processInputData ()
{
  // compute surface normals
  pcl::LinearLeastSquaresNormalEstimation<PointInT, pcl::Normal> ne;
  //ne.setMaxDepthChangeFactor(1.0f);
  //ne.setNormalSmoothingSize(9.0f);
  //ne.setDepthDependentSmoothing(true);
  ne.setInputCloud (input_);
  ne.compute (surface_normals_);

  // quantize surface normals
  quantizeSurfaceNormals ();

  // filter quantized surface normals
  filterQuantizedSurfaceNormals ();

  // spread quantized surface normals
  pcl::QuantizedMap::spreadQuantizedMap (filtered_quantized_surface_normals_,
                                         spreaded_quantized_surface_normals_,
                                         spreading_size_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::extractFeatures (const MaskMap & mask,
                                                       const size_t nr_features,
                                                       const size_t modality_index,
                                                       std::vector<QuantizedMultiModFeature> & features) const
{
  const size_t width = mask.getWidth ();
  const size_t height = mask.getHeight ();

  //cv::Mat maskImage(height, width, CV_8U, mask.mask);
  //cv::erode(maskImage, maskImage

  // create distance maps for every quantization value
  //cv::Mat distance_maps[8];
  //for (int map_index = 0; map_index < 8; ++map_index)
  //{
  //  distance_maps[map_index] = ::cv::Mat::zeros(height, width, CV_8U);
  //}

  MaskMap mask_maps[8];
  for (size_t map_index = 0; map_index < 8; ++map_index)
    mask_maps[map_index].resize (width, height);

  unsigned char map[255];
  memset(map, 0, 255);

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

  for (size_t row_index = 0; row_index < height; ++row_index)
  {
    for (size_t col_index = 0; col_index < width; ++col_index)
    {
      if (mask (col_index, row_index) != 0)
      {
        //const unsigned char quantized_value = quantized_surface_normals_ (row_index, col_index);
        const unsigned char quantized_value = filtered_quantized_surface_normals_ (col_index, row_index);

        if (quantized_value == 0) 
          continue;
        const int dist_map_index = map[quantized_value];

        distance_map_indices (col_index, row_index) = static_cast<unsigned char> (dist_map_index);
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

  const size_t off = 4;
  for (size_t row_index = off; row_index < height-off; ++row_index)
  {
    for (size_t col_index = off; col_index < width-off; ++col_index)
    {
      if (mask (col_index, row_index) != 0)
      {
        //const unsigned char quantized_value = quantized_surface_normals_ (row_index, col_index);
        const unsigned char quantized_value = filtered_quantized_surface_normals_ (col_index, row_index);

        const float nx = surface_normals_ (col_index, row_index).normal_x;
        const float ny = surface_normals_ (col_index, row_index).normal_y;
        const float nz = surface_normals_ (col_index, row_index).normal_z;

        if (quantized_value != 0 && !(pcl_isnan (nx) || pcl_isnan (ny) || pcl_isnan (nz)))
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
            candidate.bin_index = static_cast<unsigned char> (distance_map_index);

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
      feature.quantized_value = filtered_quantized_surface_normals_ (iter->x, iter->y);

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
    feature.quantized_value = filtered_quantized_surface_normals_ (iter2->x, iter2->y);

    features.push_back (feature);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::quantizeSurfaceNormals ()
{
  const size_t width = input_->width;
  const size_t height = input_->height;

  quantized_surface_normals_.resize (width, height);

  for (size_t row_index = 0; row_index < height; ++row_index)
  {
    for (size_t col_index = 0; col_index < width; ++col_index)
    {
      const float normal_x = surface_normals_ (col_index, row_index).normal_x;
      const float normal_y = surface_normals_ (col_index, row_index).normal_y;
      const float normal_z = surface_normals_ (col_index, row_index).normal_z;

      if (pcl_isnan(normal_x) || pcl_isnan(normal_y) || pcl_isnan(normal_z) || normal_z > 0)
      {
        quantized_surface_normals_ (col_index, row_index) = 0;
        continue;
      }

      //quantized_surface_normals_.data[row_index*width+col_index] =
      //  normal_lookup_(normal_x, normal_y, normal_z);

      float angle = atan2f (normal_y, normal_x)*180.0f/3.14f;

      if (angle < 0.0f) angle += 360.0f;
      if (angle >= 360.0f) angle -= 360.0f;

      int bin_index = static_cast<int> (angle*8.0f/360.0f);

      //quantized_surface_normals_.data[row_index*width+col_index] = 0x1 << bin_index;
      quantized_surface_normals_ (col_index, row_index) = static_cast<unsigned char> (bin_index);
    }
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::filterQuantizedSurfaceNormals ()
{
  const int width = input_->width;
  const int height = input_->height;

  filtered_quantized_surface_normals_.resize (width, height);

  // filter data
  for (int row_index = 1; row_index < height-1; ++row_index)
  {
    for (int col_index = 1; col_index < width-1; ++col_index)
    {
      unsigned char histogram[9] = {0,0,0,0,0,0,0,0,0};

      {
        unsigned char * dataPtr = quantized_surface_normals_.getData () + (row_index-1)*width+col_index-1;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
      }
      {
        unsigned char * dataPtr = quantized_surface_normals_.getData () + row_index*width+col_index-1;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
      }
      {
        unsigned char * dataPtr = quantized_surface_normals_.getData () + (row_index+1)*width+col_index-1;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
      }

      unsigned char max_hist_value = 0;
      int max_hist_index = -1;

      if (max_hist_value < histogram[1]) {max_hist_index = 0; max_hist_value = histogram[1];}
      if (max_hist_value < histogram[2]) {max_hist_index = 1; max_hist_value = histogram[2];}
      if (max_hist_value < histogram[3]) {max_hist_index = 2; max_hist_value = histogram[3];}
      if (max_hist_value < histogram[4]) {max_hist_index = 3; max_hist_value = histogram[4];}
      if (max_hist_value < histogram[5]) {max_hist_index = 4; max_hist_value = histogram[5];}
      if (max_hist_value < histogram[6]) {max_hist_index = 5; max_hist_value = histogram[6];}
      if (max_hist_value < histogram[7]) {max_hist_index = 6; max_hist_value = histogram[7];}
      if (max_hist_value < histogram[8]) {max_hist_index = 7; max_hist_value = histogram[8];}

      if (max_hist_index != -1 && max_hist_value >= 1)
      {
        filtered_quantized_surface_normals_ (col_index, row_index) = static_cast<unsigned char> (0x1 << max_hist_index);
      }
      else
      {
        filtered_quantized_surface_normals_ (col_index, row_index) = 0;
      }

      //filtered_quantized_color_gradients_.data[row_index*width+col_index] = quantized_color_gradients_.data[row_index*width+col_index];
    }
  }

  //cv::Mat data1(quantized_surface_normals_.height, quantized_surface_normals_.width, CV_8U, quantized_surface_normals_.data);
  //cv::Mat data2(filtered_quantized_surface_normals_.height, filtered_quantized_surface_normals_.width, CV_8U, filtered_quantized_surface_normals_.data);

  //cv::medianBlur(data1, data2, 3);

  //for (int row_index = 0; row_index < height; ++row_index)
  //{
  //  for (int col_index = 0; col_index < width; ++col_index)
  //  {
  //    filtered_quantized_surface_normals_.data[row_index*width+col_index] = 0x1 << filtered_quantized_surface_normals_.data[row_index*width+col_index];
  //  }
  //}
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::computeDistanceMap (const MaskMap & input, DistanceMap & output) const
{
  const size_t width = input.getWidth ();
  const size_t height = input.getHeight ();

  output.resize (width, height);

  // compute distance map
  //float *distance_map = new float[input_->points.size ()];
  const unsigned char * mask_map = input.getData ();
  float * distance_map = output.getData ();
  for (size_t index = 0; index < width*height; ++index)
  {
    if (mask_map[index] == 0)
      distance_map[index] = 0.0f;
    else
      distance_map[index] = static_cast<float> (width + height);
  }

  // first pass
  float * previous_row = distance_map;
  float * current_row = previous_row + width;
  for (size_t ri = 1; ri < height; ++ri)
  {
    for (size_t ci = 1; ci < width; ++ci)
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


#endif 
