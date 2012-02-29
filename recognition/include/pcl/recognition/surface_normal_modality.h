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

#ifndef PCL_FEATURES_SURFACE_NORMAL_MODALITY
#define PCL_FEATURES_SURFACE_NORMAL_MODALITY

#include "pcl/recognition/quantizable_modality.h"

#include "pcl/pcl_base.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"


namespace pcl
{

  class DistanceMap
  {
    public:
      DistanceMap () : width_ (-1), height_ (-1), data_ (NULL) {}
      virtual ~DistanceMap () {}

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
    
      inline float* 
      getData () 
      { 
        return (data_); 
      }

      void 
      initialize (const int width, const int height)
      {
        if (data_ != NULL && (width != width_ || height != height_))
          release ();

        if (data_ == NULL)
        {
          data_ = new float[width*height];
          width_ = width;
          height_ = height;
        }

        memset (data_, 0, width*height);
      }

      void 
      release ()
      {
        if (data_ != NULL) 
          delete[] data_;

        data_ = NULL;
        width_ = -1;
        height_ = -1;
      }

      inline float& 
      operator() (int col_index, int row_index)
      {
        return (data_[row_index*width_ + col_index]);
      }

      inline const float& 
      operator() (int col_index, int row_index) const
      {
        return (data_[row_index*width_ + col_index]);
      }

    protected:
      int width_;
      int height_;
      float * data_;
  };

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
      size_x (-1), size_y (-1), size (-1), lut (NULL) 
    {};

    //~QuantizedNormalLookUpTable() { if (lut != NULL) free16(lut); };
    ~QuantizedNormalLookUpTable () 
    { 
      if (lut != NULL) 
        delete[] lut; 
    };

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
      // @todo: Needs to be changed
      cv::Point3f ref_normals[nr_normals];
      
      const float normal0Angle = 40.0f * 3.14f / 180.0f;
      ref_normals[0].x = cos (normal0Angle);
      ref_normals[0].y = 0.0f;
      ref_normals[0].z = -sin (normal0Angle);

      const float invNumOfNormals = 1.0f/nr_normals;
      for (int normal_index = 1; normal_index < nr_normals; ++normal_index)
      {
        const float angle = normal_index * 3.14f * 2.0f * invNumOfNormals;

        ref_normals[normal_index].x = cos(angle)*ref_normals[0].x - sin(angle)*ref_normals[0].y;
        ref_normals[normal_index].y = sin(angle)*ref_normals[0].x + cos(angle)*ref_normals[0].y;
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
            // @todo: Needs to be changed
            cv::Point3f normal (x_index - range_x/2, y_index - range_y/2, z_index - range_z);
            const float length = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
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

              const float abs_response = fabs (response);
              if (max_response < abs_response)
              {
                max_response = abs_response;
                max_index = normal_index;
              }

              lut[z_index*size_y*size_x + y_index*size_x + x_index] = 0x1 << max_index;
            }
          }
        }
      }
    };

    inline unsigned char 
    operator() (const float x, const float y, const float z) const
    {
      const int x_index = x*offset_x + offset_x;
      const int y_index = y*offset_y + offset_y;
      const int z_index = z*range_z + range_z;

      const int index = z_index*size_y*size_x + y_index*size_x + x_index;

      return (lut[index]);
    };

    inline unsigned char 
    operator() (const int index) const
    {
      return (lut[index]);
    }
  };


  template <typename PointInT>
  class SurfaceNormalModality : public QuantizableModality, public PCLBase<PointInT>
  {
    public:

      typedef typename pcl::PointCloud<PointInT> PointCloudIn;

      SurfaceNormalModality ();

      virtual ~SurfaceNormalModality ();

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
      extractFeatures (const MaskMap & mask, int nr_features, int modality_index,
                       std::vector<QuantizedMultiModFeature> & features);

      /** \brief Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method)
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        */
      virtual void 
      setInputCloud (const typename PointCloudIn::ConstPtr &cloud) 
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
      computeDistanceMap (const MaskMap & input, DistanceMap & output);

    private:

      float feature_distance_threshold_;

      QuantizedNormalLookUpTable normal_lookup_;

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
  const int spreading_size = 8;
  pcl::QuantizedMap::spreadQuantizedMap (filtered_quantized_surface_normals_,
                                         spreaded_quantized_surface_normals_,
                                         spreading_size);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::extractFeatures (const MaskMap & mask,
                                                       const int nr_features,
                                                       const int modality_index,
                                                       std::vector<QuantizedMultiModFeature> & features)
{
  const int width = mask.getWidth ();
  const int height = mask.getHeight ();

  //cv::Mat maskImage(height, width, CV_8U, mask.mask);
  //cv::erode(maskImage, maskImage

  struct Candidate
  {
    Normal normal;
    float distance;

    unsigned char bin_index;
    
    int x;
    int y;	

    bool 
    operator< (const Candidate & rhs)
    {
      return (distance > rhs.distance);
    }
  };

  // create distance maps for every quantization value
  //cv::Mat distance_maps[8];
  //for (int map_index = 0; map_index < 8; ++map_index)
  //{
  //  distance_maps[map_index] = ::cv::Mat::zeros(height, width, CV_8U);
  //}

  MaskMap mask_maps[8];
  for (int map_index = 0; map_index < 8; ++map_index)
    mask_maps[map_index].initialize (width, height);

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

  QuantizedMap distance_map_indices;
  distance_map_indices.initialize (width, height);
  //memset (distance_map_indices.data, 0, sizeof (distance_map_indices.data[0])*width*height);

  for (int row_index = 0; row_index < height; ++row_index)
  {
    for (int col_index = 0; col_index < width; ++col_index)
    {
      if (mask (col_index, row_index) != 0)
      {
        //const unsigned char quantized_value = quantized_surface_normals_ (row_index, col_index);
        const unsigned char quantized_value = filtered_quantized_surface_normals_ (col_index, row_index);

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

  const int off = 4;
  for (int row_index = off; row_index < height-off; ++row_index)
  {
    for (int col_index = off; col_index < width-off; ++col_index)
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
            candidate.bin_index = distance_map_index;

            list1.push_back (candidate);

            ++weights[distance_map_index];
          }
        }
      }
    }
  }

  for (std::list<Candidate>::iterator iter = list1.begin (); iter != list1.end (); ++iter)
    iter->distance *= 1.0f / weights[iter->bin_index];

  list1.sort ();

  if (list1.size() <= nr_features)
  {
    features.reserve (list1.size ());
    for (std::list<Candidate>::iterator iter = list1.begin(); iter != list1.end(); ++iter)
    {
      QuantizedMultiModFeature feature;

      feature.x = iter->x;
      feature.y = iter->y;
      feature.modality_index = modality_index;
      feature.quantized_value = filtered_quantized_surface_normals_ (iter->x, iter->y);

      features.push_back (feature);
    }

    return;
  }

  int distance = list1.size () / nr_features + 1; // ???  @todo:!:!:!:!:!:!
  while (list2.size () != nr_features)
  {
    const int sqr_distance = distance*distance;
    for (std::list<Candidate>::iterator iter1 = list1.begin (); iter1 != list1.end (); ++iter1)
    {
      bool candidate_accepted = true;

      for (std::list<Candidate>::iterator iter2 = list2.begin (); iter2 != list2.end (); ++iter2)
      {
        const float dx = iter1->x - iter2->x;
        const float dy = iter1->y - iter2->y;
        const float tmp_distance = dx*dx + dy*dy;

        if (tmp_distance < sqr_distance)
        {
          candidate_accepted = false;
          break;
        }
      }

      if (candidate_accepted)
        list2.push_back (*iter1);

      if (list2.size() == nr_features) break;
    }
    --distance;
  }

  for (std::list<Candidate>::iterator iter2 = list2.begin(); iter2 != list2.end(); ++iter2)
  {
    QuantizedMultiModFeature feature;

    feature.x = iter2->x;
    feature.y = iter2->y;
    feature.modality_index = modality_index;
    feature.quantized_value = filtered_quantized_surface_normals_ (iter2->x, iter2->y);

    features.push_back (feature);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::quantizeSurfaceNormals ()
{
  const int width = input_->width;
  const int height = input_->height;

  quantized_surface_normals_.initialize (width, height);

  for (int row_index = 0; row_index < height; ++row_index)
  {
    for (int col_index = 0; col_index < width; ++col_index)
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

      float angle = atan2 (normal_y, normal_x)*180.0f/3.14f;

      if (angle < 0.0f) angle += 360.0f;
      if (angle >= 360.0f) angle -= 360.0f;

      int bin_index = static_cast<int> (angle*8.0f/360.0f);

      //quantized_surface_normals_.data[row_index*width+col_index] = 0x1 << bin_index;
      quantized_surface_normals_ (col_index, row_index) = bin_index;
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

  filtered_quantized_surface_normals_.initialize(width, height);

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
        filtered_quantized_surface_normals_ (col_index, row_index) = 0x1 << max_hist_index;
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
template <typename PointInT>void
pcl::SurfaceNormalModality<PointInT>::computeDistanceMap (
  MaskMap & input,
  DistanceMap & output)
{
  const int width = input.getWidth ();
  const int height = input.getHeight ();

  output.initialize (width, height);

  // compute distance map
  //float *distance_map = new float[input_->points.size ()];
  unsigned char *mask_map = input.getData ();
  float *distance_map = output.getData ();
  for (size_t index = 0; index < width*height; ++index)
  {
    if (mask_map[index] == 0)
      distance_map[index] = 0.0f;
    else
      distance_map[index] = width + height;
  }

  // first pass
  float* previous_row = distance_map;
  float* current_row = previous_row + width;
  for (size_t ri = 1; ri < height; ++ri)
  {
    for (size_t ci = 1; ci < width; ++ci)
    {
      const float up_left  = previous_row [ci - 1] + 1.4f; //distance_map[(ri-1)*input_->width + ci-1] + 1.4f;
      const float up      = previous_row [ci] + 1.0f;     //distance_map[(ri-1)*input_->width + ci] + 1.0f;
      const float up_right = previous_row [ci + 1] + 1.4f; //distance_map[(ri-1)*input_->width + ci+1] + 1.4f;
      const float left    = current_row  [ci - 1] + 1.0f;  //distance_map[ri*input_->width + ci-1] + 1.0f;
      const float center  = current_row  [ci];             //distance_map[ri*input_->width + ci];

      const float min_value = std::min (std::min (up_left, up), std::min (left, up_right));

      if (min_value < center)
        current_row [ci] = min_value; //distance_map[ri * input_->width + ci] = min_value;
    }
    previous_row = current_row;
    current_row += width;
  }

  float* next_row    = distance_map + width * (height - 1);
  current_row = next_row - width;
  // second pass
  for (int ri = height-2; ri >= 0; --ri)
  {
    for (int ci = width-2; ci >= 0; --ci)
    {
      const float lower_left  = next_row [ci - 1] + 1.4f;    //distance_map[(ri+1)*input_->width + ci-1] + 1.4f;
      const float lower      = next_row [ci] + 1.0f;        //distance_map[(ri+1)*input_->width + ci] + 1.0f;
      const float lower_right = next_row [ci + 1] + 1.4f;    //distance_map[(ri+1)*input_->width + ci+1] + 1.4f;
      const float right      = current_row [ci + 1] + 1.0f; //distance_map[ri*input_->width + ci+1] + 1.0f;
      const float center     = current_row [ci];            //distance_map[ri*input_->width + ci];

      const float min_value = std::min (std::min (lower_left, lower), std::min (right, lower_right));

      if (min_value < center)
        current_row [ci] = min_value; //distance_map[ri*input_->width + ci] = min_value;
    }
  }
}


#endif    // PCL_FEATURES_SURFACE_NORMAL_MODALITY
