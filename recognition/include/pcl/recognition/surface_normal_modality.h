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
#include <pcl/features/linear_least_squares_normal.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
// #include <iostream>
#include <limits>
#include <list>
#include <vector>

namespace pcl
{

  /** \brief Map that stores orientations.
    * \author Stefan Holzer
    */
  struct PCL_EXPORTS LINEMOD_OrientationMap
  {
    public:
      /** \brief Constructor. */
      inline LINEMOD_OrientationMap () : width_ (0), height_ (0) {}
      /** \brief Destructor. */
      inline ~LINEMOD_OrientationMap () = default;

      /** \brief Returns the width of the modality data map. */
      inline std::size_t
      getWidth () const
      {
        return width_;
      }

      /** \brief Returns the height of the modality data map. */
      inline std::size_t
      getHeight () const
      {
        return height_;
      }

      /** \brief Resizes the map to the specific width and height and initializes 
        *        all new elements with the specified value.
        * \param[in] width the width of the resized map.
        * \param[in] height the height of the resized map.
        * \param[in] value the value all new elements will be initialized with.
        */
      inline void
      resize (const std::size_t width, const std::size_t height, const float value)
      {
        width_ = width;
        height_ = height;
        map_.clear ();
        map_.resize (width*height, value);
      }

      /** \brief Operator to access elements of the map. 
        * \param[in] col_index the column index of the element to access.
        * \param[in] row_index the row index of the element to access.
        */
      inline float &
      operator() (const std::size_t col_index, const std::size_t row_index)
      {
        return map_[row_index * width_ + col_index];
      }

      /** \brief Operator to access elements of the map. 
        * \param[in] col_index the column index of the element to access.
        * \param[in] row_index the row index of the element to access.
        */
      inline const float &
      operator() (const std::size_t col_index, const std::size_t row_index) const
      {
        return map_[row_index * width_ + col_index];
      }

    private:
      /** \brief The width of the map. */
      std::size_t width_;
      /** \brief The height of the map. */
      std::size_t height_;
      /** \brief Storage for the data of the map. */
      std::vector<float> map_;
  
  };

  /** \brief Look-up-table for fast surface normal quantization.
    * \author Stefan Holzer
    */
  struct QuantizedNormalLookUpTable
  {
    /** \brief The range of the LUT in x-direction. */
    int range_x;
    /** \brief The range of the LUT in y-direction. */
    int range_y;
    /** \brief The range of the LUT in z-direction. */
    int range_z;

    /** \brief The offset in x-direction. */
    int offset_x;
    /** \brief The offset in y-direction. */
    int offset_y;
    /** \brief The offset in z-direction. */
    int offset_z;

    /** \brief The size of the LUT in x-direction. */
    int size_x;
    /** \brief The size of the LUT in y-direction. */
    int size_y;
    /** \brief The size of the LUT in z-direction. */
    int size_z;

    /** \brief The LUT data. */
    unsigned char * lut;

    /** \brief Constructor. */
    QuantizedNormalLookUpTable () : 
      range_x (-1), range_y (-1), range_z (-1), 
      offset_x (-1), offset_y (-1), offset_z (-1), 
      size_x (-1), size_y (-1), size_z (-1), lut (nullptr) 
    {}

    /** \brief Destructor. */
    ~QuantizedNormalLookUpTable () 
    { 
      delete[] lut; 
    }

    /** \brief Initializes the LUT.
      * \param[in] range_x_arg the range of the LUT in x-direction.
      * \param[in] range_y_arg the range of the LUT in y-direction.
      * \param[in] range_z_arg the range of the LUT in z-direction.
      */
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

      delete[] lut;
      lut = new unsigned char[size_x*size_y*size_z];

      const int nr_normals = 8;
	    pcl::PointCloud<PointXYZ>::VectorType ref_normals (nr_normals);
      
      const float normal0_angle = 40.0f * 3.14f / 180.0f;
      ref_normals[0].x = std::cos (normal0_angle);
      ref_normals[0].y = 0.0f;
      ref_normals[0].z = -sinf (normal0_angle);

      const float inv_nr_normals = 1.0f / static_cast<float> (nr_normals);
      for (int normal_index = 1; normal_index < nr_normals; ++normal_index)
      {
        const float angle = 2.0f * static_cast<float> (M_PI * normal_index * inv_nr_normals);

        ref_normals[normal_index].x = std::cos (angle) * ref_normals[0].x - sinf (angle) * ref_normals[0].y;
        ref_normals[normal_index].y = sinf (angle) * ref_normals[0].x + std::cos (angle) * ref_normals[0].y;
        ref_normals[normal_index].z = ref_normals[0].z;
      }

      // normalize normals
      for (int normal_index = 0; normal_index < nr_normals; ++normal_index)
      {
        const float length = std::sqrt (ref_normals[normal_index].x * ref_normals[normal_index].x +
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
            const float length = std::sqrt (normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
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

              const float abs_response = std::abs (response);
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

    /** \brief Operator to access an element in the LUT.
      * \param[in] x the x-component of the normal.
      * \param[in] y the y-component of the normal.
      * \param[in] z the z-component of the normal. 
      */
    inline unsigned char 
    operator() (const float x, const float y, const float z) const
    {
      const auto x_index = static_cast<std::size_t> (x * static_cast<float> (offset_x) + static_cast<float> (offset_x));
      const auto y_index = static_cast<std::size_t> (y * static_cast<float> (offset_y) + static_cast<float> (offset_y));
      const auto z_index = static_cast<std::size_t> (z * static_cast<float> (range_z) + static_cast<float> (range_z));

      const std::size_t index = z_index*size_y*size_x + y_index*size_x + x_index;

      return (lut[index]);
    }

    /** \brief Operator to access an element in the LUT.
      * \param[in] index the index of the element. 
      */
    inline unsigned char 
    operator() (const int index) const
    {
      return (lut[index]);
    }
  };


  /** \brief Modality based on surface normals.
    * \author Stefan Holzer
    * \ingroup recognition
    */
  template <typename PointInT>
  class SurfaceNormalModality : public QuantizableModality, public PCLBase<PointInT>
  {
    protected:
      using PCLBase<PointInT>::input_;

      /** \brief Candidate for a feature (used in feature extraction methods). */
      struct Candidate
      {
        /** \brief Constructor. */
        Candidate () : distance (0.0f), bin_index (0), x (0), y (0) {}

        /** \brief Normal. */
        Normal normal;
        /** \brief Distance to the next different quantized value. */
        float distance;

        /** \brief Quantized value. */
        unsigned char bin_index;
    
        /** \brief x-position of the feature. */
        std::size_t x;
        /** \brief y-position of the feature. */
        std::size_t y;	

        /** \brief Compares two candidates based on their distance to the next different quantized value. 
          * \param[in] rhs the candidate to compare with. 
          */
        bool 
        operator< (const Candidate & rhs) const
        {
          return (distance > rhs.distance);
        }
      };

    public:
      using PointCloudIn = pcl::PointCloud<PointInT>;

      /** \brief Constructor. */
      SurfaceNormalModality ();
      /** \brief Destructor. */
      ~SurfaceNormalModality () override;

      /** \brief Sets the spreading size.
        * \param[in] spreading_size the spreading size.
        */
      inline void
      setSpreadingSize (const std::size_t spreading_size)
      {
        spreading_size_ = spreading_size;
      }

      /** \brief Enables/disables the use of extracting a variable number of features.
        * \param[in] enabled specifies whether extraction of a variable number of features will be enabled/disabled.
        */
      inline void
      setVariableFeatureNr (const bool enabled)
      {
        variable_feature_nr_ = enabled;
      }

      /** \brief Returns the surface normals. */
      inline pcl::PointCloud<pcl::Normal> &
      getSurfaceNormals ()
      {
        return surface_normals_;
      }

      /** \brief Returns the surface normals. */
      inline const pcl::PointCloud<pcl::Normal> &
      getSurfaceNormals () const
      {
        return surface_normals_;
      }

      /** \brief Returns a reference to the internal quantized map. */
      inline QuantizedMap &
      getQuantizedMap () override 
      { 
        return (filtered_quantized_surface_normals_); 
      }

      /** \brief Returns a reference to the internal spread quantized map. */
      inline QuantizedMap &
      getSpreadedQuantizedMap () override 
      { 
        return (spreaded_quantized_surface_normals_); 
      }

      /** \brief Returns a reference to the orientation map. */
      inline LINEMOD_OrientationMap &
      getOrientationMap ()
      {
        return (surface_normal_orientations_);
      }

      /** \brief Extracts features from this modality within the specified mask.
        * \param[in] mask defines the areas where features are searched in. 
        * \param[in] nr_features defines the number of features to be extracted 
        *            (might be less if not sufficient information is present in the modality).
        * \param[in] modality_index the index which is stored in the extracted features.
        * \param[out] features the destination for the extracted features.
        */
      void 
      extractFeatures (const MaskMap & mask, std::size_t nr_features, std::size_t modality_index,
                       std::vector<QuantizedMultiModFeature> & features) const override;

      /** \brief Extracts all possible features from the modality within the specified mask.
        * \param[in] mask defines the areas where features are searched in. 
        * \param[in] nr_features IGNORED (TODO: remove this parameter).
        * \param[in] modality_index the index which is stored in the extracted features.
        * \param[out] features the destination for the extracted features.
        */
      void 
      extractAllFeatures (const MaskMap & mask, std::size_t nr_features, std::size_t modality_index,
                          std::vector<QuantizedMultiModFeature> & features) const override;

      /** \brief Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method)
        * \param[in] cloud the const boost shared pointer to a PointCloud message
        */
      void 
      setInputCloud (const typename PointCloudIn::ConstPtr & cloud) override 
      { 
        input_ = cloud;
      }

      /** \brief Processes the input data (smoothing, computing gradients, quantizing, filtering, spreading). */
      virtual void
      processInputData ();

      /** \brief Processes the input data assuming that everything up to filtering is already done/available 
        *        (so only spreading is performed). */
      virtual void
      processInputDataFromFiltered ();

  protected:

      /** \brief Computes the surface normals from the input cloud. */
      void
      computeSurfaceNormals ();

      /** \brief Computes and quantizes the surface normals. */
      void
      computeAndQuantizeSurfaceNormals ();

      /** \brief Computes and quantizes the surface normals. */
      void
      computeAndQuantizeSurfaceNormals2 ();

      /** \brief Quantizes the surface normals. */
      void
      quantizeSurfaceNormals ();

      /** \brief Filters the quantized surface normals. */
      void
      filterQuantizedSurfaceNormals ();

      /** \brief Computes a distance map from the supplied input mask. 
        * \param[in] input the mask for which a distance map will be computed.
        * \param[out] output the destination for the distance map. 
        */
      void
      computeDistanceMap (const MaskMap & input, DistanceMap & output) const;

    private:

      /** \brief Determines whether variable numbers of features are extracted or not. */
      bool variable_feature_nr_;

      /** \brief The feature distance threshold. */
      float feature_distance_threshold_;
      /** \brief Minimum distance of a feature to a border. */
      float min_distance_to_border_;

      /** \brief Look-up-table for quantizing surface normals. */
      QuantizedNormalLookUpTable normal_lookup_;

      /** \brief The spreading size. */
      std::size_t spreading_size_;

      /** \brief Point cloud holding the computed surface normals. */
      pcl::PointCloud<pcl::Normal> surface_normals_;
      /** \brief Quantized surface normals. */
      pcl::QuantizedMap quantized_surface_normals_;
      /** \brief Filtered quantized surface normals. */
      pcl::QuantizedMap filtered_quantized_surface_normals_;
      /** \brief Spread quantized surface normals. */
      pcl::QuantizedMap spreaded_quantized_surface_normals_;

      /** \brief Map containing surface normal orientations. */
      pcl::LINEMOD_OrientationMap surface_normal_orientations_;

  };

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::SurfaceNormalModality<PointInT>::
SurfaceNormalModality ()
  : variable_feature_nr_ (false)
  , feature_distance_threshold_ (2.0f)
  , min_distance_to_border_ (2.0f)
  , spreading_size_ (8)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::SurfaceNormalModality<PointInT>::~SurfaceNormalModality () = default;

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::processInputData ()
{
  // compute surface normals
  //computeSurfaceNormals ();

  // quantize surface normals
  //quantizeSurfaceNormals ();

  computeAndQuantizeSurfaceNormals2 ();

  // filter quantized surface normals
  filterQuantizedSurfaceNormals ();

  // spread quantized surface normals
  pcl::QuantizedMap::spreadQuantizedMap (filtered_quantized_surface_normals_,
                                         spreaded_quantized_surface_normals_,
                                         spreading_size_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::processInputDataFromFiltered ()
{
  // spread quantized surface normals
  pcl::QuantizedMap::spreadQuantizedMap (filtered_quantized_surface_normals_,
                                         spreaded_quantized_surface_normals_,
                                         spreading_size_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::computeSurfaceNormals ()
{
  // compute surface normals
  pcl::LinearLeastSquaresNormalEstimation<PointInT, pcl::Normal> ne;
  ne.setMaxDepthChangeFactor(0.05f);
  ne.setNormalSmoothingSize(5.0f);
  ne.setDepthDependentSmoothing(false);
  ne.setInputCloud (input_);
  ne.compute (surface_normals_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::computeAndQuantizeSurfaceNormals ()
{
  // compute surface normals
  //pcl::LinearLeastSquaresNormalEstimation<PointInT, pcl::Normal> ne;
  //ne.setMaxDepthChangeFactor(0.05f);
  //ne.setNormalSmoothingSize(5.0f);
  //ne.setDepthDependentSmoothing(false);
  //ne.setInputCloud (input_);
  //ne.compute (surface_normals_);


  const float bad_point = std::numeric_limits<float>::quiet_NaN ();

  const int width = input_->width;
  const int height = input_->height;

  surface_normals_.resize (width*height);
  surface_normals_.width = width;
  surface_normals_.height = height;
  surface_normals_.is_dense = false;

  quantized_surface_normals_.resize (width, height);

  // we compute the normals as follows:
  // ----------------------------------
  // 
  // for the depth-gradient you can make the following first-order Taylor approximation:
  //   D(x + dx) - D(x) = dx^T \Delta D + h.o.t.
  //     
  // build linear system by stacking up equation for 8 neighbor points:
  //   Y = X \Delta D
  // 
  // => \Delta D = (X^T X)^{-1} X^T Y
  // => \Delta D = (A)^{-1} b

  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      const int index = y * width + x;

      const float px = (*input_)[index].x;
      const float py = (*input_)[index].y;
      const float pz = (*input_)[index].z;

      if (std::isnan(px) || pz > 2.0f) 
      {
        surface_normals_[index].normal_x = bad_point;
        surface_normals_[index].normal_y = bad_point;
        surface_normals_[index].normal_z = bad_point;
        surface_normals_[index].curvature = bad_point;

        quantized_surface_normals_ (x, y) = 0;

        continue;
      }

      const int smoothingSizeInt = 5;

      float matA0 = 0.0f;
      float matA1 = 0.0f;
      float matA3 = 0.0f;

      float vecb0 = 0.0f;
      float vecb1 = 0.0f;

      for (int v = y - smoothingSizeInt; v <= y + smoothingSizeInt; v += smoothingSizeInt)
      {
        for (int u = x - smoothingSizeInt; u <= x + smoothingSizeInt; u += smoothingSizeInt)
        {
          if (u < 0 || u >= width || v < 0 || v >= height) continue;

          const std::size_t index2 = v * width + u;

          const float qx = (*input_)[index2].x;
          const float qy = (*input_)[index2].y;
          const float qz = (*input_)[index2].z;

          if (std::isnan(qx)) continue;

          const float delta = qz - pz;
          const float i = qx - px;
          const float j = qy - py;

          const float f = std::abs(delta) < 0.05f ? 1.0f : 0.0f;

          matA0 += f * i * i;
          matA1 += f * i * j;
          matA3 += f * j * j;
          vecb0 += f * i * delta;
          vecb1 += f * j * delta;
        }
      }

      const float det = matA0 * matA3 - matA1 * matA1;
      const float ddx = matA3 * vecb0 - matA1 * vecb1;
      const float ddy = -matA1 * vecb0 + matA0 * vecb1;

      const float nx = ddx;
      const float ny = ddy;
      const float nz = -det * pz;

      const float length = nx * nx + ny * ny + nz * nz;

      if (length <= 0.0f)
      {
        surface_normals_[index].normal_x = bad_point;
        surface_normals_[index].normal_y = bad_point;
        surface_normals_[index].normal_z = bad_point;
        surface_normals_[index].curvature = bad_point;

        quantized_surface_normals_ (x, y) = 0;
      }
      else
      {
        const float normInv = 1.0f / std::sqrt (length);

        const float normal_x = nx * normInv;
        const float normal_y = ny * normInv;
        const float normal_z = nz * normInv;

        surface_normals_[index].normal_x = normal_x;
        surface_normals_[index].normal_y = normal_y;
        surface_normals_[index].normal_z = normal_z;
        surface_normals_[index].curvature = bad_point;

        float angle = 11.25f + std::atan2 (normal_y, normal_x)*180.0f/3.14f;

        if (angle < 0.0f) angle += 360.0f;
        if (angle >= 360.0f) angle -= 360.0f;

        int bin_index = static_cast<int> (angle*8.0f/360.0f + 1);
        bin_index = (bin_index < 1) ? 1 : (8 < bin_index) ? 8 : bin_index;

        quantized_surface_normals_ (x, y) = static_cast<unsigned char> (bin_index);
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
// Contains GRANULARITY and NORMAL_LUT
//#include "normal_lut.i"

static void accumBilateral(long delta, long i, long j, long * A, long * b, int threshold)
{
  long f = std::abs(delta) < threshold ? 1 : 0;

  const long fi = f * i;
  const long fj = f * j;

  A[0] += fi * i;
  A[1] += fi * j;
  A[3] += fj * j;
  b[0]  += fi * delta;
  b[1]  += fj * delta;
}

/**
 * \brief Compute quantized normal image from depth image.
 *
 * Implements section 2.6 "Extension to Dense Depth Sensors."
 *
 * \todo Should also need camera model, or at least focal lengths? Replace distance_threshold with mask?
 */
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::computeAndQuantizeSurfaceNormals2 ()
{
  const int width = input_->width;
  const int height = input_->height;

  auto * lp_depth = new unsigned short[width*height]{};
  auto * lp_normals = new unsigned char[width*height]{};

  surface_normal_orientations_.resize (width, height, 0.0f);

  for (int row_index = 0; row_index < height; ++row_index)
  {
    for (int col_index = 0; col_index < width; ++col_index)
    {
      const float value = (*input_)[row_index*width + col_index].z;
      if (std::isfinite (value))
      {
        lp_depth[row_index*width + col_index] = static_cast<unsigned short> (value * 1000.0f);
      }
      else
      {
        lp_depth[row_index*width + col_index] = 0;
      }
    }
  }

  const int l_W = width;
  const int l_H = height;

  const int l_r = 5; // used to be 7
  //const int l_offset0 = -l_r - l_r * l_W;
  //const int l_offset1 =    0 - l_r * l_W;
  //const int l_offset2 = +l_r - l_r * l_W;
  //const int l_offset3 = -l_r;
  //const int l_offset4 = +l_r;
  //const int l_offset5 = -l_r + l_r * l_W;
  //const int l_offset6 =    0 + l_r * l_W;
  //const int l_offset7 = +l_r + l_r * l_W;

  const int offsets_i[] = {-l_r, 0, l_r, -l_r, l_r, -l_r, 0, l_r};
  const int offsets_j[] = {-l_r, -l_r, -l_r, 0, 0, l_r, l_r, l_r};
  const int offsets[] = { offsets_i[0] + offsets_j[0] * l_W
                        , offsets_i[1] + offsets_j[1] * l_W
                        , offsets_i[2] + offsets_j[2] * l_W
                        , offsets_i[3] + offsets_j[3] * l_W
                        , offsets_i[4] + offsets_j[4] * l_W
                        , offsets_i[5] + offsets_j[5] * l_W
                        , offsets_i[6] + offsets_j[6] * l_W
                        , offsets_i[7] + offsets_j[7] * l_W };


  //const int l_offsetx = GRANULARITY / 2;
  //const int l_offsety = GRANULARITY / 2;

  const int difference_threshold = 50;
  const int distance_threshold = 2000;

  //const double scale = 1000.0;
  //const double difference_threshold = 0.05 * scale;
  //const double distance_threshold = 2.0 * scale;

  for (int l_y = l_r; l_y < l_H - l_r - 1; ++l_y)
  {
    unsigned short * lp_line = lp_depth + (l_y * l_W + l_r);
    unsigned char * lp_norm = lp_normals + (l_y * l_W + l_r);

    for (int l_x = l_r; l_x < l_W - l_r - 1; ++l_x)
    {
      long l_d = lp_line[0];
      //float l_d = (*input_)[(l_y * l_W + l_r) + l_x].z;
      //float px = (*input_)[(l_y * l_W + l_r) + l_x].x;
      //float py = (*input_)[(l_y * l_W + l_r) + l_x].y;

      if (l_d < distance_threshold)
      {
        // accum
        long l_A[4]; l_A[0] = l_A[1] = l_A[2] = l_A[3] = 0;
        long l_b[2]; l_b[0] = l_b[1] = 0;
        //double l_A[4]; l_A[0] = l_A[1] = l_A[2] = l_A[3] = 0;
        //double l_b[2]; l_b[0] = l_b[1] = 0;

        accumBilateral(lp_line[offsets[0]] - l_d, offsets_i[0], offsets_j[0], l_A, l_b, difference_threshold);
        accumBilateral(lp_line[offsets[1]] - l_d, offsets_i[1], offsets_j[1], l_A, l_b, difference_threshold);
        accumBilateral(lp_line[offsets[2]] - l_d, offsets_i[2], offsets_j[2], l_A, l_b, difference_threshold);
        accumBilateral(lp_line[offsets[3]] - l_d, offsets_i[3], offsets_j[3], l_A, l_b, difference_threshold);
        accumBilateral(lp_line[offsets[4]] - l_d, offsets_i[4], offsets_j[4], l_A, l_b, difference_threshold);
        accumBilateral(lp_line[offsets[5]] - l_d, offsets_i[5], offsets_j[5], l_A, l_b, difference_threshold);
        accumBilateral(lp_line[offsets[6]] - l_d, offsets_i[6], offsets_j[6], l_A, l_b, difference_threshold);
        accumBilateral(lp_line[offsets[7]] - l_d, offsets_i[7], offsets_j[7], l_A, l_b, difference_threshold);

        //for (std::size_t index = 0; index < 8; ++index)
        //{
        //  //accumBilateral(lp_line[offsets[index]] - l_d, offsets_i[index], offsets_j[index], l_A, l_b, difference_threshold);

        //  //const long delta = lp_line[offsets[index]] - l_d;
        //  //const long i = offsets_i[index];
        //  //const long j = offsets_j[index];
        //  //long * A = l_A;
        //  //long * b = l_b;
        //  //const int threshold = difference_threshold;

        //  //const long f = std::abs(delta) < threshold ? 1 : 0;

        //  //const long fi = f * i;
        //  //const long fj = f * j;

        //  //A[0] += fi * i;
        //  //A[1] += fi * j;
        //  //A[3] += fj * j;
        //  //b[0] += fi * delta;
        //  //b[1] += fj * delta;


        //  const double delta = 1000.0f * ((*input_)[(l_y * l_W + l_r) + l_x + offsets[index]].z - l_d);
        //  const double i = offsets_i[index];
        //  const double j = offsets_j[index];
        //  //const float i = (*input_)[(l_y * l_W + l_r) + l_x + offsets[index]].x - px;//offsets_i[index];
        //  //const float j = (*input_)[(l_y * l_W + l_r) + l_x + offsets[index]].y - py;//offsets_j[index];
        //  double * A = l_A;
        //  double * b = l_b;
        //  const double threshold = difference_threshold;

        //  const double f = std::fabs(delta) < threshold ? 1.0f : 0.0f;

        //  const double fi = f * i;
        //  const double fj = f * j;

        //  A[0] += fi * i;
        //  A[1] += fi * j;
        //  A[3] += fj * j;
        //  b[0] += fi * delta;
        //  b[1] += fj * delta;
        //}

        //long f = std::abs(delta) < threshold ? 1 : 0;

        //const long fi = f * i;
        //const long fj = f * j;

        //A[0] += fi * i;
        //A[1] += fi * j;
        //A[3] += fj * j;
        //b[0]  += fi * delta;
        //b[1]  += fj * delta;


        // solve
        long l_det =  l_A[0] * l_A[3] - l_A[1] * l_A[1];
        long l_ddx =  l_A[3] * l_b[0] - l_A[1] * l_b[1];
        long l_ddy = -l_A[1] * l_b[0] + l_A[0] * l_b[1];

        /// @todo Magic number 1150 is focal length? This is something like
        /// f in SXGA mode, but in VGA is more like 530.
        float l_nx = static_cast<float>(1150 * l_ddx);
        float l_ny = static_cast<float>(1150 * l_ddy);
        float l_nz = static_cast<float>(-l_det * l_d);

        //// solve
        //double l_det =  l_A[0] * l_A[3] - l_A[1] * l_A[1];
        //double l_ddx =  l_A[3] * l_b[0] - l_A[1] * l_b[1];
        //double l_ddy = -l_A[1] * l_b[0] + l_A[0] * l_b[1];

        ///// @todo Magic number 1150 is focal length? This is something like
        ///// f in SXGA mode, but in VGA is more like 530.
        //const double dummy_focal_length = 1150.0f;
        //double l_nx = l_ddx * dummy_focal_length;
        //double l_ny = l_ddy * dummy_focal_length;
        //double l_nz = -l_det * l_d;

        float l_sqrt = std::sqrt (l_nx * l_nx + l_ny * l_ny + l_nz * l_nz);

        if (l_sqrt > 0)
        {
          float l_norminv = 1.0f / (l_sqrt);

          l_nx *= l_norminv;
          l_ny *= l_norminv;
          l_nz *= l_norminv;

          float angle = 11.25f + std::atan2 (l_ny, l_nx) * 180.0f / 3.14f;

          if (angle < 0.0f) angle += 360.0f;
          if (angle >= 360.0f) angle -= 360.0f;

          int bin_index = static_cast<int> (angle*8.0f/360.0f);

          surface_normal_orientations_ (l_x, l_y) = angle;

          //*lp_norm = std::abs(l_nz)*255;

          //int l_val1 = static_cast<int>(l_nx * l_offsetx + l_offsetx);
          //int l_val2 = static_cast<int>(l_ny * l_offsety + l_offsety);
          //int l_val3 = static_cast<int>(l_nz * GRANULARITY + GRANULARITY);

          //*lp_norm = NORMAL_LUT[l_val3][l_val2][l_val1];
          *lp_norm = static_cast<unsigned char> (0x1 << bin_index);
        }
        else
        {
          *lp_norm = 0; // Discard shadows from depth sensor
        }
      }
      else
      {
        *lp_norm = 0; //out of depth
      }
      ++lp_line;
      ++lp_norm;
    }
  }
  /*cvSmooth(m_dep[0], m_dep[0], CV_MEDIAN, 5, 5);*/

  unsigned char map[255]{};

  map[0x1<<0] = 1;
  map[0x1<<1] = 2;
  map[0x1<<2] = 3;
  map[0x1<<3] = 4;
  map[0x1<<4] = 5;
  map[0x1<<5] = 6;
  map[0x1<<6] = 7;
  map[0x1<<7] = 8;

  quantized_surface_normals_.resize (width, height);
  for (int row_index = 0; row_index < height; ++row_index)
  {
    for (int col_index = 0; col_index < width; ++col_index)
    {
      quantized_surface_normals_ (col_index, row_index) = map[lp_normals[row_index*width + col_index]];
    }
  }

  delete[] lp_depth;
  delete[] lp_normals;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::extractFeatures (const MaskMap & mask,
                                                       const std::size_t nr_features,
                                                       const std::size_t modality_index,
                                                       std::vector<QuantizedMultiModFeature> & features) const
{
  const std::size_t width = mask.getWidth ();
  const std::size_t height = mask.getHeight ();

  //cv::Mat maskImage(height, width, CV_8U, mask.mask);
  //cv::erode(maskImage, maskImage

  // create distance maps for every quantization value
  //cv::Mat distance_maps[8];
  //for (int map_index = 0; map_index < 8; ++map_index)
  //{
  //  distance_maps[map_index] = ::cv::Mat::zeros(height, width, CV_8U);
  //}

  MaskMap mask_maps[8];
  for (auto &mask_map : mask_maps)
    mask_map.resize (width, height);

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

  DistanceMap mask_distance_maps;
  computeDistanceMap (mask, mask_distance_maps);

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
        const unsigned char quantized_value = filtered_quantized_surface_normals_ (col_index, row_index);

        //const float nx = surface_normals_ (col_index, row_index).normal_x;
        //const float ny = surface_normals_ (col_index, row_index).normal_y;
        //const float nz = surface_normals_ (col_index, row_index).normal_z;

        if (quantized_value != 0)// && !(std::isnan (nx) || std::isnan (ny) || std::isnan (nz)))
        {
          const int distance_map_index = map[quantized_value];

          //const float distance = distance_maps[distance_map_index].at<float> (row_index, col_index);
          const float distance = distance_maps[distance_map_index] (col_index, row_index);
          const float distance_to_border = mask_distance_maps (col_index, row_index);

          if (distance >= feature_distance_threshold_ && distance_to_border >= min_distance_to_border_)
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

  for (auto iter = list1.begin (); iter != list1.end (); ++iter)
    iter->distance *= 1.0f / weights[iter->bin_index];

  list1.sort ();

  if (variable_feature_nr_)
  {
    int distance = static_cast<int> (list1.size ());
    bool feature_selection_finished = false;
    while (!feature_selection_finished)
    {
      const int sqr_distance = distance*distance;
      for (auto iter1 = list1.begin (); iter1 != list1.end (); ++iter1)
      {
        bool candidate_accepted = true;
        for (auto iter2 = list2.begin (); iter2 != list2.end (); ++iter2)
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


        float min_min_sqr_distance = std::numeric_limits<float>::max ();
        float max_min_sqr_distance = 0;
        for (auto iter2 = list2.begin (); iter2 != list2.end (); ++iter2)
        {
          float min_sqr_distance = std::numeric_limits<float>::max ();
          for (auto iter3 = list2.begin (); iter3 != list2.end (); ++iter3)
          {
            if (iter2 == iter3)
              continue;

            const float dx = static_cast<float> (iter2->x) - static_cast<float> (iter3->x);
            const float dy = static_cast<float> (iter2->y) - static_cast<float> (iter3->y);

            const float sqr_distance = dx*dx + dy*dy;

            if (sqr_distance < min_sqr_distance)
            {
              min_sqr_distance = sqr_distance;
            }

            //std::cerr << min_sqr_distance;
          }
          //std::cerr << std::endl;

          // check current feature
          {
            const float dx = static_cast<float> (iter2->x) - static_cast<float> (iter1->x);
            const float dy = static_cast<float> (iter2->y) - static_cast<float> (iter1->y);

            const float sqr_distance = dx*dx + dy*dy;

            if (sqr_distance < min_sqr_distance)
            {
              min_sqr_distance = sqr_distance;
            }
          }

          if (min_sqr_distance < min_min_sqr_distance)
            min_min_sqr_distance = min_sqr_distance;
          if (min_sqr_distance > max_min_sqr_distance)
            max_min_sqr_distance = min_sqr_distance;

          //std::cerr << min_sqr_distance << ", " << min_min_sqr_distance << ", " << max_min_sqr_distance << std::endl;
        }

        if (candidate_accepted)
        {
          //std::cerr << "feature_index: " << list2.size () << std::endl;
          //std::cerr << "min_min_sqr_distance: " << min_min_sqr_distance << std::endl;
          //std::cerr << "max_min_sqr_distance: " << max_min_sqr_distance << std::endl;

          if (min_min_sqr_distance < 50)
          {
            feature_selection_finished = true;
            break;
          }

          list2.push_back (*iter1);
        }

        //if (list2.size () == nr_features) 
        //{
        //  feature_selection_finished = true;
        //  break;
        //}
      }
      --distance;
    }
  }
  else
  {
    if (list1.size () <= nr_features)
    {
      features.reserve (list1.size ());
      for (auto iter = list1.begin (); iter != list1.end (); ++iter)
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
      for (auto iter1 = list1.begin (); iter1 != list1.end (); ++iter1)
      {
        bool candidate_accepted = true;

        for (auto iter2 = list2.begin (); iter2 != list2.end (); ++iter2)
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
  }

  for (auto iter2 = list2.begin (); iter2 != list2.end (); ++iter2)
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
pcl::SurfaceNormalModality<PointInT>::extractAllFeatures (
    const MaskMap & mask, const std::size_t, const std::size_t modality_index,
    std::vector<QuantizedMultiModFeature> & features) const
{
  const std::size_t width = mask.getWidth ();
  const std::size_t height = mask.getHeight ();

  //cv::Mat maskImage(height, width, CV_8U, mask.mask);
  //cv::erode(maskImage, maskImage

  // create distance maps for every quantization value
  //cv::Mat distance_maps[8];
  //for (int map_index = 0; map_index < 8; ++map_index)
  //{
  //  distance_maps[map_index] = ::cv::Mat::zeros(height, width, CV_8U);
  //}

  MaskMap mask_maps[8];
  for (auto &mask_map : mask_maps)
    mask_map.resize (width, height);

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

  DistanceMap mask_distance_maps;
  computeDistanceMap (mask, mask_distance_maps);

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
        const unsigned char quantized_value = filtered_quantized_surface_normals_ (col_index, row_index);

        //const float nx = surface_normals_ (col_index, row_index).normal_x;
        //const float ny = surface_normals_ (col_index, row_index).normal_y;
        //const float nz = surface_normals_ (col_index, row_index).normal_z;

        if (quantized_value != 0)// && !(std::isnan (nx) || std::isnan (ny) || std::isnan (nz)))
        {
          const int distance_map_index = map[quantized_value];

          //const float distance = distance_maps[distance_map_index].at<float> (row_index, col_index);
          const float distance = distance_maps[distance_map_index] (col_index, row_index);
          const float distance_to_border = mask_distance_maps (col_index, row_index);

          if (distance >= feature_distance_threshold_ && distance_to_border >= min_distance_to_border_)
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

  for (auto iter = list1.begin (); iter != list1.end (); ++iter)
    iter->distance *= 1.0f / weights[iter->bin_index];

  list1.sort ();

  features.reserve (list1.size ());
  for (auto iter = list1.begin (); iter != list1.end (); ++iter)
  {
    QuantizedMultiModFeature feature;

    feature.x = static_cast<int> (iter->x);
    feature.y = static_cast<int> (iter->y);
    feature.modality_index = modality_index;
    feature.quantized_value = filtered_quantized_surface_normals_ (iter->x, iter->y);

    features.push_back (feature);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::SurfaceNormalModality<PointInT>::quantizeSurfaceNormals ()
{
  const std::size_t width = input_->width;
  const std::size_t height = input_->height;

  quantized_surface_normals_.resize (width, height);

  for (std::size_t row_index = 0; row_index < height; ++row_index)
  {
    for (std::size_t col_index = 0; col_index < width; ++col_index)
    {
      const float normal_x = surface_normals_ (col_index, row_index).normal_x;
      const float normal_y = surface_normals_ (col_index, row_index).normal_y;
      const float normal_z = surface_normals_ (col_index, row_index).normal_z;

      if (std::isnan(normal_x) || std::isnan(normal_y) || std::isnan(normal_z) || normal_z > 0 || (normal_x == 0 && normal_y == 0))
      {
        quantized_surface_normals_ (col_index, row_index) = 0;
        continue;
      }

      //quantized_surface_normals_.data[row_index*width+col_index] =
      //  normal_lookup_(normal_x, normal_y, normal_z);

      float angle = 11.25f + std::atan2 (normal_y, normal_x)*180.0f/3.14f;

      if (angle < 0.0f) angle += 360.0f;
      if (angle >= 360.0f) angle -= 360.0f;

      int bin_index = static_cast<int> (angle*8.0f/360.0f + 1);
      bin_index = (bin_index < 1) ? 1 : (8 < bin_index) ? 8 : bin_index;

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

  //for (int row_index = 2; row_index < height-2; ++row_index)
  //{
  //  for (int col_index = 2; col_index < width-2; ++col_index)
  //  {
  //    std::list<unsigned char> values;
  //    values.reserve (25);

  //    unsigned char * dataPtr = quantized_surface_normals_.getData () + (row_index-2)*width+col_index-2;
  //    values.push_back (dataPtr[0]);
  //    values.push_back (dataPtr[1]);
  //    values.push_back (dataPtr[2]);
  //    values.push_back (dataPtr[3]);
  //    values.push_back (dataPtr[4]);
  //    dataPtr += width;
  //    values.push_back (dataPtr[0]);
  //    values.push_back (dataPtr[1]);
  //    values.push_back (dataPtr[2]);
  //    values.push_back (dataPtr[3]);
  //    values.push_back (dataPtr[4]);
  //    dataPtr += width;
  //    values.push_back (dataPtr[0]);
  //    values.push_back (dataPtr[1]);
  //    values.push_back (dataPtr[2]);
  //    values.push_back (dataPtr[3]);
  //    values.push_back (dataPtr[4]);
  //    dataPtr += width;
  //    values.push_back (dataPtr[0]);
  //    values.push_back (dataPtr[1]);
  //    values.push_back (dataPtr[2]);
  //    values.push_back (dataPtr[3]);
  //    values.push_back (dataPtr[4]);
  //    dataPtr += width;
  //    values.push_back (dataPtr[0]);
  //    values.push_back (dataPtr[1]);
  //    values.push_back (dataPtr[2]);
  //    values.push_back (dataPtr[3]);
  //    values.push_back (dataPtr[4]);

  //    values.sort ();

  //    filtered_quantized_surface_normals_ (col_index, row_index) = values[12];
  //  }
  //}


  //for (int row_index = 2; row_index < height-2; ++row_index)
  //{
  //  for (int col_index = 2; col_index < width-2; ++col_index)
  //  {
  //    filtered_quantized_surface_normals_ (col_index, row_index) = static_cast<unsigned char> (0x1 << (quantized_surface_normals_ (col_index, row_index) - 1));
  //  }
  //}


  // filter data
  for (int row_index = 2; row_index < height-2; ++row_index)
  {
    for (int col_index = 2; col_index < width-2; ++col_index)
    {
      unsigned char histogram[9] = {0,0,0,0,0,0,0,0,0};

      //{
      //  unsigned char * dataPtr = quantized_surface_normals_.getData () + (row_index-1)*width+col_index-1;
      //  ++histogram[dataPtr[0]];
      //  ++histogram[dataPtr[1]];
      //  ++histogram[dataPtr[2]];
      //}
      //{
      //  unsigned char * dataPtr = quantized_surface_normals_.getData () + row_index*width+col_index-1;
      //  ++histogram[dataPtr[0]];
      //  ++histogram[dataPtr[1]];
      //  ++histogram[dataPtr[2]];
      //}
      //{
      //  unsigned char * dataPtr = quantized_surface_normals_.getData () + (row_index+1)*width+col_index-1;
      //  ++histogram[dataPtr[0]];
      //  ++histogram[dataPtr[1]];
      //  ++histogram[dataPtr[2]];
      //}

      {
        unsigned char * dataPtr = quantized_surface_normals_.getData () + (row_index-2)*width+col_index-2;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
        ++histogram[dataPtr[3]];
        ++histogram[dataPtr[4]];
      }
      {
        unsigned char * dataPtr = quantized_surface_normals_.getData () + (row_index-1)*width+col_index-2;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
        ++histogram[dataPtr[3]];
        ++histogram[dataPtr[4]];
      }
      {
        unsigned char * dataPtr = quantized_surface_normals_.getData () + (row_index)*width+col_index-2;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
        ++histogram[dataPtr[3]];
        ++histogram[dataPtr[4]];
      }
      {
        unsigned char * dataPtr = quantized_surface_normals_.getData () + (row_index+1)*width+col_index-2;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
        ++histogram[dataPtr[3]];
        ++histogram[dataPtr[4]];
      }
      {
        unsigned char * dataPtr = quantized_surface_normals_.getData () + (row_index+2)*width+col_index-2;
        ++histogram[dataPtr[0]];
        ++histogram[dataPtr[1]];
        ++histogram[dataPtr[2]];
        ++histogram[dataPtr[3]];
        ++histogram[dataPtr[4]];
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
