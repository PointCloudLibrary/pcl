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

#ifndef PCL_RECOGNITION_GRAYSCALE_GRADIENT_MODALITY
#define PCL_RECOGNITION_GRAYSCALE_GRADIENT_MODALITY

#include <pcl/recognition/quantizable_modality.h>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/recognition/point_types.h>
#include <pcl/filters/convolution.h>

#include <cmath>
#include <list>
#include <vector>

namespace pcl
{

  /** \brief Modality based on max-RGB gradients.
    * \author Stefan Holzer
    */
  template <typename PointInT>
  class GrayscaleGradientModality
    : public QuantizableModality, public PCLBase<PointInT>
  {
    protected:
      using PCLBase<PointInT>::input_;

      /** \brief Candidate for a feature (used in feature extraction methods). */
      struct Candidate
      {
        /** \brief The gradient. */
        //GradientXY gradient;
    
        float angle;
        float magnitude;

        /** \brief The x-position. */
        float x;
        /** \brief The y-position. */
        float y;
    
        /** \brief Operator for comparing to candidates (by magnitude of the gradient).
          * \param[in] rhs the candidate to compare with.
          */
        bool operator< (const Candidate & rhs) const
        {
          return (magnitude > rhs.magnitude);
        }
      };

    public:
      typedef typename pcl::PointCloud<PointInT> PointCloudIn;

      /** \brief Different methods for feature selection/extraction. */
      enum FeatureSelectionMethod
      {
        MASK_BORDER_HIGH_GRADIENTS,
        MASK_BORDER_EQUALLY, // this gives templates most equally to the OpenCV implementation
        DISTANCE_MAGNITUDE_SCORE
      };

      /** \brief Constructor. */
      GrayscaleGradientModality ();
      /** \brief Destructor. */
      virtual ~GrayscaleGradientModality ();
  
      /** \brief Sets the threshold for the gradient magnitude which is used when quantizing the data.
        *        Gradients with a smaller magnitude are ignored. 
        * \param[in] threshold the new gradient magnitude threshold.
        */
      inline void
      setGradientMagnitudeThreshold (const float threshold)
      {
        gradient_magnitude_threshold_ = threshold;
      }

      /** \brief Sets the threshold for the gradient magnitude which is used for feature extraction.
        *        Gradients with a smaller magnitude are ignored. 
        * \param[in] threshold the new gradient magnitude threshold.
        */
      inline void
      setGradientMagnitudeThresholdForFeatureExtraction (const float threshold)
      {
        gradient_magnitude_threshold_feature_extraction_ = threshold;
      }

      /** \brief Sets the feature selection method.
        * \param[in] method the feature selection method.
        */
      inline void
      setFeatureSelectionMethod (const FeatureSelectionMethod method)
      {
        feature_selection_method_ = method;
      }
  
      /** \brief Sets the spreading size for spreading the quantized data. */
      inline void
      setSpreadingSize (const size_t spreading_size)
      {
        spreading_size_ = spreading_size;
      }

      /** \brief Sets whether variable feature numbers for feature extraction is enabled.
        * \param[in] enabled enables/disables variable feature numbers for feature extraction.
        */
      inline void
      setVariableFeatureNr (const bool enabled)
      {
        variable_feature_nr_ = enabled;
      }

      /** \brief Returns a reference to the internally computed quantized map. */
      inline QuantizedMap &
      getQuantizedMap () 
      { 
        return (filtered_quantized_gradients_);
      }
  
      /** \brief Returns a reference to the internally computed spreaded quantized map. */
      inline QuantizedMap &
      getSpreadedQuantizedMap () 
      { 
        return (spreaded_filtered_quantized_gradients_);
      }

      /** \brief Returns a point cloud containing the max-RGB gradients. */
      inline pcl::PointCloud<pcl::GradientXY> &
      getMaxGradients ()
      {
        return (gradients_);
      }
  
      /** \brief Extracts features from this modality within the specified mask.
        * \param[in] mask defines the areas where features are searched in. 
        * \param[in] nr_features defines the number of features to be extracted 
        *            (might be less if not sufficient information is present in the modality).
        * \param[in] modalityIndex the index which is stored in the extracted features.
        * \param[out] features the destination for the extracted features.
        */
      void
      extractFeatures (const MaskMap & mask, size_t nr_features, size_t modalityIndex,
                       std::vector<QuantizedMultiModFeature> & features) const;
  
      /** \brief Extracts all possible features from the modality within the specified mask.
        * \param[in] mask defines the areas where features are searched in. 
        * \param[in] nr_features IGNORED (TODO: remove this parameter).
        * \param[in] modalityIndex the index which is stored in the extracted features.
        * \param[out] features the destination for the extracted features.
        */
      void
      extractAllFeatures (const MaskMap & mask, size_t nr_features, size_t modalityIndex,
                          std::vector<QuantizedMultiModFeature> & features) const;
  
      /** \brief Provide a pointer to the input dataset (overwrites the PCLBase::setInputCloud method)
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      virtual void 
      setInputCloud (const typename PointCloudIn::ConstPtr & cloud) 
      { 
        input_ = cloud;
      }

      /** \brief Processes the input data (smoothing, computing gradients, quantizing, filtering, spreading). */
      // virtual void
      // processInputData ();

      /** \brief Processes the input data assuming that everything up to filtering is already done/available 
        *        (so only spreading is performed). */
      virtual void
      processInputDataFromFiltered ();

      /** \brief Computes the Gaussian kernel used for smoothing. 
        * \param[in] kernel_size the size of the Gaussian kernel. 
        * \param[in] sigma the sigma.
        * \param[out] kernel_values the destination for the values of the kernel. */
      void
      computeGaussianKernel (const size_t kernel_size, const float sigma, std::vector <float> & kernel_values);

      /** \brief Computes the max-RGB gradients for the specified cloud.
        * \param[in] cloud the cloud for which the gradients are computed.
        */
      void
      computeMaxGradients (const typename pcl::PointCloud<pcl::Intensity8u>::ConstPtr & cloud);

      /** \brief Computes the max-RGB gradients for the specified cloud using sobel.
        * \param[in] cloud the cloud for which the gradients are computed.
        */
      void
      computeMaxGradientsSobel (const typename pcl::PointCloud<pcl::Intensity8u>::ConstPtr & cloud);
  
      /** \brief Filters the quantized gradients. */
      void
      filterQuantizedGradients ();

    protected:

      /** \brief Erodes a mask.
        * \param[in] mask_in the mask which will be eroded.
        * \param[out] mask_out the destination for the eroded mask.
        */
      static void
      erode (const pcl::MaskMap & mask_in, pcl::MaskMap & mask_out);

      static uint8_t quantizedAngleFromXY(float x, float y);
  
    private:

      /** \brief Determines whether variable numbers of features are extracted or not. */
      bool variable_feature_nr_;

      /** \brief Stores a smoothed verion of the input cloud. */
      pcl::PointCloud<pcl::Intensity8u>::Ptr smoothed_input_;

      /** \brief Defines which feature selection method is used. */
      FeatureSelectionMethod feature_selection_method_;

      /** \brief The threshold applied on the gradient magnitudes (for quantization). */
      float gradient_magnitude_threshold_;
      /** \brief The threshold applied on the gradient magnitudes for feature extraction. */
      float gradient_magnitude_threshold_feature_extraction_;

      /** \brief The point cloud which holds the max-RGB gradients. */
      pcl::PointCloud<float> gradients_;

      /** \brief The spreading size. */
      size_t spreading_size_;
  
      /** \brief The map which holds the quantized max-RGB gradients. */
      pcl::QuantizedMap quantized_gradients_;
      /** \brief The map which holds the filtered quantized data. */
      pcl::QuantizedMap filtered_quantized_gradients_;
      /** \brief The map which holds the spreaded quantized data. */
      pcl::QuantizedMap spreaded_filtered_quantized_gradients_;
  
  };

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::GrayscaleGradientModality<PointInT>::
GrayscaleGradientModality ()
  : variable_feature_nr_ (false)
  , smoothed_input_ (new pcl::PointCloud<pcl::Intensity8u> ())
  , feature_selection_method_ (DISTANCE_MAGNITUDE_SCORE)
  , gradient_magnitude_threshold_ (10.0f)
  , gradient_magnitude_threshold_feature_extraction_ (55.0f)
  , gradients_ ()
  , spreading_size_ (8)
  , quantized_gradients_ ()
  , filtered_quantized_gradients_ ()
  , spreaded_filtered_quantized_gradients_ ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
pcl::GrayscaleGradientModality<PointInT>::
~GrayscaleGradientModality ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::GrayscaleGradientModality<PointInT>::
computeGaussianKernel (const size_t kernel_size, const float sigma, std::vector <float> & kernel_values)
{
  // code taken from OpenCV
  const int n = int (kernel_size);
  const int SMALL_GAUSSIAN_SIZE = 7;
  static const float small_gaussian_tab[][SMALL_GAUSSIAN_SIZE] =
  {
      {1.f},
      {0.25f, 0.5f, 0.25f},
      {0.0625f, 0.25f, 0.375f, 0.25f, 0.0625f},
      {0.03125f, 0.109375f, 0.21875f, 0.28125f, 0.21875f, 0.109375f, 0.03125f}
  };

  const float* fixed_kernel = n % 2 == 1 && n <= SMALL_GAUSSIAN_SIZE && sigma <= 0 ?
      small_gaussian_tab[n>>1] : 0;

  //CV_Assert( ktype == CV_32F || ktype == CV_64F );
  /*Mat kernel(n, 1, ktype);*/
  kernel_values.resize (n);
  float* cf = &(kernel_values[0]);
  //double* cd = (double*)kernel.data;

  double sigmaX = sigma > 0 ? sigma : ((n-1)*0.5 - 1)*0.3 + 0.8;
  double scale2X = -0.5/(sigmaX*sigmaX);
  double sum = 0;

  int i;
  for( i = 0; i < n; i++ )
  {
    double x = i - (n-1)*0.5;
    double t = fixed_kernel ? double (fixed_kernel[i]) : std::exp (scale2X*x*x);

    cf[i] = float (t);
    sum += cf[i];
  }

  sum = 1./sum;
  for (i = 0; i < n; i++ )
  {
    cf[i] = float (cf[i]*sum);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::GrayscaleGradientModality<PointInT>::
processInputDataFromFiltered ()
{
  // spread filtered quantized gradients
  //spreadFilteredQunatizedColorGradients ();
  pcl::QuantizedMap::spreadQuantizedMap (filtered_quantized_gradients_,
                                         spreaded_filtered_quantized_gradients_, 
                                         spreading_size_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void pcl::GrayscaleGradientModality<PointInT>::
extractFeatures (const MaskMap & mask, const size_t nr_features, const size_t modality_index,
                 std::vector<QuantizedMultiModFeature> & features) const
{
  const size_t width = mask.getWidth ();
  const size_t height = mask.getHeight ();
  
  std::vector<Candidate> candidateVec1;
  std::vector<Candidate> candidateVec2;
  candidateVec2.reserve(nr_features);

  if (feature_selection_method_ == DISTANCE_MAGNITUDE_SCORE)
  {
    double start = getTickCount();

    for (size_t row_index = 0; row_index < height; ++row_index)
    {
      for (size_t col_index = 0; col_index < width; ++col_index)
      {
        if (mask (col_index, row_index) != 0)
        {
          const float & gradient_mag = gradients_ (col_index, row_index);
          if (gradient_mag > gradient_magnitude_threshold_feature_extraction_
            && filtered_quantized_gradients_ (col_index, row_index) != 0)
          {
            Candidate candidate;
            candidate.magnitude = gradient_mag;
            candidate.x = static_cast<float> (col_index);
            candidate.y = static_cast<float> (row_index);

            candidateVec1.push_back (candidate);
          }
        }
      }
    }

    //candidateVec1.sort();
    std::sort(candidateVec1.begin(), candidateVec1.end());

    // printf("size %d\n", candidateVec1.size());

    // printf("1 %f\n", 1000.0*(getTickCount()-start)/1e9);
    // start = getTickCount();

    if (variable_feature_nr_)
    {
      candidateVec2.push_back (candidateVec1[0]);
      //while (candidateVec2.size () != nr_features)
      bool feature_selection_finished = false;
      while (!feature_selection_finished)
      {
        float best_score = 0.0f;
        uint32_t best_index = std::numeric_limits<uint32_t>::max();
        for (size_t id1 = 0; id1 < candidateVec1.size(); ++id1)
        {
          // find smallest distance
          float smallest_distance = std::numeric_limits<float>::max ();
          for (size_t id2 = 0; id2 < candidateVec2.size(); ++id2)
          {
            const float dx = candidateVec1[id1].x - candidateVec2[id2].x;
            const float dy = candidateVec1[id1].y - candidateVec2[id2].y;

            const float distance = dx*dx + dy*dy;

            if (distance < smallest_distance)
            {
              smallest_distance = distance;
            }
          }

          const float score = smallest_distance * candidateVec1[id1].magnitude;

          if (score > best_score)
          {
            best_score = score;
            best_index = id1;
          }
        }


        float min_min_sqr_distance = std::numeric_limits<float>::max ();
        float max_min_sqr_distance = 0;
        //for (typename std::list<Candidate>::iterator iter2 = candidateVec2.begin (); iter2 != candidateVec2.end (); ++iter2)
        for (size_t id2 = 0; id2 < candidateVec2.size(); ++id2)
        {
          float min_sqr_distance = std::numeric_limits<float>::max ();
          //for (typename std::list<Candidate>::iterator iter3 = candidateVec2.begin (); iter3 != candidateVec2.end (); ++iter3)
          for (size_t id3 = 0; id3 < candidateVec2.size(); ++id3)
          {
            if (id2 == id3)
              continue;

            const float dx = candidateVec2[id2].x - candidateVec2[id3].x;
            const float dy = candidateVec2[id2].y - candidateVec2[id3].y;

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
            const float dx = candidateVec2[id2].x - candidateVec1[best_index].x;
            const float dy = candidateVec2[id2].x - candidateVec1[best_index].y;

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

        if (best_index != std::numeric_limits<uint32_t>::max())
        {
          //std::cerr << "feature_index: " << candidateVec2.size () << std::endl;
          //std::cerr << "min_min_sqr_distance: " << min_min_sqr_distance << std::endl;
          //std::cerr << "max_min_sqr_distance: " << max_min_sqr_distance << std::endl;

          if (min_min_sqr_distance < 50)
          {
            feature_selection_finished = true;
            break;
          }

          candidateVec2.push_back(candidateVec1[best_index]);
        }
      } 
    }
    else
    {
      if (candidateVec1.size () <= nr_features)
      {
        //for (typename std::list<Candidate>::iterator iter1 = candidateVec1.begin (); iter1 != candidateVec1.end (); ++iter1)
        for (size_t id1 = 0; id1 < candidateVec1.size(); ++id1)
        {
          QuantizedMultiModFeature feature;
          const Candidate &c = candidateVec1[id1];

          feature.x = c.x;
          feature.y = c.y;
          feature.modality_index = modality_index;
          feature.quantized_value = filtered_quantized_gradients_ (c.x, c.y);

          features.push_back (feature);
        }
        return;
      }
      // printf("2 %f\n", 1000.0*(getTickCount()-start)/1e9);
      // start = getTickCount();

      candidateVec2.push_back (candidateVec1[0]);
      // printf("2.1 %f\n", 1000.0*(getTickCount()-start)/1e9);
      // start = getTickCount();
      // printf("size before%zu\n", candidateVec2.size());
      std::vector<float> candidateVec1_smallest_dist(candidateVec1.size(), std::numeric_limits<float>::max());
      while (candidateVec2.size () != nr_features)
      {
        float best_score = 0.0f;
        //typename std::list<Candidate>::iterator best_iter = candidateVec1.end ();
        uint32_t best_index = std::numeric_limits<uint32_t>::max();

        const size_t id2 = candidateVec2.size() - 1; // candidateVec2.size() is always >= 1
        size_t id1 = 0;

#if __AVX2__
        uint32_t best_index8[8] __attribute__((aligned(32)));
        float best_score8[8] __attribute__((aligned(32)));

        // TODO remove
        assert(sizeof(Candidate) == 4 * sizeof(float));

        const __m256 __candidateVec2_xy = _mm256_set_ps(
          candidateVec2[id2].y, candidateVec2[id2].x,
          candidateVec2[id2].y, candidateVec2[id2].x,
          candidateVec2[id2].y, candidateVec2[id2].x,
          candidateVec2[id2].y, candidateVec2[id2].x);

        __m256i __best_index8 = _mm256_set1_epi32(best_index);
        __m256 __best_score8 = _mm256_set1_ps(best_score);
        for (; id1 <= candidateVec1.size() - 8; id1+=8)
        {
          __m256 __candidateVec1_c01 = _mm256_loadu_ps((const float*) &candidateVec1[id1]); // Loading 2 candidates at a time
          __m256 __candidateVec1_c23 = _mm256_loadu_ps((const float*) &candidateVec1[id1 + 2]);
          __m256 __candidateVec1_c45 = _mm256_loadu_ps((const float*) &candidateVec1[id1 + 4]);
          __m256 __candidateVec1_c67 = _mm256_loadu_ps((const float*) &candidateVec1[id1 + 6]);

          // Order = 0 2 4 6 1 3 5 7
          __m256 __magnitude8 = _mm256_shuffle_ps(
              _mm256_shuffle_ps(__candidateVec1_c01, __candidateVec1_c23, (1 << 6) + (1 << 4) + (1 << 2) + 1), // 0 2 1 3 0 2 1 3
              _mm256_shuffle_ps(__candidateVec1_c45, __candidateVec1_c67, (1 << 6) + (1 << 4) + (1 << 2) + 1), // 4 6 5 7 4 6 5 7
              (2 << 6) + (0 << 4) + (2 << 2) + 0);

          __m256 __candidateVec1_xy_0213 = _mm256_shuffle_ps(__candidateVec1_c01, __candidateVec1_c23, (3 << 6) + (2 << 4) + (3 << 2) + 2);
          __m256 __candidateVec1_xy_4657 = _mm256_shuffle_ps(__candidateVec1_c45, __candidateVec1_c67, (3 << 6) + (2 << 4) + (3 << 2) + 2);

          __m256 __xy_sqr_0213 = _mm256_sub_ps(__candidateVec1_xy_0213, __candidateVec2_xy);
          __xy_sqr_0213 = _mm256_mul_ps(__xy_sqr_0213, __xy_sqr_0213);
          __m256 __xy_sqr_4657 = _mm256_sub_ps(__candidateVec1_xy_4657, __candidateVec2_xy);
          __xy_sqr_4657 = _mm256_mul_ps(__xy_sqr_4657, __xy_sqr_4657);

          // Order: 0 4 1 5 2 6 3 7
          __m256 __dist_8 = _mm256_hadd_ps(__xy_sqr_0213, __xy_sqr_4657);

          __m256 __smallest_distance8 = _mm256_loadu_ps(&candidateVec1_smallest_dist[id1]);
          __smallest_distance8 = _mm256_min_ps(__smallest_distance8, __dist_8);
          _mm256_storeu_ps(&candidateVec1_smallest_dist[id1], __smallest_distance8);

          __m256 __score8 = _mm256_mul_ps(__smallest_distance8, __magnitude8);
          __m256 __update_score_mask = _mm256_cmp_ps(__score8, __best_score8, _CMP_GT_OQ);

          __best_index8 = _mm256_blendv_epi8(__best_index8, 
            _mm256_set_epi32(id1 + 7, id1 + 5, id1 + 3, id1 + 1, id1 + 6, id1 + 4, id1 + 2, id1),
            _mm256_castps_si256(__update_score_mask));

          __best_score8 = _mm256_blendv_ps(__best_score8, __score8, __update_score_mask);
        }

        _mm256_store_ps(best_score8, __best_score8);
        _mm256_store_si256((__m256i*) best_index8, __best_index8);

        for (size_t i = 0; i < 8; ++i) {
          if (best_score8[i] > best_score)
          {
            best_score = best_score8[i];
            best_index = best_index8[i];
            //printf("best %f %u\n", best_score, best_index);
          }
        }
#endif
        for (; id1 < candidateVec1.size(); ++id1)
        {
          // find smallest distance
          //float smallest_distance = std::numeric_limits<float>::max ();
          //for (typename std::list<Candidate>::iterator iter2 = candidateVec2.begin (); iter2 != candidateVec2.end (); ++iter2)
          const float dx = static_cast<float> (candidateVec1[id1].x) - static_cast<float> (candidateVec2[id2].x);
          const float dy = static_cast<float> (candidateVec1[id1].y) - static_cast<float> (candidateVec2[id2].y);

          const float distance = dx*dx + dy*dy;

          if (distance < candidateVec1_smallest_dist[id1])
          {
            candidateVec1_smallest_dist[id1] = distance;
          }

          const float score = candidateVec1_smallest_dist[id1] * candidateVec1[id1].magnitude;

          //printf("%f %f %f %d\n", candidateVec1_smallest_dist[id1], candidateVec1[id1].magnitude, score, id1);
          //printf("best %f %u\n", best_score, best_index);
          if (score > best_score)
          {
            best_score = score;
            best_index = id1;
            //printf("best- %d %d\n", best_score, best_index);
          }
          // if (id1 == 8) {
          //   std::exit(-1);
          // }
        }

        if (best_index != std::numeric_limits<uint32_t>::max())
        {
          candidateVec2.push_back (candidateVec1[best_index]);
        }
        else
        {
          break;
        }
      }
    }
  }
  /*else if (feature_selection_method_ == MASK_BORDER_HIGH_GRADIENTS || feature_selection_method_ == MASK_BORDER_EQUALLY)
  {
    MaskMap eroded_mask;
    erode (mask, eroded_mask);

    MaskMap diff_mask;
    MaskMap::getDifferenceMask (mask, eroded_mask, diff_mask);

    for (size_t row_index = 0; row_index < height; ++row_index)
    {
      for (size_t col_index = 0; col_index < width; ++col_index)
      {
        if (diff_mask (col_index, row_index) != 0)
        {
          const GradientXY & gradient = color_gradients_ (col_index, row_index);
          if ((feature_selection_method_ == MASK_BORDER_EQUALLY || gradient.magnitude > gradient_magnitude_threshold_feature_extraction_)
            && filtered_quantized_color_gradients_ (col_index, row_index) != 0)
          {
            Candidate candidate;
            candidate.gradient = gradient;
            candidate.x = static_cast<float> (col_index);
            candidate.y = static_cast<float> (row_index);

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
      const size_t sqr_distance = distance*distance;
      for (typename std::list<Candidate>::iterator iter1 = list1.begin (); iter1 != list1.end (); ++iter1)
      {
        bool candidate_accepted = true;

        for (typename std::list<Candidate>::iterator iter2 = list2.begin (); iter2 != list2.end (); ++iter2)
        {
          const int dx = iter1->x - iter2->x;
          const int dy = iter1->y - iter2->y;
          const unsigned int tmp_distance = dx*dx + dy*dy;

          //if (tmp_distance < distance) 
          if (tmp_distance < sqr_distance)
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
  }*/

  for (size_t id2 = 0; id2 < candidateVec2.size(); ++id2)
  {
    QuantizedMultiModFeature feature;

    const Candidate &c = candidateVec2[id2];
    feature.x = c.x;
    feature.y = c.y;
    feature.modality_index = modality_index;
    feature.quantized_value = filtered_quantized_gradients_ (c.x, c.y);

    features.push_back (feature);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void 
pcl::GrayscaleGradientModality<PointInT>::
extractAllFeatures (const MaskMap & mask, const size_t, const size_t modality_index,
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
        const float& gradient_mag = gradients_ (col_index, row_index);
        if (gradient_mag > gradient_magnitude_threshold_feature_extraction_
          && filtered_quantized_gradients_ (col_index, row_index) != 0)
        {
          Candidate candidate;
          candidate.magnitude= gradient_mag;
          candidate.x = static_cast<float> (col_index);
          candidate.y = static_cast<float> (row_index);

          list1.push_back (candidate);
        }
      }
    }
  }

  list1.sort();

  for (typename std::list<Candidate>::iterator iter1 = list1.begin (); iter1 != list1.end (); ++iter1)
  {
    QuantizedMultiModFeature feature;
          
    feature.x = iter1->x;
    feature.y = iter1->y;
    feature.modality_index = modality_index;
    feature.quantized_value = filtered_quantized_gradients_ (iter1->x, iter1->y);

    features.push_back (feature);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::GrayscaleGradientModality<PointInT>::
computeMaxGradients (const typename pcl::PointCloud<pcl::Intensity8u>::ConstPtr & cloud)
{
  const int width = cloud->width;
  const int height = cloud->height;

  gradients_.points.resize (width*height);
  gradients_.width = width;
  gradients_.height = height;

  const float pi = tan (1.0f) * 2;
  for (int row_index = 0; row_index < height-2; ++row_index)
  {
    for (int col_index = 0; col_index < width-2; ++col_index)
    {
      const int index0 = row_index*width+col_index;
      const int index_c = row_index*width+col_index+2;
      const int index_r = (row_index+2)*width+col_index;

      //const int index_d = (row_index+1)*width+col_index+1;

      const unsigned char r0 = cloud->points[index0].intensity;

      const unsigned char r_c = cloud->points[index_c].intensity;

      const unsigned char r_r = cloud->points[index_r].intensity;

      const float r_dx = static_cast<float> (r_c) - static_cast<float> (r0);

      const float r_dy = static_cast<float> (r_r) - static_cast<float> (r0);

      const float sqr_mag_r = r_dx*r_dx + r_dy*r_dy;

      float gradient_mag = sqrt (sqr_mag_r);

      gradients_ (col_index+1, row_index+1) = gradient_mag;
      quantized_gradients_(col_index+1, row_index+1) = gradient_mag < gradient_magnitude_threshold_ ?
        0 : atan2 (r_dy, r_dx) * 180.0f / pi;
    }
  }

  return;
}

// Equivalent to atan2(y/x) then quantize to 8 directions
template <typename PointInT>
uint8_t
pcl::GrayscaleGradientModality<PointInT>::
quantizedAngleFromXY(float x, float y) {
  if (x == 0.0f || x == -0.0f)
   return 4 + 1;
  float a = y / x;

  uint8_t ret;

  if (a > 0.198912367379658) {
    if (a > 1.496605762665489) {
      ret = a > 5.027339492125846 ? 4 : 3;
    } else {
      ret = a > 0.6681786379192989 ? 2 : 1;
    }
  } else {
    if (a > -1.4966057626654885) {
      ret = (a > -0.1989123673796579) ?
        0 :
        (a > -0.6681786379192988) ?
          7 : 6;
    } else {
      ret = (a > -5.02733949212585) ? 5 : 4;
    }
  }

  return ret + 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::GrayscaleGradientModality<PointInT>::
computeMaxGradientsSobel (const typename pcl::PointCloud<pcl::Intensity8u>::ConstPtr & cloud)
{
  const int width = cloud->width;
  const int height = cloud->height;

  gradients_.points.resize (width*height);
  gradients_.width = width;
  gradients_.height = height;
  quantized_gradients_.resize (width, height);

#if __AVX2__
  static const __m256i __index_shuffle = _mm256_set_epi8(
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 12, 8, 4, 0,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 12, 8, 4, 0);
  static const __m256i __ones = _mm256_set1_epi32(0xFFFFFFFF);
#endif
  //#pragma omp parallel for
  for (int row_index = 1; row_index < height-1; ++row_index)
  {
    int col_index = 1;
#if __AVX2__
    for (; col_index <= width - 1 - 16; col_index += 16)
    {
      // __m256i __loadtemp = _mm256_loadu_si256((const __m256i*) &cloud->points[(row_index-1)*width + (col_index-1)]);
      // __m256i __7 = _mm256_cvtepu8_epi16(_mm256_extracti128_si256(__loadtemp, 0));
      // __m256i __loadtemp1 = _mm256_cvtepu8_epi16(_mm256_extracti128_si256(__loadtemp, 1));
      // // Although each channel is 8 bits, 16 bits are needed to prevent overflow
      // __m256i __8 = _mm256_alignr_epi8(__loadtemp1, __7, 8);
      // __m256i __9 = _mm256_alignr_epi8(__loadtemp1, __7, 16);
      // __m256i __dy = _mm256_add_epi16(_mm256_add_epi16(__7, _mm256_slli_epi16(__8, 1)), __9);

      // __loadtemp = _mm256_loadu_si256((const __m256i*) &cloud->points[(row_index+1)*width + (col_index-1)]);
      // __m256i __1 = _mm256_cvtepu8_epi16(_mm256_extracti128_si256(__loadtemp, 0));
      // __loadtemp1 = _mm256_cvtepu8_epi16(_mm256_extracti128_si256(__loadtemp, 1));
      // __m256i __2 = _mm256_alignr_epi8(__loadtemp1, __1, 8);
      // __m256i __3 = _mm256_alignr_epi8(__loadtemp1, __1, 16);
      
      // Although each channel is 8 bits, 16 bits are needed to prevent overflow
      __m256i __7 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i*) &cloud->points[(row_index-1)*width + (col_index-1)]));
      __m256i __8 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i*) &cloud->points[(row_index-1)*width + (col_index)]));
      __m256i __9 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i*) &cloud->points[(row_index-1)*width + (col_index+1)]));

      __m256i __dy = _mm256_add_epi16(_mm256_add_epi16(__7, _mm256_slli_epi16(__8, 1)), __9);
      __m256i __1 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i*) &cloud->points[(row_index+1)*width + (col_index-1)]));
      __m256i __2 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i*) &cloud->points[(row_index+1)*width + (col_index)]));
      __m256i __3 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i*) &cloud->points[(row_index+1)*width + (col_index+1)]));

      __dy = _mm256_sub_epi16(_mm256_add_epi16(_mm256_add_epi16(__1, _mm256_slli_epi16(__2, 1)), __3),
                              __dy);

      __m256i __4 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i*) &cloud->points[(row_index)*width + (col_index-1)]));
      __m256i __6 = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i*) &cloud->points[(row_index)*width + (col_index+1)]));
      __m256i __dx = _mm256_sub_epi16(_mm256_add_epi16(_mm256_add_epi16(__9, _mm256_slli_epi16(__6, 1)), __3),
                                      _mm256_add_epi16(_mm256_add_epi16(__7, _mm256_slli_epi16(__4, 1)), __1));

      
      __m256 __dx8 = _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(_mm256_extracti128_si256(__dx, 0)));
      __m256 __dy8 = _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(_mm256_extracti128_si256(__dy, 0)));
      __m256 __mag_8 = _mm256_sqrt_ps(_mm256_fmadd_ps(__dx8, __dx8, _mm256_mul_ps(__dy8, __dy8)));
      __m256 __ydx8 = _mm256_div_ps(__dy8, __dx8);

      _mm256_storeu_ps(&gradients_(col_index, row_index), __mag_8);

      __m256i __accum_mask, __update_mask;

      // Is this really faster than looping?...
      #define QUANTIZE_ANGLE(__index_var) \
      __index_var = _mm256_setzero_si256(); \
          \
      __accum_mask = _mm256_castps_si256(_mm256_cmp_ps(__mag_8, _mm256_set1_ps(gradient_magnitude_threshold_), _CMP_LT_OQ)); \
        \
      __update_mask =    \
        _mm256_andnot_si256(   \
          __accum_mask, \
          _mm256_or_si256( \
            _mm256_castps_si256(_mm256_cmp_ps(__ydx8, _mm256_set1_ps(5.027339492125846), _CMP_GT_OQ)), \
            _mm256_castps_si256(_mm256_cmp_ps(__ydx8, _mm256_set1_ps(-5.02733949212585), _CMP_LT_OQ)))); \
      __accum_mask = _mm256_or_si256(__update_mask, __accum_mask); \
      __index_var = _mm256_blendv_epi8(__index_var, _mm256_set1_epi32(5), __update_mask); \
      __update_mask = _mm256_andnot_si256(__accum_mask, _mm256_castps_si256(_mm256_cmp_ps(__ydx8, _mm256_set1_ps(1.496605762665489), _CMP_GT_OQ))); \
      __accum_mask = _mm256_or_si256(__update_mask, __accum_mask); \
      __index_var = _mm256_blendv_epi8(__index_var, _mm256_set1_epi32(4), __update_mask); \
      __update_mask = _mm256_andnot_si256(__accum_mask, _mm256_castps_si256(_mm256_cmp_ps(__ydx8, _mm256_set1_ps(0.6681786379192989), _CMP_GT_OQ))); \
      __accum_mask = _mm256_or_si256(__update_mask, __accum_mask); \
      __index_var = _mm256_blendv_epi8(__index_var, _mm256_set1_epi32(3), __update_mask); \
      __update_mask = _mm256_andnot_si256(__accum_mask, _mm256_castps_si256(_mm256_cmp_ps(__ydx8, _mm256_set1_ps(0.198912367379658), _CMP_GT_OQ))); \
      __accum_mask = _mm256_or_si256(__update_mask, __accum_mask); \
      __index_var = _mm256_blendv_epi8(__index_var, _mm256_set1_epi32(2), __update_mask); \
                                                                        \
      __update_mask = _mm256_andnot_si256(__accum_mask, _mm256_castps_si256(_mm256_cmp_ps(__ydx8, _mm256_set1_ps(-0.1989123673796579), _CMP_GT_OQ))); \
      __accum_mask = _mm256_or_si256(__update_mask, __accum_mask); \
      __index_var = _mm256_blendv_epi8(__index_var, _mm256_set1_epi32(1), __update_mask); \
                                                                         \
      __update_mask = _mm256_andnot_si256(__accum_mask, _mm256_castps_si256(_mm256_cmp_ps(__ydx8, _mm256_set1_ps(-0.6681786379192988), _CMP_GT_OQ))); \
      __accum_mask = _mm256_or_si256(__update_mask, __accum_mask);                                                                                 \
      __index_var = _mm256_blendv_epi8(__index_var, _mm256_set1_epi32(8), __update_mask);                                                                            \
                \
      __update_mask = _mm256_andnot_si256(__accum_mask, _mm256_castps_si256(_mm256_cmp_ps(__ydx8, _mm256_set1_ps(-1.4966057626654885), _CMP_GT_OQ))); \
      __accum_mask = _mm256_or_si256(__update_mask, __accum_mask); \
      __index_var = _mm256_blendv_epi8(__index_var, _mm256_set1_epi32(7), __update_mask); \
                \
      __update_mask = _mm256_andnot_si256(__accum_mask, __ones); \
      __index_var = _mm256_blendv_epi8(__index_var, _mm256_set1_epi32(6), __update_mask); \
      __index_var = _mm256_shuffle_epi8(__index_var, __index_shuffle); \
      __index_var = _mm256_permutevar8x32_epi32(__index_var, _mm256_set_epi32(4, 0, 4, 0, 4, 0, 4, 0));

      // index 0-7 = [0:63]
      __m256i __index_first8;
      QUANTIZE_ANGLE(__index_first8);

      // Second half
      __dx8 = _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(_mm256_extractf128_si256(__dx, 1)));
      __dy8 = _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(_mm256_extractf128_si256(__dy, 1)));
      __ydx8 = _mm256_div_ps(__dy8, __dx8);
      __mag_8 = _mm256_sqrt_ps(_mm256_fmadd_ps(__dx8, __dx8, _mm256_mul_ps(__dy8, __dy8)));

      // index 8-15 = [0:63]
      __m256i __index_second8;
      QUANTIZE_ANGLE(__index_second8);

      #undef QUANTIZE_ANGLE

      _mm256_storeu_ps(&gradients_(col_index + 8, row_index), __mag_8);

      _mm_storeu_si128(
        (__m128i*) &quantized_gradients_ (col_index, row_index),
        _mm256_extracti128_si256(_mm256_castps_si256(_mm256_shuffle_ps(_mm256_castsi256_ps(__index_first8), _mm256_castsi256_ps(__index_second8), (1 << 6) + (0 << 4) + (1 << 2) + 0)), 0));
    }
#endif
    for (; col_index < width-1; ++col_index)
    {
      const int r7 = static_cast<int> (cloud->points[(row_index-1)*width + (col_index-1)].intensity);
      const int r8 = static_cast<int> (cloud->points[(row_index-1)*width + (col_index)].intensity);
      const int r9 = static_cast<int> (cloud->points[(row_index-1)*width + (col_index+1)].intensity);
      const int r4 = static_cast<int> (cloud->points[(row_index)*width + (col_index-1)].intensity);
      const int r6 = static_cast<int> (cloud->points[(row_index)*width + (col_index+1)].intensity);
      const int r1 = static_cast<int> (cloud->points[(row_index+1)*width + (col_index-1)].intensity);
      const int r2 = static_cast<int> (cloud->points[(row_index+1)*width + (col_index)].intensity);
      const int r3 = static_cast<int> (cloud->points[(row_index+1)*width + (col_index+1)].intensity);

      const int r_dx = r9 + 2*r6 + r3 - (r7 + 2*r4 + r1);
      const int r_dy = r1 + 2*r2 + r3 - (r7 + 2*r8 + r9);

      const int sqr_mag_r = r_dx*r_dx + r_dy*r_dy;

      float gradient_mag = sqrtf (static_cast<float> (sqr_mag_r));
      //gradient.angle = static_cast<float>(quantizedAngleFromXY(r_dx, r_dy));

      gradients_ (col_index, row_index) = gradient_mag;
      quantized_gradients_ (col_index, row_index) = gradient_mag < gradient_magnitude_threshold_ ?
        0 : quantizedAngleFromXY(r_dx, r_dy);
    }
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::GrayscaleGradientModality<PointInT>::
filterQuantizedGradients ()
{
  const size_t width = input_->width;
  const size_t height = input_->height;

  filtered_quantized_gradients_.resize (width, height);

  // filter data
  #pragma omp parallel for
  for (size_t row_index = 1; row_index < height-1; ++row_index)
  {
    for (size_t col_index = 1; col_index < width-1; ++col_index)
    {
      unsigned char histogram[9] = {0,0,0,0,0,0,0,0,0};

      const unsigned char * data_ptr = quantized_gradients_.getData () + (row_index-1)*width+col_index-1;
      ++histogram[data_ptr[0]];
      ++histogram[data_ptr[1]];
      ++histogram[data_ptr[2]];
      data_ptr += width;
      ++histogram[data_ptr[0]];
      ++histogram[data_ptr[1]];
      ++histogram[data_ptr[2]];
      data_ptr += width;
      ++histogram[data_ptr[0]];
      ++histogram[data_ptr[1]];
      ++histogram[data_ptr[2]];

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
      // __m128i __hist = _mm_castpd_si128(_mm_load1_pd((const double*) &histogram[1]));
      // __m128i __max = _mm_max_epi8(__hist, _mm_alignr_epi8(__hist, __hist, 4));
      // __max = _mm_max_epi8(__max, _mm_alignr_epi8(__max, __max, 2));
      // __max = _mm_max_epi8(__max, _mm_alignr_epi8(__max, __max, 1));
      // int max_index = __builtin_ctz(_mm_movemask_epi8(_mm_cmpeq_epi8(__hist, __max)));

      if (max_hist_value < histogram[1]) {max_hist_index = 0; max_hist_value = histogram[1];}
      if (max_hist_value < histogram[2]) {max_hist_index = 1; max_hist_value = histogram[2];}
      if (max_hist_value < histogram[3]) {max_hist_index = 2; max_hist_value = histogram[3];}
      if (max_hist_value < histogram[4]) {max_hist_index = 3; max_hist_value = histogram[4];}
      if (max_hist_value < histogram[5]) {max_hist_index = 4; max_hist_value = histogram[5];}
      if (max_hist_value < histogram[6]) {max_hist_index = 5; max_hist_value = histogram[6];}
      if (max_hist_value < histogram[7]) {max_hist_index = 6; max_hist_value = histogram[7];}
      if (max_hist_value < histogram[8]) {max_hist_index = 7; max_hist_value = histogram[8];}

      if (max_hist_index != -1 && max_hist_value >= 5)
        filtered_quantized_gradients_ (col_index, row_index) = static_cast<unsigned char> (0x1 << max_hist_index);
      else
        filtered_quantized_gradients_ (col_index, row_index) = 0;

      // filtered_quantized_color_gradients_(col_index, row_index) = histogram[max_index + 1] >= 5 ?
      //   static_cast<unsigned char> (0x1 << max_index) : 0;

    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
pcl::GrayscaleGradientModality<PointInT>::
erode (const pcl::MaskMap & mask_in, 
       pcl::MaskMap & mask_out)
{
  const size_t width = mask_in.getWidth ();
  const size_t height = mask_in.getHeight ();

  mask_out.resize (width, height);

  for (size_t row_index = 1; row_index < height-1; ++row_index)
  {
    for (size_t col_index = 1; col_index < width-1; ++col_index)
    {
      if (mask_in (col_index, row_index-1) == 0 ||
          mask_in (col_index-1, row_index) == 0 ||
          mask_in (col_index+1, row_index) == 0 ||
          mask_in (col_index, row_index+1) == 0)
      {
        mask_out (col_index, row_index) = 0;
      }
      else
      {
        mask_out (col_index, row_index) = 255;
      }
    }
  }
}

#endif 
