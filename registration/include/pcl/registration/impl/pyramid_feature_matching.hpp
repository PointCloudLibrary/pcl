/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id$
 *
 */

#ifndef PCL_REGISTRATION_IMPL_PYRAMID_FEATURE_MATCHING_H_
#define PCL_REGISTRATION_IMPL_PYRAMID_FEATURE_MATCHING_H_

#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/console/print.h>
#include <pcl/pcl_macros.h>

namespace pcl {

template <typename PointFeature>
float
PyramidFeatureHistogram<PointFeature>::comparePyramidFeatureHistograms(
    const PyramidFeatureHistogramPtr& pyramid_a,
    const PyramidFeatureHistogramPtr& pyramid_b)
{
  // do a few consistency checks before and during the computation
  if (pyramid_a->nr_dimensions != pyramid_b->nr_dimensions) {
    PCL_ERROR("[pcl::PyramidFeatureMatching::comparePyramidFeatureHistograms] The two "
              "given pyramids have different numbers of dimensions: %u vs %u\n",
              pyramid_a->nr_dimensions,
              pyramid_b->nr_dimensions);
    return -1;
  }
  if (pyramid_a->nr_levels != pyramid_b->nr_levels) {
    PCL_ERROR("[pcl::PyramidFeatureMatching::comparePyramidFeatureHistograms] The two "
              "given pyramids have different numbers of levels: %u vs %u\n",
              pyramid_a->nr_levels,
              pyramid_b->nr_levels);
    return -1;
  }

  // calculate for level 0 first
  if (pyramid_a->hist_levels[0].hist.size() != pyramid_b->hist_levels[0].hist.size()) {
    PCL_ERROR("[pcl::PyramidFeatureMatching::comparePyramidFeatureHistograms] The two "
              "given pyramids have different numbers of bins on level 0: %u vs %u\n",
              pyramid_a->hist_levels[0].hist.size(),
              pyramid_b->hist_levels[0].hist.size());
    return -1;
  }
  float match_count_level = 0.0f;
  for (std::size_t bin_i = 0; bin_i < pyramid_a->hist_levels[0].hist.size(); ++bin_i) {
    if (pyramid_a->hist_levels[0].hist[bin_i] < pyramid_b->hist_levels[0].hist[bin_i])
      match_count_level += static_cast<float>(pyramid_a->hist_levels[0].hist[bin_i]);
    else
      match_count_level += static_cast<float>(pyramid_b->hist_levels[0].hist[bin_i]);
  }

  float match_count = match_count_level;
  for (std::size_t level_i = 1; level_i < pyramid_a->nr_levels; ++level_i) {
    if (pyramid_a->hist_levels[level_i].hist.size() !=
        pyramid_b->hist_levels[level_i].hist.size()) {
      PCL_ERROR(
          "[pcl::PyramidFeatureMatching::comparePyramidFeatureHistograms] The two "
          "given pyramids have different numbers of bins on level %u: %u vs %u\n",
          level_i,
          pyramid_a->hist_levels[level_i].hist.size(),
          pyramid_b->hist_levels[level_i].hist.size());
      return -1;
    }

    float match_count_prev_level = match_count_level;
    match_count_level = 0.0f;
    for (std::size_t bin_i = 0; bin_i < pyramid_a->hist_levels[level_i].hist.size();
         ++bin_i) {
      if (pyramid_a->hist_levels[level_i].hist[bin_i] <
          pyramid_b->hist_levels[level_i].hist[bin_i])
        match_count_level +=
            static_cast<float>(pyramid_a->hist_levels[level_i].hist[bin_i]);
      else
        match_count_level +=
            static_cast<float>(pyramid_b->hist_levels[level_i].hist[bin_i]);
    }

    float level_normalization_factor = powf(2.0f, static_cast<float>(level_i));
    match_count +=
        (match_count_level - match_count_prev_level) / level_normalization_factor;
  }

  // include self-similarity factors
  float self_similarity_a = static_cast<float>(pyramid_a->nr_features),
        self_similarity_b = static_cast<float>(pyramid_b->nr_features);
  PCL_DEBUG("[pcl::PyramidFeatureMatching::comparePyramidFeatureHistograms] Self "
            "similarity measures: %f, %f\n",
            self_similarity_a,
            self_similarity_b);
  match_count /= std::sqrt(self_similarity_a * self_similarity_b);

  return match_count;
}

template <typename PointFeature>
PyramidFeatureHistogram<PointFeature>::PyramidFeatureHistogram()
: nr_dimensions(0)
, nr_levels(0)
, nr_features(0)
, feature_representation_(new DefaultPointRepresentation<PointFeature>)
, is_computed_(false)
, hist_levels()
{}

template <typename PointFeature>
void
PyramidFeatureHistogram<
    PointFeature>::PyramidFeatureHistogramLevel::initializeHistogramLevel()
{
  std::size_t total_vector_size = 1;
  for (const auto& bin : bins_per_dimension) {
    total_vector_size *= bin;
  }

  hist.resize(total_vector_size, 0);
}

template <typename PointFeature>
bool
PyramidFeatureHistogram<PointFeature>::initializeHistogram()
{
  // a few consistency checks before starting the computations
  if (!PCLBase<PointFeature>::initCompute()) {
    PCL_ERROR("[pcl::PyramidFeatureHistogram::initializeHistogram] PCLBase initCompute "
              "failed\n");
    return false;
  }

  if (dimension_range_input_.empty()) {
    PCL_ERROR("[pcl::PyramidFeatureHistogram::initializeHistogram] Input dimension "
              "range was not set\n");
    return false;
  }

  if (dimension_range_target_.empty()) {
    PCL_ERROR("[pcl::PyramidFeatureHistogram::initializeHistogram] Target dimension "
              "range was not set\n");
    return false;
  }

  if (dimension_range_input_.size() != dimension_range_target_.size()) {
    PCL_ERROR("[pcl::PyramidFeatureHistogram::initializeHistogram] Input and target "
              "dimension ranges do not agree in size: %u vs %u\n",
              dimension_range_input_.size(),
              dimension_range_target_.size());
    return false;
  }

  nr_dimensions = dimension_range_target_.size();
  nr_features = input_->size();
  float D = 0.0f;
  for (const auto& dim : dimension_range_target_) {
    float aux = dim.first - dim.second;
    D += aux * aux;
  }
  D = std::sqrt(D);
  nr_levels = static_cast<std::size_t>(std::ceil(std::log2(D)));
  PCL_DEBUG("[pcl::PyramidFeatureHistogram::initializeHistogram] Pyramid will have %u "
            "levels with a hyper-parallelepiped diagonal size of %f\n",
            nr_levels,
            D);

  hist_levels.resize(nr_levels);
  for (std::size_t level_i = 0; level_i < nr_levels; ++level_i) {
    std::vector<std::size_t> bins_per_dimension(nr_dimensions);
    std::vector<float> bin_step(nr_dimensions);
    for (std::size_t dim_i = 0; dim_i < nr_dimensions; ++dim_i) {
      bins_per_dimension[dim_i] = static_cast<std::size_t>(
          std::ceil((dimension_range_target_[dim_i].second -
                     dimension_range_target_[dim_i].first) /
                    (powf(2.0f, static_cast<float>(level_i)) *
                     std::sqrt(static_cast<float>(nr_dimensions)))));
      bin_step[dim_i] = powf(2.0f, static_cast<float>(level_i)) *
                        std::sqrt(static_cast<float>(nr_dimensions));
    }
    hist_levels[level_i] = PyramidFeatureHistogramLevel(bins_per_dimension, bin_step);

    PCL_DEBUG("[pcl::PyramidFeatureHistogram::initializeHistogram] Created vector of "
              "size %u at level %u\nwith #bins per dimension:",
              hist_levels.back().hist.size(),
              level_i);
    for (std::size_t dim_i = 0; dim_i < nr_dimensions; ++dim_i)
      PCL_DEBUG("%u ", bins_per_dimension[dim_i]);
    PCL_DEBUG("\n");
  }

  return true;
}

template <typename PointFeature>
unsigned int&
PyramidFeatureHistogram<PointFeature>::at(std::vector<std::size_t>& access,
                                          std::size_t& level)
{
  if (access.size() != nr_dimensions) {
    PCL_ERROR(
        "[pcl::PyramidFeatureHistogram::at] Cannot access histogram position because "
        "the access point does not have the right number of dimensions\n");
    return hist_levels.front().hist.front();
  }
  if (level >= hist_levels.size()) {
    PCL_ERROR(
        "[pcl::PyramidFeatureHistogram::at] Trying to access a too large level\n");
    return hist_levels.front().hist.front();
  }

  std::size_t vector_position = 0;
  std::size_t dim_accumulator = 1;

  for (int i = static_cast<int>(access.size()) - 1; i >= 0; --i) {
    vector_position += access[i] * dim_accumulator;
    dim_accumulator *= hist_levels[level].bins_per_dimension[i];
  }

  return hist_levels[level].hist[vector_position];
}

template <typename PointFeature>
unsigned int&
PyramidFeatureHistogram<PointFeature>::at(std::vector<float>& feature,
                                          std::size_t& level)
{
  if (feature.size() != nr_dimensions) {
    PCL_ERROR("[pcl::PyramidFeatureHistogram::at] The given feature vector does not "
              "match the feature dimensions of the pyramid histogram: %u vs %u\n",
              feature.size(),
              nr_dimensions);
    return hist_levels.front().hist.front();
  }
  if (level >= hist_levels.size()) {
    PCL_ERROR(
        "[pcl::PyramidFeatureHistogram::at] Trying to access a too large level\n");
    return hist_levels.front().hist.front();
  }

  std::vector<std::size_t> access;
  for (std::size_t dim_i = 0; dim_i < nr_dimensions; ++dim_i)
    access.push_back(static_cast<std::size_t>(
        std::floor((feature[dim_i] - dimension_range_target_[dim_i].first) /
                   hist_levels[level].bin_step[dim_i])));

  return at(access, level);
}

template <typename PointFeature>
void
PyramidFeatureHistogram<PointFeature>::convertFeatureToVector(
    const PointFeature& feature, std::vector<float>& feature_vector)
{
  // convert feature to vector representation
  feature_vector.resize(feature_representation_->getNumberOfDimensions());
  feature_representation_->vectorize(feature, feature_vector);

  // adapt the values from the input range to the target range
  for (std::size_t i = 0; i < feature_vector.size(); ++i)
    feature_vector[i] =
        (feature_vector[i] - dimension_range_input_[i].first) /
            (dimension_range_input_[i].second - dimension_range_input_[i].first) *
            (dimension_range_target_[i].second - dimension_range_target_[i].first) +
        dimension_range_target_[i].first;
}

template <typename PointFeature>
void
PyramidFeatureHistogram<PointFeature>::compute()
{
  if (!initializeHistogram())
    return;

  for (const auto& point : *input_) {
    std::vector<float> feature_vector;
    // NaN is converted to very high number that gives out of bound exception.
    if (!pcl::isFinite(point))
      continue;
    convertFeatureToVector(point, feature_vector);
    addFeature(feature_vector);
  }

  is_computed_ = true;
}

template <typename PointFeature>
void
PyramidFeatureHistogram<PointFeature>::addFeature(std::vector<float>& feature)
{
  for (std::size_t level_i = 0; level_i < nr_levels; ++level_i)
    at(feature, level_i)++;
}

} // namespace pcl

#define PCL_INSTANTIATE_PyramidFeatureHistogram(PointFeature)                          \
  template class PCL_EXPORTS pcl::PyramidFeatureHistogram<PointFeature>;

#endif /* PCL_REGISTRATION_IMPL_PYRAMID_FEATURE_MATCHING_H_ */
