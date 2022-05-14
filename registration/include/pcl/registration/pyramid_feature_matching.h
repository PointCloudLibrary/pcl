/*
 * Software License Agreement (BSD License)
 *
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

#pragma once

#include <pcl/pcl_base.h>
#include <pcl/point_representation.h>

namespace pcl {
/**
 * \brief Class that compares two sets of features by using a multiscale representation
 * of the features inside a pyramid. Each level of the pyramid offers information about
 * the similarity of the two feature sets. \note Works with any Point/Feature type which
 * has a PointRepresentation implementation \note The only parameters it needs are the
 * input dimension ranges and the output dimension ranges. The input dimension ranges
 * represent the ranges in which each dimension of the feature vector lies. As described
 * in the paper, a minimum inter-vector distance of sqrt(nr_dims)/2 is needed. As such,
 * the target dimension range parameter is used in order to augment/reduce the range for
 * each dimension in order to obtain the necessary minimal inter-vector distance and to
 * add/subtract weight to/from certain dimensions of the feature vector.
 *
 * Follows the algorithm presented in the publication:
 *    Grauman, K. & Darrell, T.
 *    The Pyramid Match Kernel: Discriminative Classification with Sets of Image
 * Features Tenth IEEE International Conference on Computer Vision ICCV05 Volume 1
 *    October 2005
 *
 * \author Alexandru-Eugen Ichim
 */
template <typename PointFeature>
class PyramidFeatureHistogram : public PCLBase<PointFeature> {
public:
  using PCLBase<PointFeature>::input_;

  using Ptr = shared_ptr<PyramidFeatureHistogram<PointFeature>>;
  using ConstPtr = shared_ptr<const PyramidFeatureHistogram<PointFeature>>;
  using PyramidFeatureHistogramPtr = Ptr;
  using FeatureRepresentationConstPtr =
      shared_ptr<const pcl::PointRepresentation<PointFeature>>;

  /** \brief Empty constructor that instantiates the feature representation variable */
  PyramidFeatureHistogram();

  /** \brief Method for setting the input dimension range parameter.
   * \note Please check the PyramidHistogram class description for more details about
   * this parameter.
   */
  inline void
  setInputDimensionRange(std::vector<std::pair<float, float>>& dimension_range_input)
  {
    dimension_range_input_ = dimension_range_input;
  }

  /** \brief Method for retrieving the input dimension range vector */
  inline std::vector<std::pair<float, float>>
  getInputDimensionRange()
  {
    return dimension_range_input_;
  }

  /** \brief Method to set the target dimension range parameter.
   * \note Please check the PyramidHistogram class description for more details about
   * this parameter.
   */
  inline void
  setTargetDimensionRange(std::vector<std::pair<float, float>>& dimension_range_target)
  {
    dimension_range_target_ = dimension_range_target;
  }

  /** \brief Method for retrieving the target dimension range vector */
  inline std::vector<std::pair<float, float>>
  getTargetDimensionRange()
  {
    return dimension_range_target_;
  }

  /** \brief Provide a pointer to the feature representation to use to convert features
   * to k-D vectors. \param feature_representation the const boost shared pointer to a
   * PointRepresentation
   */
  inline void
  setPointRepresentation(const FeatureRepresentationConstPtr& feature_representation)
  {
    feature_representation_ = feature_representation;
  }

  /** \brief Get a pointer to the feature representation used when converting features
   * into k-D vectors. */
  inline FeatureRepresentationConstPtr const
  getPointRepresentation()
  {
    return feature_representation_;
  }

  /** \brief The central method for inserting the feature set inside the pyramid and
   * obtaining the complete pyramid */
  void
  compute();

  /** \brief Checks whether the pyramid histogram has been computed */
  inline bool
  isComputed()
  {
    return is_computed_;
  }

  /** \brief Static method for comparing two pyramid histograms that returns a floating
   * point value between 0 and 1, representing the similarity between the feature sets
   * on which the two pyramid histograms are based. \param pyramid_a Pointer to the
   * first pyramid to be compared (needs to be computed already). \param pyramid_b
   * Pointer to the second pyramid to be compared (needs to be computed already).
   */
  static float
  comparePyramidFeatureHistograms(const PyramidFeatureHistogramPtr& pyramid_a,
                                  const PyramidFeatureHistogramPtr& pyramid_b);

private:
  std::size_t nr_dimensions, nr_levels, nr_features;
  std::vector<std::pair<float, float>> dimension_range_input_, dimension_range_target_;
  FeatureRepresentationConstPtr feature_representation_;
  bool is_computed_;

  /** \brief Checks for input inconsistencies and initializes the underlying data
   * structures */
  bool
  initializeHistogram();

  /** \brief Converts a feature in templated form to an STL vector. This is the point
   * where the conversion from the input dimension range to the target dimension range
   * is done.
   */
  void
  convertFeatureToVector(const PointFeature& feature,
                         std::vector<float>& feature_vector);

  /** \brief Adds a feature vector to its corresponding bin at each level in the pyramid
   */
  void
  addFeature(std::vector<float>& feature);

  /** \brief Access the pyramid bin given the position of the bin at the given pyramid
   * level and the pyramid level \param access index of the bin at the respective level
   * \param level the level in the pyramid
   */
  inline unsigned int&
  at(std::vector<std::size_t>& access, std::size_t& level);

  /** \brief Access the pyramid bin given a feature vector and the pyramid level
   * \param feature the feature in vectorized form
   * \param level the level in the pyramid
   */
  inline unsigned int&
  at(std::vector<float>& feature, std::size_t& level);

  /** \brief Structure for representing a single pyramid histogram level */
  struct PyramidFeatureHistogramLevel {
    PyramidFeatureHistogramLevel() {}

    PyramidFeatureHistogramLevel(std::vector<std::size_t>& a_bins_per_dimension,
                                 std::vector<float>& a_bin_step)
    : bins_per_dimension(a_bins_per_dimension), bin_step(a_bin_step)
    {
      initializeHistogramLevel();
    }

    void
    initializeHistogramLevel();

    std::vector<unsigned int> hist;
    std::vector<std::size_t> bins_per_dimension;
    std::vector<float> bin_step;
  };
  std::vector<PyramidFeatureHistogramLevel> hist_levels;
};
} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/registration/impl/pyramid_feature_matching.hpp>
#endif
