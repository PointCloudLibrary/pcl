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

#include <pcl/common/common.h>
#include <pcl/ml/feature_handler.h>
#include <pcl/ml/ferns/fern.h>
#include <pcl/ml/stats_estimator.h>

#include <vector>

namespace pcl {

/** Trainer for a Fern. */
template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
class PCL_EXPORTS FernTrainer {

public:
  /** Constructor. */
  FernTrainer();

  /** Sets the feature handler used to create and evaluate features.
   *
   * \param[in] feature_handler the feature handler
   */
  inline void
  setFeatureHandler(
      pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler)
  {
    feature_handler_ = &feature_handler;
  }

  /** Sets the object for estimating the statistics for tree nodes.
   *
   * \param[in] stats_estimator the statistics estimator
   */
  inline void
  setStatsEstimator(
      pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>& stats_estimator)
  {
    stats_estimator_ = &stats_estimator;
  }

  /** Sets the maximum depth of the learned tree.
   *
   * \param[in] fern_depth maximum depth of the learned tree
   */
  inline void
  setFernDepth(const std::size_t fern_depth)
  {
    fern_depth_ = fern_depth;
  }

  /** Sets the number of features used to find optimal decision features.
   *
   * \param[in] num_of_features the number of features
   */
  inline void
  setNumOfFeatures(const std::size_t num_of_features)
  {
    num_of_features_ = num_of_features;
  }

  /** Sets the number of thresholds tested for finding the optimal decision
   *  threshold on the feature responses.
   *
   * \param[in] num_of_threshold the number of thresholds
   */
  inline void
  setNumOfThresholds(const std::size_t num_of_threshold)
  {
    num_of_thresholds_ = num_of_threshold;
  }

  /** Sets the input data set used for training.
   *
   * \param[in] data_set the data set used for training
   */
  inline void
  setTrainingDataSet(DataSet& data_set)
  {
    data_set_ = data_set;
  }

  /** Example indices that specify the data used for training.
   *
   * \param[in] examples the examples
   */
  inline void
  setExamples(std::vector<ExampleIndex>& examples)
  {
    examples_ = examples;
  }

  /** Sets the label data corresponding to the example data.
   *
   * \param[in] label_data the label data
   */
  inline void
  setLabelData(std::vector<LabelType>& label_data)
  {
    label_data_ = label_data;
  }

  /** Trains a decision tree using the set training data and settings.
   *
   * \param[out] fern destination for the trained tree
   */
  void
  train(Fern<FeatureType, NodeType>& fern);

protected:
  /** Creates uniformely distrebuted thresholds over the range of the supplied
   *  values.
   *
   * \param[in] num_of_thresholds the number of thresholds to create
   * \param[in] values the values for estimating the expected value range
   * \param[out] thresholds the resulting thresholds
   */
  static void
  createThresholdsUniform(const std::size_t num_of_thresholds,
                          std::vector<float>& values,
                          std::vector<float>& thresholds);

private:
  /** Desired depth of the learned fern. */
  std::size_t fern_depth_;
  /** Number of features used to find optimal decision features. */
  std::size_t num_of_features_;
  /** Number of thresholds. */
  std::size_t num_of_thresholds_;

  /** FeatureHandler instance, responsible for creating and evaluating features. */
  pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>* feature_handler_;
  /** StatsEstimator instance, responsible for gathering stats about a node. */
  pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>* stats_estimator_;

  /** The training data set. */
  DataSet data_set_;
  /** The label data. */
  std::vector<LabelType> label_data_;
  /** The example data. */
  std::vector<ExampleIndex> examples_;
};

} // namespace pcl

#include <pcl/ml/impl/ferns/fern_trainer.hpp>
