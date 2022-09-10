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
#include <pcl/ml/dt/decision_tree.h>
#include <pcl/ml/dt/decision_tree_data_provider.h>
#include <pcl/ml/feature_handler.h>
#include <pcl/ml/stats_estimator.h>

#include <vector>

namespace pcl {

/** Trainer for decision trees. */
template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
class PCL_EXPORTS DecisionTreeTrainer {

public:
  /** Constructor. */
  DecisionTreeTrainer();

  /** Destructor. */
  virtual ~DecisionTreeTrainer();

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
   * \param[in] max_tree_depth maximum depth of the learned tree
   */
  inline void
  setMaxTreeDepth(const std::size_t max_tree_depth)
  {
    max_tree_depth_ = max_tree_depth;
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

  /** Sets the minimum number of examples to continue growing a tree.
   *
   * \param[in] n number of examples
   */
  inline void
  setMinExamplesForSplit(std::size_t n)
  {
    min_examples_for_split_ = n;
  }

  /** Specify the thresholds to be used when evaluating features.
   *
   * \param[in] thres the threshold values
   */
  void
  setThresholds(std::vector<float>& thres)
  {
    thresholds_ = thres;
  }

  /** Specify the data provider.
   *
   * \param[in] dtdp the data provider that should implement getDatasetAndLabels()
   *            function
   */
  void
  setDecisionTreeDataProvider(
      typename pcl::DecisionTreeTrainerDataProvider<FeatureType,
                                                    DataSet,
                                                    LabelType,
                                                    ExampleIndex,
                                                    NodeType>::Ptr& dtdp)
  {
    decision_tree_trainer_data_provider_ = dtdp;
  }

  /** Specify if the features are randomly generated at each split node.
   *
   * \param[in] b do it or not
   */
  void
  setRandomFeaturesAtSplitNode(bool b)
  {
    random_features_at_split_node_ = b;
  }

  /** Trains a decision tree using the set training data and settings.
   *
   * \param[out] tree destination for the trained tree
   */
  void
  train(DecisionTree<NodeType>& tree);

protected:
  /** Trains a decision tree node from the specified features, label data, and
   *  examples.
   *
   * \param[in] features the feature pool used for training
   * \param[in] examples the examples used for training
   * \param[in] label_data the label data corresponding to the examples
   * \param[in] max_depth the maximum depth of the remaining tree
   * \param[out] node the resulting node
   */
  void
  trainDecisionTreeNode(std::vector<FeatureType>& features,
                        std::vector<ExampleIndex>& examples,
                        std::vector<LabelType>& label_data,
                        std::size_t max_depth,
                        NodeType& node);

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
  /** Maximum depth of the learned tree. */
  std::size_t max_tree_depth_;
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

  /** Minimum number of examples to split a node. */
  std::size_t min_examples_for_split_;
  /** Thresholds to be used instead of generating uniform distributed thresholds. */
  std::vector<float> thresholds_;
  /** The data provider which is called before training a specific tree, if pointer is
   *   NULL, then data_set_ is used. */
  typename pcl::DecisionTreeTrainerDataProvider<FeatureType,
                                                DataSet,
                                                LabelType,
                                                ExampleIndex,
                                                NodeType>::Ptr
      decision_tree_trainer_data_provider_;
  /** If true, random features are generated at each node, otherwise, at start of
   *  training the tree */
  bool random_features_at_split_node_;
};

} // namespace pcl

#include <pcl/ml/impl/dt/decision_tree_trainer.hpp>
