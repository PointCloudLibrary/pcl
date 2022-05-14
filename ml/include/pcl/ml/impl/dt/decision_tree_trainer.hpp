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

namespace pcl {

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
DecisionTreeTrainer<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    DecisionTreeTrainer()
: max_tree_depth_(15)
, num_of_features_(1000)
, num_of_thresholds_(10)
, feature_handler_(nullptr)
, stats_estimator_(nullptr)
, data_set_()
, label_data_()
, examples_()
, decision_tree_trainer_data_provider_()
, random_features_at_split_node_(false)
{}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
DecisionTreeTrainer<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    ~DecisionTreeTrainer()
{}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
DecisionTreeTrainer<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::train(
    pcl::DecisionTree<NodeType>& tree)
{
  // create random features
  std::vector<FeatureType> features;

  if (!random_features_at_split_node_)
    feature_handler_->createRandomFeatures(num_of_features_, features);

  // recursively build decision tree
  NodeType root_node;
  tree.setRoot(root_node);

  if (decision_tree_trainer_data_provider_) {
    std::cerr << "use decision_tree_trainer_data_provider_" << std::endl;

    decision_tree_trainer_data_provider_->getDatasetAndLabels(
        data_set_, label_data_, examples_);
    trainDecisionTreeNode(
        features, examples_, label_data_, max_tree_depth_, tree.getRoot());
    label_data_.clear();
    data_set_.clear();
    examples_.clear();
  }
  else {
    trainDecisionTreeNode(
        features, examples_, label_data_, max_tree_depth_, tree.getRoot());
  }
}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
DecisionTreeTrainer<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    trainDecisionTreeNode(std::vector<FeatureType>& features,
                          std::vector<ExampleIndex>& examples,
                          std::vector<LabelType>& label_data,
                          const std::size_t max_depth,
                          NodeType& node)
{
  const std::size_t num_of_examples = examples.size();
  if (num_of_examples == 0) {
    PCL_ERROR(
        "Reached invalid point in decision tree training: Number of examples is 0!\n");
    return;
  };

  if (max_depth == 0) {
    stats_estimator_->computeAndSetNodeStats(data_set_, examples, label_data, node);
    return;
  };

  if (examples.size() < min_examples_for_split_) {
    stats_estimator_->computeAndSetNodeStats(data_set_, examples, label_data, node);
    return;
  }

  if (random_features_at_split_node_) {
    features.clear();
    feature_handler_->createRandomFeatures(num_of_features_, features);
  }

  std::vector<float> feature_results;
  std::vector<unsigned char> flags;

  feature_results.reserve(num_of_examples);
  flags.reserve(num_of_examples);

  // find best feature for split
  int best_feature_index = -1;
  float best_feature_threshold = 0.0f;
  float best_feature_information_gain = 0.0f;

  const std::size_t num_of_features = features.size();
  for (std::size_t feature_index = 0; feature_index < num_of_features;
       ++feature_index) {
    // evaluate features
    feature_handler_->evaluateFeature(
        features[feature_index], data_set_, examples, feature_results, flags);

    // get list of thresholds
    if (!thresholds_.empty()) {
      // compute information gain for each threshold and store threshold with highest
      // information gain
      for (std::size_t threshold_index = 0; threshold_index < thresholds_.size();
           ++threshold_index) {

        const float information_gain =
            stats_estimator_->computeInformationGain(data_set_,
                                                     examples,
                                                     label_data,
                                                     feature_results,
                                                     flags,
                                                     thresholds_[threshold_index]);

        if (information_gain > best_feature_information_gain) {
          best_feature_information_gain = information_gain;
          best_feature_index = static_cast<int>(feature_index);
          best_feature_threshold = thresholds_[threshold_index];
        }
      }
    }
    else {
      std::vector<float> thresholds;
      thresholds.reserve(num_of_thresholds_);
      createThresholdsUniform(num_of_thresholds_, feature_results, thresholds);

      // compute information gain for each threshold and store threshold with highest
      // information gain
      for (std::size_t threshold_index = 0; threshold_index < num_of_thresholds_;
           ++threshold_index) {
        const float threshold = thresholds[threshold_index];

        // compute information gain
        const float information_gain = stats_estimator_->computeInformationGain(
            data_set_, examples, label_data, feature_results, flags, threshold);

        if (information_gain > best_feature_information_gain) {
          best_feature_information_gain = information_gain;
          best_feature_index = static_cast<int>(feature_index);
          best_feature_threshold = threshold;
        }
      }
    }
  }

  if (best_feature_index == -1) {
    stats_estimator_->computeAndSetNodeStats(data_set_, examples, label_data, node);
    return;
  }

  // get branch indices for best feature and best threshold
  std::vector<unsigned char> branch_indices;
  branch_indices.reserve(num_of_examples);
  {
    feature_handler_->evaluateFeature(
        features[best_feature_index], data_set_, examples, feature_results, flags);

    stats_estimator_->computeBranchIndices(
        feature_results, flags, best_feature_threshold, branch_indices);
  }

  stats_estimator_->computeAndSetNodeStats(data_set_, examples, label_data, node);

  // separate data
  {
    const std::size_t num_of_branches = stats_estimator_->getNumOfBranches();

    std::vector<std::size_t> branch_counts(num_of_branches, 0);
    for (std::size_t example_index = 0; example_index < num_of_examples;
         ++example_index) {
      ++branch_counts[branch_indices[example_index]];
    }

    node.feature = features[best_feature_index];
    node.threshold = best_feature_threshold;
    node.sub_nodes.resize(num_of_branches);

    for (std::size_t branch_index = 0; branch_index < num_of_branches; ++branch_index) {
      if (branch_counts[branch_index] == 0) {
        NodeType branch_node;
        stats_estimator_->computeAndSetNodeStats(
            data_set_, examples, label_data, branch_node);
        // branch_node->num_of_sub_nodes = 0;

        node.sub_nodes[branch_index] = branch_node;

        continue;
      }

      std::vector<LabelType> branch_labels;
      std::vector<ExampleIndex> branch_examples;
      branch_labels.reserve(branch_counts[branch_index]);
      branch_examples.reserve(branch_counts[branch_index]);

      for (std::size_t example_index = 0; example_index < num_of_examples;
           ++example_index) {
        if (branch_indices[example_index] == branch_index) {
          branch_examples.push_back(examples[example_index]);
          branch_labels.push_back(label_data[example_index]);
        }
      }

      trainDecisionTreeNode(features,
                            branch_examples,
                            branch_labels,
                            max_depth - 1,
                            node.sub_nodes[branch_index]);
    }
  }
}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
DecisionTreeTrainer<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    createThresholdsUniform(const std::size_t num_of_thresholds,
                            std::vector<float>& values,
                            std::vector<float>& thresholds)
{
  // estimate range of values
  float min_value = ::std::numeric_limits<float>::max();
  float max_value = -::std::numeric_limits<float>::max();

  const std::size_t num_of_values = values.size();
  for (std::size_t value_index = 0; value_index < num_of_values; ++value_index) {
    const float value = values[value_index];

    if (value < min_value)
      min_value = value;
    if (value > max_value)
      max_value = value;
  }

  const float range = max_value - min_value;
  const float step = range / static_cast<float>(num_of_thresholds + 2);

  // compute thresholds
  thresholds.resize(num_of_thresholds);

  for (std::size_t threshold_index = 0; threshold_index < num_of_thresholds;
       ++threshold_index) {
    thresholds[threshold_index] =
        min_value + step * (static_cast<float>(threshold_index + 1));
  }
}

} // namespace pcl
