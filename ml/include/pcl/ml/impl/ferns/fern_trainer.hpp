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
FernTrainer<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::FernTrainer()
: fern_depth_(10)
, num_of_features_(1000)
, num_of_thresholds_(10)
, feature_handler_(nullptr)
, stats_estimator_(nullptr)
, data_set_()
, label_data_()
, examples_()
{}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
FernTrainer<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::train(
    pcl::Fern<FeatureType, NodeType>& fern)
{
  const std::size_t num_of_branches = stats_estimator_->getNumOfBranches();
  const std::size_t num_of_examples = examples_.size();

  // create random features
  std::vector<FeatureType> features;
  feature_handler_->createRandomFeatures(num_of_features_, features);

  // setup fern
  fern.initialize(fern_depth_);

  // evaluate all features
  std::vector<std::vector<float>> feature_results(num_of_features_);
  std::vector<std::vector<unsigned char>> flags(num_of_features_);

  for (std::size_t feature_index = 0; feature_index < num_of_features_;
       ++feature_index) {
    feature_results[feature_index].reserve(num_of_examples);
    flags[feature_index].reserve(num_of_examples);

    feature_handler_->evaluateFeature(features[feature_index],
                                      data_set_,
                                      examples_,
                                      feature_results[feature_index],
                                      flags[feature_index]);
  }

  // iteratively select features and thresholds
  std::vector<std::vector<std::vector<float>>> branch_feature_results(
      num_of_features_); // [feature_index][branch_index][result_index]
  std::vector<std::vector<std::vector<unsigned char>>> branch_flags(
      num_of_features_); // [feature_index][branch_index][flag_index]
  std::vector<std::vector<std::vector<ExampleIndex>>> branch_examples(
      num_of_features_); // [feature_index][branch_index][result_index]
  std::vector<std::vector<std::vector<LabelType>>> branch_label_data(
      num_of_features_); // [feature_index][branch_index][flag_index]

  // - initialize branch feature results and flags
  for (std::size_t feature_index = 0; feature_index < num_of_features_;
       ++feature_index) {
    branch_feature_results[feature_index].resize(1);
    branch_flags[feature_index].resize(1);
    branch_examples[feature_index].resize(1);
    branch_label_data[feature_index].resize(1);

    branch_feature_results[feature_index][0] = feature_results[feature_index];
    branch_flags[feature_index][0] = flags[feature_index];
    branch_examples[feature_index][0] = examples_;
    branch_label_data[feature_index][0] = label_data_;
  }

  for (std::size_t depth_index = 0; depth_index < fern_depth_; ++depth_index) {
    // get thresholds
    std::vector<std::vector<float>> thresholds(num_of_features_);

    for (std::size_t feature_index = 0; feature_index < num_of_features_;
         ++feature_index) {
      thresholds.reserve(num_of_thresholds_);
      createThresholdsUniform(num_of_thresholds_,
                              feature_results[feature_index],
                              thresholds[feature_index]);
    }

    // compute information gain
    int best_feature_index = -1;
    float best_feature_threshold = 0.0f;
    float best_feature_information_gain = 0.0f;

    for (std::size_t feature_index = 0; feature_index < num_of_features_;
         ++feature_index) {
      for (std::size_t threshold_index = 0; threshold_index < num_of_thresholds_;
           ++threshold_index) {
        float information_gain = 0.0f;
        for (std::size_t branch_index = 0;
             branch_index < branch_feature_results[feature_index].size();
             ++branch_index) {
          const float branch_information_gain =
              stats_estimator_->computeInformationGain(
                  data_set_,
                  branch_examples[feature_index][branch_index],
                  branch_label_data[feature_index][branch_index],
                  branch_feature_results[feature_index][branch_index],
                  branch_flags[feature_index][branch_index],
                  thresholds[feature_index][threshold_index]);

          information_gain +=
              branch_information_gain *
              branch_feature_results[feature_index][branch_index].size();
        }

        if (information_gain > best_feature_information_gain) {
          best_feature_information_gain = information_gain;
          best_feature_index = static_cast<int>(feature_index);
          best_feature_threshold = thresholds[feature_index][threshold_index];
        }
      }
    }

    // add feature to the feature list of the fern
    fern.accessFeature(depth_index) = features[best_feature_index];
    fern.accessThreshold(depth_index) = best_feature_threshold;

    // update branch feature results and flags
    for (std::size_t feature_index = 0; feature_index < num_of_features_;
         ++feature_index) {
      std::vector<std::vector<float>>& cur_branch_feature_results =
          branch_feature_results[feature_index];
      std::vector<std::vector<unsigned char>>& cur_branch_flags =
          branch_flags[feature_index];
      std::vector<std::vector<ExampleIndex>>& cur_branch_examples =
          branch_examples[feature_index];
      std::vector<std::vector<LabelType>>& cur_branch_label_data =
          branch_label_data[feature_index];

      const std::size_t total_num_of_new_branches =
          num_of_branches * cur_branch_feature_results.size();

      std::vector<std::vector<float>> new_branch_feature_results(
          total_num_of_new_branches); // [branch_index][example_index]
      std::vector<std::vector<unsigned char>> new_branch_flags(
          total_num_of_new_branches); // [branch_index][example_index]
      std::vector<std::vector<ExampleIndex>> new_branch_examples(
          total_num_of_new_branches); // [branch_index][example_index]
      std::vector<std::vector<LabelType>> new_branch_label_data(
          total_num_of_new_branches); // [branch_index][example_index]

      for (std::size_t branch_index = 0;
           branch_index < cur_branch_feature_results.size();
           ++branch_index) {
        const std::size_t num_of_examples_in_this_branch =
            cur_branch_feature_results[branch_index].size();

        std::vector<unsigned char> branch_indices;
        branch_indices.reserve(num_of_examples_in_this_branch);

        stats_estimator_->computeBranchIndices(cur_branch_feature_results[branch_index],
                                               cur_branch_flags[branch_index],
                                               best_feature_threshold,
                                               branch_indices);

        // split results into different branches
        const std::size_t base_branch_index = branch_index * num_of_branches;
        for (std::size_t example_index = 0;
             example_index < num_of_examples_in_this_branch;
             ++example_index) {
          const std::size_t combined_branch_index =
              base_branch_index + branch_indices[example_index];

          new_branch_feature_results[combined_branch_index].push_back(
              cur_branch_feature_results[branch_index][example_index]);
          new_branch_flags[combined_branch_index].push_back(
              cur_branch_flags[branch_index][example_index]);
          new_branch_examples[combined_branch_index].push_back(
              cur_branch_examples[branch_index][example_index]);
          new_branch_label_data[combined_branch_index].push_back(
              cur_branch_label_data[branch_index][example_index]);
        }
      }

      branch_feature_results[feature_index] = new_branch_feature_results;
      branch_flags[feature_index] = new_branch_flags;
      branch_examples[feature_index] = new_branch_examples;
      branch_label_data[feature_index] = new_branch_label_data;
    }
  }

  // set node statistics
  // - re-evaluate selected features
  std::vector<std::vector<float>> final_feature_results(
      fern_depth_); // [feature_index][example_index]
  std::vector<std::vector<unsigned char>> final_flags(
      fern_depth_); // [feature_index][example_index]
  std::vector<std::vector<unsigned char>> final_branch_indices(
      fern_depth_); // [feature_index][example_index]
  for (std::size_t depth_index = 0; depth_index < fern_depth_; ++depth_index) {
    final_feature_results[depth_index].reserve(num_of_examples);
    final_flags[depth_index].reserve(num_of_examples);
    final_branch_indices[depth_index].reserve(num_of_examples);

    feature_handler_->evaluateFeature(fern.accessFeature(depth_index),
                                      data_set_,
                                      examples_,
                                      final_feature_results[depth_index],
                                      final_flags[depth_index]);

    stats_estimator_->computeBranchIndices(final_feature_results[depth_index],
                                           final_flags[depth_index],
                                           fern.accessThreshold(depth_index),
                                           final_branch_indices[depth_index]);
  }

  // - distribute examples to nodes
  std::vector<std::vector<LabelType>> node_labels(
      0x1 << fern_depth_); // [node_index][example_index]
  std::vector<std::vector<ExampleIndex>> node_examples(
      0x1 << fern_depth_); // [node_index][example_index]

  for (std::size_t example_index = 0; example_index < num_of_examples;
       ++example_index) {
    std::size_t node_index = 0;
    for (std::size_t depth_index = 0; depth_index < fern_depth_; ++depth_index) {
      node_index *= num_of_branches;
      node_index += final_branch_indices[depth_index][example_index];
    }

    node_labels[node_index].push_back(label_data_[example_index]);
    node_examples[node_index].push_back(examples_[example_index]);
  }

  // - compute and set statistics for every node
  const std::size_t num_of_nodes = 0x1 << fern_depth_;
  for (std::size_t node_index = 0; node_index < num_of_nodes; ++node_index) {
    stats_estimator_->computeAndSetNodeStats(data_set_,
                                             node_examples[node_index],
                                             node_labels[node_index],
                                             fern[node_index]);
  }
}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
FernTrainer<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    createThresholdsUniform(const std::size_t num_of_thresholds,
                            std::vector<float>& values,
                            std::vector<float>& thresholds)
{
  // estimate range of values
  float min_value = ::std::numeric_limits<float>::max();
  float max_value = -::std::numeric_limits<float>::max();

  const std::size_t num_of_values = values.size();
  for (int value_index = 0; value_index < num_of_values; ++value_index) {
    const float value = values[value_index];

    if (value < min_value)
      min_value = value;
    if (value > max_value)
      max_value = value;
  }

  const float range = max_value - min_value;
  const float step = range / (num_of_thresholds + 2);

  // compute thresholds
  thresholds.resize(num_of_thresholds);

  for (int threshold_index = 0; threshold_index < num_of_thresholds;
       ++threshold_index) {
    thresholds[threshold_index] = min_value + step * (threshold_index + 1);
  }
}

} // namespace pcl
