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

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
FernEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::FernEvaluator()
{}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
FernEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::evaluate(
    pcl::Fern<FeatureType, NodeType>& fern,
    pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
    pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>& stats_estimator,
    DataSet& data_set,
    std::vector<ExampleIndex>& examples,
    std::vector<LabelType>& label_data)
{
  const std::size_t num_of_examples = examples.size();
  const std::size_t num_of_branches = stats_estimator.getNumOfBranches();
  const std::size_t num_of_features = fern.getNumOfFeatures();

  label_data.resize(num_of_examples);

  std::vector<std::vector<float>> results(num_of_features);
  std::vector<std::vector<unsigned char>> flags(num_of_features);
  std::vector<std::vector<unsigned char>> branch_indices(num_of_features);

  for (std::size_t feature_index = 0; feature_index < num_of_features;
       ++feature_index) {
    results[feature_index].reserve(num_of_examples);
    flags[feature_index].reserve(num_of_examples);
    branch_indices[feature_index].reserve(num_of_examples);

    feature_handler.evaluateFeature(fern.accessFeature(feature_index),
                                    data_set,
                                    examples,
                                    results[feature_index],
                                    flags[feature_index]);
    stats_estimator.computeBranchIndices(results[feature_index],
                                         flags[feature_index],
                                         fern.accessThreshold(feature_index),
                                         branch_indices[feature_index]);
  }

  for (std::size_t example_index = 0; example_index < num_of_examples;
       ++example_index) {
    std::size_t node_index = 0;
    for (std::size_t feature_index = 0; feature_index < num_of_features;
         ++feature_index) {
      node_index *= num_of_branches;
      node_index += branch_indices[feature_index][example_index];
    }

    label_data[example_index] = stats_estimator.getLabelOfNode(fern[node_index]);
  }
}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
FernEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::evaluateAndAdd(
    pcl::Fern<FeatureType, NodeType>& fern,
    pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
    pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>& stats_estimator,
    DataSet& data_set,
    std::vector<ExampleIndex>& examples,
    std::vector<LabelType>& label_data)
{
  const std::size_t num_of_examples = examples.size();
  const std::size_t num_of_branches = stats_estimator.getNumOfBranches();
  const std::size_t num_of_features = fern.getNumOfFeatures();

  std::vector<std::vector<float>> results(num_of_features);
  std::vector<std::vector<unsigned char>> flags(num_of_features);
  std::vector<std::vector<unsigned char>> branch_indices(num_of_features);

  for (std::size_t feature_index = 0; feature_index < num_of_features;
       ++feature_index) {
    results[feature_index].reserve(num_of_examples);
    flags[feature_index].reserve(num_of_examples);
    branch_indices[feature_index].reserve(num_of_examples);

    feature_handler.evaluateFeature(fern.accessFeature(feature_index),
                                    data_set,
                                    examples,
                                    results[feature_index],
                                    flags[feature_index]);
    stats_estimator.computeBranchIndices(results[feature_index],
                                         flags[feature_index],
                                         fern.accessThreshold(feature_index),
                                         branch_indices[feature_index]);
  }

  for (std::size_t example_index = 0; example_index < num_of_examples;
       ++example_index) {
    std::size_t node_index = 0;
    for (std::size_t feature_index = 0; feature_index < num_of_features;
         ++feature_index) {
      node_index *= num_of_branches;
      node_index += branch_indices[feature_index][example_index];
    }

    label_data[example_index] = stats_estimator.getLabelOfNode(fern[node_index]);
  }
}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
FernEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::getNodes(
    pcl::Fern<FeatureType, NodeType>& fern,
    pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
    pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>& stats_estimator,
    DataSet& data_set,
    std::vector<ExampleIndex>& examples,
    std::vector<NodeType*>& nodes)
{
  const std::size_t num_of_examples = examples.size();
  const std::size_t num_of_branches = stats_estimator.getNumOfBranches();
  const std::size_t num_of_features = fern.getNumOfFeatures();

  nodes.reserve(num_of_examples);

  std::vector<std::vector<float>> results(num_of_features);
  std::vector<std::vector<unsigned char>> flags(num_of_features);
  std::vector<std::vector<unsigned char>> branch_indices(num_of_features);

  for (std::size_t feature_index = 0; feature_index < num_of_features;
       ++feature_index) {
    results[feature_index].reserve(num_of_examples);
    flags[feature_index].reserve(num_of_examples);
    branch_indices[feature_index].reserve(num_of_examples);

    feature_handler.evaluateFeature(fern.accessFeature(feature_index),
                                    data_set,
                                    examples,
                                    results[feature_index],
                                    flags[feature_index]);
    stats_estimator.computeBranchIndices(results[feature_index],
                                         flags[feature_index],
                                         fern.accessThreshold(feature_index),
                                         branch_indices[feature_index]);
  }

  for (std::size_t example_index = 0; example_index < num_of_examples;
       ++example_index) {
    std::size_t node_index = 0;
    for (std::size_t feature_index = 0; feature_index < num_of_features;
         ++feature_index) {
      node_index *= num_of_branches;
      node_index += branch_indices[feature_index][example_index];
    }

    nodes.push_back(&(fern[node_index]));
  }
}

} // namespace pcl
