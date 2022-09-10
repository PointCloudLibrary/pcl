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
#include <pcl/ml/feature_handler.h>
#include <pcl/ml/stats_estimator.h>

#include <vector>

namespace pcl {

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
DecisionTreeEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    DecisionTreeEvaluator() = default;

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
DecisionTreeEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    ~DecisionTreeEvaluator() = default;

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
DecisionTreeEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    evaluate(pcl::DecisionTree<NodeType>& tree,
             pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
             pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>&
                 stats_estimator,
             DataSet& data_set,
             std::vector<ExampleIndex>& examples,
             std::vector<LabelType>& label_data)
{
  const std::size_t num_of_examples = examples.size();
  label_data.resize(num_of_examples);
  for (int example_index = 0; example_index < num_of_examples; ++example_index) {
    NodeType* node = &(tree.getRoot());

    while (node->sub_nodes.size() != 0) {
      float feature_result = 0.0f;
      unsigned char flag = 0;
      unsigned char branch_index = 0;

      feature_handler.evaluateFeature(
          node->feature, data_set, examples[example_index], feature_result, flag);
      stats_estimator.computeBranchIndex(
          feature_result, flag, node->threshold, branch_index);

      node = &(node->sub_nodes[branch_index]);
    }

    label_data[example_index] = stats_estimator.getLabelOfNode(*node);
  }
}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
DecisionTreeEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    evaluateAndAdd(
        pcl::DecisionTree<NodeType>& tree,
        pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
        pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>&
            stats_estimator,
        DataSet& data_set,
        std::vector<ExampleIndex>& examples,
        std::vector<LabelType>& label_data)
{
  const std::size_t num_of_examples = examples.size();
  for (int example_index = 0; example_index < num_of_examples; ++example_index) {
    NodeType* node = &(tree.getRoot());

    while (node->sub_nodes.size() != 0) {
      float feature_result = 0.0f;
      unsigned char flag = 0;
      unsigned char branch_index = 0;

      feature_handler.evaluateFeature(
          node->feature, data_set, examples[example_index], feature_result, flag);
      stats_estimator.computeBranchIndex(
          feature_result, flag, node->threshold, branch_index);

      node = &(node->sub_nodes[branch_index]);
    }

    label_data[example_index] += stats_estimator.getLabelOfNode(*node);
  }
}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
DecisionTreeEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    evaluate(pcl::DecisionTree<NodeType>& tree,
             pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
             pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>&
                 stats_estimator,
             DataSet& data_set,
             ExampleIndex example,
             NodeType& leave)
{
  NodeType* node = &(tree.getRoot());

  while (!node->sub_nodes.empty()) {
    float feature_result = 0.0f;
    unsigned char flag = 0;
    unsigned char branch_index = 0;

    feature_handler.evaluateFeature(
        node->feature, data_set, example, feature_result, flag);
    stats_estimator.computeBranchIndex(
        feature_result, flag, node->threshold, branch_index);

    node = &(node->sub_nodes[branch_index]);
  }

  leave = *node;
}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
DecisionTreeEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    getNodes(pcl::DecisionTree<NodeType>& tree,
             pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
             pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>&
                 stats_estimator,
             DataSet& data_set,
             std::vector<ExampleIndex>& examples,
             std::vector<NodeType*>& nodes)
{
  const std::size_t num_of_examples = examples.size();
  for (int example_index = 0; example_index < num_of_examples; ++example_index) {
    NodeType* node = &(tree.getRoot());

    while (node->sub_nodes.size() != 0) {
      float feature_result = 0.0f;
      unsigned char flag = 0;
      unsigned char branch_index = 0;

      feature_handler.evaluateFeature(
          node->feature, data_set, examples[example_index], feature_result, flag);
      stats_estimator.computeBranchIndex(
          feature_result, node->threshold, flag, branch_index);

      node = &(node->subNodes[branch_index]);
    }

    nodes.push_back(node);
  }
}

} // namespace pcl
