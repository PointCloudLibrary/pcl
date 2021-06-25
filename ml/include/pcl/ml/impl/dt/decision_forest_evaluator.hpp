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
#include <pcl/ml/dt/decision_forest.h>
#include <pcl/ml/dt/decision_forest_evaluator.h>
#include <pcl/ml/feature_handler.h>
#include <pcl/ml/stats_estimator.h>

#include <vector>

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
pcl::DecisionForestEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    DecisionForestEvaluator()
: tree_evaluator_()
{}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
pcl::DecisionForestEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    ~DecisionForestEvaluator() = default;

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
pcl::DecisionForestEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    evaluate(pcl::DecisionForest<NodeType>& forest,
             pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
             pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>&
                 stats_estimator,
             DataSet& data_set,
             std::vector<ExampleIndex>& examples,
             std::vector<LabelType>& label_data)
{
  const std::size_t num_of_examples = examples.size();
  label_data.resize(num_of_examples, 0);

  for (std::size_t forest_index = 0; forest_index < forest.size(); ++forest_index) {
    tree_evaluator_.evaluateAndAdd(forest[forest_index],
                                   feature_handler,
                                   stats_estimator,
                                   data_set,
                                   examples,
                                   label_data);
  }

  const float inv_num_of_trees = 1.0f / static_cast<float>(forest.size());
  for (std::size_t label_index = 0; label_index < label_data.size(); ++label_index) {
    label_data[label_index] *= inv_num_of_trees;
  }
}

template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
void
pcl::DecisionForestEvaluator<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::
    evaluate(pcl::DecisionForest<NodeType>& forest,
             pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
             pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>&
                 stats_estimator,
             DataSet& data_set,
             ExampleIndex example,
             std::vector<NodeType>& leaves)
{
  leaves.resize(forest.size());
  for (std::size_t forest_index = 0; forest_index < forest.size(); ++forest_index) {
    NodeType leave;
    tree_evaluator_.evaluate(forest[forest_index],
                             feature_handler,
                             stats_estimator,
                             data_set,
                             example,
                             leave);
    leaves[forest_index] = leave;
  }
}
