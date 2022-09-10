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

/** Utility class for evaluating a decision tree. */
template <class FeatureType,
          class DataSet,
          class LabelType,
          class ExampleIndex,
          class NodeType>
class DecisionTreeEvaluator {

public:
  /** Constructor. */
  DecisionTreeEvaluator();

  /** Destructor. */
  virtual ~DecisionTreeEvaluator();

  /** Evaluates the specified examples using the supplied tree.
   *
   * \param[in] tree the decision tree
   * \param[in] feature_handler the feature handler used to train the tree
   * \param[in] stats_estimator the statistics estimation instance used while training
   *            the tree
   * \param[in] data_set the data set used for evaluation
   * \param[in] examples the examples that have to be evaluated
   * \param[out] label_data the destination for the resulting label data
   */
  void
  evaluate(
      pcl::DecisionTree<NodeType>& tree,
      pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
      pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>& stats_estimator,
      DataSet& data_set,
      std::vector<ExampleIndex>& examples,
      std::vector<LabelType>& label_data);

  /** Evaluates the specified examples using the supplied tree and adds the
   *  results to the supplied results array.
   *
   * \param[in] tree the decision tree
   * \param[in] feature_handler the feature handler used to train the tree
   * \param[in] stats_estimator the statistics estimation instance used while training
   *            the tree
   * \param[in] data_set the data set used for evaluation
   * \param[in] examples the examples that have to be evaluated
   * \param[out] label_data the destination where the resulting label data is added to
   */
  void
  evaluateAndAdd(
      pcl::DecisionTree<NodeType>& tree,
      pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
      pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>& stats_estimator,
      DataSet& data_set,
      std::vector<ExampleIndex>& examples,
      std::vector<LabelType>& label_data);

  /** Evaluates the specified examples using the supplied tree.
   *
   * \param[in] tree the decision tree
   * \param[in] feature_handler the feature handler used to train the tree
   * \param[in] stats_estimator the statistics estimation instance used while training
   *            the tree
   * \param[in] data_set the data set used for evaluation
   * \param[in] example the example that has to be evaluated
   * \param[out] leave The leave reached by the examples.
   */
  void
  evaluate(
      pcl::DecisionTree<NodeType>& tree,
      pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
      pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>& stats_estimator,
      DataSet& data_set,
      ExampleIndex example,
      NodeType& leave);

  /** Evaluates the specified examples using the supplied tree.
   *
   * \param[in] tree the decision tree
   * \param[in] feature_handler the feature handler used to train the tree
   * \param[in] stats_estimator the statistics estimation instance used while training
   *            the tree
   * \param[in] data_set the data set used for evaluation
   * \param[in] examples the examples that have to be evaluated
   * \param[out] nodes the leaf-nodes reached while evaluation
   */
  void
  getNodes(
      pcl::DecisionTree<NodeType>& tree,
      pcl::FeatureHandler<FeatureType, DataSet, ExampleIndex>& feature_handler,
      pcl::StatsEstimator<LabelType, NodeType, DataSet, ExampleIndex>& stats_estimator,
      DataSet& data_set,
      std::vector<ExampleIndex>& examples,
      std::vector<NodeType*>& nodes);
};

} // namespace pcl

#include <pcl/ml/impl/dt/decision_tree_evaluator.hpp>
