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
  
#ifndef PCL_ML_REGRESSION_VARIANCE_STATS_ESTIMATOR_H_
#define PCL_ML_REGRESSION_VARIANCE_STATS_ESTIMATOR_H_

#include <pcl/common/common.h>
#include <pcl/ml/stats_estimator.h>
#include <pcl/ml/branch_estimator.h>

#include <istream>
#include <ostream>

namespace pcl
{

  /** \brief Node for a regression trees which optimizes variance. */
  template <class FeatureType, class LabelType>
  class PCL_EXPORTS RegressionVarianceNode 
  {
    public:
      /** \brief Constructor. */
      RegressionVarianceNode () : value(0), variance(0), threshold(0), sub_nodes() {}
      /** \brief Destructor. */
      virtual ~RegressionVarianceNode () {}

      /** \brief Serializes the node to the specified stream.
        * \param[out] stream The destination for the serialization.
        */
      inline void 
      serialize (std::ostream & stream) const
      {
        feature.serialize (stream);

        stream.write (reinterpret_cast<const char*> (&threshold), sizeof (threshold));

        stream.write (reinterpret_cast<const char*> (&value), sizeof (value));
        stream.write (reinterpret_cast<const char*> (&variance), sizeof (variance));

        const int num_of_sub_nodes = static_cast<int> (sub_nodes.size ());
        stream.write (reinterpret_cast<const char*> (&num_of_sub_nodes), sizeof (num_of_sub_nodes));
        for (int sub_node_index = 0; sub_node_index < num_of_sub_nodes; ++sub_node_index)
        {
          sub_nodes[sub_node_index].serialize (stream);        
        }
      }

      /** \brief Deserializes a node from the specified stream.
        * \param[in] stream The source for the deserialization.
        */
      inline void 
      deserialize (std::istream & stream)
      {
        feature.deserialize (stream);

        stream.read (reinterpret_cast<char*> (&threshold), sizeof (threshold));

        stream.read (reinterpret_cast<char*> (&value), sizeof (value));
        stream.read (reinterpret_cast<char*> (&variance), sizeof (variance));

        int num_of_sub_nodes;
        stream.read (reinterpret_cast<char*> (&num_of_sub_nodes), sizeof (num_of_sub_nodes));
        sub_nodes.resize (num_of_sub_nodes);

        if (num_of_sub_nodes > 0)
        {
          for (int sub_node_index = 0; sub_node_index < num_of_sub_nodes; ++sub_node_index)
          {
            sub_nodes[sub_node_index].deserialize (stream);
          }
        }
      }

    public:
      /** \brief The feature associated with the node. */
      FeatureType feature;
      /** \brief The threshold applied on the feature response. */
      float threshold;

      /** \brief The label value of this node. */
      LabelType value;
      /** \brief The variance of the labels that ended up at this node during training. */
      LabelType variance;

      /** \brief The child nodes. */
      std::vector<RegressionVarianceNode> sub_nodes;
  };

  /** \brief Statistics estimator for regression trees which optimizes variance. */
  template <class LabelDataType, class NodeType, class DataSet, class ExampleIndex>
  class PCL_EXPORTS RegressionVarianceStatsEstimator
    : public pcl::StatsEstimator<LabelDataType, NodeType, DataSet, ExampleIndex>
  {
  
    public:
      /** \brief Constructor. */
      RegressionVarianceStatsEstimator (BranchEstimator * branch_estimator) 
        : branch_estimator_ (branch_estimator)
      {}
      /** \brief Destructor. */
      virtual ~RegressionVarianceStatsEstimator () {}
  
      /** \brief Returns the number of branches the corresponding tree has. */
      inline size_t 
      getNumOfBranches () const 
      { 
        //return 2; 
        return branch_estimator_->getNumOfBranches ();
      }

      /** \brief Returns the label of the specified node. 
        * \param[in] node The node which label is returned.
        */
      inline LabelDataType 
      getLabelOfNode (
        NodeType & node) const
      {
        return node.value;
      }

      /** \brief Computes the information gain obtained by the specified threshold.
        * \param[in] data_set The data set corresponding to the supplied result data.
        * \param[in] examples The examples used for extracting the supplied result data.
        * \param[in] label_data The label data corresponding to the specified examples.
        * \param[in] results The results computed using the specifed examples.
        * \param[in] flags The flags corresponding to the results.
        * \param[in] threshold The threshold for which the information gain is computed.
        */
      float 
      computeInformationGain (
        DataSet & data_set,
        std::vector<ExampleIndex> & examples,
        std::vector<LabelDataType> & label_data,
        std::vector<float> & results,
        std::vector<unsigned char> & flags,
        const float threshold) const
      {
        const size_t num_of_examples = examples.size ();
        const size_t num_of_branches = getNumOfBranches();

        // compute variance
        std::vector<LabelDataType> sums (num_of_branches+1, 0);
        std::vector<LabelDataType> sqr_sums (num_of_branches+1, 0);
        std::vector<size_t> branch_element_count (num_of_branches+1, 0);

        for (size_t branch_index = 0; branch_index < num_of_branches; ++branch_index)
        {
          branch_element_count[branch_index] = 1;
          ++branch_element_count[num_of_branches];
        }

        for (size_t example_index = 0; example_index < num_of_examples; ++example_index)
        {
          unsigned char branch_index;
          computeBranchIndex (results[example_index], flags[example_index], threshold, branch_index);

          LabelDataType label = label_data[example_index];

          sums[branch_index] += label;
          sums[num_of_branches] += label;

          sqr_sums[branch_index] += label*label;
          sqr_sums[num_of_branches] += label*label;

          ++branch_element_count[branch_index];
          ++branch_element_count[num_of_branches];
        }

        std::vector<float> variances (num_of_branches+1, 0);
        for (size_t branch_index = 0; branch_index < num_of_branches+1; ++branch_index)
        {
          const float mean_sum = static_cast<float>(sums[branch_index]) / branch_element_count[branch_index];
          const float mean_sqr_sum = static_cast<float>(sqr_sums[branch_index]) / branch_element_count[branch_index];
          variances[branch_index] = mean_sqr_sum - mean_sum*mean_sum;
        }

        float information_gain = variances[num_of_branches];
        for (size_t branch_index = 0; branch_index < num_of_branches; ++branch_index)
        {
          //const float weight = static_cast<float>(sums[branchIndex]) / sums[numOfBranches];
          const float weight = static_cast<float>(branch_element_count[branch_index]) / static_cast<float>(branch_element_count[num_of_branches]);
          information_gain -= weight*variances[branch_index];
        }

        return information_gain;
      }

      /** \brief Computes the branch indices for all supplied results.
        * \param[in] results The results the branch indices will be computed for.
        * \param[in] flags The flags corresponding to the specified results.
        * \param[in] threshold The threshold used to compute the branch indices.
        * \param[out] branch_indices The destination for the computed branch indices.
        */
      void 
      computeBranchIndices (
        std::vector<float> & results,
        std::vector<unsigned char> & flags,
        const float threshold,
        std::vector<unsigned char> & branch_indices) const
      {
        const size_t num_of_results = results.size ();
        const size_t num_of_branches = getNumOfBranches();

        branch_indices.resize (num_of_results);
        for (size_t result_index = 0; result_index < num_of_results; ++result_index)
        {
          unsigned char branch_index;
          computeBranchIndex (results[result_index], flags[result_index], threshold, branch_index);
          branch_indices[result_index] = branch_index;
        }
      }

      /** \brief Computes the branch index for the specified result.
        * \param[in] result The result the branch index will be computed for.
        * \param[in] flag The flag corresponding to the specified result.
        * \param[in] threshold The threshold used to compute the branch index.
        * \param[out] branch_index The destination for the computed branch index.
        */
      inline void 
      computeBranchIndex(
        const float result,
        const unsigned char flag,
        const float threshold,
        unsigned char & branch_index) const
      {
        branch_estimator_->computeBranchIndex (result, flag, threshold, branch_index);
        //branch_index = (result > threshold) ? 1 : 0;
      }

      /** \brief Computes and sets the statistics for a node.
        * \param[in] data_set The data set which is evaluated.
        * \param[in] examples The examples which define which parts of the data set are used for evaluation.
        * \param[in] label_data The label_data corresponding to the examples.
        * \param[out] node The destination node for the statistics.
        */
      void 
      computeAndSetNodeStats (
        DataSet & data_set,
        std::vector<ExampleIndex> & examples,
        std::vector<LabelDataType> & label_data,
        NodeType & node) const
      {
        const size_t num_of_examples = examples.size ();

        LabelDataType sum = 0.0f;
        LabelDataType sqr_sum = 0.0f;
        for (size_t example_index = 0; example_index < num_of_examples; ++example_index)
        {
          const LabelDataType label = label_data[example_index];

          sum += label;
          sqr_sum += label*label;
        }

        sum /= num_of_examples;
        sqr_sum /= num_of_examples;

        const float variance = sqr_sum - sum*sum;

        node.value = sum;
        node.variance = variance;
      }

      /** \brief Generates code for branch index computation.
        * \param[in] node The node for which code is generated.
        * \param[out] stream The destination for the generated code.
        */
      void 
      generateCodeForBranchIndexComputation (
        NodeType & node,
        std::ostream & stream) const
      {
        stream << "ERROR: RegressionVarianceStatsEstimator does not implement generateCodeForBranchIndex(...)";
      }

      /** \brief Generates code for label output.
        * \param[in] node The node for which code is generated.
        * \param[out] stream The destination for the generated code.
        */
      void 
      generateCodeForOutput (
        NodeType & node,
        std::ostream & stream) const
      {
        stream << "ERROR: RegressionVarianceStatsEstimator does not implement generateCodeForBranchIndex(...)";
      }

    private:
      /** \brief The branch estimator. */
      pcl::BranchEstimator * branch_estimator_;
  };

}

#endif
