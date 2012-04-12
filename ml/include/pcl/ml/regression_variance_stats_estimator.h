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
#include <pcl/ml/dt/stats_estimator.h>

#include <istream>
#include <ostream>

namespace pcl
{

  /** \brief Statistics estimator for regression trees optimizing variance. */
  template <class LabelDataType, class NodeType, class DataSet, class ExampleIndex>
  class RegressionVarianceStatsEstimator
    : public pcl::StatsEstimator<LabelDataType, NodeType, DataSet, ExampleIndex>
  {
  
    public:
      /** \brief Constructor. */
      RegressionVarianceStatsEstimator () {}
      /** \brief Destructor. */
      virtual ~RegressionVarianceStatsEstimator () {}
  
      /** \brief Returns the number of branches the corresponding tree has. */
      size_t 
      getNumOfBranches () const 
      { 
        return 2; 
      }

      /** \brief Returns the label of the specified node. 
        * \param[in] node The node which label is returned.
        */
      LabelDataType 
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
        const int numOfElements = examples.size ();

        const int numOfBranches = getNumOfBranches();

        // compute variance
        std::vector<LabelDataType> sums (numOfBranches+1, 0);
        std::vector<LabelDataType> sqrSums (numOfBranches+1, 0);
        std::vector<int> branchElementCount (numOfBranches+1, 0);

        for (int elementIndex = 0; elementIndex < numOfElements; ++elementIndex)
        {
          const int branchIndex = results[elementIndex] > threshold;
          LabelDataType label = label_data[elementIndex];

          sums[branchIndex] += label;
          sums[numOfBranches] += label;

          sqrSums[branchIndex] += label*label;
          sqrSums[numOfBranches] += label*label;

          ++branchElementCount[branchIndex];
          ++branchElementCount[numOfBranches];
        }

        std::vector<float> variances (numOfBranches+1, 0);
        for (int branchIndex = 0; branchIndex < numOfBranches+1; ++branchIndex)
        {
          const float meanSum = static_cast<float>(sums[branchIndex]) / branchElementCount[branchIndex];
          const float meanSqrSum = static_cast<float>(sqrSums[branchIndex]) / branchElementCount[branchIndex];
          variances[branchIndex] = meanSqrSum - meanSum*meanSum;
        }

        float informationGain = variances[numOfBranches];
        for (int branchIndex = 0; branchIndex < numOfBranches; ++branchIndex)
        {
          //const float weight = static_cast<float>(sums[branchIndex]) / sums[numOfBranches];
          const float weight = static_cast<float>(branchElementCount[branchIndex]) / static_cast<float>(branchElementCount[numOfBranches]);
          informationGain -= weight*variances[branchIndex];
        }

        return informationGain;
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
        const int numOfBranches = 2;
        const int numOfElements = results.size ();

        branch_indices.resize (numOfElements);
        for (int elementIndex = 0; elementIndex < numOfElements; ++elementIndex)
        {
          const int branchIndex = results[elementIndex] > threshold;
          branch_indices[elementIndex] = branchIndex;
        }
      }

      /** \brief Computes the branch index for the specified result.
        * \param[in] result The result the branch index will be computed for.
        * \param[in] flag The flag corresponding to the specified result.
        * \param[in] threshold The threshold used to compute the branch index.
        * \param[out] branch_index The destination for the computed branch index.
        */
      void 
      computeBranchIndex(
        const float result,
        const unsigned char flag,
        const float threshold,
        unsigned char & branch_index) const
      {
        branch_index = result > threshold;
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
        const int numOfElements = examples.size ();

        LabelDataType sum = 0.0f;
        LabelDataType sqrSum = 0.0f;
        for (int elementIndex = 0; elementIndex < numOfElements; ++elementIndex)
        {
          const LabelDataType label = label_data[elementIndex];

          sum += label;
          sqrSum += label*label;
        }

        sum /= numOfElements;
        sqrSum /= numOfElements;

        const float variance = sqrSum - sum*sum;

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

  };

}

#endif
