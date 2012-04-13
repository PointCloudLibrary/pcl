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
  
#ifndef PCL_ML_DT_STATS_ESTIMATOR_H_
#define PCL_ML_DT_STATS_ESTIMATOR_H_

#include <pcl/common/common.h>

#include <ostream>
#include <vector>

namespace pcl
{

  /** \brief Class interface for gathering statistics for decision tree learning. */
  template <
    class LabelDataType, 
    class NodeType, 
    class DataSet,
    class ExampleIndex >
  class PCL_EXPORTS StatsEstimator
  {
  
    public:

      /** \brief Destructor. */
      virtual 
      ~StatsEstimator () {};

      /** \brief Returns the number of brances a node can have (e.g. a binary tree has 2). */
      virtual size_t 
      getNumOfBranches () const = 0;

      /** \brief Computes and sets the statistics for a node. 
        * \param[in] data_set The data set used for training.
        * \param[in] examples The examples used for computing the statistics for the specified node.
        * \param[in] label_data The labels corresponding to the examples.
        * \param[out] node The destination node for the statistics.
        */
      virtual void 
      computeAndSetNodeStats (DataSet & data_set,
                              std::vector<ExampleIndex> & examples,
                              std::vector<LabelDataType> & label_data,
                              NodeType & node ) const = 0;

      /** \brief Returns the label of the specified node. 
        * \param[in] node The node from which the label is extracted. */
      virtual LabelDataType 
      getLabelOfNode (NodeType & node) const = 0;

      /** \brief Computes the information gain obtained by the specified threshold on the supplied feature evaluation results.
        * \param[in] data_set The data set used for extracting the supplied result values.
        * \param[in] examples The examples used to extract the supplied result values.
        * \param[in] label_data The labels corresponding to the examples.
        * \param[in] results The results obtained from the feature evaluation.
        * \param[in] flags The flags obtained together with the results.
        * \param[in] threshold The threshold which is used to compute the information gain.
        */
      virtual float 
      computeInformationGain (DataSet & data_set,
                              std::vector<ExampleIndex> & examples,
                              std::vector<LabelDataType> & label_data,
                              std::vector<float> & results,
                              std::vector<unsigned char> & flags,
                              const float threshold) const = 0;

      /** \brief Computes the branch indices obtained by the specified threshold on the supplied feature evaluation results.
        * \param[in] results The results obtained from the feature evaluation.
        * \param[in] flags The flags obtained together with the results.
        * \param[in] threshold The threshold which is used to compute the branch indices.
        * \param[out] branch_indices The destination for the computed branch indices.
        */
      virtual void 
      computeBranchIndices (std::vector<float> & results,
                            std::vector<unsigned char> & flags,
                            const float threshold,
                            std::vector<unsigned char> & branch_indices) const = 0;

      /** \brief Computes the branch indices obtained by the specified threshold on the supplied feature evaluation results.
        * \param[in] result The result obtained from the feature evaluation.
        * \param[in] flag The flag obtained together with the result.
        * \param[in] threshold The threshold which is used to compute the branch index.
        * \param[out] branch_index The destination for the computed branch index.
        */
      virtual void 
      computeBranchIndex (const float result,
                          const unsigned char flag,
                          const float threshold,
                          unsigned char & branch_index) const = 0;

      /** \brief Generates code for computing the branch indices for the specified node and writes it to the specified stream.
        * \param[in] node The node for which the branch index estimation code is generated.
        * \param[out] stream The destionation for the code.
        */
      virtual void 
      generateCodeForBranchIndexComputation (NodeType & node,
                                             std::ostream & stream) const = 0;

      /** \brief Generates code for computing the output for the specified node and writes it to the specified stream.
        * \param[in] node The node for which the output estimation code is generated.
        * \param[out] stream The destionation for the code.
        */
      virtual void 
      generateCodeForOutput (NodeType & node,
                             std::ostream & stream ) const = 0;

  };

}

#endif
