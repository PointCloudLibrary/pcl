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
#include <pcl/common/utils.h> // pcl::utils::ignore
#include <pcl/ml/stats_estimator.h>

#include <istream>
#include <ostream>

namespace pcl {

/** Interface for branch estimators. */
class PCL_EXPORTS BranchEstimator {
public:
  /** Destructor. */
  virtual ~BranchEstimator() {}

  /** Returns the number of branches the corresponding tree has. */
  virtual std::size_t
  getNumOfBranches() const = 0;

  /** Computes the branch index for the specified result.
   *
   * \param[in] result the result the branch index will be computed for
   * \param[in] flag the flag corresponding to the specified result
   * \param[in] threshold the threshold used to compute the branch index
   * \param[out] branch_index the destination for the computed branch index
   */
  virtual void
  computeBranchIndex(const float result,
                     const unsigned char flag,
                     const float threshold,
                     unsigned char& branch_index) const = 0;
};

/** Branch estimator for binary trees where the branch is computed only from the
 *  threshold. */
class PCL_EXPORTS BinaryTreeThresholdBasedBranchEstimator : public BranchEstimator {
public:
  /** Constructor. */
  inline BinaryTreeThresholdBasedBranchEstimator() {}
  /** Destructor. */
  inline ~BinaryTreeThresholdBasedBranchEstimator() {}

  /** Returns the number of branches the corresponding tree has. */
  inline std::size_t
  getNumOfBranches() const override
  {
    return 2;
  }

  /** Computes the branch index for the specified result.
   *
   * \param[in] result the result the branch index will be computed for
   * \param[in] flag the flag corresponding to the specified result
   * \param[in] threshold the threshold used to compute the branch index
   * \param[out] branch_index the destination for the computed branch index
   */
  inline void
  computeBranchIndex(const float result,
                     const unsigned char flag,
                     const float threshold,
                     unsigned char& branch_index) const override
  {
    pcl::utils::ignore(flag);
    branch_index = (result > threshold) ? 1 : 0;
  }
};

/** Branch estimator for ternary trees where one branch is used for missing data
 *  (indicated by flag != 0). */
class PCL_EXPORTS TernaryTreeMissingDataBranchEstimator : public BranchEstimator {
public:
  /** Constructor. */
  inline TernaryTreeMissingDataBranchEstimator() {}
  /** Destructor. */
  inline ~TernaryTreeMissingDataBranchEstimator() {}

  /** \brief Returns the number of branches the corresponding tree has. */
  inline std::size_t
  getNumOfBranches() const override
  {
    return 3;
  }

  /** Computes the branch index for the specified result.
   *
   * \param[in] result the result the branch index will be computed for
   * \param[in] flag the flag corresponding to the specified result
   * \param[in] threshold the threshold used to compute the branch index
   * \param[out] branch_index the destination for the computed branch index
   */
  inline void
  computeBranchIndex(const float result,
                     const unsigned char flag,
                     const float threshold,
                     unsigned char& branch_index) const override
  {
    if (flag == 0)
      branch_index = (result > threshold) ? 1 : 0;
    else
      branch_index = 2;
  }
};

} // namespace pcl
