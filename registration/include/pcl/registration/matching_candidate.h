/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Copyright (C) 2008 Ben Gurion University of the Negev, Beer Sheva, Israel.
 *
 *  All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
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
#include <pcl/registration/registration.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>

namespace pcl {
namespace registration {
/** \brief Container for matching candidate consisting of
 *
 * * fitness score value as a result of the matching algorithm
 * * correspondences between source and target data set
 * * transformation matrix calculated based on the correspondences
 *
 */
struct MatchingCandidate {
  /** \brief Constructor. */
  MatchingCandidate()
  : fitness_score(FLT_MAX), transformation(Eigen::Matrix4f::Identity()){};

  /** \brief Value constructor. */
  MatchingCandidate(float s, const pcl::Correspondences& c, const Eigen::Matrix4f& m)
  : fitness_score(s), correspondences(c), transformation(m){};

  /** \brief Destructor. */
  ~MatchingCandidate(){};

  /** \brief Fitness score of current candidate resulting from matching algorithm. */
  float fitness_score;

  /** \brief Correspondences between source <-> target. */
  pcl::Correspondences correspondences;

  /** \brief Corresponding transformation matrix retrieved using \a corrs. */
  Eigen::Matrix4f transformation;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

using MatchingCandidates =
    std::vector<MatchingCandidate, Eigen::aligned_allocator<MatchingCandidate>>;

/** \brief Sorting of candidates based on fitness score value. */
struct by_score {
  /** \brief Operator used to sort candidates based on fitness score. */
  bool
  operator()(MatchingCandidate const& left, MatchingCandidate const& right)
  {
    return (left.fitness_score < right.fitness_score);
  }
};

}; // namespace registration
}; // namespace pcl
