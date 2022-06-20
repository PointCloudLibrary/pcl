/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_REGISTRATION_GICP_CONVERGENCE_CRITERIA_HPP_
#define PCL_REGISTRATION_GICP_CONVERGENCE_CRITERIA_HPP_

#include <pcl/console/print.h>

namespace pcl {

namespace registration {

template <typename Scalar>
bool
GICPConvergenceCriteria<Scalar>::hasConverged()
{
  if (convergence_state_ != CONVERGENCE_CRITERIA_NOT_CONVERGED) {
    // If it already converged or failed before, reset.
    iterations_similar_transforms_ = 0;
    convergence_state_ = CONVERGENCE_CRITERIA_NOT_CONVERGED;
  }

  bool is_similar = false;

  PCL_DEBUG("[pcl::GICPConvergenceCriteria::hasConverged] Iteration %d out of %d.\n",
            iterations_,
            max_iterations_);
  // 1. Number of iterations has reached the maximum user imposed number of iterations
  if (iterations_ >= max_iterations_) {
    if (!failure_after_max_iter_) {
      convergence_state_ = CONVERGENCE_CRITERIA_ITERATIONS;
      return (true);
    }
    convergence_state_ = CONVERGENCE_CRITERIA_FAILURE_AFTER_MAX_ITERATIONS;
  }

  // 2. The epsilon (difference) between the previous transformation and the current
  // estimated transformation
  /* compute the delta from this iteration */
  // Difference between consecutive transforms
  double delta = 0.;
  for (int k = 0; k < 4; k++) {
    for (int l = 0; l < 4; l++) {
      double ratio = 1;
      if (k < 3 && l < 3) // rotation part of the transform
        ratio = 1. / rotation_epsilon_;
      else
        ratio = 1. / translation_threshold_;
      double c_delta =
          ratio * std::abs(previous_transformation_(k, l) - transformation_(k, l));
      if (c_delta > delta)
        delta = c_delta;
    }
  }
  if (delta < 1) {
    if (iterations_similar_transforms_ >= max_iterations_similar_transforms_) {
      convergence_state_ = CONVERGENCE_CRITERIA_TRANSFORM;
      return (true);
    }
    is_similar = true;
  }

  correspondences_cur_mse_ = calculateMSE(correspondences_);
  PCL_DEBUG("[pcl::GICPConvergenceCriteria::hasConverged] Previous / Current MSE "
            "for correspondences distances is: %f / %f.\n",
            correspondences_prev_mse_,
            correspondences_cur_mse_);

  // 3. The relative sum of Euclidean squared errors is smaller than a user defined
  // threshold Absolute
  if (std::abs(correspondences_cur_mse_ - correspondences_prev_mse_) <
      mse_threshold_absolute_) {
    if (iterations_similar_transforms_ >= max_iterations_similar_transforms_) {
      convergence_state_ = CONVERGENCE_CRITERIA_ABS_MSE;
      return (true);
    }
    is_similar = true;
  }

  // Relative
  if (std::abs(correspondences_cur_mse_ - correspondences_prev_mse_) /
          correspondences_prev_mse_ <
      mse_threshold_relative_) {
    if (iterations_similar_transforms_ >= max_iterations_similar_transforms_) {
      convergence_state_ = CONVERGENCE_CRITERIA_REL_MSE;
      return (true);
    }
    is_similar = true;
  }

  if (is_similar) {
    // Increment the number of transforms that the thresholds are allowed to be similar
    ++iterations_similar_transforms_;
  }
  else {
    // When the transform becomes large, reset.
    iterations_similar_transforms_ = 0;
  }

  correspondences_prev_mse_ = correspondences_cur_mse_;

  return (false);
}

} // namespace registration
} // namespace pcl

#endif // PCL_REGISTRATION_GICP_CONVERGENCE_CRITERIA_HPP_
