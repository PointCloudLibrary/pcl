/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_PERPENDICULAR_PLANE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_PERPENDICULAR_PLANE_H_

#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelPerpendicularPlane<PointT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }

  SampleConsensusModelPlane<PointT>::selectWithinDistance (model_coefficients, threshold, inliers);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::SampleConsensusModelPerpendicularPlane<PointT>::countWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return (0);

  return (SampleConsensusModelPlane<PointT>::countWithinDistance (model_coefficients, threshold));
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelPerpendicularPlane<PointT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }

  SampleConsensusModelPlane<PointT>::getDistancesToModel (model_coefficients, distances);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelPerpendicularPlane<PointT>::isModelValid (const Eigen::VectorXf &model_coefficients)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 4)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelPerpendicularPlane::isModelValid] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return (false);
  }

  // Check against template, if given
  if (eps_angle_ > 0.0)
  {
    // Obtain the plane normal
    Eigen::Vector4f coeff = model_coefficients;
    coeff[3] = 0;

    Eigen::Vector4f axis (axis_[0], axis_[1], axis_[2], 0);
    double angle_diff = fabs (getAngle3D (axis, coeff));
    angle_diff = (std::min) (angle_diff, M_PI - angle_diff);
    // Check whether the current plane model satisfies our angle threshold criterion with respect to the given axis
    if (angle_diff > eps_angle_)
      return (false);
  }

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelPerpendicularPlane(T) template class PCL_EXPORTS pcl::SampleConsensusModelPerpendicularPlane<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_PERPENDICULAR_PLANE_H_

