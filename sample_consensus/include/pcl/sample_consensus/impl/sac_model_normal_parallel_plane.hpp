/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2010, Willow Garage, Inc.
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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_NORMAL_PARALLEL_PLANE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_NORMAL_PARALLEL_PLANE_H_

#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelNormalParallelPlane<PointT, PointNT>::isModelValid (const Eigen::VectorXf &model_coefficients)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 4)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelNormalParallelPlane::isModelValid] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return (false);
  }

  // Check against template, if given
  if (eps_angle_ > 0.0)
  {
    // Obtain the plane normal
    Eigen::Vector4f coeff = model_coefficients;
    coeff[3] = 0;
    coeff.normalize ();

    if (fabs (axis_.dot (coeff)) < cos_angle_)
      return  (false);
  }

  if (eps_dist_ > 0.0)
  {
    if (fabs (-model_coefficients[3] - distance_from_origin_) > eps_dist_)
      return (false);
  }

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelNormalParallelPlane(PointT, PointNT) template class PCL_EXPORTS pcl::SampleConsensusModelNormalParallelPlane<PointT, PointNT>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_NORMAL_PARALLEL_PLANE_H_


