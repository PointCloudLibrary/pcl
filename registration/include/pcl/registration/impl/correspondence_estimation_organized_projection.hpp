/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_ORGANIZED_PROJECTION_IMPL_HPP_
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_ORGANIZED_PROJECTION_IMPL_HPP_

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> bool
pcl::registration::CorrespondenceEstimationOrganizedProjection<PointSource, PointTarget, Scalar>::initCompute ()
{
  // Set the target_cloud_updated_ variable to true, so that the kd-tree is not built - it is not needed for this class
  target_cloud_updated_ = false;
  if (!CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute ())
    return (false);

  /// Check if the target cloud is organized
  if (!target_->isOrganized ())
  {
    PCL_WARN ("[pcl::registration::%s::initCompute] Target cloud is not organized.\n", getClassName ().c_str ());
    return (false);
  }

  /// Put the projection matrix together
  projection_matrix_ (0, 0) = fx_;
  projection_matrix_ (1, 1) = fy_;
  projection_matrix_ (0, 2) = cx_;
  projection_matrix_ (1, 2) = cy_;

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::CorrespondenceEstimationOrganizedProjection<PointSource, PointTarget, Scalar>::determineCorrespondences (
    pcl::Correspondences &correspondences,
    double max_distance)
{
  if (!initCompute ())
    return;

  correspondences.resize (indices_->size ());
  size_t c_index = 0;

  for (std::vector<int>::const_iterator src_it = indices_->begin (); src_it != indices_->end (); ++src_it)
  {
    if (isFinite (input_->points[*src_it]))
    {
      Eigen::Vector4f p_src (src_to_tgt_transformation_ * input_->points[*src_it].getVector4fMap ());
      Eigen::Vector3f p_src3 (p_src[0], p_src[1], p_src[2]);
      Eigen::Vector3f uv (projection_matrix_ * p_src3);

      /// Check if the point was behind the camera
      if (uv[2] <= 0)
        continue;

      int u = static_cast<int> (uv[0] / uv[2]);
      int v = static_cast<int> (uv[1] / uv[2]);

      if (u >= 0 && u < static_cast<int> (target_->width) &&
          v >= 0 && v < static_cast<int> (target_->height))
      {
        const PointTarget &pt_tgt = target_->at (u, v);
        if (!isFinite (pt_tgt))
          continue;
        /// Check if the depth difference is larger than the threshold
        if (fabs (uv[2] - pt_tgt.z) > depth_threshold_)
          continue;

        double dist = (p_src3 - pt_tgt.getVector3fMap ()).norm ();
        if (dist < max_distance)
          correspondences[c_index++] =  pcl::Correspondence (*src_it, v * target_->width + u, static_cast<float> (dist));
      }
    }
  }

  correspondences.resize (c_index);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
pcl::registration::CorrespondenceEstimationOrganizedProjection<PointSource, PointTarget, Scalar>::determineReciprocalCorrespondences (
    pcl::Correspondences &correspondences,
    double max_distance)
{
  // Call the normal determineCorrespondences (...), as doing it both ways will not improve the results
  determineCorrespondences (correspondences, max_distance);
}

#endif    // PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_ORGANIZED_PROJECTION_IMPL_HPP_

