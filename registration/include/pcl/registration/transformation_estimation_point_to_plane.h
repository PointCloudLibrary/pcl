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

#pragma once

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid.h>

namespace pcl {
namespace registration {
/** @b TransformationEstimationPointToPlane uses Levenberg Marquardt optimization to
 * find the transformation that minimizes the point-to-plane distance between the given
 * correspondences.
 *
 * \author Michael Dixon
 * \ingroup registration
 */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class TransformationEstimationPointToPlane
: public TransformationEstimationLM<PointSource, PointTarget, Scalar> {
public:
  using Ptr = shared_ptr<
      TransformationEstimationPointToPlane<PointSource, PointTarget, Scalar>>;
  using ConstPtr = shared_ptr<
      const TransformationEstimationPointToPlane<PointSource, PointTarget, Scalar>>;

  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;
  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointIndicesPtr = PointIndices::Ptr;
  using PointIndicesConstPtr = PointIndices::ConstPtr;

  using Vector4 = Eigen::Matrix<Scalar, 4, 1>;

  TransformationEstimationPointToPlane() = default;
  ~TransformationEstimationPointToPlane() override = default;

protected:
  Scalar
  computeDistance(const PointSource& p_src, const PointTarget& p_tgt) const override
  {
    // Compute the point-to-plane distance
    Vector4 s(p_src.x, p_src.y, p_src.z, 0);
    Vector4 t(p_tgt.x, p_tgt.y, p_tgt.z, 0);
    Vector4 n(p_tgt.normal_x, p_tgt.normal_y, p_tgt.normal_z, 0);
    return ((s - t).dot(n));
  }

  Scalar
  computeDistance(const Vector4& p_src, const PointTarget& p_tgt) const override
  {
    // Compute the point-to-plane distance
    Vector4 t(p_tgt.x, p_tgt.y, p_tgt.z, 0);
    Vector4 n(p_tgt.normal_x, p_tgt.normal_y, p_tgt.normal_z, 0);
    return ((p_src - t).dot(n));
  }
};
} // namespace registration
} // namespace pcl
