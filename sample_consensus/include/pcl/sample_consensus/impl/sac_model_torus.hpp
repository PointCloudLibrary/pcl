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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CYLINDER_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CYLINDER_H_

#include <pcl/sample_consensus/sac_model_torus.h>
#include <pcl/common/common.h> // for getAngle3D
#include <pcl/common/concatenate.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelTorus<PointT, PointNT>::isSampleGood (const Indices &samples) const
{
  //TODO implement
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelTorus<PointT, PointNT>::computeModelCoefficients (
      const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
  //TODO implement
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
void pcl::SampleConsensusModelTorus<PointT, PointNT>::projectPointToPlane(const Eigen::Vector4f& p,
                         const Eigen::Vector4d& plane_coefficientsd,
                         Eigen::Vector4f& q) const {

  Eigen::Vector4f plane_coefficients = plane_coefficientsd.cast<float>();
  //TODO careful with Vector4f
  // use normalized coefficients to calculate the scalar projection
  float distance_to_plane = p.dot(plane_coefficients);
  q = p - distance_to_plane * plane_coefficients;

  assert(q[3] == 0.f);


}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelTorus<PointT, PointNT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelTorus<PointT, PointNT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold, Indices &inliers)
{
  //TODO implement
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> std::size_t
pcl::SampleConsensusModelTorus<PointT, PointNT>::countWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  //TODO implement
  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelTorus<PointT, PointNT>::optimizeModelCoefficients (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
{
  //TODO implement
}

Eigen::Matrix3d toRotationMatrix(double theta, double rho) {
  Eigen::Matrix3d rx{
      {1, 0, 0}, {0, cos(theta), -sin(theta)}, {0, sin(theta), cos(theta)}};

  Eigen::Matrix3d ry{
      {cos(rho), 0, sin(rho)}, {0, 1, 0}, {-sin(rho), 0, cos(rho)}};
  return ry * rx;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelTorus<PointT, PointNT>::projectPoints (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields) const
{
    // Needs a valid set of model coefficients
    if (!isModelValid (model_coefficients))
    {
      PCL_ERROR ("[pcl::SampleConsensusModelCylinder::projectPoints] Given model is invalid!\n");
      return;
    }

    projected_points.header = input_->header;
    projected_points.is_dense = input_->is_dense;



    // Fetch optimization parameters
    const double& R = model_coefficients[0];
    const double& r = model_coefficients[1];

    const double& x0 = model_coefficients[2];
    const double& y0 = model_coefficients[3];
    const double& z0 = model_coefficients[4];

    const double& theta = model_coefficients[5];
    const double& rho = model_coefficients[6];

    // Normal of the plane where the torus circle lies
    Eigen::Matrix3f rot = toRotationMatrix(theta, rho).cast<float>();
    Eigen::Vector4f n{0, 0, 1, 0};
    Eigen::Vector4f pt0{x0, y0, z0, 0};

    // Rotate the normal
    n.head<3>() = rot * n.head<3>();
    // Ax + By + Cz + D = 0
    double D = - n.dot(pt0);
    Eigen::Vector4d planeCoeffs{n[0], n[1], n[2], D};
    planeCoeffs.normalized();

    for (const auto &inlier : inliers)
    {
      Eigen::Vector4f p ((*input_)[inlier].x,
                         (*input_)[inlier].y,
                         (*input_)[inlier].z,
                         0);


      // Project to the torus circle plane
      Eigen::Vector4f pt_proj;
      projectPointToPlane(p, planeCoeffs, pt_proj);


      // TODO expect singularities, mainly pt_proj_e == pt0 || pt_e ==
      // Closest point from the inner circle to the current point
      Eigen::Vector4f circle_closest;
      circle_closest = (pt_proj - pt0).normalized() * R + pt0;



      // From the that closest point we move towards the goal point until we
      // meet the surface of the torus
      Eigen::Vector4f torus_closest =
          (p - circle_closest).normalized() * r + circle_closest;


      pcl::Vector4fMap pp = projected_points[inlier].getVector4fMap ();

      pp = Eigen::Vector4f{torus_closest[0], torus_closest[1], torus_closest[2], 0};
   }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelTorus<PointT, PointNT>::doSamplesVerifyModel (
      const std::set<index_t> &indices, const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  //TODO implement
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelTorus<PointT, PointNT>::projectPointToTorus (
      const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients, Eigen::Vector4f &pt_proj) const
{
  //TODO implement
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelTorus<PointT, PointNT>::isModelValid (const Eigen::VectorXf &model_coefficients) const
{
  if (!SampleConsensusModel<PointT>::isModelValid (model_coefficients))
    return (false);

}

#define PCL_INSTANTIATE_SampleConsensusModelTorus(PointT, PointNT)	template class PCL_EXPORTS pcl::SampleConsensusModelTorus<PointT, PointNT>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CYLINDER_H_

