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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_TORUS_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_TORUS_H_

#include <pcl/sample_consensus/sac_model_torus.h>
#include <pcl/common/common.h> // for getAngle3D
#include <pcl/common/concatenate.h>
#include <unsupported/Eigen/NonLinearOptimization> // for LevenbergMarquardt

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

  std::cout << "compute model" << std::endl;
  // Make sure that the samples are valid
  if (!isSampleGood (samples) && false)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCylinder::computeModelCoefficients] Invalid set of samples given!\n");
    return (false);
  }



  Eigen::Vector4f center{0.f, 0.f, 0.f, 0.f};
  Eigen::MatrixXf A (samples.size(), 3);

  Eigen::MatrixXf b (samples.size(), 1);
  b.setZero();

  size_t i = 0;



  for (auto index : samples){
    Eigen::Vector3f a =   Eigen::Vector3f((*input_)[index].getVector3fMap());
    A.row(i) = a;
    b(i, 0) = A(i, 2);
    A(i, 2) = 1;
    center += ((*input_)[index].getVector4fMap());
    i++;



  }

  center *= 1 / samples.size();

  float R = 0.f;
  float r = std::numeric_limits<float>::max();

  for (auto index : samples){
    Eigen::Vector3f a =   Eigen::Vector3f((*input_)[index].getVector3fMap());

    float dsq = (a - center.head<3>()).norm();
    if(dsq  > R ){
      R = dsq;
      continue;
    }

    if(dsq < r){
      r = dsq;
      continue;
    }
  }

 //  Eigen::Vector4f x = A.colPivHouseholderQr().solve(b);

//std::cout << "print A" << A << std::endl;
//std::cout << "solution is :" << x << std::endl;
//std::cout << "-------" << std::endl;
//
//std::cout << x << std::endl;











  model_coefficients.resize (model_size_);

  std::cout << model_size_ << std::endl;

  // Fetch optimization parameters
  //const double& R = model_coefficients[0];
  //const double& r = model_coefficients[1];

  model_coefficients [0] = R;
  model_coefficients [1] = r;

  model_coefficients[2] = center[0];
  model_coefficients[3] = center[1];
  model_coefficients[4] = center[2];

  //const double& theta = model_coefficients[5];
  //const double& rho = model_coefficients[6];

  optimizeModelCoefficients(samples, model_coefficients, model_coefficients);
  return true;

  //A*x = b













}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
void pcl::SampleConsensusModelTorus<PointT, PointNT>::projectPointToPlane(const Eigen::Vector3f& p,
                         const Eigen::Vector4f& plane_coefficients,
                         Eigen::Vector3f& q) const {

  //TODO careful with Vector4f
  // use normalized coefficients to calculate the scalar projection
  float distance_to_plane = p.dot(plane_coefficients.head<3>()) + plane_coefficients[3];
  q = p - distance_to_plane * plane_coefficients.head<3>();

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelTorus<PointT, PointNT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{

  std::cout << "dtm" << std::endl;

  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }

  distances.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances to the closest torus point
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector3f pt = (*input_)[(*indices_)[i]].getVector3fMap();

    Eigen::Vector3f torus_closest;
    projectPointToTorus(pt, model_coefficients, torus_closest);

    assert(torus_closest[3] == 0.f);

    distances[i] = (torus_closest - pt).norm();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelTorus<PointT, PointNT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold, Indices &inliers)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }
  inliers.clear ();
  error_sqr_dists_.clear ();
  inliers.reserve (indices_->size ());
  error_sqr_dists_.reserve (indices_->size ());

  std::cout << "within" << std::endl;
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector3f pt = (*input_)[(*indices_)[i]].getVector3fMap();

    Eigen::Vector3f torus_closest;
    projectPointToTorus(pt, model_coefficients, torus_closest);

    float distance = (torus_closest - pt).norm();

    if (distance < threshold)
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      inliers.push_back ((*indices_)[i]);
      error_sqr_dists_.push_back (distance);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> std::size_t
pcl::SampleConsensusModelTorus<PointT, PointNT>::countWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  if (!isModelValid (model_coefficients))
    return (0);

  std::size_t nr_p = 0;

  std::cout << "count within" << std::endl;

  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector3f pt = (*input_)[(*indices_)[i]].getVector3fMap();

    Eigen::Vector3f torus_closest;
    projectPointToTorus(pt, model_coefficients, torus_closest);

    float distance = (torus_closest - pt).norm();

    if (distance < threshold)
    {
      nr_p++;
    }
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelTorus<PointT, PointNT>::optimizeModelCoefficients (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
{

  std::cout << "optimize model coeffs" << std::endl;
  optimized_coefficients = model_coefficients;

  // Needs a set of valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::optimizeModelCoefficients] Given model is invalid!\n");
    return;
  }

  // Need more than the minimum sample size to make a difference
  if (inliers.size () <= sample_size_ && false)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::optimizeModelCoefficients] Not enough inliers to refine/optimize the model's coefficients (%lu)! Returning the same coefficients.\n", inliers.size ());
    //return;
  }

  OptimizationFunctor2 functor(this, inliers);
  Eigen::NumericalDiff<OptimizationFunctor2> num_diff(functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor2>, double> lm(num_diff);
  Eigen::VectorXd coeff(7);
  std::cout << "is this happening? " << std::endl;
  int info = lm.minimize(coeff);
  for (Eigen::Index i = 0; i < coeff.size (); ++i)
    optimized_coefficients[i] = static_cast<float> (coeff[i]);

  std::cout << optimized_coefficients << std::endl;
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
pcl::SampleConsensusModelTorus<PointT, PointNT>::projectPointToTorus(
       const Eigen::Vector3f& p_in,
       const Eigen::VectorXf& model_coefficients,
       Eigen::Vector3f& q) const
{

  //std::cout << "projectPointToTorus" << std::endl;



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
    Eigen::Vector3f n{0, 0, 1};
    Eigen::Vector3f pt0{x0, y0, z0};

    // Rotate the normal
    n = rot * n;
    // Ax + By + Cz + D = 0
    double D = - n.dot(pt0);
    Eigen::Vector4f planeCoeffs{n[0], n[1], n[2], D};
    planeCoeffs.normalized();
    Eigen::Vector3f p (p_in);


    // Project to the torus circle plane
    Eigen::Vector3f pt_proj;
    projectPointToPlane(p, planeCoeffs, pt_proj);


    // TODO expect singularities, mainly pt_proj_e == pt0 || pt_e ==
    // Closest point from the inner circle to the current point
    Eigen::Vector3f circle_closest;
    circle_closest = (pt_proj - pt0).normalized() * R + pt0;



    // From the that closest point we move towards the goal point until we
    // meet the surface of the torus
    Eigen::Vector3f torus_closest =
        (p - circle_closest).normalized() * r + circle_closest;

    q = torus_closest;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelTorus<PointT, PointNT>::projectPoints (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields) const
{
    std::cout << "projectPoints" << std::endl;
    // Needs a valid set of model coefficients
    if (!isModelValid (model_coefficients))
    {
      PCL_ERROR ("[pcl::SampleConsensusModelCylinder::projectPoints] Given model is invalid!\n");
      return;
    }

    projected_points.header = input_->header;
    projected_points.is_dense = input_->is_dense;


    std::cout << "project points" << std::endl;

    for (const auto &inlier : inliers)
    {

      Eigen::Vector3f q;
      projectPointToTorus((*input_)[inlier].getVector3fMap(), model_coefficients, q);
      projected_points[inlier].getVector3fMap() = q;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelTorus<PointT, PointNT>::doSamplesVerifyModel (
      const std::set<index_t> &indices, const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  return true;
  //TODO implement
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelTorus<PointT, PointNT>::isModelValid (const Eigen::VectorXf &model_coefficients) const
{
  return true;
  if (!SampleConsensusModel<PointT>::isModelValid (model_coefficients))
    return (false);

}

#define PCL_INSTANTIATE_SampleConsensusModelTorus(PointT, PointNT)	template class PCL_EXPORTS pcl::SampleConsensusModelTorus<PointT, PointNT>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CYLINDER_H_

