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

#include <pcl/common/concatenate.h>
#include <pcl/sample_consensus/sac_model_torus.h>

#include <unsupported/Eigen/NonLinearOptimization> // for LevenbergMarquardt

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool
pcl::SampleConsensusModelTorus<PointT>::isSampleGood(const Indices& samples) const
{
  // TODO implement
  (void)samples;
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool
pcl::SampleConsensusModelTorus<PointT>::computeModelCoefficients(
    const Indices& samples, Eigen::VectorXf& model_coefficients) const
{

  // Make sure that the samples are valid
  if (!isSampleGood(samples)) {
    PCL_ERROR("[pcl::SampleConsensusModelCylinder::computeModelCoefficients] Invalid "
              "set of samples given!\n");
    return (false);
  }

  // First pass: Centroid will be estimated to average
  Eigen::Vector3f centroid{0.f, 0.f, 0.f};

  for (auto index : samples) {
    Eigen::Vector3f curr_pt = Eigen::Vector3f((*input_)[index].getVector3fMap());
    centroid = centroid + curr_pt;
  }
  centroid = centroid * (1 / static_cast<float>(samples.size()));

  // Now with the centroid lets do another pass to guess min and max radius relative to
  // centroid
  float R_to_centroid_sq = 0.f;
  float r_to_centroid_sq = std::numeric_limits<float>::max();

  for (auto index : samples) {
    Eigen::Vector3f curr_pt = Eigen::Vector3f((*input_)[index].getVector3fMap());

    float dsq = (curr_pt - centroid).norm();
    if (dsq > R_to_centroid_sq) {
      R_to_centroid_sq = dsq;
      continue; // May be problematic in edge cases since r_to_centroid does not get set
    }

    if (dsq < r_to_centroid_sq) {
      r_to_centroid_sq = dsq;
    }
  }

  model_coefficients.resize(model_size_);

  // The (big) radius of the torus is the radius of the circunference
  float R = (std::sqrt(R_to_centroid_sq) + std::sqrt(r_to_centroid_sq)) / 2;
  // The (small) radius is the distance from circumference to points
  float r = (std::sqrt(R_to_centroid_sq) - std::sqrt(r_to_centroid_sq)) / 2;

  // Third pass is for normal estimation, it can be merged with the second pass, in the
  // future
  size_t i = 0;
  Eigen::MatrixXf A(samples.size(), 3);
  for (auto index : samples) {
    Eigen::Vector3f curr_pt =
        Eigen::Vector3f((*input_)[index].getVector3fMap()) - centroid;
    A.row(i) = curr_pt;
    A(i, 0) = curr_pt[0];
    A(i, 1) = curr_pt[1];
    A(i, 2) = curr_pt[2];
    i++; // TODO, maybe not range-for here
  }

  // SVD to get the plane normal
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::VectorXf n = Eigen::MatrixXf(svd.matrixV().rightCols(1)).normalized();

  // Flip normals to look up on z axis
  if (n[2] < 0.f)
    n = -n;

  model_coefficients[0] = R;
  model_coefficients[1] = r;

  model_coefficients[2] = centroid[0];
  model_coefficients[3] = centroid[1];
  model_coefficients[4] = centroid[2];

  model_coefficients[5] = n[0];
  model_coefficients[6] = n[1];
  model_coefficients[7] = n[2];

  // TODO: this is not right, I still need to deal with failure in here
  optimizeModelCoefficients(samples, model_coefficients, model_coefficients);
  model_coefficients.tail<3>().normalize();

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::SampleConsensusModelTorus<PointT>::projectPointToPlane(
    const Eigen::Vector3f& p,
    const Eigen::Vector4f& plane_coefficients,
    Eigen::Vector3f& q) const
{

  // TODO careful with Vector4f
  //  use normalized coefficients to calculate the scalar projection
  float distance_to_plane = p.dot(plane_coefficients.head<3>()) + plane_coefficients[3];
  q = p - distance_to_plane * plane_coefficients.head<3>();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::SampleConsensusModelTorus<PointT>::getDistancesToModel(
    const Eigen::VectorXf& model_coefficients, std::vector<double>& distances) const
{

  if (!isModelValid(model_coefficients)) {
    distances.clear();
    return;
  }

  distances.resize(indices_->size());

  // Iterate through the 3d points and calculate the distances to the closest torus
  // point
  for (std::size_t i = 0; i < indices_->size(); ++i) {
    Eigen::Vector3f pt = (*input_)[(*indices_)[i]].getVector3fMap();

    Eigen::Vector3f torus_closest;
    projectPointToTorus(pt, model_coefficients, torus_closest);

    assert(torus_closest[3] == 0.f);

    distances[i] = (torus_closest - pt).norm();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::SampleConsensusModelTorus<PointT>::selectWithinDistance(
    const Eigen::VectorXf& model_coefficients, const double threshold, Indices& inliers)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid(model_coefficients)) {
    inliers.clear();
    return;
  }
  inliers.clear();
  error_sqr_dists_.clear();
  inliers.reserve(indices_->size());
  error_sqr_dists_.reserve(indices_->size());

  for (std::size_t i = 0; i < indices_->size(); ++i) {
    Eigen::Vector3f pt = (*input_)[(*indices_)[i]].getVector3fMap();

    Eigen::Vector3f torus_closest;
    projectPointToTorus(pt, model_coefficients, torus_closest);

    float distance = (torus_closest - pt).norm();

    if (distance < threshold) {
      // Returns the indices of the points whose distances are smaller than the
      // threshold
      inliers.push_back((*indices_)[i]);
      error_sqr_dists_.push_back(distance);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
std::size_t
pcl::SampleConsensusModelTorus<PointT>::countWithinDistance(
    const Eigen::VectorXf& model_coefficients, const double threshold) const
{
  if (!isModelValid(model_coefficients))
    return (0);

  std::size_t nr_p = 0;

  for (std::size_t i = 0; i < indices_->size(); ++i) {
    Eigen::Vector3f pt = (*input_)[(*indices_)[i]].getVector3fMap();

    Eigen::Vector3f torus_closest;
    projectPointToTorus(pt, model_coefficients, torus_closest);

    float distance = (torus_closest - pt).norm();

    if (distance < threshold) {
      nr_p++;
    }
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::SampleConsensusModelTorus<PointT>::optimizeModelCoefficients(
    const Indices& inliers,
    const Eigen::VectorXf& model_coefficients,
    Eigen::VectorXf& optimized_coefficients) const
{

  optimized_coefficients = model_coefficients;

  // Needs a set of valid model coefficients
  if (!isModelValid(model_coefficients)) {
    PCL_ERROR("[pcl::SampleConsensusModelTorus::optimizeModelCoefficients] Given model "
              "is invalid!\n");
    return;
  }

  // Need more than the minimum sample size to make a difference
  if (inliers.size() <= sample_size_) {
    PCL_ERROR("[pcl::SampleConsensusModelTorus::optimizeModelCoefficients] Not enough "
              "inliers to refine/optimize the model's coefficients (%lu)! Returning "
              "the same coefficients.\n",
              inliers.size());
    return;
  }

  OptimizationFunctor functor(this, inliers);
  Eigen::NumericalDiff<OptimizationFunctor> num_diff(functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, double> lm(
      num_diff);
  Eigen::VectorXd coeff(model_size_);
  int info = lm.minimize(coeff);

  if (info) {
    PCL_ERROR("[pcl::SampleConsensusModelTorus::optimizeModelCoefficients] Not enough "
              "inliers to refine/optimize the model's coefficients (%lu)! Returning "
              "the same coefficients.\n",
              inliers.size());
    return;
  }
  else {
    for (Eigen::Index i = 0; i < coeff.size(); ++i)
      optimized_coefficients[i] = static_cast<float>(coeff[i]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void
pcl::SampleConsensusModelTorus<PointT>::projectPointToTorus(
    const Eigen::Vector3f& p_in,
    const Eigen::VectorXf& model_coefficients,
    Eigen::Vector3f& q) const
{

  // Fetch optimization parameters
  const float& R = model_coefficients[0];
  const float& r = model_coefficients[1];

  const float& x0 = model_coefficients[2];
  const float& y0 = model_coefficients[3];
  const float& z0 = model_coefficients[4];

  const float& nx = model_coefficients[5];
  const float& ny = model_coefficients[6];
  const float& nz = model_coefficients[7];

  // Normal of the plane where the torus circle lies
  Eigen::Vector3f n{nx, ny, nz};
  n.normalize();

  Eigen::Vector3f pt0{x0, y0, z0};

  // Ax + By + Cz + D = 0
  float D = -n.dot(pt0);
  Eigen::Vector4f planeCoeffs{n[0], n[1], n[2], D};
  planeCoeffs.normalized();
  Eigen::Vector3f p(p_in);

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
template <typename PointT>
void
pcl::SampleConsensusModelTorus<PointT>::projectPoints(
    const Indices& inliers,
    const Eigen::VectorXf& model_coefficients,
    PointCloud& projected_points,
    bool copy_data_fields) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid(model_coefficients)) {
    PCL_ERROR(
        "[pcl::SampleConsensusModelCylinder::projectPoints] Given model is invalid!\n");
    return;
  }

  // TODO this is not right, copy other implementations
  (void)copy_data_fields;

  projected_points.header = input_->header;
  projected_points.is_dense = input_->is_dense;

  for (const auto& inlier : inliers) {

    Eigen::Vector3f q;
    projectPointToTorus((*input_)[inlier].getVector3fMap(), model_coefficients, q);
    projected_points[inlier].getVector3fMap() = q;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool
pcl::SampleConsensusModelTorus<PointT>::doSamplesVerifyModel(
    const std::set<index_t>& indices,
    const Eigen::VectorXf& model_coefficients,
    const double threshold) const
{
  return true;
  // TODO implement
  (void)indices;
  (void)model_coefficients;
  (void)threshold;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool
pcl::SampleConsensusModelTorus<PointT>::isModelValid(
    const Eigen::VectorXf& model_coefficients) const
{
  return true;
  if (!SampleConsensusModel<PointT>::isModelValid(model_coefficients))
    return (false);
}

#define PCL_INSTANTIATE_SampleConsensusModelTorus(PointT)                              \
  template class PCL_EXPORTS pcl::SampleConsensusModelTorus<PointT>;

#endif // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CYLINDER_H_
