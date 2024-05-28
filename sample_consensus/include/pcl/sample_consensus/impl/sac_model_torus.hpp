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

// clang-format off
#include <pcl/sample_consensus/sac_model_torus.h>
#include <pcl/common/concatenate.h>
// clang-format on

#include <unsupported/Eigen/NonLinearOptimization> // for LevenbergMarquardt
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
bool
pcl::SampleConsensusModelTorus<PointT, PointNT>::isSampleGood(
    const Indices& samples) const
{
  if (samples.size() != sample_size_) {
    PCL_ERROR("[pcl::SampleConsensusTorus::isSampleGood] Wrong number of samples (is "
              "%lu, should be %lu)!\n",
              samples.size(),
              sample_size_);
    return (false);
  }

  Eigen::Vector3f n0 = Eigen::Vector3f((*normals_)[samples[0]].getNormalVector3fMap());
  Eigen::Vector3f n1 = Eigen::Vector3f((*normals_)[samples[1]].getNormalVector3fMap());
  Eigen::Vector3f n2 = Eigen::Vector3f((*normals_)[samples[2]].getNormalVector3fMap());
  Eigen::Vector3f n3 = Eigen::Vector3f((*normals_)[samples[3]].getNormalVector3fMap());

  // Required for numeric stability on computeModelCoefficients
  if (std::abs((n0).cross(n1).squaredNorm()) <
          Eigen::NumTraits<float>::dummy_precision() ||
      std::abs((n0).cross(n2).squaredNorm()) <
          Eigen::NumTraits<float>::dummy_precision() ||
      std::abs((n0).cross(n3).squaredNorm()) <
          Eigen::NumTraits<float>::dummy_precision() ||
      std::abs((n1).cross(n2).squaredNorm()) <
          Eigen::NumTraits<float>::dummy_precision() ||
      std::abs((n1).cross(n3).squaredNorm()) <
          Eigen::NumTraits<float>::dummy_precision()) {
    PCL_ERROR("[pcl::SampleConsensusModelEllipse3D::isSampleGood] Sample points "
              "normals too similar or collinear!\n");
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
crossDot(Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3)
{
  return v1.cross(v2).dot(v3);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
bool
pcl::SampleConsensusModelTorus<PointT, PointNT>::computeModelCoefficients(
    const Indices& samples, Eigen::VectorXf& model_coefficients) const
{

  // Make sure that the samples are valid
  if (!isSampleGood(samples)) {
    PCL_ERROR("[pcl::SampleConsensusModelTorus::computeModelCoefficients] Invalid set "
              "of samples given!\n");
    return (false);
  }

  if (!normals_) {
    PCL_ERROR("[pcl::SampleConsensusModelTorus::computeModelCoefficients] No input "
              "dataset containing normals was given!\n");
    return (false);
  }
  // Find axis using:

  // @article{article,
  // author = {Lukacs, G. and Marshall, David and Martin, R.},
  // year = {2001},
  // month = {09},
  // pages = {},
  // title = {Geometric Least-Squares Fitting of Spheres, Cylinders, Cones and Tori}
  //}

  const Eigen::Vector3f n0 = Eigen::Vector3f((*normals_)[samples[0]].getNormalVector3fMap());
  const Eigen::Vector3f n1 = Eigen::Vector3f((*normals_)[samples[1]].getNormalVector3fMap());
  const Eigen::Vector3f n2 = Eigen::Vector3f((*normals_)[samples[2]].getNormalVector3fMap());
  const Eigen::Vector3f n3 = Eigen::Vector3f((*normals_)[samples[3]].getNormalVector3fMap());

  const Eigen::Vector3f p0 = Eigen::Vector3f((*input_)[samples[0]].getVector3fMap());
  const Eigen::Vector3f p1 = Eigen::Vector3f((*input_)[samples[1]].getVector3fMap());
  const Eigen::Vector3f p2 = Eigen::Vector3f((*input_)[samples[2]].getVector3fMap());
  const Eigen::Vector3f p3 = Eigen::Vector3f((*input_)[samples[3]].getVector3fMap());

  const float a01 = crossDot(n0, n1, n2);
  const float b01 = crossDot(n0, n1, n3);
  const float a0 = crossDot(p2 - p1, n0, n2);
  const float a1 = crossDot(p0 - p2, n1, n2);
  const float b0 = crossDot(p3 - p1, n0, n3);
  const float b1 = crossDot(p0 - p3, n1, n3);
  const float a = crossDot(p0 - p2, p1 - p0, n2);
  const float b = crossDot(p0 - p3, p1 - p0, n3);

  // a10*t0*t1 + a0*t0 + a1*t1 + a = 0
  // b10*t0*t1 + b0*t0 + b1*t1 + b = 0
  //
  // (a0 - b0*a10/b10)* t0 + (a1-b1*a10/b10) *t1 + a - b*a10/b10
  // t0 = k * t1 + p

  const float k = -(a1 - b1 * a01 / b01) / (a0 - b0 * a01 / b01);
  const float p = -(a - b * a01 / b01) / (a0 - b0 * a01 / b01);

  // Second deg eqn.
  //
  // b10*k*t1*t1 + b10*p*t1  | + b0*k *t1 + b0*p | + b1*t1 | + b = 0
  //
  // (b10*k) * t1*t1 + (b10*p + b0*k + b1) * t1  + (b0*p + b)

  const float _a = (b01 * k);
  const float _b = (b01 * p + b0 * k + b1);
  const float _c = (b0 * p + b);

  const float eps = Eigen::NumTraits<float>::dummy_precision();

  // Check for imaginary solutions, or small denominators.
  if ((_b * _b - 4 * _a * _c) < 0 || std::abs(a0 - b0 * a01) < eps ||
      std::abs(b01) < eps || std::abs(_a) < eps) {
    PCL_DEBUG("[pcl::SampleConsensusModelTorus::computeModelCoefficients] Can't "
              "compute model coefficients with this method!\n");
    return (false);
  }

  const float s0 = (-_b + std::sqrt(_b * _b - 4 * _a * _c)) / (2 * _a);
  const float s1 = (-_b - std::sqrt(_b * _b - 4 * _a * _c)) / (2 * _a);

  float r_maj_stddev_cycle1 = std::numeric_limits<float>::max();

  for (float s : {s0, s1}) {

    const float t1 = s;
    const float t0 = k * t1 + p;

    // Direction vector
    Eigen::Vector3f d = ((p1 + n1 * t1) - (p0 + n0 * t0));
    d.normalize();
    // Flip direction, so that the first element of the direction vector is
    // positive, for consistency.
    if (d[0] < 0) {
      d *= -1;
    }

    // Flip normals if required. Note |d| = 1
    // d
    // if (n0.dot(d) / n0.norm() < M_PI / 2 ) n0 = -n0;
    // if (n1.dot(d) / n1.norm() < M_PI / 2 ) n1 = -n1;
    // if (n2.dot(d) / n2.norm() < M_PI / 2 ) n2 = -n2;
    // if (n3.dot(d) / n3.norm() < M_PI / 2 ) n3 = -n3;

    // We fit the points to the plane of the torus.
    // Ax + By + Cz + D = 0
    // We know that all for each point plus its normal
    // times the minor radius will give us a point
    // in that plane
    // Pplane_i = P_i + n_i * r
    // we substitute A,x,B,y,C,z
    // dx *( P_i_x + n_i_x * r ) + dy *( P_i_y + n_i_y * r ) +dz *( P_i_z + n_i_z * r )
    // + D = 0 and finally (dx*P_i_x + dy*P_i_y + dz*P_i_z) + (dx*n_i_x + dy*n_i_y +
    // dz*n_i_z ) * r + D = 0 We can set up a linear least squares system of two
    // variables r and D
    //
    Eigen::MatrixXf A(4, 2);
    A << d.dot(n0), 1, d.dot(n1), 1, d.dot(n2), 1, d.dot(n3), 1;

    Eigen::Matrix<float, -1, -1> B(4, 1);
    B << -d.dot(p0), -d.dot(p1), -d.dot(p2), -d.dot(p3);

    Eigen::Matrix<float, -1, -1> sol;
    sol = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    const float r_min = -sol(0);
    const float D = sol(1);

    // Axis line and plane intersect to find the centroid of the torus
    // We take a random point on the line. We find P_rand + lambda * d belongs in the
    // plane

    const Eigen::Vector3f Pany = (p1 + n1 * t1);

    const float lambda = (-d.dot(Pany) - D) / d.dot(d);

    const Eigen::Vector3f centroid = Pany + d * lambda;

    // Finally, the major radius. The least square solution will be
    // the average in this case.
    const float r_maj = std::sqrt(((p0 - r_min * n0 - centroid).squaredNorm() +
                             (p1 - r_min * n1 - centroid).squaredNorm() +
                             (p2 - r_min * n2 - centroid).squaredNorm() +
                             (p3 - r_min * n3 - centroid).squaredNorm()) /
                            4.f);

    const float r_maj_stddev =
        std::sqrt((std::pow(r_maj - (p0 - r_min * n0 - centroid).norm(), 2) +
                   std::pow(r_maj - (p1 - r_min * n1 - centroid).norm(), 2) +
                   std::pow(r_maj - (p2 - r_min * n2 - centroid).norm(), 2) +
                   std::pow(r_maj - (p3 - r_min * n3 - centroid).norm(), 2)) /
                  4.f);
    // We select the minimum stddev cycle
    if (r_maj_stddev < r_maj_stddev_cycle1) {
      r_maj_stddev_cycle1 = r_maj_stddev;
    }
    else {
      break;
    }

    model_coefficients.resize(model_size_);
    model_coefficients[0] = r_maj;
    model_coefficients[1] = r_min;

    model_coefficients[2] = centroid[0];
    model_coefficients[3] = centroid[1];
    model_coefficients[4] = centroid[2];

    model_coefficients[5] = d[0];
    model_coefficients[6] = d[1];
    model_coefficients[7] = d[2];
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
void
pcl::SampleConsensusModelTorus<PointT, PointNT>::getDistancesToModel(
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
    const Eigen::Vector3f pt = (*input_)[(*indices_)[i]].getVector3fMap();
    const Eigen::Vector3f pt_n = (*normals_)[(*indices_)[i]].getNormalVector3fMap();

    Eigen::Vector3f torus_closest;
    projectPointToTorus(pt, pt_n, model_coefficients, torus_closest);

    distances[i] = (torus_closest - pt).norm();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
void
pcl::SampleConsensusModelTorus<PointT, PointNT>::selectWithinDistance(
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
    const Eigen::Vector3f pt = (*input_)[(*indices_)[i]].getVector3fMap();
    const Eigen::Vector3f pt_n = (*normals_)[(*indices_)[i]].getNormalVector3fMap();

    Eigen::Vector3f torus_closest;
    projectPointToTorus(pt, pt_n, model_coefficients, torus_closest);

    const float distance = (torus_closest - pt).norm();

    if (distance < threshold) {
      // Returns the indices of the points whose distances are smaller than the
      // threshold
      inliers.push_back((*indices_)[i]);
      error_sqr_dists_.push_back(distance);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
std::size_t
pcl::SampleConsensusModelTorus<PointT, PointNT>::countWithinDistance(
    const Eigen::VectorXf& model_coefficients, const double threshold) const
{
  if (!isModelValid(model_coefficients))
    return (0);

  std::size_t nr_p = 0;

  for (std::size_t i = 0; i < indices_->size(); ++i) {
    const Eigen::Vector3f pt = (*input_)[(*indices_)[i]].getVector3fMap();
    const Eigen::Vector3f pt_n = (*normals_)[(*indices_)[i]].getNormalVector3fMap();

    Eigen::Vector3f torus_closest;
    projectPointToTorus(pt, pt_n, model_coefficients, torus_closest);

    const float distance = (torus_closest - pt).norm();

    if (distance < threshold) {
      nr_p++;
    }
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
void
pcl::SampleConsensusModelTorus<PointT, PointNT>::optimizeModelCoefficients(
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

  Eigen::VectorXd coeff = model_coefficients.cast<double>();
  int info = lm.minimize(coeff);

  if (!info) {
    PCL_ERROR(
        "[pcl::SampleConsensusModelTorus::optimizeModelCoefficients] Optimizer returned"
        "with error (%i)! Returning ",
        info);
    return;
  }

  // Normalize direction vector
  coeff.tail<3>().normalize();
  optimized_coefficients = coeff.cast<float>();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
void
pcl::SampleConsensusModelTorus<PointT, PointNT>::projectPointToTorus(
    const Eigen::Vector3f& p_in,
    const Eigen::Vector3f& p_n,
    const Eigen::VectorXf& model_coefficients,
    Eigen::Vector3f& pt_out) const
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
  const float D = -n.dot(pt0);

  // Project to the torus circle plane folling the point normal
  // we want to find lambda such that p + pn_n*lambda lies on the
  // torus plane.
  // A*(pt_x + lambda*pn_x) + B *(pt_y + lambda*pn_y) + ... + D = 0
  // given that: n = [A,B,C]
  // n.dot(P) + lambda*n.dot(pn) + D = 0
  //

  Eigen::Vector3f pt_proj;
  // If the point lies in the torus plane, we just use it as projected

  // C++20 -> [[likely]]
  if (std::abs(n.dot(p_n)) > Eigen::NumTraits<float>::dummy_precision()) {
    float lambda = (-D - n.dot(p_in)) / n.dot(p_n);
    pt_proj = p_in + lambda * p_n;
  }
  else {
    pt_proj = p_in;
  }

  // Closest point from the inner circle to the current point
  const Eigen::Vector3f circle_closest = (pt_proj - pt0).normalized() * R + pt0;

  // From the that closest point we move towards the goal point until we
  // meet the surface of the torus
  pt_out = (p_in - circle_closest).normalized() * r + circle_closest;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
void
pcl::SampleConsensusModelTorus<PointT, PointNT>::projectPoints(
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

  // Copy all the data fields from the input cloud to the projected one?
  if (copy_data_fields) {
    // Allocate enough space and copy the basics
    projected_points.resize(input_->size());
    projected_points.width = input_->width;
    projected_points.height = input_->height;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < input_->size(); ++i)
      // Iterate over each dimension
      pcl::for_each_type<FieldList>(
          NdConcatenateFunctor<PointT, PointT>((*input_)[i], projected_points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (const auto& inlier : inliers) {
      Eigen::Vector3f q;
      const Eigen::Vector3f pt_n = (*normals_)[inlier].getNormalVector3fMap();
      projectPointToTorus(
          (*input_)[inlier].getVector3fMap(), pt_n, model_coefficients, q);
      projected_points[inlier].getVector3fMap() = q;
    }
  }
  else {
    // Allocate enough space and copy the basics
    projected_points.resize(inliers.size());
    projected_points.width = inliers.size();
    projected_points.height = 1;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < inliers.size(); ++i) {
      // Iterate over each dimension
      pcl::for_each_type<FieldList>(NdConcatenateFunctor<PointT, PointT>(
          (*input_)[inliers[i]], projected_points[i]));
    }

    for (const auto& inlier : inliers) {
      Eigen::Vector3f q;
      const Eigen::Vector3f pt_n = (*normals_)[inlier].getNormalVector3fMap();
      projectPointToTorus(
          (*input_)[inlier].getVector3fMap(), pt_n, model_coefficients, q);
      projected_points[inlier].getVector3fMap() = q;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
bool
pcl::SampleConsensusModelTorus<PointT, PointNT>::doSamplesVerifyModel(
    const std::set<index_t>& indices,
    const Eigen::VectorXf& model_coefficients,
    const double threshold) const
{

  for (const auto& index : indices) {
    const Eigen::Vector3f pt_n = (*normals_)[index].getNormalVector3fMap();
    Eigen::Vector3f torus_closest;
    projectPointToTorus((*input_)[index].getVector3fMap(), pt_n, model_coefficients, torus_closest);

    if (((*input_)[index].getVector3fMap() - torus_closest).squaredNorm() > threshold * threshold)
      return (false);
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT>
bool
pcl::SampleConsensusModelTorus<PointT, PointNT>::isModelValid(
    const Eigen::VectorXf& model_coefficients) const
{
  if (!SampleConsensusModel<PointT>::isModelValid(model_coefficients))
    return (false);

  if (radius_min_ != std::numeric_limits<double>::lowest() &&
      (model_coefficients[0] < radius_min_ || model_coefficients[1] < radius_min_)) {
    PCL_DEBUG(
        "[pcl::SampleConsensusModelTorus::isModelValid] Major radius OR minor radius "
        "of torus is/are too small: should be larger than %g, but are {%g, %g}.\n",
        radius_min_,
        model_coefficients[0],
        model_coefficients[1]);
    return (false);
  }
  if (radius_max_ != std::numeric_limits<double>::max() &&
      (model_coefficients[0] > radius_max_ || model_coefficients[1] > radius_max_)) {
    PCL_DEBUG(
        "[pcl::SampleConsensusModelTorus::isModelValid] Major radius OR minor radius "
        "of torus is/are too big: should be smaller than %g, but are {%g, %g}.\n",
        radius_max_,
        model_coefficients[0],
        model_coefficients[1]);
    return (false);
  }
  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelTorus(PointT, PointNT)                     \
  template class PCL_EXPORTS pcl::SampleConsensusModelTorus<PointT, PointNT>;

#endif // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_TORUS_H_
