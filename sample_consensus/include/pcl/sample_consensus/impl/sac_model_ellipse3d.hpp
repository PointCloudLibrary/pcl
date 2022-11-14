/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#pragma once

#include <limits>

#include <unsupported/Eigen/NonLinearOptimization> // for LevenbergMarquardt
#include <pcl/sample_consensus/sac_model_ellipse3d.h>
#include <pcl/common/concatenate.h>

#include <Eigen/Eigenvalues>
#include <complex>


//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelEllipse3D<PointT>::isSampleGood (
    const Indices &samples) const
{
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::isSampleGood] Wrong number of samples (is %lu, should be %lu)!\n", samples.size (), sample_size_);
    return (false);
  }

  // Use three points out of the 6 samples
  const Eigen::Vector3d p0 ((*input_)[samples[0]].x, (*input_)[samples[0]].y, (*input_)[samples[0]].z);
  const Eigen::Vector3d p1 ((*input_)[samples[1]].x, (*input_)[samples[1]].y, (*input_)[samples[1]].z);
  const Eigen::Vector3d p2 ((*input_)[samples[2]].x, (*input_)[samples[2]].y, (*input_)[samples[2]].z);

  // Check if the squared norm of the cross-product is non-zero, otherwise
  // common_helper_vec, which plays an important role in computeModelCoefficients,
  // would likely be ill-formed.
  if ((p1 - p0).cross(p1 - p2).squaredNorm() < Eigen::NumTraits<float>::dummy_precision ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::isSampleGood] Sample points too similar or collinear!\n");
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelEllipse3D<PointT>::computeModelCoefficients (const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
  // Uses 6 samples
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
    return (false);
  }

  model_coefficients.resize (model_size_); // 11 coefs

  const Eigen::Vector3f p0((*input_)[samples[0]].x, (*input_)[samples[0]].y, (*input_)[samples[0]].z);
  const Eigen::Vector3f p1((*input_)[samples[1]].x, (*input_)[samples[1]].y, (*input_)[samples[1]].z);
  const Eigen::Vector3f p2((*input_)[samples[2]].x, (*input_)[samples[2]].y, (*input_)[samples[2]].z);
  const Eigen::Vector3f p3((*input_)[samples[3]].x, (*input_)[samples[3]].y, (*input_)[samples[3]].z);
  const Eigen::Vector3f p4((*input_)[samples[4]].x, (*input_)[samples[4]].y, (*input_)[samples[4]].z);
  const Eigen::Vector3f p5((*input_)[samples[5]].x, (*input_)[samples[5]].y, (*input_)[samples[5]].z);

  const Eigen::Vector3f common_helper_vec = (p1 - p0).cross(p1 - p2);
  const Eigen::Vector3f ellipse_normal = common_helper_vec.normalized();

  // The same check is implemented in isSampleGood, so be sure to look there too
  // if you find the need to change something here.
  if (common_helper_vec.squaredNorm() < Eigen::NumTraits<float>::dummy_precision ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::computeModelCoefficients] Sample points too similar or collinear!\n");
    return (false);
  }

  // Definition of the local reference frame of the ellipse
  Eigen::Vector3f x_axis = (p1 - p0).normalized();
  const Eigen::Vector3f z_axis = ellipse_normal.normalized();
  const Eigen::Vector3f y_axis = z_axis.cross(x_axis).normalized();
  
  // Compute the rotation matrix and its transpose
  const Eigen::Matrix3f Rot = (Eigen::Matrix3f(3,3)
    << x_axis(0), y_axis(0), z_axis(0),
    x_axis(1), y_axis(1), z_axis(1),
    x_axis(2), y_axis(2), z_axis(2))
    .finished();
  const Eigen::Matrix3f Rot_T = Rot.transpose();
  
  // Convert the points to local coordinates
  const Eigen::Vector3f p0_ = Rot_T * (p0 - p0);
  const Eigen::Vector3f p1_ = Rot_T * (p1 - p0);
  const Eigen::Vector3f p2_ = Rot_T * (p2 - p0);
  const Eigen::Vector3f p3_ = Rot_T * (p3 - p0);
  const Eigen::Vector3f p4_ = Rot_T * (p4 - p0);
  const Eigen::Vector3f p5_ = Rot_T * (p5 - p0);


  // Fit an ellipse to the samples to obtain its conic equation parameters
  // (this implementation follows the manuscript "Direct Least Square Fitting of Ellipses"
  //  A. Fitzgibbon, M. Pilu and R. Fisher, IEEE TPAMI, 21(5) : 476–480, May 1999).

  // xOy projections only
  const Eigen::VectorXf X = (Eigen::VectorXf(6) << p0_(0), p1_(0), p2_(0), p3_(0), p4_(0), p5_(0)).finished();
  const Eigen::VectorXf Y = (Eigen::VectorXf(6) << p0_(1), p1_(1), p2_(1), p3_(1), p4_(1), p5_(1)).finished();

  // Design matrix D
  const Eigen::MatrixXf D = (Eigen::MatrixXf(6,6)
    << X(0) * X(0), X(0) * Y(0), Y(0) * Y(0), X(0), Y(0), 1.0,
      X(1) * X(1), X(1) * Y(1), Y(1) * Y(1), X(1), Y(1), 1.0,
      X(2) * X(2), X(2) * Y(2), Y(2) * Y(2), X(2), Y(2), 1.0,
      X(3) * X(3), X(3) * Y(3), Y(3) * Y(3), X(3), Y(3), 1.0,
      X(4) * X(4), X(4) * Y(4), Y(4) * Y(4), X(4), Y(4), 1.0,
      X(5) * X(5), X(5) * Y(5), Y(5) * Y(5), X(5), Y(5), 1.0)
    .finished();

  // Scatter matrix S
  const Eigen::MatrixXf S = D.transpose() * D;

  // Constraint matrix C
  const Eigen::MatrixXf C = (Eigen::MatrixXf(6,6)
    << 0.0, 0.0, -2.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      -2.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    .finished();

  // Solve the Generalized Eigensystem: S*a = lambda*C*a
  Eigen::GeneralizedEigenSolver<Eigen::MatrixXf> solver;
  solver.compute(S, C);
  const Eigen::VectorXf eigvals = solver.eigenvalues().real();

  // Find the negative eigenvalue 'neigvec' (the largest, if many exist)
  int idx(-1);
  float absmin(0.0);
  for (size_t i(0); i < static_cast<size_t>(eigvals.size()); ++i) {
    if (eigvals(i) < absmin && !std::isinf(eigvals(i))) {
      idx = i;
    }
  }
  // Return "false" in case the negative eigenvalue was not found
  if (idx == -1) {
    PCL_DEBUG("[pcl::SampleConsensusModelEllipse3D::computeModelCoefficients] Failed to find the negative eigenvalue in the GES.\n");
    return (false);
  }
  const Eigen::VectorXf neigvec = solver.eigenvectors().real().col(idx).normalized();


  // Convert the conic model parameters to parametric ones

  // Conic parameters
  const float con_A(neigvec(0));
  const float con_B(neigvec(1));
  const float con_C(neigvec(2));
  const float con_D(neigvec(3));
  const float con_E(neigvec(4));
  const float con_F(neigvec(5));

  // Build matrix M0
  const Eigen::MatrixXf M0 = (Eigen::MatrixXf(3, 3)
    << con_F, con_D/2.0, con_E/2.0,
      con_D/2.0, con_A, con_B/2.0,
      con_E/2.0, con_B/2.0, con_C)
    .finished();

  // Build matrix M
  const Eigen::MatrixXf M = (Eigen::MatrixXf(2, 2)
    << con_A, con_B/2.0,
      con_B/2.0, con_C)
    .finished();

  // Calculate the eigenvalues and eigenvectors of matrix M
  Eigen::EigenSolver<Eigen::MatrixXf> solver_M(M);

  Eigen::VectorXf eigvals_M = solver_M.eigenvalues().real();

  // Order the eigenvalues so that |lambda_0 - con_A| <= |lambda_0 - con_C|
  float aux_eigval(0.0);
  if (std::abs(eigvals_M(0) - con_A) > std::abs(eigvals_M(0) - con_C)) {
    aux_eigval = eigvals_M(0);
    eigvals_M(0) = eigvals_M(1);
    eigvals_M(1) = aux_eigval;
  }

  // Parametric parameters of the ellipse
  float par_a = std::sqrt(-M0.determinant() / (M.determinant() * eigvals_M(0)));
  float par_b = std::sqrt(-M0.determinant() / (M.determinant() * eigvals_M(1)));
  const float par_h = (con_B * con_E - 2.0 * con_C * con_D) / (4.0 * con_A * con_C - std::pow(con_B, 2));
  const float par_k = (con_B * con_D - 2.0 * con_A * con_E) / (4.0 * con_A * con_C - std::pow(con_B, 2));
  const float par_t = (M_PI / 2.0 - std::atan((con_A - con_C) / con_B)) / 2.0; // equivalent to: acot((con_A - con_C) / con_B) / 2.0;

  // Convert the center point of the ellipse to global coordinates
  // (the if statement ensures that 'par_a' always refers to the semi-major axis length)
  Eigen::Vector3f p_ctr;
  float aux_par(0.0);
  if (par_a > par_b) {
    p_ctr = p0 + Rot * Eigen::Vector3f(par_h, par_k, 0.0);
  } else {
    aux_par = par_a;
    par_a = par_b;
    par_b = aux_par;
    p_ctr = p0 + Rot * Eigen::Vector3f(par_k, par_h, 0.0);
  }

  // Center (x, y, z)
  model_coefficients[0] = static_cast<float>(p_ctr(0));
  model_coefficients[1] = static_cast<float>(p_ctr(1));
  model_coefficients[2] = static_cast<float>(p_ctr(2));

  // Semi-major axis length 'a' (along the local x-axis)
  model_coefficients[3] = static_cast<float>(par_a);
  // Semi-minor axis length 'b' (along the local y-axis)
  model_coefficients[4] = static_cast<float>(par_b);

  // Ellipse normal
  model_coefficients[5] = static_cast<float>(ellipse_normal[0]);
  model_coefficients[6] = static_cast<float>(ellipse_normal[1]);
  model_coefficients[7] = static_cast<float>(ellipse_normal[2]);

  // Retrive the ellipse point at the tilt angle t (par_t), along the local x-axis
  const Eigen::VectorXf params = (Eigen::VectorXf(5) << par_a, par_b, par_h, par_k, par_t).finished();
  Eigen::Vector3f p_th_(0.0, 0.0, 0.0);
  get_ellipse_point(params, par_t, p_th_(0), p_th_(1));

  // Redefinition of the x-axis of the ellipse's local reference frame
  x_axis = (Rot * p_th_).normalized();
  model_coefficients[8] = static_cast<float>(x_axis[0]);
  model_coefficients[9] = static_cast<float>(x_axis[1]);
  model_coefficients[10] = static_cast<float>(x_axis[2]);


  PCL_DEBUG ("[pcl::SampleConsensusModelEllipse3D::computeModelCoefficients] Model is (%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g).\n",
             model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3],
             model_coefficients[4], model_coefficients[5], model_coefficients[6], model_coefficients[7],
             model_coefficients[8], model_coefficients[9], model_coefficients[10]);
  return (true);
}


//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelEllipse3D<PointT>::getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }
  distances.resize (indices_->size ());

  // c : Ellipse Center
  const Eigen::Vector3f c(model_coefficients[0], model_coefficients[1], model_coefficients[2]);
  // n : Ellipse (Plane) Normal
  const Eigen::Vector3f n_axis(model_coefficients[5], model_coefficients[6], model_coefficients[7]);
  // x : Ellipse (Plane) X-Axis
  const Eigen::Vector3f x_axis(model_coefficients[8], model_coefficients[9], model_coefficients[10]);
  // y : Ellipse (Plane) Y-Axis
  const Eigen::Vector3f y_axis = n_axis.cross(x_axis).normalized();
  // a : Ellipse semi-major axis (X) length
  const float par_a(model_coefficients[3]);
  // b : Ellipse semi-minor axis (Y) length
  const float par_b(model_coefficients[4]);

  // Compute the rotation matrix and its transpose
  const Eigen::Matrix3f Rot = (Eigen::Matrix3f(3,3)
    << x_axis(0), y_axis(0), n_axis(0),
    x_axis(1), y_axis(1), n_axis(1),
    x_axis(2), y_axis(2), n_axis(2))
    .finished();
  const Eigen::Matrix3f Rot_T = Rot.transpose();

  // Ellipse parameters
  const Eigen::VectorXf params = (Eigen::VectorXf(5) << par_a, par_b, 0.0, 0.0, 0.0).finished();
  float th_opt;

  // Iterate through the 3D points and calculate the distances from them to the ellipse
  for (std::size_t i = 0; i < indices_->size (); ++i)
  // Calculate the distance from the point to the ellipse:
  // 1.   calculate intersection point of the plane in which the ellipse lies and the
  //      line from the sample point with the direction of the plane normal (projected point)
  // 2.   calculate the intersection point of the line from the ellipse center to the projected point
  //      with the ellipse
  // 3.   calculate distance from corresponding point on the ellipse to the sample point
  {
    // p : Sample Point
    const Eigen::Vector3f p((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z);
    
    // Local coordinates of sample point p
    const Eigen::Vector3f p_ = Rot_T * (p - c);

    // k : Point on Ellipse
    // Calculate the shortest distance from the point to the ellipse which is given by
    // the norm of a vector that is normal to the ellipse tangent calculated at the
    // point it intersects the tangent.
    const Eigen::Vector2f distanceVector = dvec2ellipse(params, p_(0), p_(1), th_opt);

    distances[i] = distanceVector.norm();
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelEllipse3D<PointT>::selectWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold,
    Indices &inliers)
{
  inliers.clear();
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    return;
  }
  inliers.reserve (indices_->size ());

  // c : Ellipse Center
  const Eigen::Vector3f c(model_coefficients[0], model_coefficients[1], model_coefficients[2]);
  // n : Ellipse (Plane) Normal
  const Eigen::Vector3f n_axis(model_coefficients[5], model_coefficients[6], model_coefficients[7]);
  // x : Ellipse (Plane) X-Axis
  const Eigen::Vector3f x_axis(model_coefficients[8], model_coefficients[9], model_coefficients[10]);
  // y : Ellipse (Plane) Y-Axis
  const Eigen::Vector3f y_axis = n_axis.cross(x_axis).normalized();
  // a : Ellipse semi-major axis (X) length
  const float par_a(model_coefficients[3]);
  // b : Ellipse semi-minor axis (Y) length
  const float par_b(model_coefficients[4]);

  // Compute the rotation matrix and its transpose
  const Eigen::Matrix3f Rot = (Eigen::Matrix3f(3,3)
    << x_axis(0), y_axis(0), n_axis(0),
    x_axis(1), y_axis(1), n_axis(1),
    x_axis(2), y_axis(2), n_axis(2))
    .finished();
  const Eigen::Matrix3f Rot_T = Rot.transpose();

  const auto squared_threshold = threshold * threshold;
  // Iterate through the 3d points and calculate the distances from them to the ellipse
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    // p : Sample Point
    const Eigen::Vector3f p((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z);

    // Local coordinates of sample point p
    const Eigen::Vector3f p_ = Rot_T * (p - c);

    // k : Point on Ellipse
    // Calculate the shortest distance from the point to the ellipse which is given by
    // the norm of a vector that is normal to the ellipse tangent calculated at the
    // point it intersects the tangent.
    const Eigen::VectorXf params = (Eigen::VectorXf(5) << par_a, par_b, 0.0, 0.0, 0.0).finished();
    float th_opt;
    const Eigen::Vector2f distanceVector = dvec2ellipse(params, p_(0), p_(1), th_opt);

    if (distanceVector.squaredNorm() < squared_threshold)
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      inliers.push_back ((*indices_)[i]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> std::size_t
pcl::SampleConsensusModelEllipse3D<PointT>::countWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return (0);
  std::size_t nr_p = 0;

  // c : Ellipse Center
  const Eigen::Vector3f c(model_coefficients[0], model_coefficients[1], model_coefficients[2]);
  // n : Ellipse (Plane) Normal
  const Eigen::Vector3f n_axis(model_coefficients[5], model_coefficients[6], model_coefficients[7]);
  // x : Ellipse (Plane) X-Axis
  const Eigen::Vector3f x_axis(model_coefficients[8], model_coefficients[9], model_coefficients[10]);
  // y : Ellipse (Plane) Y-Axis
  const Eigen::Vector3f y_axis = n_axis.cross(x_axis).normalized();
  // a : Ellipse semi-major axis (X) length
  const float par_a(model_coefficients[3]);
  // b : Ellipse semi-minor axis (Y) length
  const float par_b(model_coefficients[4]);

  // Compute the rotation matrix and its transpose
  const Eigen::Matrix3f Rot = (Eigen::Matrix3f(3,3)
    << x_axis(0), y_axis(0), n_axis(0),
    x_axis(1), y_axis(1), n_axis(1),
    x_axis(2), y_axis(2), n_axis(2))
    .finished();
  const Eigen::Matrix3f Rot_T = Rot.transpose();

  const auto squared_threshold = threshold * threshold;
  // Iterate through the 3d points and calculate the distances from them to the ellipse
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    // p : Sample Point
    const Eigen::Vector3f p((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z);

    // Local coordinates of sample point p
    const Eigen::Vector3f p_ = Rot_T * (p - c);

    // k : Point on Ellipse
    // Calculate the shortest distance from the point to the ellipse which is given by
    // the norm of a vector that is normal to the ellipse tangent calculated at the
    // point it intersects the tangent.
    const Eigen::VectorXf params = (Eigen::VectorXf(5) << par_a, par_b, 0.0, 0.0, 0.0).finished();
    float th_opt;
    const Eigen::Vector2f distanceVector = dvec2ellipse(params, p_(0), p_(1), th_opt);

    if (distanceVector.squaredNorm() < squared_threshold)
      nr_p++;
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelEllipse3D<PointT>::optimizeModelCoefficients (
      const Indices &inliers,
      const Eigen::VectorXf &model_coefficients,
      Eigen::VectorXf &optimized_coefficients) const
{
  optimized_coefficients = model_coefficients;

  // Needs a set of valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::optimizeModelCoefficients] Given model is invalid!\n");
    return;
  }

  // Need more than the minimum sample size to make a difference
  if (inliers.size () <= sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::optimizeModelCoefficients] Not enough inliers to refine/optimize the model's coefficients (%lu)! Returning the same coefficients.\n", inliers.size ());
    return;
  }

  OptimizationFunctor functor(this, inliers);
  Eigen::NumericalDiff<OptimizationFunctor> num_diff(functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, double> lm(num_diff);
  Eigen::VectorXd coeff;
  int info = lm.minimize(coeff);
  for (Eigen::Index i = 0; i < coeff.size (); ++i)
    optimized_coefficients[i] = static_cast<float> (coeff[i]);

  // Compute the L2 norm of the residuals
  PCL_DEBUG ("[pcl::SampleConsensusModelEllipse3D::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g %g %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g %g %g %g %g %g %g %g\n",
            info, lm.fvec.norm (),

            model_coefficients[0],
            model_coefficients[1],
            model_coefficients[2],
            model_coefficients[3],
            model_coefficients[4],
            model_coefficients[5],
            model_coefficients[6],
            model_coefficients[7],
            model_coefficients[8],
            model_coefficients[9],
            model_coefficients[10],

            optimized_coefficients[0],
            optimized_coefficients[1],
            optimized_coefficients[2],
            optimized_coefficients[3],
            optimized_coefficients[4],
            optimized_coefficients[5],
            optimized_coefficients[6],
            optimized_coefficients[7],
            optimized_coefficients[8],
            optimized_coefficients[9],
            optimized_coefficients[10]);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelEllipse3D<PointT>::projectPoints (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients,
      PointCloud &projected_points, bool copy_data_fields) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::projectPoints] Given model is invalid!\n");
    return;
  }

  projected_points.header   = input_->header;
  projected_points.is_dense = input_->is_dense;

  // Copy all the data fields from the input cloud to the projected one?
  if (copy_data_fields)
  {
    // Allocate enough space and copy the basics
    projected_points.resize (input_->size ());
    projected_points.width    = input_->width;
    projected_points.height   = input_->height;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < projected_points.size(); ++i)
    {
      // Iterate over each dimension
      pcl::for_each_type<FieldList>(NdConcatenateFunctor<PointT, PointT>((*input_)[i], projected_points[i]));
    }

    // c : Ellipse Center
    const Eigen::Vector3f c(model_coefficients[0], model_coefficients[1], model_coefficients[2]);
    // n : Ellipse (Plane) Normal
    const Eigen::Vector3f n_axis(model_coefficients[5], model_coefficients[6], model_coefficients[7]);
    // x : Ellipse (Plane) X-Axis
    const Eigen::Vector3f x_axis(model_coefficients[8], model_coefficients[9], model_coefficients[10]);
    // y : Ellipse (Plane) Y-Axis
    const Eigen::Vector3f y_axis = n_axis.cross(x_axis).normalized();
    // a : Ellipse semi-major axis (X) length
    const float par_a(model_coefficients[3]);
    // b : Ellipse semi-minor axis (Y) length
    const float par_b(model_coefficients[4]);

    // Compute the rotation matrix and its transpose
    const Eigen::Matrix3f Rot = (Eigen::Matrix3f(3,3)
      << x_axis(0), y_axis(0), n_axis(0),
      x_axis(1), y_axis(1), n_axis(1),
      x_axis(2), y_axis(2), n_axis(2))
      .finished();
    const Eigen::Matrix3f Rot_T = Rot.transpose();

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (std::size_t i = 0; i < inliers.size (); ++i)
    {
      // p : Sample Point
      const Eigen::Vector3f p((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z);

      // Local coordinates of sample point p
      const Eigen::Vector3f p_ = Rot_T * (p - c);

      // k : Point on Ellipse
      // Calculate the shortest distance from the point to the ellipse which is given by
      // the norm of a vector that is normal to the ellipse tangent calculated at the
      // point it intersects the tangent.
      const Eigen::VectorXf params = (Eigen::VectorXf(5) << par_a, par_b, 0.0, 0.0, 0.0).finished();
      float th_opt;
      dvec2ellipse(params, p_(0), p_(1), th_opt);

      // Retrive the ellipse point at the tilt angle t, along the local x-axis
      Eigen::Vector3f k_(0.0, 0.0, 0.0);
      get_ellipse_point(params, th_opt, k_[0], k_[1]);

      const Eigen::Vector3f k = c + Rot * k_;

      projected_points[i].x = static_cast<float> (k[0]);
      projected_points[i].y = static_cast<float> (k[1]);
      projected_points[i].z = static_cast<float> (k[2]);
    }
  }
  else
  {
    // Allocate enough space and copy the basics
    projected_points.resize (inliers.size ());
    projected_points.width    = inliers.size ();
    projected_points.height   = 1;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < inliers.size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> ((*input_)[inliers[i]], projected_points[i]));

    // c : Ellipse Center
    const Eigen::Vector3f c(model_coefficients[0], model_coefficients[1], model_coefficients[2]);
    // n : Ellipse (Plane) Normal
    const Eigen::Vector3f n_axis(model_coefficients[5], model_coefficients[6], model_coefficients[7]);
    // x : Ellipse (Plane) X-Axis
    const Eigen::Vector3f x_axis(model_coefficients[8], model_coefficients[9], model_coefficients[10]);
    // y : Ellipse (Plane) Y-Axis
    const Eigen::Vector3f y_axis = n_axis.cross(x_axis).normalized();
    // a : Ellipse semi-major axis (X) length
    const float par_a(model_coefficients[3]);
    // b : Ellipse semi-minor axis (Y) length
    const float par_b(model_coefficients[4]);

    // Compute the rotation matrix and its transpose
    const Eigen::Matrix3f Rot = (Eigen::Matrix3f(3,3)
      << x_axis(0), y_axis(0), n_axis(0),
      x_axis(1), y_axis(1), n_axis(1),
      x_axis(2), y_axis(2), n_axis(2))
      .finished();
    const Eigen::Matrix3f Rot_T = Rot.transpose();

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (std::size_t i = 0; i < inliers.size (); ++i)
    {
      // p : Sample Point
      const Eigen::Vector3f p((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z);

      // Local coordinates of sample point p
      const Eigen::Vector3f p_ = Rot_T * (p - c);

      // k : Point on Ellipse
      // Calculate the shortest distance from the point to the ellipse which is given by
      // the norm of a vector that is normal to the ellipse tangent calculated at the
      // point it intersects the tangent.
      const Eigen::VectorXf params = (Eigen::VectorXf(5) << par_a, par_b, 0.0, 0.0, 0.0).finished();
      float th_opt;
      dvec2ellipse(params, p_(0), p_(1), th_opt);

      // Retrive the ellipse point at the tilt angle t, along the local x-axis
      //// model_coefficients[5] = static_cast<float>(par_t);
      Eigen::Vector3f k_(0.0, 0.0, 0.0);
      get_ellipse_point(params, th_opt, k_[0], k_[1]);

      const Eigen::Vector3f k = c + Rot * k_;

      projected_points[i].x = static_cast<float> (k[0]);
      projected_points[i].y = static_cast<float> (k[1]);
      projected_points[i].z = static_cast<float> (k[2]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelEllipse3D<PointT>::doSamplesVerifyModel (
      const std::set<index_t> &indices,
      const Eigen::VectorXf &model_coefficients,
      const double threshold) const
{
  // Needs a valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelEllipse3D::doSamplesVerifyModel] Given model is invalid!\n");
    return (false);
  }

  // c : Ellipse Center
  const Eigen::Vector3f c(model_coefficients[0], model_coefficients[1], model_coefficients[2]);
  // n : Ellipse (Plane) Normal
  const Eigen::Vector3f n_axis(model_coefficients[5], model_coefficients[6], model_coefficients[7]);
  // x : Ellipse (Plane) X-Axis
  const Eigen::Vector3f x_axis(model_coefficients[8], model_coefficients[9], model_coefficients[10]);
  // y : Ellipse (Plane) Y-Axis
  const Eigen::Vector3f y_axis = n_axis.cross(x_axis).normalized();
  // a : Ellipse semi-major axis (X) length
  const float par_a(model_coefficients[3]);
  // b : Ellipse semi-minor axis (Y) length
  const float par_b(model_coefficients[4]);

  // Compute the rotation matrix and its transpose
  const Eigen::Matrix3f Rot = (Eigen::Matrix3f(3,3)
    << x_axis(0), y_axis(0), n_axis(0),
    x_axis(1), y_axis(1), n_axis(1),
    x_axis(2), y_axis(2), n_axis(2))
    .finished();
  const Eigen::Matrix3f Rot_T = Rot.transpose();

  const auto squared_threshold = threshold * threshold;
  for (const auto &index : indices)
  {
    // p : Sample Point
    const Eigen::Vector3f p((*input_)[index].x, (*input_)[index].y, (*input_)[index].z);

    // Local coordinates of sample point p
    const Eigen::Vector3f p_ = Rot_T * (p - c);

    // k : Point on Ellipse
    // Calculate the shortest distance from the point to the ellipse which is given by
    // the norm of a vector that is normal to the ellipse tangent calculated at the
    // point it intersects the tangent.
    const Eigen::VectorXf params = (Eigen::VectorXf(5) << par_a, par_b, 0.0, 0.0, 0.0).finished();
    float th_opt;
    const Eigen::Vector2f distanceVector = dvec2ellipse(params, p_(0), p_(1), th_opt);

    if (distanceVector.squaredNorm() > squared_threshold)
      return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelEllipse3D<PointT>::isModelValid (const Eigen::VectorXf &model_coefficients) const
{
  if (!SampleConsensusModel<PointT>::isModelValid (model_coefficients))
    return (false);

  if (radius_min_ != std::numeric_limits<double>::lowest() && (model_coefficients[3] < radius_min_ || model_coefficients[4] < radius_min_))
  {
    PCL_DEBUG ("[pcl::SampleConsensusModelEllipse3D::isModelValid] Semi-minor axis OR semi-major axis (radii) of ellipse is/are too small: should be larger than %g, but are {%g, %g}.\n",
               radius_min_, model_coefficients[3], model_coefficients[4]);
    return (false);
  }
  if (radius_max_ != std::numeric_limits<double>::max() && (model_coefficients[3] > radius_max_ || model_coefficients[4] > radius_max_))
  {
    PCL_DEBUG ("[pcl::SampleConsensusModelEllipse3D::isModelValid] Semi-minor axis OR semi-major axis (radii) of ellipse is/are too big: should be smaller than %g, but are {%g, %g}.\n",
               radius_max_, model_coefficients[3], model_coefficients[4]);
    return (false);
  }

  return (true);
}



//////////////////////////////////////////////////////////////////////////
template <typename PointT>
void inline pcl::SampleConsensusModelEllipse3D<PointT>::get_ellipse_point(
    const Eigen::VectorXf& par, float th, float& x, float& y)
{
  /*
   * Calculates a point on the ellipse model 'par' using the angle 'th'.
   */

  // Parametric eq.params
  const float par_a(par[0]);
  const float par_b(par[1]);
  const float par_h(par[2]);
  const float par_k(par[3]);
  const float par_t(par[4]);

  x = par_h + std::cos(par_t) * par_a * std::cos(th) -
      std::sin(par_t) * par_b * std::sin(th);
  y = par_k + std::sin(par_t) * par_a * std::cos(th) +
      std::cos(par_t) * par_b * std::sin(th);

  return;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT>
Eigen::Vector2f inline pcl::SampleConsensusModelEllipse3D<PointT>::dvec2ellipse(
    const Eigen::VectorXf& par, float u, float v, float& th_opt)
{
  /*
   * Minimum distance vector from point p=(u,v) to the ellipse model 'par'.
   */

  // Parametric eq.params
  // (par_a, par_b, and par_t do not need to be declared)
  const float par_h = par[2];
  const float par_k = par[3];

  const Eigen::Vector2f center(par_h, par_k);
  Eigen::Vector2f p(u, v);
  p -= center;

  // Local x-axis of the ellipse
  Eigen::Vector2f x_axis(0.0, 0.0);
  get_ellipse_point(par, 0.0, x_axis(0), x_axis(1));
  x_axis -= center;

  // Local y-axis of the ellipse
  Eigen::Vector2f y_axis(0.0, 0.0);
  get_ellipse_point(par, M_PI / 2.0, y_axis(0), y_axis(1));
  y_axis -= center;

  // Convert the point p=(u,v) to local ellipse coordinates
  const float x_proj = p.dot(x_axis) / x_axis.norm();
  const float y_proj = p.dot(y_axis) / y_axis.norm();

  // Find the ellipse quandrant to where the point p=(u,v) belongs,
  // and limit the search interval to 'th_min' and 'th_max'.
  float th_min(0.0), th_max(0.0);
  const float th = std::atan2(y_proj, x_proj);

  if (-M_PI <= th && th < -M_PI / 2.0) {
    // Q3
    th_min = -M_PI;
    th_max = -M_PI / 2.0;
  }
  if (-M_PI / 2.0 <= th && th < 0.0) {
    // Q4
    th_min = -M_PI / 2.0;
    th_max = 0.0;
  }
  if (0.0 <= th && th < M_PI / 2.0) {
    // Q1
    th_min = 0.0;
    th_max = M_PI / 2.0;
  }
  if (M_PI / 2.0 <= th && th <= M_PI) {
    // Q2
    th_min = M_PI / 2.0;
    th_max = M_PI;
  }

  // Use an unconstrained line search optimizer to find the optimal th_opt
  th_opt = golden_section_search(par, u, v, th_min, th_max, 1.e-3);

  // Distance vector from a point (u,v) to a given point in the ellipse model 'par' at an angle 'th_opt'.
  float x(0.0), y(0.0);
  get_ellipse_point(par, th_opt, x, y);
  Eigen::Vector2f distanceVector(u - x, v - y);
  return distanceVector;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT>
float inline pcl::SampleConsensusModelEllipse3D<PointT>::golden_section_search(
    const Eigen::VectorXf& par,
    float u,
    float v,
    float th_min,
    float th_max,
    float epsilon)
{
  /*
   * Golden section search
   */

  constexpr float phi(1.61803398874989484820f); // Golden ratio

  // tl (theta lower bound), tu (theta upper bound)
  float tl(th_min), tu(th_max);
  float ta = tl + (tu - tl) * (1 - 1 / phi);
  float tb = tl + (tu - tl) * 1 / phi;

  while ((tu - tl) > epsilon) {

    // theta a
    float x_a(0.0), y_a(0.0);
    get_ellipse_point(par, ta, x_a, y_a);
    float squared_dist_ta = (u - x_a) * (u - x_a) + (v - y_a) * (v - y_a);

    // theta b
    float x_b(0.0), y_b(0.0);
    get_ellipse_point(par, tb, x_b, y_b);
    float squared_dist_tb = (u - x_b) * (u - x_b) + (v - y_b) * (v - y_b);

    if (squared_dist_ta < squared_dist_tb) {
      tu = tb;
      tb = ta;
      ta = tl + (tu - tl) * (1 - 1 / phi);
    }
    else if (squared_dist_ta > squared_dist_tb) {
      tl = ta;
      ta = tb;
      tb = tl + (tu - tl) * 1 / phi;
    }
    else {
      tl = ta;
      tu = tb;
      ta = tl + (tu - tl) * (1 - 1 / phi);
      tb = tl + (tu - tl) * 1 / phi;
    }
  }
  return (tl + tu) / 2.0;
}


#define PCL_INSTANTIATE_SampleConsensusModelEllipse3D(T) template class PCL_EXPORTS pcl::SampleConsensusModelEllipse3D<T>;
