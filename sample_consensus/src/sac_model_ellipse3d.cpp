/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#include <pcl/sample_consensus/sac_model_ellipse3d.h>
#include <pcl/sample_consensus/impl/sac_model_ellipse3d.hpp>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <Eigen/Eigenvalues>
#include <pcl/common/utils.h>

namespace pcl
{
  namespace internal
  {
    void
    getEllipsePoint (const Eigen::VectorXf& par, float th, float& x, float& y)
    {
      const float par_a (par [0]);
      const float par_b (par [1]);
      const float par_h (par [2]);
      const float par_k (par [3]);
      const float par_t (par [4]);

      x = par_h + std::cos (par_t) * par_a * std::cos (th) - std::sin (par_t) * par_b * std::sin (th);
      y = par_k + std::sin (par_t) * par_a * std::cos (th) + std::cos (par_t) * par_b * std::sin (th);
    }

    float
    goldenSectionSearch (const Eigen::VectorXf& par, float u, float v, float th_min, float th_max, float epsilon)
    {
      constexpr float phi (1.61803398874989484820f);
      float tl (th_min), tu (th_max);
      float ta = tl + (tu - tl) * (1.0f - 1.0f / phi);
      float tb = tl + (tu - tl) * 1.0f / phi;

      while ((tu - tl) > epsilon)
      {
        float x_a (0.0f), y_a (0.0f);
        getEllipsePoint (par, ta, x_a, y_a);
        float squared_dist_ta = (u - x_a) * (u - x_a) + (v - y_a) * (v - y_a);

        float x_b (0.0f), y_b (0.0f);
        getEllipsePoint (par, tb, x_b, y_b);
        float squared_dist_tb = (u - x_b) * (u - x_b) + (v - y_b) * (v - y_b);

        if (squared_dist_ta < squared_dist_tb)
        {
          tu = tb;
          tb = ta;
          ta = tl + (tu - tl) * (1.0f - 1.0f / phi);
        }
        else if (squared_dist_ta > squared_dist_tb)
        {
          tl = ta;
          ta = tb;
          tb = tl + (tu - tl) * 1.0f / phi;
        }
        else
        {
          tl = ta;
          tu = tb;
          ta = tl + (tu - tl) * (1.0f - 1.0f / phi);
          tb = tl + (tu - tl) * 1.0f / phi;
        }
      }
      return (tl + tu) / 2.0f;
    }

    Eigen::Vector2f
    dVec2Ellipse (const Eigen::VectorXf& par, float u, float v, float& th_opt)
    {
      const float par_h = par [2];
      const float par_k = par [3];

      const Eigen::Vector2f center (par_h, par_k);
      Eigen::Vector2f p (u, v);
      p -= center;

      Eigen::Vector2f x_axis (0.0f, 0.0f);
      getEllipsePoint (par, 0.0f, x_axis (0), x_axis (1));
      x_axis -= center;

      Eigen::Vector2f y_axis (0.0f, 0.0f);
      getEllipsePoint (par, static_cast<float> (M_PI / 2.0), y_axis (0), y_axis (1));
      y_axis -= center;

      const float x_proj = p.dot (x_axis) / x_axis.norm ();
      const float y_proj = p.dot (y_axis) / y_axis.norm ();

      float th_min (0.0f), th_max (0.0f);
      const float th = std::atan2 (y_proj, x_proj);

      if (th < -static_cast<float> (M_PI / 2.0))
      {
        th_min = -static_cast<float> (M_PI);
        th_max = -static_cast<float> (M_PI / 2.0);
      }
      else if (th < 0.0f)
      {
        th_min = -static_cast<float> (M_PI / 2.0);
        th_max = 0.0f;
      }
      else if (th < static_cast<float> (M_PI / 2.0))
      {
        th_min = 0.0f;
        th_max = static_cast<float> (M_PI / 2.0);
      }
      else
      {
        th_min = static_cast<float> (M_PI / 2.0);
        th_max = static_cast<float> (M_PI);
      }

      th_opt = goldenSectionSearch (par, u, v, th_min, th_max, 1.e-3f);
      float x (0.0f), y (0.0f);
      getEllipsePoint (par, th_opt, x, y);
      return Eigen::Vector2f (u - x, v - y);
    }

    struct OptimizationFunctor : pcl::Functor<double>
    {
      OptimizationFunctor (const Eigen::ArrayXf& x, const Eigen::ArrayXf& y, const Eigen::ArrayXf& z) :
        pcl::Functor<double> (static_cast<int>(x.size ())), x_ (x), y_ (y), z_ (z) {}

      int operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
      {
        const Eigen::Vector3f c (static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));
        const float par_a (static_cast<float>(x [3]));
        const float par_b (static_cast<float>(x [4]));
        const Eigen::Vector3f n_axis = Eigen::Vector3f (static_cast<float>(x[5]), static_cast<float>(x[6]), static_cast<float>(x[7])).normalized ();
        Eigen::Vector3f x_ax = Eigen::Vector3f (static_cast<float>(x[8]), static_cast<float>(x[9]), static_cast<float>(x[10]));
        x_ax = (x_ax - x_ax.dot (n_axis) * n_axis).normalized ();
        const Eigen::Vector3f y_ax = n_axis.cross (x_ax).normalized ();

        const Eigen::Matrix3f Rot = (Eigen::Matrix3f (3,3)
          << x_ax (0), y_ax(0), n_axis(0),
          x_ax (1), y_ax(1), n_axis(1),
          x_ax (2), y_ax(2), n_axis(2))
          .finished ();
        const Eigen::Matrix3f Rot_T = Rot.transpose ();

        const Eigen::VectorXf params = (Eigen::VectorXf (5) << par_a, par_b, 0.0f, 0.0f, 0.0f).finished ();
        for (int i = 0; i < values (); ++i)
        {
          const Eigen::Vector3f p (x_ [i], y_ [i], z_ [i]);
          const Eigen::Vector3f p_ = Rot_T * (p - c);
          float th_opt;
          fvec [i] = static_cast<double>(dVec2Ellipse (params, p_ (0), p_ (1), th_opt).norm ());
        }
        return (0);
      }

      const Eigen::ArrayXf &x_, &y_, &z_;
    };

    bool
    computeModelCoefficientsEllipse3D (const Eigen::Matrix<float, 6, 3> &pts, Eigen::VectorXf &model_coefficients)
    {
      const Eigen::Vector3f p0 = pts.row (0);
      const Eigen::Vector3f p1 = pts.row (1);
      const Eigen::Vector3f p2 = pts.row (2);
      const Eigen::Vector3f p3 = pts.row (3);
      const Eigen::Vector3f p4 = pts.row (4);
      const Eigen::Vector3f p5 = pts.row (5);

      const Eigen::Vector3f common_helper_vec = (p1 - p0).cross (p1 - p2);
      const Eigen::Vector3f ellipse_normal = common_helper_vec.normalized ();

      if (common_helper_vec.squaredNorm () < Eigen::NumTraits<float>::dummy_precision ())
        return (false);

      const Eigen::Vector3f x_axis = (p1 - p0).normalized ();
      const Eigen::Vector3f z_axis = ellipse_normal;
      const Eigen::Vector3f y_axis = z_axis.cross (x_axis).normalized ();

      const Eigen::Matrix3f Rot = (Eigen::Matrix3f (3,3)
        << x_axis (0), y_axis (0), z_axis (0),
        x_axis (1), y_axis (1), z_axis (1),
        x_axis (2), y_axis (2), z_axis (2))
        .finished ();
      const Eigen::Matrix3f Rot_T = Rot.transpose ();

      const Eigen::Vector3f p1_ = Rot_T * (p1 - p0);
      const Eigen::Vector3f p2_ = Rot_T * (p2 - p0);
      const Eigen::Vector3f p3_ = Rot_T * (p3 - p0);
      const Eigen::Vector3f p4_ = Rot_T * (p4 - p0);
      const Eigen::Vector3f p5_ = Rot_T * (p5 - p0);

      const Eigen::VectorXf X = (Eigen::VectorXf (6) << 0.0f, p1_ (0), p2_ (0), p3_ (0), p4_ (0), p5_ (0)).finished ();
      const Eigen::VectorXf Y = (Eigen::VectorXf (6) << 0.0f, p1_ (1), p2_ (1), p3_ (1), p4_ (1), p5_ (1)).finished ();

      const Eigen::MatrixXf D = (Eigen::MatrixXf (6,6)
        << X(0) * X(0), X(0) * Y(0), Y(0) * Y(0), X(0), Y(0), 1.0f,
           X(1) * X(1), X(1) * Y(1), Y(1) * Y(1), X(1), Y(1), 1.0f,
           X(2) * X(2), X(2) * Y(2), Y(2) * Y(2), X(2), Y(2), 1.0f,
           X(3) * X(3), X(3) * Y(3), Y(3) * Y(3), X(3), Y(3), 1.0f,
           X(4) * X(4), X(4) * Y(4), Y(4) * Y(4), X(4), Y(4), 1.0f,
           X(5) * X(5), X(5) * Y(5), Y(5) * Y(5), X(5), Y(5), 1.0f).finished ();

      // Scatter matrix S
      const Eigen::MatrixXf S = D.transpose () * D;
      // Constraint matrix C
      const Eigen::MatrixXf C = (Eigen::MatrixXf (6,6)
        << 0.0f, 0.0f, -2.0f, 0.0f, 0.0f, 0.0f,
           0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
           -2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
           0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
           0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
           0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f).finished ();

      // Solve the Generalized Eigensystem: S*a = lambda*C*a
      Eigen::GeneralizedEigenSolver<Eigen::MatrixXf> solver;
      solver.compute (S, C);
      const Eigen::VectorXf eigvals = solver.eigenvalues ().real ();

      // Find the negative eigenvalue 'neigvec' (the largest, if many exist)
      int idx (-1);
      float absmin (0.0f);
      for (int i = 0; i < static_cast<int>(eigvals.size ()); ++i)
        if (eigvals (i) < absmin && !std::isinf (eigvals (i)))
          idx = i;

      // Return "false" in case the negative eigenvalue was not found
      if (idx == -1) return (false);
      const Eigen::VectorXf neigvec = solver.eigenvectors ().real ().col (idx).normalized ();

      // Convert the conic model parameters to parametric ones
      // Conic parameters
      const float con_A (neigvec (0));
      const float con_B (neigvec (1));
      const float con_C (neigvec (2));
      const float con_D (neigvec (3));
      const float con_E (neigvec (4));
      const float con_F (neigvec (5));

      // Build matrix M0
      const Eigen::Matrix3f M0 = (Eigen::Matrix3f ()
        << con_F, con_D / 2.0f, con_E / 2.0f,
           con_D / 2.0f, con_A, con_B / 2.0f,
           con_E / 2.0f, con_B / 2.0f, con_C).finished ();
      
      // Build matrix M
      const Eigen::Matrix2f M = (Eigen::Matrix2f () << con_A, con_B / 2.0f, con_B / 2.0f, con_C).finished ();
      
      // Calculate the eigenvalues and eigenvectors of matrix M
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver_M (M, Eigen::EigenvaluesOnly);
      Eigen::Vector2f eigvals_M = solver_M.eigenvalues ();

      // Order the eigenvalues so that |lambda_0 - con_A| <= |lambda_0 - con_C|
      if (std::abs (eigvals_M (0) - con_A) > std::abs (eigvals_M (0) - con_C))
        std::swap (eigvals_M (0), eigvals_M (1));

      // Parametric parameters of the ellipse
      float par_a = std::sqrt (-M0.determinant () / (M.determinant () * eigvals_M (0)));
      float par_b = std::sqrt (-M0.determinant () / (M.determinant () * eigvals_M (1)));
      const float par_h = (con_B * con_E - 2.0f * con_C * con_D) / (4.0f * con_A * con_C - std::pow (con_B, 2));
      const float par_k = (con_B * con_D - 2.0f * con_A * con_E) / (4.0f * con_A * con_C - std::pow (con_B, 2));
      const float par_t = (static_cast<float> (M_PI / 2.0) - std::atan ((con_A - con_C) / con_B)) / 2.0f;

      // Convert the center point of the ellipse to global coordinates
      // (the if statement ensures that 'par_a' always refers to the semi-major axis length)
      Eigen::Vector3f p_ctr;
      if (par_a > par_b)
        p_ctr.noalias () = p0 + Rot * Eigen::Vector3f (par_h, par_k, 0.0f);
      else
      {
        std::swap (par_a, par_b);
        p_ctr.noalias () = p0 + Rot * Eigen::Vector3f (par_k, par_h, 0.0f);
      }

      model_coefficients [0] = p_ctr (0);
      model_coefficients [1] = p_ctr (1);
      model_coefficients [2] = p_ctr (2);
      model_coefficients [3] = par_a;
      model_coefficients [4] = par_b;
      model_coefficients [5] = ellipse_normal [0];
      model_coefficients [6] = ellipse_normal [1];
      model_coefficients [7] = ellipse_normal [2];

      const Eigen::VectorXf params_t = (Eigen::VectorXf (5) << par_a, par_b, par_h, par_k, par_t).finished ();
      Eigen::Vector3f p_th_ (0.0f, 0.0f, 0.0f);
      getEllipsePoint (params_t, par_t, p_th_ (0), p_th_ (1));
      const Eigen::Vector3f x_ax = (Rot * p_th_).normalized ();
      model_coefficients [8] = x_ax [0];
      model_coefficients [9] = x_ax [1];
      model_coefficients [10] = x_ax [2];

      return (true);
    }

    void
    getDistancesToModelEllipse3D (const Eigen::VectorXf &model_coefficients, const Eigen::ArrayXf &x, const Eigen::ArrayXf &y, const Eigen::ArrayXf &z, std::vector<double> &distances)
    {
      // c : Ellipse Center
      const Eigen::Vector3f c (model_coefficients [0], model_coefficients [1], model_coefficients [2]);
      // n : Ellipse (Plane) Normal
      const Eigen::Vector3f n_axis (model_coefficients [5], model_coefficients [6], model_coefficients [7]);
      // x : Ellipse (Plane) X-Axis
      const Eigen::Vector3f x_axis (model_coefficients [8], model_coefficients [9], model_coefficients [10]);
      // y : Ellipse (Plane) Y-Axis
      const Eigen::Vector3f y_axis = n_axis.cross (x_axis).normalized ();
      // a : Ellipse semi-major axis (X) length
      const float par_a (model_coefficients [3]);
      // b : Ellipse semi-minor axis (Y) length
      const float par_b (model_coefficients [4]);

      // Compute the rotation matrix and its transpose
      const Eigen::Matrix3f Rot = (Eigen::Matrix3f (3,3) << x_axis (0), y_axis (0), n_axis (0), x_axis (1), y_axis (1), n_axis (1), x_axis (2), y_axis (2), n_axis (2)).finished ();
      const Eigen::Matrix3f Rot_T = Rot.transpose ();
      
      // Ellipse parameters
      const Eigen::VectorXf params = (Eigen::VectorXf (5) << par_a, par_b, 0.0f, 0.0f, 0.0f).finished ();

      // Iterate through the 3D points and calculate the distances from them to the ellipse
      distances.resize (x.size ());
      for (Eigen::Index i = 0; i < x.size (); ++i)
      {
        float th_opt;
        const Eigen::Vector3f p (x [i], y [i], z [i]);
        // Local coordinates of sample point p
        const Eigen::Vector3f p_ = Rot_T * (p - c);
        
        // Calculate the shortest distance from the point to the ellipse which is given by
        // the norm of a vector that is normal to the ellipse tangent calculated at the
        // point it intersects the tangent.
        distances [i] = static_cast<double>(dVec2Ellipse (params, p_(0), p_(1), th_opt).norm ());
      }
    }

    void
    optimizeModelCoefficientsEllipse3D (const Eigen::ArrayXf &x, const Eigen::ArrayXf &y, const Eigen::ArrayXf &z, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients)
    {
      OptimizationFunctor functor (x, y, z);
      Eigen::NumericalDiff<OptimizationFunctor> num_diff (functor);
      Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, double> lm (num_diff);
      Eigen::VectorXd coeff = model_coefficients.cast<double> ();
      lm.minimize (coeff);
      optimized_coefficients = coeff.cast<float> ();
      
      const Eigen::Vector3f n_axis = Eigen::Vector3f (optimized_coefficients[5], optimized_coefficients[6], optimized_coefficients[7]).normalized ();
      optimized_coefficients[5] = n_axis[0];
      optimized_coefficients[6] = n_axis[1];
      optimized_coefficients[7] = n_axis[2];

      Eigen::Vector3f x_ax = Eigen::Vector3f (optimized_coefficients[8], optimized_coefficients[9], optimized_coefficients[10]);
      x_ax = (x_ax - x_ax.dot (n_axis) * n_axis).normalized ();
      optimized_coefficients[8] = x_ax[0];
      optimized_coefficients[9] = x_ax[1];
      optimized_coefficients[10] = x_ax[2];
    }
  }
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

template class pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>;
template class pcl::SampleConsensusModelEllipse3D<pcl::PointXYZI>;
template class pcl::SampleConsensusModelEllipse3D<pcl::PointXYZRGBA>;
template class pcl::SampleConsensusModelEllipse3D<pcl::PointXYZRGB>;
template class pcl::SampleConsensusModelEllipse3D<pcl::PointXYZRGBNormal>;
#endif
