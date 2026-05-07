/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#define PCL_SAC_MODEL_ELLIPSE3D_INSTANTIATE
#include <pcl/sample_consensus/sac_model_ellipse3d.h>
#include <pcl/sample_consensus/impl/sac_model_ellipse3d.hpp>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

namespace pcl
{
  namespace internal
  {
    /** \brief Internal function to compute ellipse point from parametric coefficients and angle.
      * \param[in] par the parametric coefficients (a, b, h, k, slant)
      * \param[in] th the angle (in radians)
      * \param[out] x the resultant X coordinate in local frame
      * \param[out] y the resultant Y coordinate in local frame
      */
    void
    get_ellipse_point (const Eigen::VectorXf& par, float th, float& x, float& y)
    {
      const float par_a (par [0]);
      const float par_b (par [1]);
      const float par_h (par [2]);
      const float par_k (par [3]);
      const float par_t (par [4]);

      x = par_h + std::cos (par_t) * par_a * std::cos (th) - std::sin (par_t) * par_b * std::sin (th);
      y = par_k + std::sin (par_t) * par_a * std::cos (th) + std::cos (par_t) * par_b * std::sin (th);
    }

    /** \brief Internal function to find the optimal angle using Golden Section Search.
      * \param[in] par the ellipse coefficients (a, b, h, k, slant)
      * \param[in] u point X coordinate in local frame
      * \param[in] v point Y coordinate in local frame
      * \param[in] th_min search interval lower bound
      * \param[in] th_max search interval upper bound
      * \param[in] epsilon search convergence tolerance
      * \return the optimal angle (in radians)
      */
    float
    golden_section_search (const Eigen::VectorXf& par, float u, float v, float th_min, float th_max, float epsilon)
    {
      constexpr float phi (1.61803398874989484820f);
      float tl (th_min), tu (th_max);
      float ta = tl + (tu - tl) * (1.0f - 1.0f / phi);
      float tb = tl + (tu - tl) * 1.0f / phi;

      while ((tu - tl) > epsilon)
      {
        float x_a (0.0f), y_a (0.0f);
        get_ellipse_point (par, ta, x_a, y_a);
        float squared_dist_ta = (u - x_a) * (u - x_a) + (v - y_a) * (v - y_a);

        float x_b (0.0f), y_b (0.0f);
        get_ellipse_point (par, tb, x_b, y_b);
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

    /** \brief Internal function to compute the shortest distance vector from a point to an ellipse.
      * \param[in] par the ellipse coefficients (a, b, h, k, slant)
      * \param[in] u point X coordinate in local frame
      * \param[in] v point Y coordinate in local frame
      * \param[out] th_opt the resultant optimal angle on the ellipse
      * \return the distance vector from the point to its projection on the ellipse
      */
    Eigen::Vector2f
    dvec2ellipse (const Eigen::VectorXf& par, float u, float v, float& th_opt)
    {
      const float par_h = par [2];
      const float par_k = par [3];

      const Eigen::Vector2f center (par_h, par_k);
      Eigen::Vector2f p (u, v);
      p -= center;

      Eigen::Vector2f x_axis (0.0f, 0.0f);
      get_ellipse_point (par, 0.0f, x_axis (0), x_axis (1));
      x_axis -= center;

      Eigen::Vector2f y_axis (0.0f, 0.0f);
      get_ellipse_point (par, static_cast<float> (M_PI / 2.0), y_axis (0), y_axis (1));
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

      th_opt = golden_section_search (par, u, v, th_min, th_max, 1.e-3f);
      float x (0.0f), y (0.0f);
      get_ellipse_point (par, th_opt, x, y);
      return {u - x, v - y};
    }

    int
    optimizeModelCoefficientsEllipse3D (Eigen::VectorXf &coeff, const Eigen::ArrayXf &pts_x, const Eigen::ArrayXf &pts_y, const Eigen::ArrayXf &pts_z)
    {
      struct Ellipse3DOptimizationFunctor : pcl::Functor<double>
      {
        Ellipse3DOptimizationFunctor (const Eigen::ArrayXf& x, const Eigen::ArrayXf& y, const Eigen::ArrayXf& z) :
          pcl::Functor<double> (static_cast<int>(x.size ())), x_ (x), y_ (y), z_ (z) {}

        int operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
        {
          // c : Ellipse Center
          const Eigen::Vector3f c (static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));
          // a : Ellipse semi-major axis (X) length
          const float par_a (static_cast<float>(x [3]));
          // b : Ellipse semi-minor axis (Y) length
          const float par_b (static_cast<float>(x [4]));
          // n : Ellipse (Plane) Normal
          const Eigen::Vector3f n_axis = Eigen::Vector3f (static_cast<float>(x[5]), static_cast<float>(x[6]), static_cast<float>(x[7])).normalized ();
          // x : Ellipse (Plane) X-Axis
          Eigen::Vector3f x_ax = Eigen::Vector3f (static_cast<float>(x[8]), static_cast<float>(x[9]), static_cast<float>(x[10]));
          x_ax = (x_ax - x_ax.dot (n_axis) * n_axis).normalized ();
          // y : Ellipse (Plane) Y-Axis
          const Eigen::Vector3f y_ax = n_axis.cross (x_ax).normalized ();

          // Compute the rotation matrix and its transpose
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
            // k : Point on Ellipse
            // Calculate the shortest distance from the point to the ellipse which is
            // given by the norm of a vector that is normal to the ellipse tangent
            // calculated at the point it intersects the tangent.
            fvec [i] = static_cast<double>(dvec2ellipse (params, p_ (0), p_ (1), th_opt).norm ());
          }
          return (0);
        }

        const Eigen::ArrayXf &x_, &y_, &z_;
      };

      Ellipse3DOptimizationFunctor functor (pts_x, pts_y, pts_z);
      Eigen::NumericalDiff<Ellipse3DOptimizationFunctor> num_diff (functor);
      Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Ellipse3DOptimizationFunctor>, double> lm (num_diff);
      Eigen::VectorXd coeff_double = coeff.cast<double> ();
      const int info = lm.minimize (coeff_double);
      coeff = coeff_double.cast<float> ();
      
      const Eigen::Vector3f n_axis = Eigen::Vector3f (coeff[5], coeff[6], coeff[7]).normalized ();
      coeff[5] = n_axis[0];
      coeff[6] = n_axis[1];
      coeff[7] = n_axis[2];

      Eigen::Vector3f x_ax = Eigen::Vector3f (coeff[8], coeff[9], coeff[10]);
      x_ax = (x_ax - x_ax.dot (n_axis) * n_axis).normalized ();
      coeff[8] = x_ax[0];
      coeff[9] = x_ax[1];
      coeff[10] = x_ax[2];

      return info;
    }
  }
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
  PCL_INSTANTIATE(SampleConsensusModelEllipse3D, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)(pcl::PointXYZRGBNormal))
#else
  PCL_INSTANTIATE(SampleConsensusModelEllipse3D, PCL_XYZ_POINT_TYPES)
#endif
#endif    // PCL_NO_PRECOMPILE
