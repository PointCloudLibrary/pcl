/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

#ifndef PCL_REGISTRATION_NDT_IMPL_H_
#define PCL_REGISTRATION_NDT_IMPL_H_

//#include <pcl/registration/ndt.h>

using namespace std;

template<typename PointSource, typename PointTarget>
pcl::NormalDistributionsTransform<PointSource, PointTarget>::NormalDistributionsTransform()
{
  reg_name_ = "NormalDistributionsTransform";

  double _gauss_c1, _gauss_c2, _gauss_d3;

  outlier_ratio_ = 0.55;

  // Initializes the guassian fitting parameters (eq. 6.8) [Magnusson 2009]
  _gauss_c1 = 10.0 * (1 - outlier_ratio_);
  _gauss_c2 = outlier_ratio_ / pow (resolution_, 3);
  _gauss_d3 = -log (_gauss_c2);
  gauss_d1_ = -log ( _gauss_c1 + _gauss_c2 ) - _gauss_d3;
  gauss_d2_ = -2 * log ((-log ( _gauss_c1 * exp ( -0.5 ) + _gauss_c2 ) - _gauss_d3) / gauss_d1_);

  transformation_epsilon_ = 0.1;
  step_size_ = 0.1;
  resolution_ = 1.0;
  max_iterations_ = 35;
}

template<typename PointSource, typename PointTarget> void
pcl::NormalDistributionsTransform<PointSource, PointTarget>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess)
{
  nr_iterations_ = 0;
  converged_ = false;

  double _gauss_c1, _gauss_c2, _gauss_d3;

  // Initializes the guassian fitting parameters (eq. 6.8) [Magnusson 2009]
  _gauss_c1 = 10 * (1 - outlier_ratio_);
  _gauss_c2 = outlier_ratio_ / pow (resolution_, 3);
  _gauss_d3 = -log (_gauss_c2);
  gauss_d1_ = -log ( _gauss_c1 + _gauss_c2 ) - _gauss_d3;
  gauss_d2_ = -2 * log ((-log ( _gauss_c1 * exp ( -0.5 ) + _gauss_c2 ) - _gauss_d3) / gauss_d1_);

  if (guess != Eigen::Matrix4f::Identity ())
  {
    // Initialise final transformation to the guessed one
    final_transformation_ = guess;
    // Apply guessed transformation prior to search for neighbours
    transformPointCloud (output, output, guess);
  }

  // Initialize Point Gradient and Hessian
  point_gradient_.setZero ();
  point_gradient_.block<3, 3>(0, 0).setIdentity ();
  point_hessian_.setZero ();

  Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> _eig_transformation;
  _eig_transformation.matrix () = final_transformation_;

  // Convert initial guess matrix to 6 element transformation vector
  Eigen::Matrix<double, 6, 1> _p, _delta_p, _score_gradient;
  Eigen::Vector3f _init_translation = _eig_transformation.translation ();
  Eigen::Vector3f _init_rotation = _eig_transformation.rotation ().eulerAngles (0, 1, 2);
  _p << _init_translation (0), _init_translation (1), _init_translation (2),
  _init_rotation (0), _init_rotation (1), _init_rotation (2);

  Eigen::Matrix<double, 6, 6> _hessian;

  double _score = 0;
  double _delta_p_norm;

  // Calculate derivates of initial transform vector, subsequent derivative calculations are done in the step length determination.
  _score = computeDerivatives (_score_gradient, _hessian, output, _p);

  while (!converged_)
  {
    // Store previous transformation
    previous_transformation_ = transformation_;

    // Solve for decent direction using newton method, line 23 in Algorithm 2 [Magnusson 2009]
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6> > sv (_hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Negative for maximization as opposed to minimization
    _delta_p = sv.solve (-_score_gradient);

    //Calculate step length with guarnteed sufficient decrease [More, Thuente 1994]
    _delta_p_norm = _delta_p.norm ();
    _delta_p.normalize ();
    _delta_p_norm = computeStepLengthMT (_p, _delta_p, _delta_p_norm, step_size_, transformation_epsilon_ / 2, _score, _score_gradient, _hessian, output);
    _delta_p *= _delta_p_norm;


    transformation_ = (Eigen::Translation<float, 3>(_delta_p (0), _delta_p (1), _delta_p (2)) *
                       Eigen::AngleAxis<float>(_delta_p (3), Eigen::Vector3f::UnitX ()) *
                       Eigen::AngleAxis<float>(_delta_p (4), Eigen::Vector3f::UnitY ()) *
                       Eigen::AngleAxis<float>(_delta_p (5), Eigen::Vector3f::UnitZ ())).matrix ();


    _p = _p + _delta_p;

    // Update Visualizer (untested)
    if (update_visualizer_ != 0)
      update_visualizer_ (output, std::vector<int>(), *target_, std::vector<int>() );

    if (nr_iterations_ > max_iterations_ ||
        (nr_iterations_ && (std::fabs (_delta_p_norm) < transformation_epsilon_)))
    {
      converged_ = true;
    }

    nr_iterations_++;

  }

  // Store transformation probability.  The realtive differences within each scan registration are accurate
  // but the normalization constants need to be modified for it to be globally accurate
  trans_probability_ = _score / input_->points.size ();
}


template<typename PointSource, typename PointTarget> double
pcl::NormalDistributionsTransform<PointSource, PointTarget>::computeDerivatives (Eigen::Matrix<double, 6, 1> &score_gradient,
                                                                                 Eigen::Matrix<double, 6, 6> &hessian,
                                                                                 PointCloudSource &trans_cloud,
                                                                                 Eigen::Matrix<double, 6, 1> &p,
                                                                                 bool compute_hessian)
{
  // Original Point and Transformed Point
  PointSource _x_pt, _x_trans_pt;
  // Original Point and Transformed Point (for math)
  Eigen::Vector3d _x, _x_trans;
  // Occupied Voxel
  TargetGridLeafConstPtr _cell;
  // Inverse Covariance of Occupied Voxel
  Eigen::Matrix3d _c_inv;

  score_gradient.setZero ();
  hessian.setZero ();
  double _score = 0;

  // Precompute Angular Derivatives (eq. 6.19 and 6.21)[Magnusson 2009]
  computeAngleDerivatives (p);

  // Update gradient and hessian for each point, line 17 in Algorithm 2 [Magnusson 2009]
  for (size_t _idx = 0; _idx < input_->points.size (); _idx++)
  {
    _x_trans_pt = trans_cloud.points[_idx];

    // Find nieghbors (Radius search has been experimentally faster than direct neighbor checking.
    vector<TargetGridLeafConstPtr> _neighborhood;
    vector<float> _distances;
    target_cells_.radiusSearch (_x_trans_pt, resolution_, _neighborhood, _distances);

    for (typename vector<TargetGridLeafConstPtr>::iterator _it = _neighborhood.begin (); _it != _neighborhood.end (); _it++)
    {
      _cell = *_it;
      _x_pt = input_->points[_idx];
      _x = Eigen::Vector3d (_x_pt.x, _x_pt.y, _x_pt.z);

      _x_trans = Eigen::Vector3d (_x_trans_pt.x, _x_trans_pt.y, _x_trans_pt.z);

      // Denorm point, x_k' in Equations 6.12 and 6.13 [Magnusson 2009]
      _x_trans -= _cell->getMean ();
      // Uses precomputed covariance for speed.
      _c_inv = _cell->getInverseCov ();

      // Compute derivative of transform function w.r.t. transform vector, J_E and H_E in Equations 6.18 and 6.20 [Magnusson 2009]
      computePointDerivatives (_x);
      // Update score, gradient and hessian, lines 19-21 in Algorithm 2, according to Equations 6.10, 6.12 and 6.13, respectively [Magnusson 2009]
      _score += updateDerivatives (score_gradient, hessian, _x_trans, _c_inv, compute_hessian);

    }
  }
  return _score;
}

template<typename PointSource, typename PointTarget> void
pcl::NormalDistributionsTransform<PointSource, PointTarget>::computeAngleDerivatives (Eigen::Matrix<double, 6, 1> &p, bool compute_hessian)
{
  // Simplified math for near 0 angles
  double _cx, _cy, _cz, _sx, _sy, _sz;
  if (fabsf (p (3)) < 10e-5)
  {
    //p(3) = 0;
    _cx = 1.0;
    _sx = 0.0;
  }
  else
  {
    _cx = cos (p (3));
    _sx = sin (p (3));
  }
  if (fabsf (p (4)) < 10e-5)
  {
    //p(4) = 0;
    _cy = 1.0;
    _sy = 0.0;
  }
  else
  {
    _cy = cos (p (4));
    _sy = sin (p (4));
  }

  if (fabsf (p (5)) < 10e-5)
  {
    //p(5) = 0;
    _cz = 1.0;
    _sz = 0.0;
  }
  else
  {
    _cz = cos (p (5));
    _sz = sin (p (5));
  }

  // Precomputed angular gradiant components. Letters correspond to Equation 6.19 [Magnusson 2009]
  j_ang_a_ << (-_sx * _sz + _cx * _sy * _cz), (-_sx * _cz - _cx * _sy * _sz), (-_cx * _cy);
  j_ang_b_ << (_cx * _sz + _sx * _sy * _cz), (_cx * _cz - _sx * _sy * _sz), (-_sx * _cy);
  j_ang_c_ << (-_sy * _cz), _sy * _sz, _cy;
  j_ang_d_ << _sx * _cy * _cz, (-_sx * _cy * _sz), _sx * _sy;
  j_ang_e_ << (-_cx * _cy * _cz), _cx * _cy * _sz, (-_cx * _sy);
  j_ang_f_ << (-_cy * _sz), (-_cy * _cz), 0;
  j_ang_g_ << (_cx * _cz - _sx * _sy * _sz), (-_cx * _sz - _sx * _sy * _cz), 0;
  j_ang_h_ << (_sx * _cz + _cx * _sy * _sz), (_cx * _sy * _cz - _sx * _sz), 0;

  if (compute_hessian)
  {
    // Precomputed angular hessian components. Letters correspond to Equation 6.21 and numbers correspond to row index [Magnusson 2009]
    h_ang_a2_ << (-_cx * _sz - _sx * _sy * _cz), (-_cx * _cz + _sx * _sy * _sz), _sx * _cy;
    h_ang_a3_ << (-_sx * _sz + _cx * _sy * _cz), (-_cx * _sy * _sz - _sx * _cz), (-_cx * _cy);

    h_ang_b2_ << (_cx * _cy * _cz), (-_cx * _cy * _sz), (_cx * _sy);
    h_ang_b3_ << (_sx * _cy * _cz), (-_sx * _cy * _sz), (_sx * _sy);

    h_ang_c2_ << (-_sx * _cz - _cx * _sy * _sz), (_sx * _sz - _cx * _sy * _cz), 0;
    h_ang_c3_ << (_cx * _cz - _sx * _sy * _sz), (-_sx * _sy * _cz - _cx * _sz), 0;

    h_ang_d1_ << (-_cy * _cz), (_cy * _sz), (_sy);
    h_ang_d2_ << (-_sx * _sy * _cz), (_sx * _sy * _sz), (_sx * _cy);
    h_ang_d3_ << (_cx * _sy * _cz), (-_cx * _sy * _sz), (-_cx * _cy);

    h_ang_e1_ << (_sy * _sz), (_sy * _cz), 0;
    h_ang_e2_ << (-_sx * _cy * _sz), (-_sx * _cy * _cz), 0;
    h_ang_e3_ << (_cx * _cy * _sz), (_cx * _cy * _cz), 0;

    h_ang_f1_ << (-_cy * _cz), (_cy * _sz), 0;
    h_ang_f2_ << (-_cx * _sz - _sx * _sy * _cz), (-_cx * _cz + _sx * _sy * _sz), 0;
    h_ang_f3_ << (-_sx * _sz + _cx * _sy * _cz), (-_cx * _sy * _sz - _sx * _cz), 0;
  }
}

template<typename PointSource, typename PointTarget> void
pcl::NormalDistributionsTransform<PointSource, PointTarget>::computePointDerivatives (Eigen::Vector3d &x, bool compute_hessian)
{
  // Calculate first derivative of Transformation Equation 6.17 w.r.t. transform vector p.
  // Derivative w.r.t. ith element of transform vector corresponds to column i, Equation 6.18 and 6.19 [Magnusson 2009]
  point_gradient_ (1, 3) = x.dot (j_ang_a_);
  point_gradient_ (2, 3) = x.dot (j_ang_b_);
  point_gradient_ (0, 4) = x.dot (j_ang_c_);
  point_gradient_ (1, 4) = x.dot (j_ang_d_);
  point_gradient_ (2, 4) = x.dot (j_ang_e_);
  point_gradient_ (0, 5) = x.dot (j_ang_f_);
  point_gradient_ (1, 5) = x.dot (j_ang_g_);
  point_gradient_ (2, 5) = x.dot (j_ang_h_);

  if (compute_hessian)
  {
    // Vectors from Equation 6.21 [Magnusson 2009]
    Eigen::Vector3d a, b, c, d, e, f;

    a << 0, x.dot (h_ang_a2_), x.dot (h_ang_a3_);
    b << 0, x.dot (h_ang_b2_), x.dot (h_ang_b3_);
    c << 0, x.dot (h_ang_c2_), x.dot (h_ang_c3_);
    d << x.dot (h_ang_d1_), x.dot (h_ang_d2_), x.dot (h_ang_d3_);
    e << x.dot (h_ang_e1_), x.dot (h_ang_e2_), x.dot (h_ang_e3_);
    f << x.dot (h_ang_f1_), x.dot (h_ang_f2_), x.dot (h_ang_f3_);

    // Calculate second derivative of Transformation Equation 6.17 w.r.t. transform vector p.
    // Derivative w.r.t. ith and jth elements of transform vector corresponds to the 3x1 block matrix starting at (3i,j), Equation 6.20 and 6.21 [Magnusson 2009]
    point_hessian_.block<3, 1>(9, 3) = a;
    point_hessian_.block<3, 1>(12, 3) = b;
    point_hessian_.block<3, 1>(15, 3) = c;
    point_hessian_.block<3, 1>(9, 4) = b;
    point_hessian_.block<3, 1>(12, 4) = d;
    point_hessian_.block<3, 1>(15, 4) = e;
    point_hessian_.block<3, 1>(9, 5) = c;
    point_hessian_.block<3, 1>(12, 5) = e;
    point_hessian_.block<3, 1>(15, 5) = f;
  }
}

template<typename PointSource, typename PointTarget> double
pcl::NormalDistributionsTransform<PointSource, PointTarget>::updateDerivatives (Eigen::Matrix<double, 6, 1> &score_gradient,
                                                                                Eigen::Matrix<double, 6, 6> &hessian,
                                                                                Eigen::Vector3d &x_trans, Eigen::Matrix3d &c_inv,
                                                                                bool compute_hessian)
{
  Eigen::Vector3d _CdxdPi;
  // e^(-d_2/2 * (x_k - mu_k)^T Sigma_k^-1 (x_k - mu_k)) Equation 6.9 [Magnusson 2009]
  double _e_xCx = exp (-gauss_d2_ * x_trans.dot (c_inv * x_trans) / 2);
  // Calculate probability of transtormed points existance, Equation 6.9 [Magnusson 2009]
  double score_inc = -gauss_d1_ * _e_xCx;

  _e_xCx = gauss_d2_ * _e_xCx;

  // Error checking for invalid values.
  if (_e_xCx > 1 || _e_xCx < 0 || _e_xCx != _e_xCx)
  {
    return 0;
  }

  // Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]
  _e_xCx *= gauss_d1_;


  for (int i = 0; i < 6; i++)
  {
    // Sigma_k^-1 d(T(x,p))/dpi, Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]
    _CdxdPi = c_inv * point_gradient_.col (i);

    // Update gradient, Equation 6.12 [Magnusson 2009]
    score_gradient (i) += x_trans.dot (_CdxdPi) * _e_xCx;

    if (compute_hessian)
    {
      for (int j = 0; j < hessian.cols (); j++)
      {
        // Update hessian, Equation 6.13 [Magnusson 2009]
        hessian (i, j) += _e_xCx * (-gauss_d2_ * x_trans.dot (_CdxdPi) * x_trans.dot (c_inv * point_gradient_.col (j)) +
                                    x_trans.dot (c_inv * point_hessian_.block<3, 1>(3 * i, j)) +
                                    point_gradient_.col (j).dot (_CdxdPi) );
      }
    }
  }

  return score_inc;
}

template<typename PointSource, typename PointTarget> void
pcl::NormalDistributionsTransform<PointSource, PointTarget>::computeHessian (Eigen::Matrix<double, 6, 6> &hessian,
                                                                             PointCloudSource &trans_cloud, Eigen::Matrix<double, 6, 1> &p)
{
  // Original Point and Transformed Point
  PointSource _x_pt, _x_trans_pt;
  // Original Point and Transformed Point (for math)
  Eigen::Vector3d _x, _x_trans;
  // Occupied Voxel
  TargetGridLeafConstPtr _cell;
  // Inverse Covariance of Occupied Voxel
  Eigen::Matrix3d _c_inv;

  hessian.setZero ();

  // Precompute Angular Derivatives unessisary because only used after regular derivative calculation

  // Update hessian for each point, line 17 in Algorithm 2 [Magnusson 2009]
  for (size_t _idx = 0; _idx < input_->points.size (); _idx++)
  {
    _x_trans_pt = trans_cloud.points[_idx];

    // Find nieghbors (Radius search has been experimentally faster than direct neighbor checking.
    vector<TargetGridLeafConstPtr> _neighborhood;
    vector<float> _distances;
    target_cells_.radiusSearch (_x_trans_pt, resolution_, _neighborhood, _distances);

    for (typename vector<TargetGridLeafConstPtr>::iterator _it = _neighborhood.begin (); _it != _neighborhood.end (); _it++)
    {
      _cell = *_it;

      {
        _x_pt = input_->points[_idx];
        _x = Eigen::Vector3d (_x_pt.x, _x_pt.y, _x_pt.z);

        _x_trans = Eigen::Vector3d (_x_trans_pt.x, _x_trans_pt.y, _x_trans_pt.z);

        // Denorm point, x_k' in Equations 6.12 and 6.13 [Magnusson 2009]
        _x_trans -= _cell->getMean ();
        // Uses precomputed covariance for speed.
        _c_inv = _cell->getInverseCov ();

        // Compute derivative of transform function w.r.t. transform vector, J_E and H_E in Equations 6.18 and 6.20 [Magnusson 2009]
        computePointDerivatives (_x);
        // Update hessian, lines 21 in Algorithm 2, according to Equations 6.10, 6.12 and 6.13, respectively [Magnusson 2009]
        updateHessian (hessian, _x_trans, _c_inv);
      }
    }
  }
}

template<typename PointSource, typename PointTarget> void
pcl::NormalDistributionsTransform<PointSource, PointTarget>::updateHessian (Eigen::Matrix<double, 6, 6> &hessian, Eigen::Vector3d &x_trans, Eigen::Matrix3d &c_inv)
{
  Eigen::Vector3d _CdxdPi;
  // e^(-d_2/2 * (x_k - mu_k)^T Sigma_k^-1 (x_k - mu_k)) Equation 6.9 [Magnusson 2009]
  double _e_xCx = gauss_d2_ * exp (-gauss_d2_ * x_trans.dot (c_inv * x_trans) / 2);

  // Error checking for invalid values.
  if (_e_xCx > 1 || _e_xCx < 0 || _e_xCx != _e_xCx)
  {
    return;
  }

  // Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]
  _e_xCx *= gauss_d1_;

  for (int i = 0; i < 6; i++)
  {
    // Sigma_k^-1 d(T(x,p))/dpi, Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]
    _CdxdPi = c_inv * point_gradient_.col (i);

    for (int j = 0; j < hessian.cols (); j++)
    {
      // Update hessian, Equation 6.13 [Magnusson 2009]
      hessian (i, j) += _e_xCx * (-gauss_d2_ * x_trans.dot (_CdxdPi) * x_trans.dot (c_inv * point_gradient_.col (j)) +
                                  x_trans.dot (c_inv * point_hessian_.block<3, 1>(3 * i, j)) +
                                  point_gradient_.col (j).dot (_CdxdPi) );
    }
  }

}


template<typename PointSource, typename PointTarget> bool
pcl::NormalDistributionsTransform<PointSource, PointTarget>::updateIntervalMT (double &a_l, double &f_l, double &g_l,
                                                                               double &a_u, double &f_u, double &g_u,
                                                                               double a_t, double f_t, double g_t)
{
  // Case U1 in Update Algorithm and Case a in Modified Update Algorithm [More, Thuente 1994]
  if (f_t > f_l)
  {
    a_u = a_t;
    f_u = f_t;
    g_u = g_t;
    return false;
  }
  // Case U2 in Update Algorithm and Case b in Modified Update Algorithm [More, Thuente 1994]
  else
  if (g_t * (a_l - a_t) > 0)
  {
    a_l = a_t;
    f_l = f_t;
    g_l = g_t;
    return false;
  }
  // Case U3 in Update Algorithm and Case c in Modified Update Algorithm [More, Thuente 1994]
  else
  if (g_t * (a_l - a_t) < 0)
  {
    a_u = a_l;
    f_u = f_l;
    g_u = g_l;

    a_l = a_t;
    f_l = f_t;
    g_l = g_t;
    return false;
  }
  // Interval Converged
  else
    return true;
}

template<typename PointSource, typename PointTarget> double
pcl::NormalDistributionsTransform<PointSource, PointTarget>::trialValueSelectionMT (double a_l, double f_l, double g_l,
                                                                                    double a_u, double f_u, double g_u,
                                                                                    double a_t, double f_t, double g_t)
{
  // Case 1 in Trial Value Selection [More, Thuente 1994]
  if (f_t > f_l)
  {
    // Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
    // Equation 2.4.52 [Sun, Yuan 2006]
    double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
    double w = std::sqrt (z * z - g_t * g_l);
    // Equation 2.4.56 [Sun, Yuan 2006]
    double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

    // Calculate the minimizer of the quadratic that interpolates f_l, f_t and g_l
    // Equation 2.4.2 [Sun, Yuan 2006]
    double a_q = a_l - 0.5 * (a_l - a_t) * g_l / (g_l - (f_l - f_t) / (a_l - a_t));

    if (std::fabs (a_c - a_l) < std::fabs (a_q - a_l))
      return a_c;
    else
      return 0.5 * (a_q + a_c);
  }
  // Case 2 in Trial Value Selection [More, Thuente 1994]
  else
  if (g_t * g_l < 0)
  {
    // Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
    // Equation 2.4.52 [Sun, Yuan 2006]
    double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
    double w = std::sqrt (z * z - g_t * g_l);
    // Equation 2.4.56 [Sun, Yuan 2006]
    double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

    // Calculate the minimizer of the quadratic that interpolates f_l, g_l and g_t
    // Equation 2.4.5 [Sun, Yuan 2006]
    double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

    if (std::fabs (a_c - a_t) >= std::fabs (a_s - a_t))
      return a_c;
    else
      return a_s;
  }
  // Case 3 in Trial Value Selection [More, Thuente 1994]
  else
  if (std::fabs (g_t) <= std::fabs (g_l))
  {
    // Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
    // Equation 2.4.52 [Sun, Yuan 2006]
    double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
    double w = std::sqrt (z * z - g_t * g_l);
    double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

    // Calculate the minimizer of the quadratic that interpolates g_l and g_t
    // Equation 2.4.5 [Sun, Yuan 2006]
    double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

    double a_t_next;

    if (std::fabs (a_c - a_t) < std::fabs (a_s - a_t))
      a_t_next = a_c;
    else
      a_t_next = a_s;

    if (a_t > a_l)
      return std::min (a_t + 0.66 * (a_u - a_t), a_t_next);
    else
      return std::max (a_t + 0.66 * (a_u - a_t), a_t_next);
  }
  // Case 4 in Trial Value Selection [More, Thuente 1994]
  else
  {
    // Calculate the minimizer of the cubic that interpolates f_u, f_t, g_u and g_t
    // Equation 2.4.52 [Sun, Yuan 2006]
    double z = 3 * (f_t - f_u) / (a_t - a_u) - g_t - g_u;
    double w = std::sqrt (z * z - g_t * g_u);
    // Equation 2.4.56 [Sun, Yuan 2006]
    return a_u + (a_t - a_u) * (w - g_u - z) / (g_t - g_u + 2 * w);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget> double
pcl::NormalDistributionsTransform<PointSource, PointTarget>::computeStepLengthMT (
    const Eigen::Matrix<double, 6, 1> &x, 
    const Eigen::Matrix<double, 6, 1> &step_dir, 
    double step_init, double step_max, double step_min, 
    double &score, 
    Eigen::Matrix<double, 6, 1> &score_gradient, 
    Eigen::Matrix<double, 6, 6> &hessian,
    PointCloudSource &trans_cloud)
{
  // Set the value of phi(0), Equation 1.3 [More, Thuente 1994]
  double _phi_0 = -score;
  // Set the value of phi'(0), Equation 1.3 [More, Thuente 1994]
  double _d_phi_0 = -(score_gradient.dot (step_dir));

  Eigen::Matrix<double, 6, 1>  _x_t;

  // Direction if step, negative if step derection is not a decent direction
  double step_sign = 1;

  if (_d_phi_0 >= 0)
  {
    // Not a decent direction
    if (_d_phi_0 == 0)
      return 0;
    else
    {
      // Reverse step direction and calculate optimal step.
      _d_phi_0 *= -1;
      step_dir *= -1;
      step_sign = -1;

    }
  }

  // The Search Algorithm for T(mu) [More, Thuente 1994]

  int _max_step_iterations = 10;
  int _step_iterations = 0;

  // Sufficient decreace constant, Equation 1.1 [More, Thuete 1994]
  double _mu = 1.e-4;
  // Curvature condition constant, Equation 1.2 [More, Thuete 1994]
  double _nu = 0.9;

  // Initial endpoints of Interval I,
  double _a_l = 0, _a_u = 0;

  // Auxiliary function psi is used until I is determined ot be a closed interval, Equation 2.1 [More, Thuente 1994]
  double _f_l = auxilaryFunction_PsiMT (_a_l, _phi_0, _phi_0, _d_phi_0, _mu);
  double _g_l = auxilaryFunction_dPsiMT (_d_phi_0, _d_phi_0, _mu);

  double _f_u = auxilaryFunction_PsiMT (_a_u, _phi_0, _phi_0, _d_phi_0, _mu);
  double _g_u = auxilaryFunction_dPsiMT (_d_phi_0, _d_phi_0, _mu);

  // Check used to allow More-Thuente step length calculation to be skipped by making step_min == step_max
  bool _interval_converged = (step_max - step_min) > 0, _open_interval = true;

  double _a_t = step_init;
  _a_t = std::min (_a_t, step_max);
  _a_t = std::max (_a_t, step_min);

  _x_t = x + step_dir * _a_t;

  final_transformation_ = (Eigen::Translation<float, 3>(_x_t (0), _x_t (1), _x_t (2)) *
                           Eigen::AngleAxis<float>(_x_t (3), Eigen::Vector3f::UnitX ()) *
                           Eigen::AngleAxis<float>(_x_t (4), Eigen::Vector3f::UnitY ()) *
                           Eigen::AngleAxis<float>(_x_t (5), Eigen::Vector3f::UnitZ ())).matrix ();

  // New transformed point cloud
  transformPointCloud (*input_, trans_cloud, final_transformation_);

  // Updates score, gradient and hessian.  Hessian calculation is unessisary but testing showed that most step calculations use the
  // initial step suggestion and recalculation the reusable portions of the hessian would intail more computation time.
  score = computeDerivatives (score_gradient, hessian, trans_cloud, _x_t, true);

  // Calculate phi(alpha_t)
  double _phi_t = -score;
  // Calculate phi'(alpha_t)
  double _d_phi_t = -(score_gradient.dot (step_dir));

  // Calculate psi(alpha_t)
  double _psi_t = auxilaryFunction_PsiMT (_a_t, _phi_t, _phi_0, _d_phi_0, _mu);
  // Calculate psi'(alpha_t)
  double _d_psi_t = auxilaryFunction_dPsiMT (_d_phi_t, _d_phi_0, _mu);

  // Iterate until max number of iterations, interval convergance or a value satisfies the sufficient decrease, Equation 1.1, and curvature condition, Equation 1.2 [More, Thuente 1994]
  while (!_interval_converged && _step_iterations < _max_step_iterations && !(_psi_t <= 0 /*Sufficient Decrease*/ && _d_phi_t <= -_nu * _d_phi_0 /*Curvature Condition*/))
  {
    // Use auxilary function if interval I is not closed
    if (_open_interval)
    {
      _a_t = trialValueSelectionMT (_a_l, _f_l, _g_l,
                                    _a_u, _f_u, _g_u,
                                    _a_t, _psi_t, _d_psi_t);
    }
    else
    {
      _a_t = trialValueSelectionMT (_a_l, _f_l, _g_l,
                                    _a_u, _f_u, _g_u,
                                    _a_t, _phi_t, _d_phi_t);
    }

    _a_t = std::min (_a_t, step_max);
    _a_t = std::max (_a_t, step_min);

    _x_t = x + step_dir * _a_t;

    final_transformation_ = (Eigen::Translation<float, 3>(_x_t (0), _x_t (1), _x_t (2)) *
                             Eigen::AngleAxis<float>(_x_t (3), Eigen::Vector3f::UnitX ()) *
                             Eigen::AngleAxis<float>(_x_t (4), Eigen::Vector3f::UnitY ()) *
                             Eigen::AngleAxis<float>(_x_t (5), Eigen::Vector3f::UnitZ ())).matrix ();

    // New transformed point cloud
    // Done on final cloud to prevent wasted computation
    transformPointCloud (*input_, trans_cloud, final_transformation_);

    // Updates score, gradient. Values stored to prevent wasted computation.
    score = computeDerivatives (score_gradient, hessian, trans_cloud, _x_t, false);

    // Calculate phi(alpha_t+)
    _phi_t = -score;
    // Calculate phi'(alpha_t+)
    _d_phi_t = -(score_gradient.dot (step_dir));

    // Calculate psi(alpha_t+)
    _psi_t = auxilaryFunction_PsiMT (_a_t, _phi_t, _phi_0, _d_phi_0, _mu);
    // Calculate psi'(alpha_t+)
    _d_psi_t = auxilaryFunction_dPsiMT (_d_phi_t, _d_phi_0, _mu);

    // Check if I is now a closed interval
    if (_open_interval && (_psi_t <= 0 && _d_psi_t >= 0))
    {
      _open_interval = false;

      // Converts _f_l and _g_l from psi to phi
      _f_l = _f_l + _phi_0 - _mu * _d_phi_0 * _a_l;
      _g_l = _g_l + _mu * _d_phi_0;

      // Converts _f_u and _g_u from psi to phi
      _f_u = _f_u + _phi_0 - _mu * _d_phi_0 * _a_u;
      _g_u = _g_u + _mu * _d_phi_0;
    }

    if (_open_interval)
    {
      // Update interval end points using Updating Algorithm [More, Thuente 1994]
      _interval_converged = updateIntervalMT (_a_l, _f_l, _g_l,
                                              _a_u, _f_u, _g_u,
                                              _a_t, _psi_t, _d_psi_t);
    }
    else
    {
      // Update interval end points using Modified Updating Algorithm [More, Thuente 1994]
      _interval_converged = updateIntervalMT (_a_l, _f_l, _g_l,
                                              _a_u, _f_u, _g_u,
                                              _a_t, _phi_t, _d_phi_t);
    }

    _step_iterations++;
  }

  // If inner loop was run then hessian needs to be calculated.
  // Hessian is unnessisary for step length determination but gradients are required
  // so derivative and transform data is stored for the next iteration.
  if (_step_iterations)
    computeHessian (hessian, trans_cloud, _x_t);

  return step_sign * _a_t;
}

#endif // PCL_REGISTRATION_NDT_IMPL_H_
