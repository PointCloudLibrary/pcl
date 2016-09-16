/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011-2012, Willow Garage, Inc.
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
#ifndef PCL_NDT_2D_IMPL_H_
#define PCL_NDT_2D_IMPL_H_
#include <cmath>

#include <pcl/registration/eigen.h>
#include <pcl/registration/boost.h>


namespace Eigen
{
  /* This NumTraits specialisation is necessary because NormalDist is used as
   * the element type of an Eigen Matrix.
   */
  template<typename PointT> struct NumTraits<pcl::ndt2d::NormalDist<PointT> >
  {
    typedef double Real;
    static Real dummy_precision () { return 1.0; }
    enum {
      IsComplex = 0,
      IsInteger = 0,
      IsSigned = 0,
      RequireInitialization = 1,
      ReadCost = 1,
      AddCost = 1,
      MulCost = 1
    };
  };
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ndt2d::NormalDist<PointT>::estimateParams (const PointCloud& cloud, double min_covar_eigvalue_mult)
{
  Eigen::Vector2d sx  = Eigen::Vector2d::Zero ();
  Eigen::Matrix2d sxx = Eigen::Matrix2d::Zero ();

  std::vector<size_t>::const_iterator i;
  for (i = pt_indices_.begin (); i != pt_indices_.end (); i++)
  {
    Eigen::Vector2d p (cloud[*i]. x, cloud[*i]. y);
    sx  += p;
    sxx += p * p.transpose ();
  }

  n_ = pt_indices_.size ();

  if (n_ >= min_n_)
  {
    mean_ = sx / static_cast<double> (n_);
    // Using maximum likelihood estimation as in the original paper
    Eigen::Matrix2d covar = (sxx - 2 * (sx * mean_.transpose ())) / static_cast<double> (n_) + mean_ * mean_.transpose ();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver (covar);
    if (solver.eigenvalues ()[0] < min_covar_eigvalue_mult * solver.eigenvalues ()[1])
    {
      PCL_DEBUG ("[pcl::NormalDist::estimateParams] NDT normal fit: adjusting eigenvalue %f\n", solver.eigenvalues ()[0]);
      Eigen::Matrix2d l = solver.eigenvalues ().asDiagonal ();
      Eigen::Matrix2d q = solver.eigenvectors ();
      // set minimum smallest eigenvalue:
      l (0,0) = l (1,1) * min_covar_eigvalue_mult;
      covar = q * l * q.transpose ();
    }
    covar_inv_ = covar.inverse ();
  }

  pt_indices_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::ndt2d::ValueAndDerivatives<3,double>
pcl::ndt2d::NormalDist<PointT>::test (const PointT& transformed_pt, const double& cos_theta, const double& sin_theta) const
{
  if (n_ < min_n_)
    return ValueAndDerivatives<3,double>::Zero ();

  ValueAndDerivatives<3,double> r;
  const double x = transformed_pt.x;
  const double y = transformed_pt.y;
  const Eigen::Vector2d p_xy (transformed_pt.x, transformed_pt.y);
  const Eigen::Vector2d q = p_xy - mean_;
  const Eigen::RowVector2d qt_cvi (q.transpose () * covar_inv_);
  const double exp_qt_cvi_q = std::exp (-0.5 * double (qt_cvi * q));
  r.value = -exp_qt_cvi_q;

  Eigen::Matrix<double, 2, 3> jacobian;
  jacobian <<
    1, 0, -(x * sin_theta + y*cos_theta),
    0, 1,   x * cos_theta - y*sin_theta;

  for (size_t i = 0; i < 3; i++)
    r.grad[i] = double (qt_cvi * jacobian.col (i)) * exp_qt_cvi_q;

  // second derivative only for i == j == 2:
  const Eigen::Vector2d d2q_didj (
      y * sin_theta - x*cos_theta,
    -(x * sin_theta + y*cos_theta)
  );

  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 3; j++)
      r.hessian (i,j) = -exp_qt_cvi_q * (
        double (-qt_cvi*jacobian.col (i)) * double (-qt_cvi*jacobian.col (j)) +
        (-qt_cvi * ((i==2 && j==2)? d2q_didj : Eigen::Vector2d::Zero ())) +
        (-jacobian.col (j).transpose () * covar_inv_ * jacobian.col (i))
      );

  return r;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::NormalDistributionsTransform2D<PointSource, PointTarget>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess)
{
  if (!target_ndt_)
  {
    PCL_ERROR ("Missing NDT2D target point cloud");
    return;
  }
  ndt2d::NDT2D<PointTarget>& target_ndt = *target_ndt_;

  PointCloudSource intm_cloud;

  if (guess != Eigen::Matrix4f::Identity ())
  {
    transformation_ = guess;
    transformPointCloud (output, intm_cloud, transformation_);
  }
  else
    intm_cloud = output;

  // can't seem to use .block<> () member function on transformation_
  // directly... gcc bug? 
  Eigen::Matrix4f& transformation = transformation_;


  // work with x translation, y translation and z rotation: extending to 3D
  // would be some tricky maths, but not impossible.
  const Eigen::Matrix3f initial_rot (transformation.block<3,3> (0,0));
  const Eigen::Vector3f rot_x (initial_rot*Eigen::Vector3f::UnitX ());
  const double z_rotation = std::atan2 (rot_x[1], rot_x[0]);

  Eigen::Vector3d xytheta_transformation (
    transformation (0,3),
    transformation (1,3),
    z_rotation
  );

  while (!converged_)
  {
    const double cos_theta = std::cos (xytheta_transformation[2]);
    const double sin_theta = std::sin (xytheta_transformation[2]);
    previous_transformation_ = transformation;    

    ndt2d::ValueAndDerivatives<3, double> score = ndt2d::ValueAndDerivatives<3, double>::Zero ();
    for (size_t i = 0; i < intm_cloud.size (); i++)
      score += target_ndt.test (intm_cloud[i], cos_theta, sin_theta);
    
    PCL_DEBUG ("[pcl::NormalDistributionsTransform2D::computeTransformation] NDT score %f (x=%f,y=%f,r=%f)\n",
      float (score.value), xytheta_transformation[0], xytheta_transformation[1], xytheta_transformation[2]
    );

    if (score.value != 0)
    {
      // test for positive definiteness, and adjust to ensure it if necessary:
      Eigen::EigenSolver<Eigen::Matrix3d> solver;
      solver.compute (score.hessian, false);
      double min_eigenvalue = 0;
      for (int i = 0; i <3; i++)
        if (solver.eigenvalues ()[i].real () < min_eigenvalue)
            min_eigenvalue = solver.eigenvalues ()[i].real ();

      // ensure "safe" positive definiteness: this is a detail missing
      // from the original paper
      if (min_eigenvalue < 0)
      {
        double lambda = 1.1 * min_eigenvalue - 1;
        score.hessian += Eigen::Vector3d (-lambda, -lambda, -lambda).asDiagonal ();
        solver.compute (score.hessian, false);
        PCL_DEBUG ("[pcl::NormalDistributionsTransform2D::computeTransformation] adjust hessian: %f: new eigenvalues:%f %f %f\n",
            float (lambda),
            solver.eigenvalues ()[0].real (),
            solver.eigenvalues ()[1].real (),
            solver.eigenvalues ()[2].real ()
        );
      }
      assert (solver.eigenvalues ()[0].real () >= 0 &&
              solver.eigenvalues ()[1].real () >= 0 &&
              solver.eigenvalues ()[2].real () >= 0);
      
      Eigen::Vector3d delta_transformation (-score.hessian.inverse () * score.grad);
      Eigen::Vector3d new_transformation = xytheta_transformation + newton_lambda_.cwiseProduct (delta_transformation);

      xytheta_transformation = new_transformation;
      
      // update transformation matrix from x, y, theta:
      transformation.block<3,3> (0,0).matrix () = Eigen::Matrix3f (Eigen::AngleAxisf (static_cast<float> (xytheta_transformation[2]), Eigen::Vector3f::UnitZ ()));
      transformation.block<3,1> (0,3).matrix () = Eigen::Vector3f (static_cast<float> (xytheta_transformation[0]), static_cast<float> (xytheta_transformation[1]), 0.0f);

      //std::cout << "new transformation:\n" << transformation << std::endl;
    }
    else
    {
      PCL_ERROR ("[pcl::NormalDistributionsTransform2D::computeTransformation] no overlap: try increasing the size or reducing the step of the grid\n");
      break;
    }
    
    transformPointCloud (output, intm_cloud, transformation);

    nr_iterations_++;
    
    if (update_visualizer_ != 0)
      update_visualizer_ (output, *indices_, *target_, *indices_);

    //std::cout << "eps=" << fabs ((transformation - previous_transformation_).sum ()) << std::endl;

    if (nr_iterations_ > max_iterations_ ||
       (transformation - previous_transformation_).array ().abs ().sum () < transformation_epsilon_)
      converged_ = true;
  }
  final_transformation_ = transformation;
  output = intm_cloud;
}

#endif    // PCL_NDT_2D_IMPL_H_
 
