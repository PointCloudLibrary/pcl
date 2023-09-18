/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 */

#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
#include <unsupported/Eigen/NonLinearOptimization> // for LevenbergMarquardt

int pcl::internal::optimizeModelCoefficientsSphere (Eigen::VectorXf& coeff, const Eigen::ArrayXf& pts_x, const Eigen::ArrayXf& pts_y, const Eigen::ArrayXf& pts_z)
{
  if(pts_x.size() != pts_y.size() || pts_y.size() != pts_z.size()) {
    PCL_ERROR("[pcl::internal::optimizeModelCoefficientsSphere] Sizes not equal!\n");
    return Eigen::LevenbergMarquardtSpace::ImproperInputParameters;
  }
  if(coeff.size() != 4) {
    PCL_ERROR("[pcl::internal::optimizeModelCoefficientsSphere] Coefficients have wrong size\n");
    return Eigen::LevenbergMarquardtSpace::ImproperInputParameters;
  }
  struct SphereOptimizationFunctor : pcl::Functor<float>
  {
    SphereOptimizationFunctor (const Eigen::ArrayXf& x, const Eigen::ArrayXf& y, const Eigen::ArrayXf& z) :
      pcl::Functor<float>(x.size()), pts_x(x), pts_y(y), pts_z(z)
      {}

    int
    operator() (const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
    {
      // Compute distance of all points to center, then subtract radius
      fvec = ((Eigen::ArrayXf::Constant(pts_x.size(), x[0])-pts_x).square()
             +(Eigen::ArrayXf::Constant(pts_x.size(), x[1])-pts_y).square()
             +(Eigen::ArrayXf::Constant(pts_x.size(), x[2])-pts_z).square()).sqrt()
             -Eigen::ArrayXf::Constant(pts_x.size(), x[3]);
      return (0);
    }

    const Eigen::ArrayXf& pts_x, pts_y, pts_z;
  };

  SphereOptimizationFunctor functor (pts_x, pts_y, pts_z);
  Eigen::NumericalDiff<SphereOptimizationFunctor> num_diff (functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<SphereOptimizationFunctor>, float> lm (num_diff);
  const int info = lm.minimize (coeff);
  PCL_DEBUG ("[pcl::internal::optimizeModelCoefficientsSphere] LM solver finished with exit code %i, having a residual norm of %g.\n",
             info, lm.fvec.norm ());
  return info;
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
  PCL_INSTANTIATE(SampleConsensusModelSphere, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)(pcl::PointXYZRGBNormal))
#else
  PCL_INSTANTIATE(SampleConsensusModelSphere, PCL_XYZ_POINT_TYPES)
#endif
#endif    // PCL_NO_PRECOMPILE

