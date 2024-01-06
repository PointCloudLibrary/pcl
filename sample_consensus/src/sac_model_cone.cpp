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

#include <pcl/sample_consensus/impl/sac_model_cone.hpp>
#include <unsupported/Eigen/NonLinearOptimization> // for LevenbergMarquardt

int pcl::internal::optimizeModelCoefficientsCone (Eigen::VectorXf& coeff, const Eigen::ArrayXf& pts_x, const Eigen::ArrayXf& pts_y, const Eigen::ArrayXf& pts_z)
{
  if(pts_x.size() != pts_y.size() || pts_y.size() != pts_z.size()) {
    PCL_ERROR("[pcl::internal::optimizeModelCoefficientsCone] Sizes not equal!\n");
    return Eigen::LevenbergMarquardtSpace::ImproperInputParameters;
  }
  if(coeff.size() != 7) {
    PCL_ERROR("[pcl::internal::optimizeModelCoefficientsCone] Coefficients have wrong size\n");
    return Eigen::LevenbergMarquardtSpace::ImproperInputParameters;
  }
  struct ConeOptimizationFunctor : pcl::Functor<float>
  {
    ConeOptimizationFunctor (const Eigen::ArrayXf& x, const Eigen::ArrayXf& y, const Eigen::ArrayXf& z) :
      pcl::Functor<float>(x.size()), pts_x(x), pts_y(y), pts_z(z)
      {}

    int
    operator() (const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
    {
      Eigen::Vector3f axis_dir(x[3], x[4], x[5]);
      axis_dir.normalize();
      const Eigen::ArrayXf axis_dir_x = Eigen::ArrayXf::Constant(pts_x.size(), axis_dir.x());
      const Eigen::ArrayXf axis_dir_y = Eigen::ArrayXf::Constant(pts_x.size(), axis_dir.y());
      const Eigen::ArrayXf axis_dir_z = Eigen::ArrayXf::Constant(pts_x.size(), axis_dir.z());
      const Eigen::ArrayXf bx = Eigen::ArrayXf::Constant(pts_x.size(), x[0]) - pts_x;
      const Eigen::ArrayXf by = Eigen::ArrayXf::Constant(pts_x.size(), x[1]) - pts_y;
      const Eigen::ArrayXf bz = Eigen::ArrayXf::Constant(pts_x.size(), x[2]) - pts_z;
      const Eigen::ArrayXf actual_cone_radius = std::tan(x[6]) *
          (bx*axis_dir_x+by*axis_dir_y+bz*axis_dir_z);
      // compute the squared distance of point b to the line (cross product), then subtract the actual cone radius (squared)
      fvec = ((axis_dir_y * bz - axis_dir_z * by).square()
             +(axis_dir_z * bx - axis_dir_x * bz).square()
             +(axis_dir_x * by - axis_dir_y * bx).square())
             -actual_cone_radius.square();
      return (0);
    }

    const Eigen::ArrayXf& pts_x, pts_y, pts_z;
  };

  ConeOptimizationFunctor functor (pts_x, pts_y, pts_z);
  Eigen::NumericalDiff<ConeOptimizationFunctor> num_diff (functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<ConeOptimizationFunctor>, float> lm (num_diff);
  const int info = lm.minimize (coeff);
  PCL_DEBUG ("[pcl::internal::optimizeModelCoefficientsCone] LM solver finished with exit code %i, having a residual norm of %g.\n",
             info, lm.fvec.norm ());
  return info;
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
  PCL_INSTANTIATE_PRODUCT(SampleConsensusModelCone, ((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB))((pcl::Normal)))
#else
 PCL_INSTANTIATE_PRODUCT(SampleConsensusModelCone, (PCL_XYZ_POINT_TYPES)(PCL_NORMAL_POINT_TYPES))
#endif
#endif    // PCL_NO_PRECOMPILE

