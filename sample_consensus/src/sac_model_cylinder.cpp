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

#include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>
#include <unsupported/Eigen/NonLinearOptimization> // for LevenbergMarquardt

int pcl::internal::optimizeModelCoefficientsCylinder (Eigen::VectorXf& coeff, const Eigen::ArrayXf& pts_x, const Eigen::ArrayXf& pts_y, const Eigen::ArrayXf& pts_z)
{
  if(pts_x.size() != pts_y.size() || pts_y.size() != pts_z.size()) {
    PCL_ERROR("[pcl::internal::optimizeModelCoefficientsCylinder] Sizes not equal!\n");
    return Eigen::LevenbergMarquardtSpace::ImproperInputParameters;
  }
  if(coeff.size() != 7) {
    PCL_ERROR("[pcl::internal::optimizeModelCoefficientsCylinder] Coefficients have wrong size\n");
    return Eigen::LevenbergMarquardtSpace::ImproperInputParameters;
  }
  struct CylinderOptimizationFunctor : pcl::Functor<float>
  {
    CylinderOptimizationFunctor (const Eigen::ArrayXf& x, const Eigen::ArrayXf& y, const Eigen::ArrayXf& z,
                                 const Eigen::Vector3f& ref_pt, const Eigen::Vector3f& ref_dir,
                                 const Eigen::Vector3f& u, const Eigen::Vector3f& v) :
      pcl::Functor<float>(x.size()), pts_x(x), pts_y(y), pts_z(z), ref_pt(ref_pt), ref_dir(ref_dir), u(u), v(v)
      {}

    int
    operator() (const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
    {
      Eigen::Vector3f line_pt = ref_pt + x[0] * u + x[1] * v;
      Eigen::Vector3f line_dir = ref_dir + x[2] * u + x[3] * v;
      line_dir.normalize();
      const Eigen::ArrayXf bx = Eigen::ArrayXf::Constant(pts_x.size(), line_pt.x()) - pts_x;
      const Eigen::ArrayXf by = Eigen::ArrayXf::Constant(pts_x.size(), line_pt.y()) - pts_y;
      const Eigen::ArrayXf bz = Eigen::ArrayXf::Constant(pts_x.size(), line_pt.z()) - pts_z;
      // compute the distance of point b to the line (cross product), then subtract the model radius
      fvec = ((line_dir.y() * bz - line_dir.z() * by).square()
             +(line_dir.z() * bx - line_dir.x() * bz).square()
             +(line_dir.x() * by - line_dir.y() * bx).square()).sqrt()
             - std::abs(x[4]);
      return (0);
    }

    const Eigen::ArrayXf& pts_x, pts_y, pts_z;
    const Eigen::Vector3f& ref_pt, ref_dir, u, v;
  };

  Eigen::Vector3f line_pt(coeff[0], coeff[1], coeff[2]);
  Eigen::Vector3f line_dir(coeff[3], coeff[4], coeff[5]);
  line_dir.normalize();

  // Compute the center of the point cloud and project it onto the cylinder axis.
  // We do this so that the nonlinear optimization rotates the cylinder around the center, leading to a better convergence.
  Eigen::Vector3f center(pts_x.mean(), pts_y.mean(), pts_z.mean());
  line_pt += (center - line_pt).dot(line_dir) * line_dir;

  // find two vectors u, v spanning the plane orthogonal to line_dir
  Eigen::Vector3f u = std::abs(line_dir.x()) < std::abs(line_dir.y())
                          ? (std::abs(line_dir.x()) < std::abs(line_dir.z())
                                 ? Eigen::Vector3f(1.f, 0.f, 0.f)
                                 : Eigen::Vector3f(0.f, 0.f, 1.f))
                          : Eigen::Vector3f(0.f, 1.f, 0.f);
  u -= line_dir.dot(u) * line_dir;
  u.normalize();
  Eigen::Vector3f v = line_dir.cross(u);
  CylinderOptimizationFunctor functor (pts_x, pts_y, pts_z, line_pt, line_dir, u, v);
  Eigen::NumericalDiff<CylinderOptimizationFunctor> num_diff (functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CylinderOptimizationFunctor>, float> lm (num_diff);

  Eigen::VectorXf coeff_optim(5);
  coeff_optim << 0.0f, 0.0f, 0.0f, 0.0f, coeff[6];
  const int info = lm.minimize (coeff_optim);
  Eigen::Vector3f line_pt_optim = line_pt + coeff_optim[0] * u + coeff_optim[1] * v;
  Eigen::Vector3f line_dir_optim = line_dir + coeff_optim[2] * u + coeff_optim[3] * v;
  coeff[0] = line_pt_optim.x();
  coeff[1] = line_pt_optim.y();
  coeff[2] = line_pt_optim.z();
  coeff[3] = line_dir_optim.x();
  coeff[4] = line_dir_optim.y();
  coeff[5] = line_dir_optim.z();
  coeff[6] = std::abs(coeff_optim[4]);
  PCL_DEBUG ("[pcl::internal::optimizeModelCoefficientsCylinder] LM solver finished with exit code %i, having a residual norm of %g.\n",
             info, lm.fvec.norm ());
  return info;
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
  PCL_INSTANTIATE_PRODUCT(SampleConsensusModelCylinder, ((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB))((pcl::Normal)))
#else
 PCL_INSTANTIATE_PRODUCT(SampleConsensusModelCylinder, (PCL_XYZ_POINT_TYPES)(PCL_NORMAL_POINT_TYPES))
#endif
#endif    // PCL_NO_PRECOMPILE

