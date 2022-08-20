/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>

Eigen::Vector2f
pcl::MLSResult::calculatePrincipalCurvatures (const double u, const double v) const
{
  Eigen::Vector2f k (1e-5, 1e-5);

  // Note: this use the Monge Patch to derive the Gaussian curvature and Mean Curvature found here http://mathworld.wolfram.com/MongePatch.html
  // Then:
  //      k1 = H + sqrt(H^2 - K)
  //      k2 = H - sqrt(H^2 - K)
  if (order > 1 && c_vec.size () >= (order + 1) * (order + 2) / 2 && std::isfinite (c_vec[0]))
  {
    const PolynomialPartialDerivative d = getPolynomialPartialDerivative (u, v);
    const double Z = 1 + d.z_u * d.z_u + d.z_v * d.z_v;
    const double Zlen = std::sqrt (Z);
    const double K = (d.z_uu * d.z_vv - d.z_uv * d.z_uv) / (Z * Z);
    const double H = ((1.0 + d.z_v * d.z_v) * d.z_uu - 2.0 * d.z_u * d.z_v * d.z_uv + (1.0 + d.z_u * d.z_u) * d.z_vv) / (2.0 * Zlen * Zlen * Zlen);
    const double disc2 = H * H - K;
    assert (disc2 >= 0.0);
    const double disc = std::sqrt (disc2);
    k[0] = H + disc;
    k[1] = H - disc;

    if (std::abs (k[0]) > std::abs (k[1])) std::swap (k[0], k[1]);
  }
  else
  {
    PCL_ERROR ("No Polynomial fit data, unable to calculate the principal curvatures!\n");
  }

  return (k);
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#ifdef PCL_ONLY_CORE_POINT_TYPES
 // Instantiations of specific point types
  PCL_INSTANTIATE_PRODUCT(MovingLeastSquares, ((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointXYZRGBNormal)(pcl::PointNormal))
                                              ((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointXYZRGBNormal)(pcl::PointNormal)))
#else
  PCL_INSTANTIATE_PRODUCT(MovingLeastSquares, (PCL_XYZ_POINT_TYPES)(PCL_XYZ_POINT_TYPES))
#endif
#endif    // PCL_NO_PRECOMPILE
