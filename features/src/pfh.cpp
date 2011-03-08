/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: pfh.cpp 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#include "pcl/point_types.h"
#include "pcl/impl/instantiate.hpp"
#include "pcl/features/pfh.h"
#include "pcl/features/impl/pfh.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::computePairFeatures (const Eigen::Vector4f &p1, const Eigen::Vector4f &n1, 
                          const Eigen::Vector4f &p2, const Eigen::Vector4f &n2,
                          float &f1, float &f2, float &f3, float &f4)
{
  // Compute the Cartesian difference between the two points
  Eigen::Vector4f delta = p2 - p1;
  delta[3] = 0;

  // Compute the Euclidean norm = || p_idx - q_idx ||
  float distance_sqr = delta.squaredNorm ();

  if (distance_sqr == 0)
  {
    ROS_ERROR ("Euclidean distance between points is 0!");
    f1 = f2 = f3 = f4 = 0;
    return (false);
  }

  // Estimate f4 = || delta ||
  f4 = sqrt (distance_sqr);

  // Create a Darboux frame coordinate system u-v-w
  // u = n1; v = (p_idx - q_idx) x u / || (p_idx - q_idx) x u ||; w = u x v

  // Estimate f3 = u * delta / || delta ||
  // delta[3] = 0 (line 59)
  f3 = n1.dot (delta) / f4;

  // v = delta * u
  Eigen::Vector4f v = Eigen::Vector4f::Zero ();
  v = delta.cross3 (n1);

  distance_sqr = v.squaredNorm ();
  if (distance_sqr == 0)
  {
    ROS_ERROR ("Norm of Delta x U is 0!");
    f1 = f2 = f3 = f4 = 0;
    return (false);
  }

  // Copy the q_idx normal
  Eigen::Vector4f nq = n2;
  nq[3] = 0;

  // Normalize the vector
  v /= sqrt (distance_sqr);

  // Compute delta (w) = u x v
  delta = n1.cross3 (v);

  // Compute f2 = v * n2;
  // v[3] = 0 (line 82)
  f2 = v.dot (nq);

  // Compute f1 = arctan (w * n2, u * n2) i.e. angle of n2 in the x=u, y=w coordinate system
  // delta[3] = 0 (line 59), nq[3] = 0 (line 97)
  f1 = atan2f (delta.dot (nq), n1.dot (nq));       // @todo: optimize this

  return (true);
}

// Instantiations of specific point types
PCL_INSTANTIATE_PRODUCT(PFHEstimation, (PCL_XYZ_POINT_TYPES)(PCL_NORMAL_POINT_TYPES)((pcl::PFHSignature125)));

