/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: point_cloud.h 4696 2012-02-23 06:12:55Z rusu $
 *
 */

#pragma once

#include <pcl/point_types.h>

namespace pcl {
namespace tracking {
/* state definition */
struct ParticleXYZRPY;
struct ParticleXYR;

/* \brief return the value of normal distribution */
PCL_EXPORTS double
sampleNormal(double mean, double sigma);
} // namespace tracking
} // namespace pcl

#include <pcl/tracking/impl/tracking.hpp>

// ==============================
// =====POINT_CLOUD_REGISTER=====
// ==============================

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::tracking::_ParticleXYZRPY,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, roll, roll)
                                  (float, pitch, pitch)
                                  (float, yaw, yaw))
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::tracking::ParticleXYZRPY,
                                   pcl::tracking::_ParticleXYZRPY)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::tracking::_ParticleXYRPY,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, roll, roll)
                                  (float, pitch, pitch)
                                  (float, yaw, yaw))
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::tracking::ParticleXYRPY,
                                   pcl::tracking::_ParticleXYRPY)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::tracking::_ParticleXYRP,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, roll, roll)
                                  (float, pitch, pitch)
                                  (float, yaw, yaw))
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::tracking::ParticleXYRP,
                                   pcl::tracking::_ParticleXYRP)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::tracking::_ParticleXYR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, roll, roll)
                                  (float, pitch, pitch)
                                  (float, yaw, yaw))
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::tracking::ParticleXYR,
                                   pcl::tracking::_ParticleXYR)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::tracking::_ParticleXYZR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, roll, roll)
                                  (float, pitch, pitch)
                                  (float, yaw, yaw))
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::tracking::ParticleXYZR,
                                   pcl::tracking::_ParticleXYZR)
// clang-format on

#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/tracking.hpp>
#endif
