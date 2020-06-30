/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
#include <pcl/tracking/impl/particle_filter.hpp>
#include <pcl/tracking/impl/particle_filter_omp.hpp>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#define PCL_TRACKING_NORMAL_SUPPORTED

// clang-format off
PCL_INSTANTIATE_PRODUCT(ParticleFilterTracker,
                        ((pcl::PointNormal)
                         (pcl::PointXYZINormal)
                         (pcl::PointXYZRGBNormal))
                        (PCL_STATE_POINT_TYPES))
PCL_INSTANTIATE_PRODUCT(ParticleFilterOMPTracker,
                        ((pcl::PointNormal)
                         (pcl::PointXYZINormal)
                         (pcl::PointXYZRGBNormal))
                        (PCL_STATE_POINT_TYPES))
// clang-format on
#undef PCL_TRACKING_NORMAL_SUPPORTED
// clang-format off
PCL_INSTANTIATE_PRODUCT(ParticleFilterTracker,
                        ((pcl::PointXYZ)
                         (pcl::PointXYZI)
                         (pcl::PointXYZRGBA)
                         (pcl::PointXYZRGB)
                         (pcl::InterestPoint)
                         (pcl::PointWithRange)
                         (pcl::PointWithViewpoint)
                         (pcl::PointWithScale))
                        (PCL_STATE_POINT_TYPES))
PCL_INSTANTIATE_PRODUCT(ParticleFilterOMPTracker,
                        ((pcl::PointXYZ)
                         (pcl::PointXYZI)
                         (pcl::PointXYZRGBA)
                         (pcl::PointXYZRGB)
                         (pcl::InterestPoint)
                         (pcl::PointWithRange)
                         (pcl::PointWithViewpoint)
                         (pcl::PointWithScale))
                        (PCL_STATE_POINT_TYPES))
// clang-format on
#endif // PCL_NO_PRECOMPILE
