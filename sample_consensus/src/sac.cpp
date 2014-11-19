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

#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/impl/rransac.hpp>
#include <pcl/sample_consensus/impl/msac.hpp>
#include <pcl/sample_consensus/impl/rmsac.hpp>
#include <pcl/sample_consensus/impl/prosac.hpp>
#include <pcl/sample_consensus/impl/mlesac.hpp>
#include <pcl/sample_consensus/impl/lmeds.hpp>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
  PCL_INSTANTIATE(RandomSampleConsensus, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)(pcl::PointNormal)(pcl::PointXYZRGBNormal))
  PCL_INSTANTIATE(MEstimatorSampleConsensus, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)(pcl::PointXYZRGBNormal))
  PCL_INSTANTIATE(RandomizedMEstimatorSampleConsensus, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)(pcl::PointXYZRGBNormal))
  PCL_INSTANTIATE(RandomizedRandomSampleConsensus, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)(pcl::PointXYZRGBNormal))
  PCL_INSTANTIATE(ProgressiveSampleConsensus, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)(pcl::PointXYZRGBNormal))
  PCL_INSTANTIATE(MaximumLikelihoodSampleConsensus, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)(pcl::PointXYZRGBNormal))
  PCL_INSTANTIATE(LeastMedianSquares, (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGBA)(pcl::PointXYZRGB)(pcl::PointXYZRGBNormal))
#else
  PCL_INSTANTIATE(RandomSampleConsensus, PCL_XYZ_POINT_TYPES)
  PCL_INSTANTIATE(MEstimatorSampleConsensus, PCL_XYZ_POINT_TYPES)
  PCL_INSTANTIATE(RandomizedMEstimatorSampleConsensus, PCL_XYZ_POINT_TYPES)
  PCL_INSTANTIATE(RandomizedRandomSampleConsensus, PCL_XYZ_POINT_TYPES)
  PCL_INSTANTIATE(ProgressiveSampleConsensus, PCL_XYZ_POINT_TYPES)
  PCL_INSTANTIATE(MaximumLikelihoodSampleConsensus, PCL_XYZ_POINT_TYPES)
  PCL_INSTANTIATE(LeastMedianSquares, PCL_XYZ_POINT_TYPES)
#endif
#endif    // PCL_NO_PRECOMPILE

