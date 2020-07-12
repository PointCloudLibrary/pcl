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
 * $Id: test_normal_estimation.cpp 5066 2012-03-14 06:42:21Z rusu $
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/vfh.h>
#include <pcl/features/usc.h>
#include <pcl/features/statistical_multiscale_interest_region_extraction.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/rsd.h>
#include <pcl/features/rift.h>
#include <pcl/features/normal_based_signature.h>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, FeaturePtr)
{
  VFHEstimation<PointXYZ, PointNormal, VFHSignature308>::Ptr vfh (new VFHEstimation<PointXYZ, PointNormal, VFHSignature308> ());
  vfh->setViewPoint (1.0f, 1.0f, 1.0f);

  UniqueShapeContext<PointXYZ, UniqueShapeContext1960, ReferenceFrame>::Ptr usc (new UniqueShapeContext<PointXYZ, UniqueShapeContext1960, ReferenceFrame> ());
  usc->setMinimalRadius (5);

  StatisticalMultiscaleInterestRegionExtraction<PointXYZ>::Ptr smire (new StatisticalMultiscaleInterestRegionExtraction<PointXYZ> ());
  smire->getScalesVector ();

  SpinImageEstimation<PointXYZ, PointNormal, Histogram<153> >::Ptr spin (new SpinImageEstimation<PointXYZ, PointNormal, Histogram<153> > ());
  spin->setImageWidth (20);

//  RSDEstimation<PointXYZ, Normal, PrincipalRadiiRSD>::Ptr rsd (new RSDEstimation<PointXYZ, Normal, PrincipalRadiiRSD> ());
//  rsd->setNrSubdivisions (20);

#ifndef PCL_ONLY_CORE_POINT_TYPES
  RIFTEstimation<PointXYZI, IntensityGradient, Histogram<32> >::Ptr rift (new RIFTEstimation<PointXYZI, IntensityGradient, Histogram<32> > ());
  rift->setNrDistanceBins (10);
#endif
  NormalBasedSignatureEstimation<PointXYZ, Normal, NormalBasedSignature12>::Ptr nbs (new NormalBasedSignatureEstimation<PointXYZ, Normal, NormalBasedSignature12> ());
  nbs->setN (20);
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
