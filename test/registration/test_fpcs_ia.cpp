/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2014-, Open Perception, Inc.
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

#include <pcl/test/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ia_fpcs.h>

#include "test_fpcs_ia_data.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::registration;

PointCloud<PointXYZ> cloud_source, cloud_target;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, FPCSInitialAlignment)
{
  // transform the source cloud by a large amount
  Eigen::Vector3f initial_offset (1.f, 0.f, 0.f);
  float angle = static_cast<float> (M_PI) / 2.f;
  Eigen::Quaternionf initial_rotation (std::cos (angle / 2.f), 0, 0, std::sin (angle / 2.f));
  PointCloud<PointXYZ> cloud_source_transformed;
  transformPointCloud (cloud_source, cloud_source_transformed, initial_offset, initial_rotation);

  // create shared pointers
  PointCloud<PointXYZ>::Ptr cloud_source_ptr, cloud_target_ptr;
  cloud_source_ptr = cloud_source_transformed.makeShared ();
  cloud_target_ptr = cloud_target.makeShared ();

  // initialize fpcs
  PointCloud <PointXYZ> source_aligned;
  FPCSInitialAlignment <PointXYZ, PointXYZ> fpcs_ia;
  fpcs_ia.setInputSource (cloud_source_ptr);
  fpcs_ia.setInputTarget (cloud_target_ptr);

  fpcs_ia.setNumberOfThreads (nr_threads);
  fpcs_ia.setApproxOverlap (approx_overlap);
  fpcs_ia.setDelta (delta, true);
  fpcs_ia.setNumberOfSamples (nr_samples);

  // align
  fpcs_ia.align (source_aligned);
  EXPECT_EQ (source_aligned.size (), cloud_source.size ());

  // check for correct coarse transformation marix
  //Eigen::Matrix4f transform_res_from_fpcs = fpcs_ia.getFinalTransformation ();
  //for (int i = 0; i < 4; ++i)
  //  for (int j = 0; j < 4; ++j)
  //    EXPECT_NEAR (transform_res_from_fpcs (i,j), transform_from_fpcs[i][j], 0.5);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
main (int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "No test files given. Please download `bun0.pcd` and `bun4.pcd` pass their path to the test." << std::endl;
    return (-1);
  }

  // Input
  if (loadPCDFile (argv[1], cloud_source) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  if (loadPCDFile (argv[2], cloud_target) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun4.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
