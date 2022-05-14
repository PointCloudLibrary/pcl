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

#include <limits>

#include <pcl/test/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ia_kfpcs.h>

#include "test_kfpcs_ia_data.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::registration;

PointCloud<PointXYZI> cloud_source, cloud_target;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, KFPCSInitialAlignment)
{
  const auto previous_verbosity_level = pcl::console::getVerbosityLevel();
  pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
  // create shared pointers
  PointCloud<PointXYZI>::Ptr cloud_source_ptr, cloud_target_ptr;
  cloud_source_ptr = cloud_source.makeShared ();
  cloud_target_ptr = cloud_target.makeShared ();

  // initialize k-fpcs
  PointCloud <PointXYZI> cloud_source_aligned;
  KFPCSInitialAlignment <PointXYZI, PointXYZI> kfpcs_ia;
  kfpcs_ia.setInputSource (cloud_source_ptr);
  kfpcs_ia.setInputTarget (cloud_target_ptr);

  //kfpcs_ia.setNumberOfThreads (nr_threads);
  kfpcs_ia.setApproxOverlap (approx_overlap);
  kfpcs_ia.setDelta (voxel_size, false);
  kfpcs_ia.setScoreThreshold (abort_score);

  // repeat alignment 2 times to increase probability to ~99.99%
  const float max_angle3d = 0.1745f, max_translation3d = 1.f;
  float angle3d = std::numeric_limits<float>::max(), translation3d = std::numeric_limits<float>::max();
  for (int i = 0; i < 2; i++)
  {
    kfpcs_ia.align (cloud_source_aligned);
  
    // copy initial matrix
    Eigen::Matrix4f transform_groundtruth;
    for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      transform_groundtruth (i, j) = transformation_office1_office2[i][j];

    // check for correct transformation
    Eigen::Matrix4f transform_rest = kfpcs_ia.getFinalTransformation ().colPivHouseholderQr ().solve (transform_groundtruth);
    angle3d = std::min (angle3d, Eigen::AngleAxisf (transform_rest.block <3, 3> (0, 0)).angle ());
    translation3d = std::min (translation3d, transform_rest.block <3, 1> (0, 3).norm ());
    
    if (angle3d < max_angle3d && translation3d < max_translation3d)
      break;
  }

  EXPECT_EQ (cloud_source_aligned.size (), cloud_source.size ());
  EXPECT_NEAR (angle3d, 0.f, max_angle3d);
  EXPECT_NEAR (translation3d, 0.f, max_translation3d);
  pcl::console::setVerbosityLevel(previous_verbosity_level); // reset verbosity level
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
