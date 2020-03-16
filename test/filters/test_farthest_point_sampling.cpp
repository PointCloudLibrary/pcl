/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020-, Open Perception, Inc.
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
 */

#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/farthest_point_sampling.h>

using namespace pcl;
PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ>);

TEST (FarthestPointSampling, farthest_point_sampling)
{
  const static int SAMPLE = 100;
  PointCloud<PointXYZ>::Ptr cloud_out (new PointCloud<PointXYZ>);
  FarthestPointSampling<PointXYZ> fps;
  fps.setInputCloud(cloud_in);
  fps.setSample(SAMPLE);
  fps.filter(*cloud_out);
  EXPECT_EQ (cloud_out->points.size(),  SAMPLE);
}


int 
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download bunny.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  if (io::loadPCDFile<PointXYZ> (argv[1], *cloud_in) == -1) //* load the file
  {
    std::cerr << "Please download `bunny.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

