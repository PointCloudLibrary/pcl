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

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/uniform_sampling.h>

using namespace pcl;

PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (UniformSamplingRemoveIndicesSize, Uniform_Sampling)
{
  const bool keep_remove_indices = true;
  UniformSampling<PointXYZ> us (keep_remove_indices);
  us.setInputCloud (cloud);
  us.setRadiusSearch (0.003);
  PointCloud<PointXYZ>::Ptr cloud_filtered (new PointCloud<PointXYZ> ());
  us.filter (*cloud_filtered);

  const IndicesConstPtr& remove_indices = us.getRemovedIndices ();

  EXPECT_EQ(remove_indices->size () + cloud_filtered->points.size (),
            cloud->points.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (UniformSamplingRemoveIndicesSizeWithSetIndices, Uniform_Sampling)
{
  IndicesPtr indices (new std::vector<int>);
  for (std::size_t i = 0; i < cloud->points.size (); i += 2)
    indices->push_back (i);

  const bool keep_remove_indices = true;
  UniformSampling<PointXYZ> us (keep_remove_indices);
  us.setInputCloud (cloud);
  us.setIndices (indices);
  us.setRadiusSearch (0.003);
  PointCloud<PointXYZ>::Ptr cloud_filtered (new PointCloud<PointXYZ> ());
  us.filter (*cloud_filtered);

  const IndicesConstPtr& remove_indices = us.getRemovedIndices ();

  EXPECT_EQ(remove_indices->size () + cloud_filtered->points.size (),
            cloud->points.size () - indices->size ());
}

int
main (int argc,
      char** argv)
{
  // Load a standard PCD file from disk
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `milk_cartoon_all_small_clorox.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  char* file_name = argv[1];
  // Load a standard PCD file from disk
  io::loadPCDFile (file_name, *cloud);

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}