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

#include <cmath>
#include <random>

using namespace pcl;
PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ>);
const static int CLOUD_SIZE = 10;
const static int SAMPLE_SIZE = CLOUD_SIZE -1;
std::vector<float> x_values;

TEST (FarthestPointSampling, farthest_point_sampling)
{
  PointCloud<PointXYZ> cloud_out;
  FarthestPointSampling<PointXYZ> fps;
  fps.setInputCloud(cloud_in);
  
  //set a seed, and identify first sample point
  std::random_device rd;
  int random_seed = rd();
  fps.setSeed(random_seed);
  fps.setSample(1);
  fps.filter(cloud_out);
  float first_element = cloud_out.points[0].x;
  
  //identify index of first element
  std::vector<float>::iterator itr;
  itr = std::find(x_values.begin(), x_values.end(), first_element);
  int first_index = std::distance(x_values.begin(), itr);
  
  //resample cloud with the same seed
  fps.setSeed(random_seed);
  fps.setSample(SAMPLE_SIZE);
  fps.filter(cloud_out);

  //check get methods
  std::size_t sample_value = fps.getSample();
  int seed_value = fps.getSeed();

  //assert seed value and sample value and sample cloud size
  EXPECT_EQ(seed_value, random_seed);
  EXPECT_EQ(sample_value, SAMPLE_SIZE);
  EXPECT_EQ (cloud_out.points.size(),  SAMPLE_SIZE);

  //check if each element is in the correct order
  //by default, filtered indices should be sorted in order of distance
  int point_index, expected_index;
  for (int j = 1; j < SAMPLE_SIZE; j++)
  {
    itr = std::find(x_values.begin(), x_values.end(), cloud_out.points[j].x);
    point_index = std::distance(x_values.begin(), itr);

    if ((CLOUD_SIZE -j) == first_index)
      expected_index = 0;
    else
      expected_index = CLOUD_SIZE - j;

    EXPECT_EQ (point_index, expected_index);
  }
}

int 
main (int argc, char** argv)
{
  // Fill in the cloud data
  cloud_in->width    = CLOUD_SIZE;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);

  x_values.push_back(0);

  for (std::size_t i = 1; i < CLOUD_SIZE; ++i)
  {
    x_values.push_back(std::pow(3,i-1));
    cloud_in->points[i].x = x_values[i];
    cloud_in->points[i].y = 0;
    cloud_in->points[i].z = 0;
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

