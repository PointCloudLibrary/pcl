/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *
 *  All rights reserved.
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

