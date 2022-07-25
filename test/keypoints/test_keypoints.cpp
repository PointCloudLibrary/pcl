/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Michael Dixon */

#include <pcl/test/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/keypoints/sift_keypoint.h>

#include <set>

using namespace pcl;
using namespace pcl::io;

struct KeypointT
{
  float x, y, z, scale;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (KeypointT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, scale, scale)
)

PointCloud<PointXYZI>::Ptr cloud_xyzi (new PointCloud<PointXYZI>);

//////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, SIFTKeypoint)
{
  PointCloud<KeypointT> keypoints;

  // Compute the SIFT keypoints
  SIFTKeypoint<PointXYZI, KeypointT> sift_detector;
	search::KdTree<PointXYZI>::Ptr tree (new search::KdTree<PointXYZI>);
  sift_detector.setSearchMethod (tree);
  sift_detector.setScales (0.02f, 5, 3);
  sift_detector.setMinimumContrast (0.03f);

  sift_detector.setInputCloud (cloud_xyzi);
  sift_detector.compute (keypoints);

  ASSERT_EQ (keypoints.width, keypoints.size ());
  ASSERT_EQ (keypoints.height, 1);
  EXPECT_EQ (keypoints.size (), static_cast<std::size_t> (169));
  EXPECT_EQ (keypoints.header, cloud_xyzi->header);
  EXPECT_EQ (keypoints.sensor_origin_ (0), cloud_xyzi->sensor_origin_ (0));
  EXPECT_EQ (keypoints.sensor_origin_ (1), cloud_xyzi->sensor_origin_ (1));
  EXPECT_EQ (keypoints.sensor_origin_ (2), cloud_xyzi->sensor_origin_ (2));
  EXPECT_EQ (keypoints.sensor_origin_ (3), cloud_xyzi->sensor_origin_ (3));
  EXPECT_EQ (keypoints.sensor_orientation_.w (), cloud_xyzi->sensor_orientation_.w ());
  EXPECT_EQ (keypoints.sensor_orientation_.x (), cloud_xyzi->sensor_orientation_.x ());
  EXPECT_EQ (keypoints.sensor_orientation_.y (), cloud_xyzi->sensor_orientation_.y ());
  EXPECT_EQ (keypoints.sensor_orientation_.z (), cloud_xyzi->sensor_orientation_.z ());

  // Change the values and re-compute
  sift_detector.setScales (0.05f, 5, 3);
  sift_detector.setMinimumContrast (0.06f);
  sift_detector.compute (keypoints);

  ASSERT_EQ (keypoints.width, keypoints.size ());
  ASSERT_EQ (keypoints.height, 1);

  // Compare to previously validated output
  const std::size_t correct_nr_keypoints = 5;
  const float correct_keypoints[correct_nr_keypoints][4] = 
    { 
      // { x,  y,  z,  scale }
      {-0.9425f, -0.6381f,  1.6445f,  0.0794f},
      {-0.5083f, -0.5587f,  1.8519f,  0.0500f},
      { 1.0265f,  0.0500f,  1.7154f,  0.1000f},
      { 0.3005f, -0.3007f,  1.9526f,  0.2000f},
      {-0.1002f, -0.1002f,  1.9933f,  0.3175f}
    };

  ASSERT_EQ (keypoints.size (), correct_nr_keypoints);
  for (std::size_t i = 0; i < correct_nr_keypoints; ++i)
  {
    EXPECT_NEAR (keypoints[i].x, correct_keypoints[i][0], 1e-4);
    EXPECT_NEAR (keypoints[i].y, correct_keypoints[i][1], 1e-4);
    EXPECT_NEAR (keypoints[i].z, correct_keypoints[i][2], 1e-4);
    EXPECT_NEAR (keypoints[i].scale, correct_keypoints[i][3], 1e-4);
  }

}

TEST (PCL, SIFTKeypoint_radiusSearch)
{
  const int nr_scales_per_octave = 3;
  const float scale = 0.02f;

  KdTreeFLANN<PointXYZI>::Ptr tree_ (new KdTreeFLANN<PointXYZI>);
  auto cloud = cloud_xyzi->makeShared ();

  ApproximateVoxelGrid<PointXYZI> voxel_grid;
  const float s = 1.0 * scale;
  voxel_grid.setLeafSize (s, s, s);
  voxel_grid.setInputCloud (cloud);
  voxel_grid.filter (*cloud);
  tree_->setInputCloud (cloud);
  
  const PointCloud<PointXYZI> & input = *cloud;
  KdTreeFLANN<PointXYZI> & tree = *tree_;
  const float base_scale = scale;

  std::vector<float> scales (nr_scales_per_octave + 3);
  for (int i_scale = 0; i_scale <= nr_scales_per_octave + 2; ++i_scale)
  {
    scales[i_scale] = base_scale * std::pow (2.0f, static_cast<float> (i_scale-1) / nr_scales_per_octave);
  }
  Eigen::MatrixXf diff_of_gauss;

  pcl::Indices nn_indices;
  std::vector<float> nn_dist;
  diff_of_gauss.resize (input.size (), scales.size () - 1);

  const float max_radius = 0.10f;

  const std::size_t i_point = 500;
  tree.radiusSearch (i_point, max_radius, nn_indices, nn_dist);

  // Are they all unique?
  std::set<pcl::index_t> unique_indices;
  for (const auto &nn_index : nn_indices)
  {
    unique_indices.insert (nn_index);
  }

  EXPECT_EQ (nn_indices.size (), unique_indices.size ());
}

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `cturtle.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  // Load a sample point cloud
  if (io::loadPCDFile (argv[1], *cloud_xyzi) < 0)
  {
    std::cerr << "Failed to read test file. Please download `cturtle.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
