/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Centrum Wiskunde Informatica.
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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/compression/compression_profiles.h>

#include <exception>

int total_runs = 0;

char* pcd_file;

#define MAX_POINTS 10000.0
#define MAX_COLOR 255
#define NUMBER_OF_TEST_RUNS 3

template<typename PointT> inline PointT generateRandomPoint(const float MAX_XYZ);

template<> inline pcl::PointXYZRGBA generateRandomPoint(const float MAX_XYZ) {
  return pcl::PointXYZRGBA(static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                           static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                           static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                           static_cast<int> (MAX_COLOR * rand() / RAND_MAX),
                           static_cast<int> (MAX_COLOR * rand() / RAND_MAX),
                           static_cast<int> (MAX_COLOR * rand() / RAND_MAX),
                           static_cast<int> (MAX_COLOR * rand() / RAND_MAX));
}

template<> inline pcl::PointXYZ generateRandomPoint(const float MAX_XYZ) {
  return pcl::PointXYZ(static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                       static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                       static_cast<float> (MAX_XYZ * rand() / RAND_MAX));
}

template<typename PointT> inline
typename pcl::PointCloud<PointT>::Ptr generateRandomCloud(const float MAX_XYZ) {
  // empty point cloud hangs decoder
  const unsigned int point_count = 1 + (MAX_POINTS - 1) * rand() / RAND_MAX;
  // create shared pointcloud instances
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  for (unsigned int point = 0; point < point_count; point++) {
    cloud->push_back(generateRandomPoint<PointT>(MAX_XYZ));
  }
  return cloud;
}

template<typename PointT>
class OctreeDeCompressionTest : public testing::Test {};

using TestTypes = ::testing::Types<pcl::PointXYZ, pcl::PointXYZRGBA>;
TYPED_TEST_SUITE(OctreeDeCompressionTest, TestTypes);

TYPED_TEST (OctreeDeCompressionTest, RandomClouds)
{
  srand(static_cast<unsigned int> (time(nullptr)));
  for (const double MAX_XYZ : {1.0, 1024.0}) { // Small clouds, large clouds
    // iterate over all pre-defined compression profiles
    for (int compression_profile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
      compression_profile != pcl::io::COMPRESSION_PROFILE_COUNT; ++compression_profile) {
      // instantiate point cloud compression encoder/decoder
      pcl::io::OctreePointCloudCompression<TypeParam> pointcloud_encoder((pcl::io::compression_Profiles_e) compression_profile, false);
      pcl::io::OctreePointCloudCompression<TypeParam> pointcloud_decoder;
      typename pcl::PointCloud<TypeParam>::Ptr cloud_out(new pcl::PointCloud<TypeParam>());
      // iterate over runs
      for (int test_idx = 0; test_idx < NUMBER_OF_TEST_RUNS; test_idx++, total_runs++)
      {
        auto cloud = generateRandomCloud<TypeParam>(MAX_XYZ);
        EXPECT_EQ(cloud->height, 1);

//          std::cout << "Run: " << total_runs << " compression profile:" << compression_profile << " point_count: " << point_count;
        std::stringstream compressed_data;
        pointcloud_encoder.encodePointCloud(cloud, compressed_data);
        pointcloud_decoder.decodePointCloud(compressed_data, cloud_out);
        if (pcl::io::compressionProfiles_[compression_profile].doVoxelGridDownSampling) {
          EXPECT_GT(cloud_out->width, 0);
          EXPECT_LE(cloud_out->width, cloud->width) << "cloud width after encoding and decoding greater than before. Profile: " << compression_profile;
        }
        else {
          EXPECT_EQ(cloud_out->width, cloud->width) << "cloud width after encoding and decoding not the same. Profile: " << compression_profile;
        }
        EXPECT_EQ(cloud_out->height, 1) << "cloud height after encoding and decoding should be 1 (as before). Profile: " << compression_profile;
      } // runs
    } // compression profiles
  } // small clouds, large clouds
} // TEST

TEST (PCL, OctreeDeCompressionRandomPointXYZRGBASameCloud)
{
  // Generate a random cloud. Put it into the encoder several times and make
  // sure that the decoded cloud has correct width and height each time.
  const double MAX_XYZ = 1.0;
  srand(static_cast<unsigned int> (time(nullptr)));
  // iterate over all pre-defined compression profiles
  for (int compression_profile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    compression_profile != pcl::io::COMPRESSION_PROFILE_COUNT; ++compression_profile) {
    // instantiate point cloud compression encoder/decoder
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> pointcloud_encoder((pcl::io::compression_Profiles_e) compression_profile, false);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> pointcloud_decoder;

    auto cloud = generateRandomCloud<pcl::PointXYZRGBA>(MAX_XYZ);
    EXPECT_EQ(cloud->height, 1);

    // iterate over runs
    for (int test_idx = 0; test_idx < NUMBER_OF_TEST_RUNS; test_idx++, total_runs++)
    {
//          std::cout << "Run: " << total_runs << " compression profile:" << compression_profile << " point_count: " << point_count;
      std::stringstream compressed_data;
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>());
      pointcloud_encoder.encodePointCloud(cloud, compressed_data);
      pointcloud_decoder.decodePointCloud(compressed_data, cloud_out);
      if (pcl::io::compressionProfiles_[compression_profile].doVoxelGridDownSampling) {
        EXPECT_GT(cloud_out->width, 0);
        EXPECT_LE(cloud_out->width, cloud->width) << "cloud width after encoding and decoding greater than before. Profile: " << compression_profile;
      }
      else {
        EXPECT_EQ(cloud_out->width, cloud->width) << "cloud width after encoding and decoding not the same. Profile: " << compression_profile;
      }
      EXPECT_EQ(cloud_out->height, 1) << "cloud height after encoding and decoding should be 1 (as before). Profile: " << compression_profile;
    } // runs
  } // compression profiles
} // TEST

TEST(PCL, OctreeDeCompressionFile)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  // load point cloud from file, when present
  if (pcd_file == nullptr) return;
  int rv = pcl::io::loadPCDFile(pcd_file, *input_cloud_ptr);
  float voxel_sizes[] = { 0.1, 0.01 };

  EXPECT_EQ(rv, 0) << " loadPCDFile " << pcd_file;
  EXPECT_GT(input_cloud_ptr->width , 0) << "invalid point cloud width from " << pcd_file;
  EXPECT_GT(input_cloud_ptr->height, 0) << "invalid point cloud height from " << pcd_file;

  // iterate over compression profiles
  for (int compression_profile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
       compression_profile != pcl::io::COMPRESSION_PROFILE_COUNT; ++compression_profile) {
    // instantiate point cloud compression encoder/decoder
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> PointCloudEncoder((pcl::io::compression_Profiles_e) compression_profile, false);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> PointCloudDecoder;

    // iterate over various voxel sizes
    for (std::size_t i = 0; i < sizeof(voxel_sizes)/sizeof(voxel_sizes[0]); i++) {
      pcl::octree::OctreePointCloud<pcl::PointXYZRGB> octree(voxel_sizes[i]);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
      octree.setInputCloud((*input_cloud_ptr).makeShared());
      octree.addPointsFromInputCloud();

      std::stringstream compressedData;
      PointCloudEncoder.encodePointCloud(octree.getInputCloud(), compressedData);
      PointCloudDecoder.decodePointCloud(compressedData, cloud_out);
      EXPECT_GT(cloud_out->width, 0) << "decoded PointCloud width <= 0";
      EXPECT_GT(cloud_out->height, 0) << " decoded PointCloud height <= 0 ";
      total_runs++;
    }
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  if (argc > 1) {
    pcd_file = argv[1];
  }
  return (RUN_ALL_TESTS ());
  std::cerr << "Finished " << total_runs << " runs." << std::endl;
}
/* ]--- */
