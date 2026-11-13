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
// PointXYZI is not precompiled (only PointXYZ, PointXYZRGB and PointXYZRGBA
// are), so the implementation header is needed to instantiate the codec for
// it; see also #4516.
#include <pcl/compression/impl/octree_pointcloud_compression.hpp>
#include <pcl/compression/compression_profiles.h>

#include <algorithm>
#include <exception>
#include <vector>

int total_runs = 0;

char* pcd_file;

#define MAX_POINTS 10000.0
#define MAX_COLOR 255
#define NUMBER_OF_TEST_RUNS 3

template<typename PointT> inline PointT generateRandomPoint(const float MAX_XYZ);

template<> inline pcl::PointXYZRGBA generateRandomPoint(const float MAX_XYZ) {
  return {static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                           static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                           static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                           static_cast<std::uint8_t> (MAX_COLOR * rand() / RAND_MAX),
                           static_cast<std::uint8_t> (MAX_COLOR * rand() / RAND_MAX),
                           static_cast<std::uint8_t> (MAX_COLOR * rand() / RAND_MAX),
                           static_cast<std::uint8_t> (MAX_COLOR * rand() / RAND_MAX)};
}

template<> inline pcl::PointXYZ generateRandomPoint(const float MAX_XYZ) {
  return {static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                       static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                       static_cast<float> (MAX_XYZ * rand() / RAND_MAX)};
}

template<> inline pcl::PointXYZI generateRandomPoint(const float MAX_XYZ) {
  return {static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                       static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                       static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                       static_cast<float> (50000.0 * rand() / RAND_MAX)};
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

using TestTypes = ::testing::Types<pcl::PointXYZ, pcl::PointXYZRGBA, pcl::PointXYZI>;
TYPED_TEST_SUITE(OctreeDeCompressionTest, TestTypes);

TYPED_TEST (OctreeDeCompressionTest, RandomClouds)
{
  srand(static_cast<unsigned int> (time(nullptr)));
  for (const double MAX_XYZ : {1.0, 1024.0}) { // Small clouds, large clouds
    // iterate over all pre-defined compression profiles
    for (int compression_profile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
      compression_profile != pcl::io::COMPRESSION_PROFILE_COUNT; ++compression_profile) {
      // instantiate point cloud compression encoder/decoder
      pcl::io::OctreePointCloudCompression<TypeParam> pointcloud_encoder(static_cast<pcl::io::compression_Profiles_e>(compression_profile), false);
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
  constexpr double MAX_XYZ = 1.0;
  srand(static_cast<unsigned int> (time(nullptr)));
  // iterate over all pre-defined compression profiles
  for (int compression_profile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    compression_profile != pcl::io::COMPRESSION_PROFILE_COUNT; ++compression_profile) {
    // instantiate point cloud compression encoder/decoder
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> pointcloud_encoder(static_cast<pcl::io::compression_Profiles_e>(compression_profile), false);
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

TEST(PCL, OctreeDeCompressionPointXYZIIntensity)
{
  // Regression test for #6272: the intensity of pcl::PointXYZI points was
  // lost on decompression (decoded as 0). The intensity is encoded
  // losslessly, so the decoded intensities must match exactly.
  // The cloud uses the reporter-like coordinates and intensities from #6272;
  // the points are far enough apart to occupy distinct voxels in every
  // compression profile, so every decoded point carries its own intensity.
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> ());
  const float points[][4] = {
      {345678.33f, 7027198.37f, 0.37f, 29512.0f},
      {345678.65f, 7027188.83f, 0.45f, 29206.0f},
      {345679.22f, 7027188.36f, 0.31f, 34296.0f},
      {345698.41f, 7027190.22f, 0.36f, 45830.0f},
      {345699.26f, 7027193.52f, 0.43f, 35847.0f},
      {347698.84f, 7027198.09f, 0.38f, 28070.0f},
      {345768.38f, 7027197.18f, 0.32f, 28005.0f},
      {345987.83f, 7027198.26f, 0.34f, 31260.0f},
      {345984.69f, 7027195.44f, 0.34f, 34471.0f},
      {345678.90f, 7027190.00f, 0.40f, 0.5f},
      {345678.95f, 7027191.00f, 0.41f, 0.0f},
  };
  for (const auto& row : points) {
    pcl::PointXYZI p;
    p.x = row[0];
    p.y = row[1];
    p.z = row[2];
    p.intensity = row[3];
    cloud->push_back (p);
  }

  // iterate over all pre-defined compression profiles
  for (int compression_profile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
       compression_profile != pcl::io::COMPRESSION_PROFILE_COUNT; ++compression_profile) {
    // instantiate point cloud compression encoder/decoder
    pcl::io::OctreePointCloudCompression<pcl::PointXYZI> pointcloud_encoder (static_cast<pcl::io::compression_Profiles_e> (compression_profile), false);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZI> pointcloud_decoder;

    // encode the same cloud several times: the first frame is an I-frame,
    // the following frames are P-frames
    for (int test_idx = 0; test_idx < NUMBER_OF_TEST_RUNS; test_idx++, total_runs++)
    {
      std::stringstream compressed_data;
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZI> ());
      pointcloud_encoder.encodePointCloud (cloud, compressed_data);
      pointcloud_decoder.decodePointCloud (compressed_data, cloud_out);
      ASSERT_EQ (cloud_out->width, cloud->width) << "cloud width after encoding and decoding not the same. Profile: " << compression_profile;
      ASSERT_EQ (cloud_out->height, 1) << "cloud height after encoding and decoding should be 1 (as before). Profile: " << compression_profile;

      // match decoded points to input points by nearest xyz (the decode
      // order may differ from the input order) and compare intensities
      std::vector<bool> used (cloud_out->size (), false);
      for (std::size_t i = 0; i < cloud->size (); ++i) {
        std::size_t best = cloud_out->size ();
        float best_d2 = 1e30f;
        for (std::size_t j = 0; j < cloud_out->size (); ++j) {
          if (used[j])
            continue;
          const float dx = (*cloud)[i].x - (*cloud_out)[j].x;
          const float dy = (*cloud)[i].y - (*cloud_out)[j].y;
          const float dz = (*cloud)[i].z - (*cloud_out)[j].z;
          const float d2 = dx * dx + dy * dy + dz * dz;
          if (d2 < best_d2) {
            best_d2 = d2;
            best = j;
          }
        }
        ASSERT_LT (best, cloud_out->size ()) << "no decoded point found for input point " << i << ". Profile: " << compression_profile;
        ASSERT_LT (best_d2, 0.05f * 0.05f) << "decoded point too far from input point " << i << ". Profile: " << compression_profile;
        used[best] = true;
        EXPECT_EQ ((*cloud_out)[best].intensity, (*cloud)[i].intensity)
            << "intensity of point " << i << " not preserved. Profile: " << compression_profile << ", run: " << test_idx;
      }
    } // runs
  } // compression profiles
} // TEST

TEST(PCL, OctreeDeCompressionPointXYZIIntensityDenseCloud)
{
  // Regression test for #6272, dense cloud: many points share voxels, so
  // the differential (XOR) intensity coding path is exercised. The intensity
  // is encoded losslessly, so the multiset of decoded intensities must match
  // the input multiset exactly. Voxel-grid downsampling profiles merge
  // points within a voxel and are covered by OctreeDeCompressionPointXYZIIntensity.
  srand (42);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> ());
  for (int i = 0; i < 2000; ++i) {
    pcl::PointXYZI p;
    p.x = static_cast<float> (0.1 * rand () / RAND_MAX);
    p.y = static_cast<float> (0.1 * rand () / RAND_MAX);
    p.z = static_cast<float> (0.1 * rand () / RAND_MAX);
    p.intensity = static_cast<float> (50000.0 * rand () / RAND_MAX);
    cloud->push_back (p);
  }
  std::vector<float> intensities_in;
  for (const auto& p : *cloud)
    intensities_in.push_back (p.intensity);
  std::sort (intensities_in.begin (), intensities_in.end ());

  // iterate over all pre-defined compression profiles without voxel grid
  // downsampling
  for (int compression_profile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
       compression_profile != pcl::io::COMPRESSION_PROFILE_COUNT; ++compression_profile) {
    if (pcl::io::compressionProfiles_[compression_profile].doVoxelGridDownSampling)
      continue;
    // instantiate point cloud compression encoder/decoder
    pcl::io::OctreePointCloudCompression<pcl::PointXYZI> pointcloud_encoder (static_cast<pcl::io::compression_Profiles_e> (compression_profile), false);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZI> pointcloud_decoder;

    // encode the same cloud several times: the first frame is an I-frame,
    // the following frames are P-frames
    for (int test_idx = 0; test_idx < NUMBER_OF_TEST_RUNS; test_idx++, total_runs++)
    {
      std::stringstream compressed_data;
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZI> ());
      pointcloud_encoder.encodePointCloud (cloud, compressed_data);
      pointcloud_decoder.decodePointCloud (compressed_data, cloud_out);
      ASSERT_EQ (cloud_out->width, cloud->width) << "cloud width after encoding and decoding not the same. Profile: " << compression_profile;
      std::vector<float> intensities_out;
      for (const auto& p : *cloud_out)
        intensities_out.push_back (p.intensity);
      std::sort (intensities_out.begin (), intensities_out.end ());
      ASSERT_EQ (intensities_out.size (), intensities_in.size ());
      for (std::size_t i = 0; i < intensities_in.size (); ++i)
        EXPECT_EQ (intensities_out[i], intensities_in[i])
            << "intensity mismatch at sorted position " << i << ". Profile: " << compression_profile << ", run: " << test_idx;
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
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> PointCloudEncoder(static_cast<pcl::io::compression_Profiles_e>(compression_profile), false);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> PointCloudDecoder;

    // iterate over various voxel sizes
    for (const float& voxel_size : voxel_sizes) {
      pcl::octree::OctreePointCloud<pcl::PointXYZRGB> octree(voxel_size);
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
