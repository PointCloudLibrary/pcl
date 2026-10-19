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
#include <pcl/common/io.h> // for getFieldIndex
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

TEST (PCL, PointCoding_decodePoints_Not_Enough_Data)
{
  pcl::octree::PointCoding<pcl::PointXYZ> point_coder;
  // differential information of a single point
  point_coder.getDifferentialDataVector () = {1, 2, 3};
  point_coder.initializeDecoding ();

  const pcl::PointXYZ initial_point (-1.0f, -1.0f, -1.0f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> (2, 1, initial_point));
  const double reference_point[3] = {0.0, 0.0, 0.0};

  // two points need more differential information than there is: no point is decoded
  point_coder.decodePoints (cloud, reference_point, 0, 2);
  for (const auto& point : *cloud)
  {
    EXPECT_EQ (point.x, initial_point.x);
    EXPECT_EQ (point.y, initial_point.y);
    EXPECT_EQ (point.z, initial_point.z);
  }

  // a single point is decoded from the information left untouched by the call above
  point_coder.decodePoints (cloud, reference_point, 0, 1);
  EXPECT_FLOAT_EQ ((*cloud)[0].x, 1.5f * point_coder.getPrecision ());
  EXPECT_FLOAT_EQ ((*cloud)[0].y, 2.5f * point_coder.getPrecision ());
  EXPECT_FLOAT_EQ ((*cloud)[0].z, 3.5f * point_coder.getPrecision ());
  EXPECT_EQ ((*cloud)[1].x, initial_point.x);
}

TEST (PCL, ColorCoding_decodePoints_Not_Enough_Data)
{
  std::vector<pcl::PCLPointField> fields;
  const int rgba_index = pcl::getFieldIndex<pcl::PointXYZRGBA> ("rgba", fields);
  ASSERT_GE (rgba_index, 0);
  const auto rgba_offset = static_cast<unsigned char> (fields[rgba_index].offset);

  pcl::octree::ColorCoding<pcl::PointXYZRGBA> color_coder;
  // averaged color of one voxel and differential color of a single point
  color_coder.getAverageDataVector () = {10, 20, 30};
  color_coder.getDifferentialDataVector () = {1, 2, 3};
  color_coder.initializeDecoding ();

  const pcl::PointXYZRGBA initial_point (0.0f, 0.0f, 0.0f, 255, 255, 255, 255);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA> (2, 1, initial_point));

  // a voxel with two points needs differential color information for both: no color is decoded
  color_coder.decodePoints (cloud, 0, 2, rgba_offset);
  for (const auto& point : *cloud)
    EXPECT_EQ (point.rgba, initial_point.rgba);

  // a voxel with a single point only needs the averaged color
  color_coder.decodePoints (cloud, 0, 1, rgba_offset);
  EXPECT_EQ ((*cloud)[0].rgba, 10u | (20u << 8) | (30u << 16));

  // there is no averaged color for a further voxel
  color_coder.decodePoints (cloud, 1, 2, rgba_offset);
  EXPECT_EQ ((*cloud)[1].rgba, initial_point.rgba);
}

// Encoder that leaves out the point count of the first leaf, so that the stream holds
// one point count less than the octree has leaves.
class MissingPointCountEncoder : public pcl::io::OctreePointCloudCompression<pcl::PointXYZ>
{
  public:
    MissingPointCountEncoder ()
    : pcl::io::OctreePointCloudCompression<pcl::PointXYZ> (pcl::io::MANUAL_CONFIGURATION, false, 0.001, 0.01, false, 30, false)
    {}

  protected:
    void
    serializeTreeCallback (pcl::octree::OctreeContainerPointIndices& leaf_arg, const pcl::octree::OctreeKey& key_arg) override
    {
      pcl::io::OctreePointCloudCompression<pcl::PointXYZ>::serializeTreeCallback (leaf_arg, key_arg);
      if (first_leaf_)
      {
        point_count_data_vector_.pop_back ();
        first_leaf_ = false;
      }
    }

    bool first_leaf_ = true;
};

TEST (PCL, OctreeDeCompression_deserializeTreeCallback_Missing_Point_Count)
{
  // one point in each of ten leaves
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 10; ++i)
    cloud->push_back (pcl::PointXYZ (0.1f * static_cast<float> (i), 0.0f, 0.0f));

  MissingPointCountEncoder pointcloud_encoder;
  std::stringstream compressed_data;
  pointcloud_encoder.encodePointCloud (cloud, compressed_data);

  // the point counts run out at the last leaf, which is not decoded
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> pointcloud_decoder;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pointcloud_decoder.decodePointCloud (compressed_data, cloud_out);
  EXPECT_EQ (cloud_out->size (), cloud->size () - 1);
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
