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
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/compression/compression_profiles.h>

#include <exception>

int total_runs = 0;

char* pcd_file;

#define MAX_POINTS 10000.0
#define MAX_XYZ 1024.0
#define MAX_COLOR 255
#define NUMBER_OF_TEST_RUNS 2

TEST (PCL, OctreeDeCompressionRandomPointXYZRGBA)
{
  srand(static_cast<unsigned int> (time(NULL)));

    // iterate over all pre-defined compression profiles
  for (int compression_profile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    compression_profile != pcl::io::COMPRESSION_PROFILE_COUNT; ++compression_profile) {
    // instantiate point cloud compression encoder/decoder
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> pointcloud_encoder((pcl::io::compression_Profiles_e) compression_profile, false);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> pointcloud_decoder;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>());
    // iterate over runs
    for (int test_idx = 0; test_idx < NUMBER_OF_TEST_RUNS; test_idx++, total_runs++)
    {
      try
      {
        int point_count = MAX_POINTS * rand() / RAND_MAX;
        if (point_count < 1)
        { // empty point cloud hangs decoder
          total_runs--;
          continue;
        }
        // create shared pointcloud instances
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        // assign input point clouds to octree
        // create random point cloud
        for (int point = 0; point < point_count; point++)
        {
          // gereate a random point
          pcl::PointXYZRGBA new_point;
          new_point.x = static_cast<float> (MAX_XYZ * rand() / RAND_MAX);
          new_point.y = static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
          new_point.z = static_cast<float> (MAX_XYZ * rand() / RAND_MAX);
          new_point.r = static_cast<int> (MAX_COLOR * rand() / RAND_MAX);
          new_point.g = static_cast<int> (MAX_COLOR * rand() / RAND_MAX);
          new_point.b = static_cast<int> (MAX_COLOR * rand() / RAND_MAX);
          new_point.a = static_cast<int> (MAX_COLOR * rand() / RAND_MAX);
          // OctreePointCloudPointVector can store all points..
          cloud->push_back(new_point);
        }

//        std::cout << "Run: " << total_runs << " compression profile:" << compression_profile << " point_count: " << point_count;
        std::stringstream compressed_data;
        pointcloud_encoder.encodePointCloud(cloud, compressed_data);
        pointcloud_decoder.decodePointCloud(compressed_data, cloud_out);
        EXPECT_GT((int)cloud_out->width, 0) << "decoded PointCloud width <= 0";
        EXPECT_GT((int)cloud_out->height, 0) << " decoded PointCloud height <= 0 ";
      }
      catch (std::exception& e)
      {
        std::cout << e.what() << std::endl;
      }
    } // runs
  } // compression profiles
} // TEST

TEST (PCL, OctreeDeCompressionRandomPointXYZ)
{
  srand(static_cast<unsigned int> (time(NULL)));

  // iterate over all pre-defined compression profiles
  for (int compression_profile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
        compression_profile != pcl::io::COMPRESSION_PROFILE_COUNT; ++compression_profile)
  {
    // instantiate point cloud compression encoder/decoder
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> pointcloud_encoder((pcl::io::compression_Profiles_e) compression_profile, false);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> pointcloud_decoder;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    // loop over runs
    for (int test_idx = 0; test_idx < NUMBER_OF_TEST_RUNS; test_idx++, total_runs++)
    {
      int point_count = MAX_POINTS * rand() / RAND_MAX;
      // create shared pointcloud instances
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      // assign input point clouds to octree
      // create random point cloud
      for (int point = 0; point < point_count; point++)
      {
        // generate a random point
        pcl::PointXYZ new_point(static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                               static_cast<float> (MAX_XYZ * rand() / RAND_MAX),
                               static_cast<float> (MAX_XYZ * rand() / RAND_MAX));
        cloud->push_back(new_point);
      }
//      std::cout << "Run: " << total_runs << " compression profile:" << compression_profile << " point_count: " << point_count;
      std::stringstream compressed_data;
      try
      { // decodePointCloud() throws exceptions on errors
        pointcloud_encoder.encodePointCloud(cloud, compressed_data);
        pointcloud_decoder.decodePointCloud(compressed_data, cloud_out);
        EXPECT_GT((int)cloud_out->width, 0) << "decoded PointCloud width <= 0";
        EXPECT_GT((int)cloud_out->height, 0) << " decoded PointCloud height <= 0 ";
      }
      catch (std::exception& e)
      {
        std::cout << e.what() << std::endl;
      }
    } // runs
  } // compression profiles
} // TEST

TEST(PCL, OctreeDeCompressionFile)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  // load point cloud from file, when present
  if (pcd_file == NULL) return;
    int rv = pcl::io::loadPCDFile(pcd_file, *input_cloud_ptr);
    float voxel_sizes[] = { 0.1, 0.01 };

    EXPECT_EQ(rv, 0) << " loadPCDFile " << pcd_file;
    EXPECT_GT((int) input_cloud_ptr->width , 0) << "invalid point cloud width from " << pcd_file;
    EXPECT_GT((int) input_cloud_ptr->height, 0) << "invalid point cloud height from " << pcd_file;

    // iterate over compression profiles
    for (int compression_profile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
         compression_profile != pcl::io::COMPRESSION_PROFILE_COUNT; ++compression_profile) {
    // instantiate point cloud compression encoder/decoder
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>((pcl::io::compression_Profiles_e) compression_profile, false);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>();

    // iterate over various voxel sizes
    for (std::size_t i = 0; i < sizeof(voxel_sizes)/sizeof(voxel_sizes[0]); i++) {
      pcl::octree::OctreePointCloud<pcl::PointXYZRGB> octree(voxel_sizes[i]);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr octree_out(new pcl::PointCloud<pcl::PointXYZRGB>());
      octree.setInputCloud((*input_cloud_ptr).makeShared());
      octree.addPointsFromInputCloud();

      std::stringstream compressedData;
      PointCloudEncoder->encodePointCloud(octree.getInputCloud(), compressedData);
      PointCloudDecoder->decodePointCloud(compressedData, octree_out);
      EXPECT_GT((int)octree_out->width, 0) << "decoded PointCloud width <= 0";
      EXPECT_GT((int)octree_out->height, 0) << " decoded PointCloud height <= 0 ";
      total_runs++;
    }
    delete PointCloudDecoder;
    delete PointCloudEncoder;
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
