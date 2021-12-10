/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/common/distances.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <array> // for std::array

TEST(PCL_OctreeGPU, approxNearesSearch)
{

  /*
  the test points create an octree with bounds (-1, -1, -1) and (1, 1, 1).
  point q, represents a query point
  ------------------------------------
  |                |                 |
  |                |                 |
  |                |                 |
  |                |                 |
  |                |                 |
  |----------------------------------|
  | x     | q      |                 |
  |       |        |                 |
  |-------|--------|                 |
  |       | y      |                 |
  |       |        |                 |
  ------------------------------------
  the final two point are positioned such that point 'x' is farther from query point 'q'
  than 'y', but the voxel containing 'x' is closer to  'q' than the voxel containing 'y'
  */

  const std::array<pcl::PointXYZ, 10> coords{
      pcl::PointXYZ{-1.f, -1.f, -1.f},
      pcl::PointXYZ{-1.f, -1.f, 1.f},
      pcl::PointXYZ{-1.f, 1.f, -1.f},
      pcl::PointXYZ{-1.f, 1.f, 1.f},
      pcl::PointXYZ{1.f, -1.f, -1.f},
      pcl::PointXYZ{1.f, -1.f, 1.f},
      pcl::PointXYZ{1.f, 1.f, -1.f},
      pcl::PointXYZ{1.f, 1.f, 1.f},
      pcl::PointXYZ{-0.9f, -0.2f, -0.75f},
      pcl::PointXYZ{-0.4f, -0.6f, -0.75f},
  };

  // While the GPU implementation has a fixed depth of 10 levels, octree depth in the
  // CPU implementation can vary based on the leaf size set by the user, which can
  // affect the results. Therefore results would only tally if depths match.
  //generate custom pointcloud
  constexpr pcl::index_t point_size = 1000 * coords.size();
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(point_size, 1);

  // copy chunks of 10 points at the same time
  for (auto it = cloud->begin(); it != cloud->cend(); it += coords.size())
    std::copy(coords.cbegin(), coords.cend(), it);

  const std::vector<pcl::PointXYZ> queries = {
      {-0.4, -0.2, -0.75}, // should be different across CPU and GPU if different
                           // traversal methods are used
      {-0.6, -0.2, -0.75}, // should be same across CPU and GPU
      {1.1, 1.1, 1.1},     // out of range query
  };

  // prepare device cloud
  pcl::gpu::Octree::PointCloud cloud_device;
  cloud_device.upload(cloud->points);

  // gpu build
  pcl::gpu::Octree octree_device;
  octree_device.setCloud(cloud_device);
  octree_device.build();

  // build host octree
  constexpr float host_octree_resolution = 0.05;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_host(
      host_octree_resolution);
  octree_host.setInputCloud(cloud);
  octree_host.addPointsFromInputCloud();

  // upload queries
  pcl::gpu::Octree::Queries queries_device;
  queries_device.upload(queries);

  // prepare output buffers on device
  pcl::gpu::NeighborIndices result_device(queries.size(), 1);
  pcl::Indices result_host_pcl(queries.size());
  std::vector<int> result_host_gpu(queries.size());
  std::vector<float> dists_pcl(queries.size());
  std::vector<float> dists_gpu(queries.size());
  pcl::gpu::Octree::ResultSqrDists dists_device;

  // search GPU shared
  octree_device.approxNearestSearch(queries_device, result_device, dists_device);
  std::vector<int> downloaded;
  std::vector<float> dists_device_downloaded;
  result_device.data.download(downloaded);
  dists_device.download(dists_device_downloaded);

  for (size_t i = 0; i < queries.size(); ++i) {
    octree_host.approxNearestSearch(queries[i], result_host_pcl[i], dists_pcl[i]);
    octree_device.approxNearestSearchHost(queries[i], result_host_gpu[i], dists_gpu[i]);
  }

  ASSERT_EQ(downloaded, result_host_gpu);

  const std::array<float, 3> expected_sqr_dists {pcl::squaredEuclideanDistance(coords[8], queries[0]),
                                                  pcl::squaredEuclideanDistance(coords[8], queries[1]),
                                                  pcl::squaredEuclideanDistance(coords[7], queries[2]) };

  for (size_t i = 0; i < queries.size(); ++i) {
    ASSERT_EQ(dists_pcl[i], dists_gpu[i]);
    ASSERT_NEAR(dists_gpu[i], dists_device_downloaded[i], 0.001);
    ASSERT_NEAR(dists_device_downloaded[i], expected_sqr_dists[i], 0.001);
  }
}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */
