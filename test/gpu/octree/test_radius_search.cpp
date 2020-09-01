/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <numeric>

#if defined _MSC_VER
#pragma warning(disable : 4521)
#endif

#include <pcl/point_cloud.h>

#if defined _MSC_VER
#pragma warning(default : 4521)
#endif

#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/octree/octree.hpp>

#include "data_source.hpp"

using namespace pcl::gpu;

void
verifyResults(std::vector<std::vector<int>>& host_search,
              std::vector<std::vector<float>>& host_sqr_distances,
              const NeighborIndices& device_results,
              const Octree::ResultSqrDists& device_sqr_distances,
              const std::vector<float>& radiuses,
              const std::size_t max_answers,
              const std::size_t query_size,
              const int step)
{
  // download results
  std::vector<int> sizes;
  device_results.sizes.download(sizes);

  std::vector<int> downloaded_buffer;
  device_results.data.download(downloaded_buffer);

  std::vector<float> downloaded_sqr_distances;
  device_sqr_distances.download(downloaded_sqr_distances);

  // verification
  for (std::size_t i = 0; i < query_size; i += step) {
    std::vector<int>& results_host = host_search[i];
    std::vector<float>& sqr_dist_host = host_sqr_distances[i];

    const int beg = i / step * max_answers;
    const int end = beg + sizes[i / step];

    const auto beg_dist2 = downloaded_sqr_distances.cbegin() + beg;
    const auto end_dist2 = downloaded_sqr_distances.cbegin() + end;

    std::vector<int> results_batch(downloaded_buffer.cbegin() + beg,
                                   downloaded_buffer.cbegin() + end);
    const std::vector<float> sqr_distances_batch(beg_dist2, end_dist2);

    std::sort(results_batch.begin(), results_batch.end());
    std::sort(results_host.begin(), results_host.end());

    if (results_batch.size() == max_answers &&
        results_batch.size() < results_host.size() && max_answers) {
      results_host.resize(max_answers);
      sqr_dist_host.resize(max_answers);
    }
    const float sqr_radius = radiuses[i] * radiuses[i];
    for (std::size_t j = 0; j < sqr_dist_host.size(); j++) {
      EXPECT_LT(sqr_distances_batch[j], sqr_radius);
      EXPECT_NEAR(sqr_distances_batch[j], sqr_dist_host[j], 0.001);
    }
    EXPECT_EQ(results_batch, results_host);
  }
}

class PCL_OctreeGPUTest : public ::testing::Test {

protected:
  void
  SetUp() override
  {
    data.data_size = 871000;
    data.tests_num = 10000;
    data.cube_size = 1024.f;
    data.max_radius = data.cube_size / 30.f;
    data.shared_radius = data.cube_size / 30.f;
    data.printParams();

    // generate
    data();

    // prepare gpu cloud
    cloud_device.upload(data.points);

    // gpu build
    octree_device.setCloud(cloud_device);
    octree_device.build();

    // upload queries
    queries_device.upload(data.queries);
    radiuses_device.upload(data.radiuses);

    result_device.create(queries_device.size(), max_answers);

    host_search_shared.resize(data.tests_num);
    host_sqr_distances_shared.resize(data.tests_num);
    host_search_individual.resize(data.tests_num);
    host_sqr_distances_individual.resize(data.tests_num);

    // host search
    octree_device.internalDownload();

    for (std::size_t i = 0; i < data.tests_num; ++i)
      octree_device.radiusSearchHost(data.queries[i],
                                    data.shared_radius,
                                    host_search_shared[i],
                                    host_sqr_distances_shared[i],
                                    max_answers);

    for (std::size_t i = 0; i < data.tests_num; ++i)
      octree_device.radiusSearchHost(data.queries[i],
                                    data.radiuses[i],
                                    host_search_individual[i],
                                    host_sqr_distances_individual[i],
                                    max_answers);
  }

  DataGenerator data;
  const int max_answers = 333;
  pcl::gpu::Octree::PointCloud cloud_device;
  pcl::gpu::Octree octree_device;
  pcl::gpu::Octree::Queries queries_device;
  pcl::gpu::Octree::Radiuses radiuses_device;
  pcl::gpu::Octree::ResultSqrDists result_sqr_distances;
  pcl::gpu::NeighborIndices result_device;

  // prepare output buffers on host
  std::vector<std::vector<int>> host_search_shared;
  std::vector<std::vector<float>> host_sqr_distances_shared;
  std::vector<std::vector<int>> host_search_individual;
  std::vector<std::vector<float>> host_sqr_distances_individual;
};

TEST_F(PCL_OctreeGPUTest, shared_radius)
{
  octree_device.radiusSearch(queries_device,
                             data.shared_radius,
                             max_answers,
                             result_device,
                             result_sqr_distances);

  // verify results
  const std::vector<float> radiuses(data.tests_num, data.shared_radius);
  verifyResults(host_search_shared,
                host_sqr_distances_shared,
                result_device,
                result_sqr_distances,
                radiuses,
                max_answers,
                data.tests_num,
                1);
}

TEST_F(PCL_OctreeGPUTest, individual_radius)
{
  octree_device.radiusSearch(queries_device,
                             radiuses_device,
                             max_answers,
                             result_device,
                             result_sqr_distances);

  // verify results
  verifyResults(host_search_individual,
                host_sqr_distances_individual,
                result_device,
                result_sqr_distances,
                data.radiuses,
                max_answers,
                data.tests_num,
                1);
}

TEST_F(PCL_OctreeGPUTest, shared_radius_indices)
{
  pcl::gpu::Octree::Indices indices;
  indices.upload(data.indices);
  octree_device.radiusSearch(queries_device,
                             indices,
                             data.shared_radius,
                             max_answers,
                             result_device,
                             result_sqr_distances);

  // verify results
  const std::vector<float> radiuses(data.tests_num, data.shared_radius);
  verifyResults(host_search_shared,
                host_sqr_distances_shared,
                result_device,
                result_sqr_distances,
                radiuses,
                max_answers,
                data.tests_num,
                2);
}

/* ---[ */
int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */
