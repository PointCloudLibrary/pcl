/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/common/generate.h> // CloudGenerator
#include <pcl/common/random.h>   // UniformGenerator
#include <pcl/gpu/octree/device_format.hpp> // gpu::NeighborIndices
#include <pcl/gpu/octree/octree.hpp>        // gpu::Octree
#include <pcl/point_cloud.h> // PointCloud
#include <pcl/point_types.h> // PointXYZ

#include <cmath> // NAN
#include <iostream> // cout
#include <vector> // vector

auto
kNearestSearch(const pcl::gpu::Octree& octree,
               const pcl::gpu::Octree::Queries& queries,
               const int k)
{
  // prepare output buffers on device
  pcl::gpu::NeighborIndices indices_d(queries.size(), k);
  pcl::gpu::Octree::ResultSqrDists sqr_dists_d;

  // search GPU
  octree.nearestKSearchBatch(queries, k, indices_d, sqr_dists_d);

  // download results
  std::vector<int> indices_h;
  std::vector<float> sqr_dists_h;
  indices_d.data.download(indices_h);
  sqr_dists_d.download(sqr_dists_h);

  return std::make_pair(indices_h, sqr_dists_h);
}

auto
radiusSearch(const pcl::gpu::Octree& octree,
             const pcl::gpu::Octree::Queries& queries,
             const std::vector<float>& radiuses_h)
{
  // prepare radiuses
  pcl::gpu::Octree::Radiuses radiuses_d;
  radiuses_d.upload(radiuses_h);

  // prepare output buffers on device
  const auto max_results = octree.cloud_->size();
  pcl::gpu::NeighborIndices indices_d(queries.size(), max_results);

  // search GPU
  // uncomment once radius search PR is merged
  // pcl::gpu::Octree::ResultSqrDists sqr_dists_d;
  octree.radiusSearch(queries, radiuses_d, max_results, indices_d /*, sqr_dists_d*/);

  // download results
  std::vector<int> indices_h;
  std::vector<int> sizes;
  indices_d.data.download(indices_h);
  // sqr_dists_d.download(sqr_dists_h);
  indices_d.sizes.download(sizes);

  // rewrite once radius search pr is merged
  const std::vector<float> sqr_dists_h(indices_h.size(), NAN);
  return std::make_tuple(indices_h, sizes, sqr_dists_h);
}

auto
approxNearestSearch(const pcl::gpu::Octree& octree,
                    const pcl::gpu::Octree::Queries& queries)
{
  // prepare output buffers on device
  pcl::gpu::NeighborIndices indices_d(queries.size(), 1);
  pcl::gpu::Octree::ResultSqrDists sqr_dists_d;

  // search GPU
  octree.approxNearestSearch(queries, indices_d, sqr_dists_d);

  // download results
  std::vector<int> indices_h;
  std::vector<float> sqr_dists_h;
  indices_d.data.download(indices_h);
  sqr_dists_d.download(sqr_dists_h);

  return std::make_pair(indices_h, sqr_dists_h);
}

int
main()
{
  using Generator =
      pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::UniformGenerator<float>>;
  Generator generator(Generator::GeneratorParameters(0, 1024.f)); // min max range

  pcl::PointCloud<pcl::PointXYZ> cloud_h;
  generator.fill(1000, 1, cloud_h);

  constexpr size_t query_size = 10;
  std::vector<pcl::PointXYZ> queries_h;
  for (std::size_t i = 0; i < query_size; ++i)
    queries_h.push_back(generator.get());

  // prepare device cloud
  pcl::gpu::Octree::PointCloud cloud_d;
  cloud_d.upload(cloud_h.data(), cloud_h.size());

  // upload queries
  pcl::gpu::Octree::Queries queries_d;
  queries_d.upload(queries_h);

  // gpu build
  pcl::gpu::Octree octree;
  octree.setCloud(cloud_d);
  octree.build();

  // perform approximate nearest search
  std::vector<int> indices;
  std::vector<float> sqr_dists;
  std::tie(indices, sqr_dists) = approxNearestSearch(octree, queries_d);

  for (std::size_t i = 0; i < query_size; ++i) {
    std::cout << "Approximate nearest neighbor search #" << i << " at " << queries_h[i]
              << "\n\t" << cloud_h[indices[i]] << " (squared distance " << sqr_dists[i]
              << ")\n";
  }
  std::cout << '\n';

  // perform k = 1 nearest search
  constexpr int K = 1;
  std::tie(indices, sqr_dists) = kNearestSearch(octree, queries_d, K);

  for (std::size_t i = 0; i < query_size; ++i) {
    std::cout << "K nearest neighbor search #" << i << " at " << queries_h[i] << '\n';

    for (std::size_t j = i * K; j < (i + 1) * K; ++j)
      std::cout << '\t' << cloud_h[indices[j]] << " (squared distance " << sqr_dists[j]
                << ")\n";
  }
  std::cout << '\n';

  // perform radius search
  pcl::common::UniformGenerator<float> radius_generator(75.f, 125.f);
  std::vector<float> radiuses;
  radiuses.reserve(query_size);
  for (std::size_t i = 0; i < queries_h.size(); ++i)
    radiuses.push_back(radius_generator.run());

  std::vector<int> sizes;
  std::tie(indices, sizes, sqr_dists) = radiusSearch(octree, queries_d, radiuses);

  for (std::size_t i = 0; i < query_size; ++i) {
    std::cout << "Radius search at " << queries_h[i] << " within radius " << radiuses[i]
              << ". Found " << sizes[i] << " results.\n";

    for (std::size_t j = 0; j < static_cast<std::size_t>(sizes[i]); ++j) {
      const auto idx = i * cloud_h.size() + j;
      std::cout << "\t" << cloud_h[indices[idx]] << " (squared distance "
                << sqr_dists[idx] << ")\n";
    }
  }
  std::cout << '\n';
}
