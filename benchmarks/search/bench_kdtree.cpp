// SPDX-License-Identifier: BSD-3-Clause
// Benchmark: pcl::search::KdTree (FLANN) vs pcl::search::KdTreeNanoflann
// Covers: tree construction, kNN query, radius search
// Motivation: PCL 1.15.1 added nanoflann support (#6250); this benchmark
//             provides a regression guard and documents expected speedups.
//
// Usage:
//   ./search_kdtree_flann_vs_nanoflann                  # synthetic 100k pts
//   ./search_kdtree_flann_vs_nanoflann cloud.pcd        # real PCD file

#include <benchmark/benchmark.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/flann_search.h>

#ifdef PCL_HAVE_NANOFLANN
#include <pcl/search/kdtree_nanoflann.h>
#endif

#include <iostream>
#include <random>

// ---------------------------------------------------------------------------
// Global cloud — loaded once, shared across all benchmarks
// ---------------------------------------------------------------------------

static pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud;

static pcl::PointCloud<pcl::PointXYZ>::Ptr
makeCloud(int N, float range = 100.f, unsigned seed = 42)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(N);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> dist(-range, range);
  for (int i = 0; i < N; ++i)
    cloud->emplace_back(dist(rng), dist(rng), dist(rng));
  cloud->width  = N;
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

// ---------------------------------------------------------------------------
// FLANN KdTree — construction
// ---------------------------------------------------------------------------

static void BM_FlannKdTree_Build(benchmark::State& state)
{
  auto cloud = makeCloud(static_cast<int>(state.range(0)));
  for (auto _ : state) {
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    benchmark::DoNotOptimize(tree);
  }
  state.SetItemsProcessed(state.iterations() * state.range(0));
  state.SetLabel("FLANN build");
}
BENCHMARK(BM_FlannKdTree_Build)
  ->Arg(10000)->Arg(100000)->Arg(500000)
  ->Unit(benchmark::kMillisecond);

// ---------------------------------------------------------------------------
// FLANN KdTree — kNN query
// ---------------------------------------------------------------------------

static void BM_FlannKdTree_kNN(benchmark::State& state)
{
  const int k = static_cast<int>(state.range(0));

  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud(g_cloud);

  pcl::PointXYZ query(0.f, 0.f, 0.f);
  std::vector<int>   indices(k);
  std::vector<float> dists(k);

  for (auto _ : state)
    benchmark::DoNotOptimize(tree.nearestKSearch(query, k, indices, dists));

  state.SetItemsProcessed(state.iterations());
  state.SetLabel("FLANN kNN");
}
BENCHMARK(BM_FlannKdTree_kNN)
  ->Arg(1)->Arg(5)->Arg(20)->Arg(50)
  ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// FLANN KdTree — radius search
// ---------------------------------------------------------------------------

static void BM_FlannKdTree_Radius(benchmark::State& state)
{
  const float r = static_cast<float>(state.range(0));

  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud(g_cloud);

  pcl::PointXYZ query(0.f, 0.f, 0.f);
  std::vector<int>   indices;
  std::vector<float> dists;

  for (auto _ : state)
    benchmark::DoNotOptimize(tree.radiusSearch(query, r, indices, dists));

  state.SetItemsProcessed(state.iterations());
  state.SetLabel("FLANN radius");
}
BENCHMARK(BM_FlannKdTree_Radius)
  ->Arg(5)->Arg(10)->Arg(20)
  ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Nanoflann KdTree — same benchmarks
// ---------------------------------------------------------------------------

#ifdef PCL_HAVE_NANOFLANN

static void BM_NanoflannKdTree_Build(benchmark::State& state)
{
  auto cloud = makeCloud(static_cast<int>(state.range(0)));
  for (auto _ : state) {
    pcl::search::KdTreeNanoflann<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    benchmark::DoNotOptimize(tree);
  }
  state.SetItemsProcessed(state.iterations() * state.range(0));
  state.SetLabel("nanoflann build");
}
BENCHMARK(BM_NanoflannKdTree_Build)
  ->Arg(10000)->Arg(100000)->Arg(500000)
  ->Unit(benchmark::kMillisecond);

static void BM_NanoflannKdTree_kNN(benchmark::State& state)
{
  const int k = static_cast<int>(state.range(0));

  pcl::search::KdTreeNanoflann<pcl::PointXYZ> tree;
  tree.setInputCloud(g_cloud);

  pcl::PointXYZ query(0.f, 0.f, 0.f);
  std::vector<int>   indices(k);
  std::vector<float> dists(k);

  for (auto _ : state)
    benchmark::DoNotOptimize(tree.nearestKSearch(query, k, indices, dists));

  state.SetItemsProcessed(state.iterations());
  state.SetLabel("nanoflann kNN");
}
BENCHMARK(BM_NanoflannKdTree_kNN)
  ->Arg(1)->Arg(5)->Arg(20)->Arg(50)
  ->Unit(benchmark::kMicrosecond);

static void BM_NanoflannKdTree_Radius(benchmark::State& state)
{
  const float r = static_cast<float>(state.range(0));

  pcl::search::KdTreeNanoflann<pcl::PointXYZ> tree;
  tree.setInputCloud(g_cloud);

  pcl::PointXYZ query(0.f, 0.f, 0.f);
  std::vector<int>   indices;
  std::vector<float> dists;

  for (auto _ : state)
    benchmark::DoNotOptimize(tree.radiusSearch(query, r, indices, dists));

  state.SetItemsProcessed(state.iterations());
  state.SetLabel("nanoflann radius");
}
BENCHMARK(BM_NanoflannKdTree_Radius)
  ->Arg(5)->Arg(10)->Arg(20)
  ->Unit(benchmark::kMicrosecond);

#endif  // PCL_HAVE_NANOFLANN

// ---------------------------------------------------------------------------
// main — load cloud from argv[1] or fall back to synthetic 100k points
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  if (argc > 1) {
    g_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile(argv[1], *g_cloud) < 0) {
      std::cerr << "Failed to load " << argv[1] << " — using synthetic cloud\n";
      g_cloud = makeCloud(100000);
    } else {
      std::cout << "Loaded " << g_cloud->size() << " pts from " << argv[1] << "\n";
    }
  } else {
    g_cloud = makeCloud(100000);
  }

  ::benchmark::Initialize(&argc, argv);
  ::benchmark::RunSpecifiedBenchmarks();
  ::benchmark::Shutdown();
  return 0;
}