#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/kdtree_nanoflann.h>
#include <pcl/search/organized.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <benchmark/benchmark.h>

#include <chrono>

void
print_help()
{
  std::cout << "Usage: benchmark_radius_search <pcd filename>\n";
  std::cout << "Additional argument options:\n";
  std::cout << "Usage: benchmark_radius_search <pcd filename> <radius search> "
               "<neighbor limit>\n";
}

static void
BM_OrganizedNeighborSearch(benchmark::State& state,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                           const double searchRadius,
                           const size_t neighborLimit)
{
  pcl::search::OrganizedNeighbor<pcl::PointXYZ> organizedNeighborSearch;
  organizedNeighborSearch.setInputCloud(cloudIn);

  int radiusSearchIdx = 0;

  pcl::Indices k_indices;
  std::vector<float> k_sqr_distances;
  for (auto _ : state) {
    state.PauseTiming();
    int searchIdx = radiusSearchIdx++ % cloudIn->size();
    state.ResumeTiming();
    organizedNeighborSearch.radiusSearch(
        (*cloudIn)[searchIdx], searchRadius, k_indices, k_sqr_distances, neighborLimit);
  }
}

static void
BM_KdTree(benchmark::State& state,
          const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
          const double searchRadius,
          const size_t neighborLimit)
{
  pcl::search::KdTree<pcl::PointXYZ> kdtree(false);
  kdtree.setInputCloud(cloudIn);

  int radiusSearchIdx = 0;

  pcl::Indices k_indices;
  std::vector<float> k_sqr_distances;
  for (auto _ : state) {
    state.PauseTiming();
    int searchIdx = radiusSearchIdx++ % cloudIn->size();
    state.ResumeTiming();
    kdtree.radiusSearch(
        searchIdx, searchRadius, k_indices, k_sqr_distances, neighborLimit);
  }
}

static void
BM_KdTreeAll(benchmark::State& state,
             const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
             const double searchRadius,
             const size_t neighborLimit)
{
  pcl::search::KdTree<pcl::PointXYZ> kdtree(false);
  kdtree.setInputCloud(cloudIn);

  // Leaving indices empty to have it search through all points
  pcl::Indices indices;
  std::vector<pcl::Indices> k_indices;
  std::vector<std::vector<float>> k_sqr_distances;
  for (auto _ : state) {
    auto start_time = std::chrono::high_resolution_clock::now();
    kdtree.radiusSearch(
        *cloudIn, indices, searchRadius, k_indices, k_sqr_distances, neighborLimit);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    state.SetIterationTime(elapsed.count() / cloudIn->size());
  }

  state.SetItemsProcessed(cloudIn->size());
}

#if PCL_HAS_NANOFLANN
static void
BM_KdTreeNanoflann(benchmark::State& state,
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                   const double searchRadius,
                   const size_t neighborLimit)
{
  pcl::search::KdTreeNanoflann<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloudIn);

  int radiusSearchIdx = 0;

  pcl::Indices k_indices;
  std::vector<float> k_sqr_distances;
  for (auto _ : state) {
    state.PauseTiming();
    int searchIdx = radiusSearchIdx++ % cloudIn->size();
    state.ResumeTiming();
    kdtree.radiusSearch(
        searchIdx, searchRadius, k_indices, k_sqr_distances, neighborLimit);
  }
}
#endif

int
main(int argc, char** argv)
{
  if (argc < 2) {
    std::cerr << "No test file given. Please provide a PCD file for the benchmark."
              << std::endl;
    print_help();
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read(argv[1], *cloudIn);

  double searchRadius = 0.1;
  if (argc > 2) {
    try {
      searchRadius = std::abs(std::stod(argv[2]));
    } catch (const std::invalid_argument&) {
      std::cerr << "Error: Invalid search radius. Setting it to a default of 0.1."
                << std::endl;
    }
  }

  size_t neighborLimit = 0u;
  if (argc > 3) {
    try {
      neighborLimit = std::stoul(argv[3]);
    } catch (const std::invalid_argument&) {
      std::cerr << "Error: Invalid neighbor limit. Setting it to a default of int max."
                << std::endl;
    }
  }

  benchmark::RegisterBenchmark("OrganizedNeighborSearch",
                               &BM_OrganizedNeighborSearch,
                               cloudIn,
                               searchRadius,
                               neighborLimit)
      ->Unit(benchmark::kMicrosecond);
  benchmark::RegisterBenchmark(
      "KdTree", &BM_KdTree, cloudIn, searchRadius, neighborLimit)
      ->Unit(benchmark::kMicrosecond);
  benchmark::RegisterBenchmark(
      "KdTreeAll", &BM_KdTreeAll, cloudIn, searchRadius, neighborLimit)
      ->Unit(benchmark::kMicrosecond)
      ->UseManualTime()
      ->Iterations(1);
#if PCL_HAS_NANOFLANN
  benchmark::RegisterBenchmark(
      "KdTreeNanoflann", &BM_KdTreeNanoflann, cloudIn, searchRadius, neighborLimit)
      ->Unit(benchmark::kMicrosecond);
#endif
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
