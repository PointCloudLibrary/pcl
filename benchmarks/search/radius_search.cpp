#include <pcl/filters/filter.h>
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
BM_RadiusSearch(benchmark::State& state,
                const pcl::search::Search<pcl::PointXYZ>& search,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                const double searchRadius,
                const size_t neighborLimit)
{
  int radiusSearchIdx = 0;

  pcl::Indices k_indices;
  std::vector<float> k_sqr_distances;
  for (auto _ : state) {
    state.PauseTiming();
    int searchIdx = radiusSearchIdx++ % cloudIn->size();
    while (!pcl::isFinite((*cloudIn)[searchIdx])) {
      searchIdx = radiusSearchIdx++ % cloudIn->size();
    }
    state.ResumeTiming();
    search.radiusSearch(
        (*cloudIn)[searchIdx], searchRadius, k_indices, k_sqr_distances, neighborLimit);
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

  // Filter out nans
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*cloudIn, *cloudFiltered, indices);

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

  pcl::search::OrganizedNeighbor<pcl::PointXYZ> organized_neighbor;
  organized_neighbor.setInputCloud(cloudIn);
  benchmark::RegisterBenchmark("OrganizedNeighborSearch",
                               &BM_RadiusSearch,
                               organized_neighbor,
                               cloudIn,
                               searchRadius,
                               neighborLimit)
      ->Unit(benchmark::kMicrosecond);

  pcl::search::KdTree<pcl::PointXYZ> kdtree(false);
  kdtree.setInputCloud(cloudIn);
  benchmark::RegisterBenchmark(
      "KdTree", &BM_RadiusSearch, kdtree, cloudIn, searchRadius, neighborLimit)
      ->Unit(benchmark::kMicrosecond);

  benchmark::RegisterBenchmark(
      "KdTreeAll", &BM_KdTreeAll, cloudFiltered, searchRadius, neighborLimit)
      ->Unit(benchmark::kMicrosecond)
      ->UseManualTime()
      ->Iterations(1);

#if PCL_HAS_NANOFLANN
  pcl::search::KdTreeNanoflann<pcl::PointXYZ> kdtreeNanoflann;
  kdtreeNanoflann.setInputCloud(cloudIn);
  benchmark::RegisterBenchmark("KdTreeNanoflann",
                               &BM_RadiusSearch,
                               kdtreeNanoflann,
                               cloudIn,
                               searchRadius,
                               neighborLimit)
      ->Unit(benchmark::kMicrosecond);
#endif

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
