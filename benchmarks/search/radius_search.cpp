#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <benchmark/benchmark.h>

#include <chrono>

static void
BM_OrganizedNeighborSearch(benchmark::State& state, const std::string& file)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read(file, *cloudIn);

  pcl::search::OrganizedNeighbor<pcl::PointXYZ> organizedNeighborSearch;
  organizedNeighborSearch.setInputCloud(cloudIn);

  double radiusSearchTime = 0;
  std::vector<int> indices(cloudIn->size()); // Fixed indices from 0 to cloud size
  std::iota(indices.begin(), indices.end(), 0);
  int radiusSearchIdx = 0;

  for (auto _ : state) {
    int searchIdx = indices[radiusSearchIdx++ % indices.size()];
    double searchRadius = 0.1; // or any fixed radius like 0.05

    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;

    auto start_time = std::chrono::high_resolution_clock::now();
    organizedNeighborSearch.radiusSearch(
        (*cloudIn)[searchIdx], searchRadius, k_indices, k_sqr_distances);
    auto end_time = std::chrono::high_resolution_clock::now();

    radiusSearchTime +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time)
            .count();
  }

  state.SetItemsProcessed(state.iterations());
  state.SetIterationTime(
      radiusSearchTime /
      (state.iterations() * indices.size())); // Normalize by total points processed
}

int
main(int argc, char** argv)
{
  if (argc < 2) {
    std::cerr << "No test file given. Please provide a PCD file for the benchmark."
              << std::endl;
    return -1;
  }

  benchmark::RegisterBenchmark(
      "BM_OrganizedNeighborSearch", &BM_OrganizedNeighborSearch, argv[1])
      ->Unit(benchmark::kMillisecond);
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
