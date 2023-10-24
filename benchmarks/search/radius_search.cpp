#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/io/pcd_io.h>
#include <benchmark/benchmark.h>

static void BM_OrganizedNeighborSearch(benchmark::State& state, const std::string& file) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read(file, *cloudIn);

  pcl::search::OrganizedNeighbor<pcl::PointXYZ> organizedNeighborSearch;

  double radiusSearchTime = 0;

  for (auto _ : state) {
    organizedNeighborSearch.setInputCloud(cloudIn);

    int randomIdx = rand() % cloudIn->size();
    double searchRadius = 1.0 * ((double)rand() / (double)RAND_MAX);

    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;

    auto check_time = benchmark::Now();
    organizedNeighborSearch.radiusSearch((*cloudIn)[randomIdx], searchRadius, k_indices, k_sqr_distances);
    radiusSearchTime += benchmark::Now() - check_time;
  }

  state.SetItemsProcessed(state.iterations());
  state.SetIterationTime(radiusSearchTime / state.iterations());
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "No test file given. Please provide a PCD file for the benchmark." << std::endl;
    return -1;
  }

  benchmark::RegisterBenchmark("BM_OrganizedNeighborSearch", &BM_OrganizedNeighborSearch, argv[1])
      ->Unit(benchmark::kMillisecond);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
