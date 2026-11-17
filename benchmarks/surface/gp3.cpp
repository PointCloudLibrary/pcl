#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <benchmark/benchmark.h>

#include <chrono>

void
print_help()
{
  std::cout << "Usage: benchmark_gp3 <pcd filename>\n";
}

static void
BM_gp3(benchmark::State& state, const pcl::PointCloud<pcl::PointNormal>::Ptr cloudIn)
{
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  gp3.setSearchRadius(0.025);
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMinimumAngle(M_PI / 18);    // 10 degrees
  gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
  gp3.setNormalConsistency(false);
  pcl::PolygonMesh triangles;
  gp3.setInputCloud(cloudIn);

  for (auto _ : state) {
    gp3.reconstruct(triangles);
  }
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

  pcl::PointCloud<pcl::PointNormal>::Ptr cloudIn(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PCDReader reader;
  reader.read(argv[1], *cloudIn);

  // Filter out nans
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudFiltered(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*cloudIn, *cloudFiltered, indices);

  benchmark::RegisterBenchmark("gp3", &BM_gp3, cloudFiltered)
      ->Unit(benchmark::kMillisecond);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
