#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h> // for PCDReader

#include <benchmark/benchmark.h>

static void
BM_RadiusOutlierRemoval(benchmark::State& state, const std::string& file)
{
  // Perform setup here
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read(file, *cloud);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
  ror.setInputCloud(cloud);
  ror.setRadiusSearch(0.02);
  ror.setMinNeighborsInRadius(14);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelized(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (auto _ : state) {
    // This code gets timed
    ror.filter(*cloud_voxelized);
  }
}

static void
BM_RadiusOutlierRemovalOpenMP(benchmark::State& state, const std::string& file)
{
  // Perform setup here
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read(file, *cloud);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
  ror.setInputCloud(cloud);
  ror.setRadiusSearch(0.02);
  ror.setMinNeighborsInRadius(14);
  ror.setNumberOfThreads(0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelized(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (auto _ : state) {
    // This code gets timed
    ror.filter(*cloud_voxelized);
  }
}

int
main(int argc, char** argv)
{
  constexpr int runs = 100;

  if (argc < 3) {
    std::cerr
        << "No test files given. Please download `table_scene_mug_stereo_textured.pcd` "
           "and `milk_cartoon_all_small_clorox.pcd`, and pass their paths to the test."
        << std::endl;
    return (-1);
  }

  benchmark::RegisterBenchmark(
      "BM_RadiusOutlierRemoval_milk", &BM_RadiusOutlierRemoval, argv[2])
      ->Unit(benchmark::kMillisecond)
      ->Iterations(runs);

  benchmark::RegisterBenchmark(
      "BM_RadiusOutlierRemoval_mug", &BM_RadiusOutlierRemoval, argv[1])
      ->Unit(benchmark::kMillisecond)
      ->Iterations(runs);

  benchmark::RegisterBenchmark(
      "BM_RadiusOutlierRemovalOpenMP_milk", &BM_RadiusOutlierRemovalOpenMP, argv[2])
      ->Unit(benchmark::kMillisecond)
      ->Iterations(runs);

  benchmark::RegisterBenchmark(
      "BM_RadiusOutlierRemovalOpenMP_mug", &BM_RadiusOutlierRemovalOpenMP, argv[1])
      ->Unit(benchmark::kMillisecond)
      ->Iterations(runs);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
