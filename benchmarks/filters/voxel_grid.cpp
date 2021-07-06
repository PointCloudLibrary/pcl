#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h> // for PCDReader

#include <benchmark/benchmark.h>

static void
BM_VoxelGrid(benchmark::State& state, const std::string& file)
{
  // Perform setup here
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read(file, *cloud);

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(0.01, 0.01, 0.01);
  vg.setInputCloud(cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelized(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (auto _ : state) {
    // This code gets timed
    vg.filter(*cloud_voxelized);
  }
}

static void
BM_ApproxVoxelGrid(benchmark::State& state, const std::string& file)
{
  // Perform setup here
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read(file, *cloud);

  pcl::ApproximateVoxelGrid<pcl::PointXYZ> avg;
  avg.setLeafSize(0.01, 0.01, 0.01);
  avg.setInputCloud(cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelized(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (auto _ : state) {
    // This code gets timed
    avg.filter(*cloud_voxelized);
  }
}

int
main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr
        << "No test files given. Please download `table_scene_mug_stereo_textured.pcd` "
           "and `milk_cartoon_all_small_clorox.pcd`, and pass their paths to the test."
        << std::endl;
    return (-1);
  }

  benchmark::RegisterBenchmark("BM_VoxelGrid_milk", &BM_VoxelGrid, argv[2])
      ->Unit(benchmark::kMillisecond);
  benchmark::RegisterBenchmark(
      "BM_ApproximateVoxelGrid_milk", &BM_ApproxVoxelGrid, argv[2])
      ->Unit(benchmark::kMillisecond);

  benchmark::RegisterBenchmark("BM_VoxelGrid_mug", &BM_VoxelGrid, argv[1])
      ->Unit(benchmark::kMillisecond);
  benchmark::RegisterBenchmark(
      "BM_ApproximateVoxelGrid_mug", &BM_ApproxVoxelGrid, argv[1])
      ->Unit(benchmark::kMillisecond);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
