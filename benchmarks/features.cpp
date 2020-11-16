#include <benchmark/benchmark.h>
#include <pcl/features/normal_3d.h> // for NormalEstimation
#include <pcl/io/pcd_io.h> // for PCDReader

static void BM_NormalEstimation(benchmark::State& state, const std::string& file) {
  // Perform setup here
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read (file, *cloud);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  ne.setKSearch (100);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  for (auto _ : state) {
    // This code gets timed
    ne.compute (*cloud_normals);
  }
}

int main(int argc, char** argv) {
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `table_scene_mug_stereo_textured.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  benchmark::RegisterBenchmark("BM_NormalEstimation", BM_NormalEstimation, argv[1])->Unit(benchmark::kMillisecond);
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
