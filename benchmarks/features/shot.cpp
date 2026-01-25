#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <benchmark/benchmark.h>

static void
BM_SHOT352(benchmark::State& state, const std::string& file)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read(file, *cloud);

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setKSearch(10);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
  shot.setInputCloud(cloud);
  shot.setInputNormals(normals);
  shot.setRadiusSearch(0.04);

  pcl::PointCloud<pcl::SHOT352>::Ptr output(new pcl::PointCloud<pcl::SHOT352>);
  for (auto _ : state) {
    shot.compute(*output);
  }
}

static void
BM_SHOT352_OMP(benchmark::State& state, const std::string& file)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read(file, *cloud);

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setKSearch(10);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot(
      0, state.range(0) //
  );
  shot.setInputCloud(cloud);
  shot.setInputNormals(normals);
  shot.setRadiusSearch(0.04);

  pcl::PointCloud<pcl::SHOT352>::Ptr output(new pcl::PointCloud<pcl::SHOT352>);
  for (auto _ : state) {
    shot.compute(*output);
  }
}

static void
BM_SHOT1344(benchmark::State& state, const std::string& file)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read(file, *cloud_xyz);

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud_xyz);
  ne.setKSearch(10);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgba(
      new pcl::PointCloud<pcl::PointXYZRGBA>);

  for (int i = 0; i < static_cast<int>(cloud_xyz->size()); ++i) {
    pcl::PointXYZRGBA p;
    p.x = (*cloud_xyz)[i].x;
    p.y = (*cloud_xyz)[i].y;
    p.z = (*cloud_xyz)[i].z;

    p.rgba = ((i % 255) << 16) + (((255 - i) % 255) << 8) + ((i * 37) % 255);
    cloud_rgba->push_back(p);
  }

  pcl::SHOTColorEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344> shot(true,
                                                                               true);
  shot.setInputCloud(cloud_rgba);
  shot.setInputNormals(normals);
  shot.setRadiusSearch(0.04);

  pcl::PointCloud<pcl::SHOT1344>::Ptr output(new pcl::PointCloud<pcl::SHOT1344>);
  for (auto _ : state) {
    shot.compute(*output);
  }
}

static void
BM_SHOT1344_OMP(benchmark::State& state, const std::string& file)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read(file, *cloud_xyz);

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud_xyz);
  ne.setKSearch(10);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgba(
      new pcl::PointCloud<pcl::PointXYZRGBA>);

  for (int i = 0; i < static_cast<int>(cloud_xyz->size()); ++i) {
    pcl::PointXYZRGBA p;
    p.x = (*cloud_xyz)[i].x;
    p.y = (*cloud_xyz)[i].y;
    p.z = (*cloud_xyz)[i].z;

    p.rgba = ((i % 255) << 16) + (((255 - i) % 255) << 8) + ((i * 37) % 255);
    cloud_rgba->push_back(p);
  }

  pcl::SHOTColorEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344> shot(
      true, true, 0, state.range(0));
  shot.setInputCloud(cloud_rgba);
  shot.setInputNormals(normals);
  shot.setRadiusSearch(0.04);

  pcl::PointCloud<pcl::SHOT1344>::Ptr output(new pcl::PointCloud<pcl::SHOT1344>);
  for (auto _ : state) {
    shot.compute(*output);
  }
}

int
main(int argc, char** argv)
{
  if (argc < 2) {
    std::cerr << "No test files given. Please provide a PCD file paths for SHOT352 "
                 "and SHOT1344 benchmarks."
              << std::endl;
    return (-1);
  }

  constexpr int runs = 100;

  benchmark::RegisterBenchmark("BM_SHOT352", &BM_SHOT352, argv[1])
      ->Unit(benchmark::kMillisecond)
      ->Iterations(runs);
  benchmark::RegisterBenchmark("BM_SHOT352_OMP", &BM_SHOT352_OMP, argv[1])
      ->Arg(64)
      ->Arg(128)
      ->Arg(256)
      ->Unit(benchmark::kMillisecond)
      ->Iterations(runs);

  benchmark::RegisterBenchmark("BM_SHOT1344", &BM_SHOT1344, argv[1])
      ->Unit(benchmark::kMillisecond)
      ->Iterations(runs);
  benchmark::RegisterBenchmark("BM_SHOT1344_OMP", &BM_SHOT1344_OMP, argv[1])
      ->Arg(64)
      ->Arg(128)
      ->Arg(256)
      ->Unit(benchmark::kMillisecond)
      ->Iterations(runs);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
