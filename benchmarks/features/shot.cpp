#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <benchmark/benchmark.h>

constexpr float lrf_radius = 0.04f;
constexpr float shot_search_radius = 0.04f;
constexpr float uniform_sampling_radius = 0.03f;

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

  pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
  uniform_sampling.setInputCloud(cloud);
  uniform_sampling.setRadiusSearch(uniform_sampling_radius);
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
  uniform_sampling.filter(*keypoints);

  pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
  shot.setInputCloud(keypoints);
  shot.setInputNormals(normals);
  shot.setSearchSurface(cloud);
  shot.setRadiusSearch(shot_search_radius);
  shot.setLRFRadius(lrf_radius);

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

  pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
  uniform_sampling.setInputCloud(cloud);
  uniform_sampling.setRadiusSearch(uniform_sampling_radius);
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
  uniform_sampling.filter(*keypoints);

  pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot(
      0, state.range(0) //
  );
  shot.setInputCloud(keypoints);
  shot.setInputNormals(normals);
  shot.setSearchSurface(cloud);
  shot.setRadiusSearch(shot_search_radius);
  shot.setLRFRadius(lrf_radius);

  pcl::PointCloud<pcl::SHOT352>::Ptr output(new pcl::PointCloud<pcl::SHOT352>);
  for (auto _ : state) {
    shot.compute(*output);
  }
}

static void
BM_SHOT1344(benchmark::State& state, const std::string& file)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgba(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PCDReader reader;
  reader.read(file, *cloud_rgba);

  pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setInputCloud(cloud_rgba);
  ne.setKSearch(10);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  pcl::UniformSampling<pcl::PointXYZRGBA> uniform_sampling;
  uniform_sampling.setInputCloud(cloud_rgba);
  uniform_sampling.setRadiusSearch(uniform_sampling_radius);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  uniform_sampling.filter(*keypoints);

  pcl::SHOTColorEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344> shot(true,
                                                                               true);
  shot.setInputCloud(keypoints);
  shot.setInputNormals(normals);
  shot.setSearchSurface(cloud_rgba);
  shot.setRadiusSearch(shot_search_radius);
  shot.setLRFRadius(lrf_radius);

  pcl::PointCloud<pcl::SHOT1344>::Ptr output(new pcl::PointCloud<pcl::SHOT1344>);
  for (auto _ : state) {
    shot.compute(*output);
  }
}

static void
BM_SHOT1344_OMP(benchmark::State& state, const std::string& file)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgba(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PCDReader reader;
  reader.read(file, *cloud_rgba);

  pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setInputCloud(cloud_rgba);
  ne.setKSearch(10);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  pcl::UniformSampling<pcl::PointXYZRGBA> uniform_sampling;
  uniform_sampling.setInputCloud(cloud_rgba);
  uniform_sampling.setRadiusSearch(uniform_sampling_radius);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  uniform_sampling.filter(*keypoints);

  pcl::SHOTColorEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT1344> shot(
      true, true, 0, state.range(0));
  shot.setInputCloud(keypoints);
  shot.setInputNormals(normals);
  shot.setSearchSurface(cloud_rgba);
  shot.setRadiusSearch(shot_search_radius);
  shot.setLRFRadius(lrf_radius);

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
