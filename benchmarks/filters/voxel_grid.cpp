#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/experimental/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h> // for PCDReader

#include <benchmark/benchmark.h>

#include <array>
#include <string>
#include <vector>

using namespace pcl;

template <typename PointT, template <typename> class GridFilter>
static void
BM_grid_filter(benchmark::State& state, const typename PointCloud<PointT>::Ptr cloud)
{
  // Perform setup here
  GridFilter<PointT> f;
  f.setLeafSize(0.01, 0.01, 0.01);
  f.setInputCloud(cloud);

  PointCloud<PointT> cloud_voxelized;
  for (auto _ : state) {
    // This code gets timed
    f.filter(cloud_voxelized);
  }
}

template <typename PointT, template <typename> class T>
void
BM_register(const std::string& class_name,
            const typename PointCloud<PointT>::Ptr dataset,
            const std::string& dataset_name,
            const benchmark::TimeUnit unit)
{
  const std::string bm_name = dataset_name + "_" + class_name;
  benchmark::RegisterBenchmark(bm_name.c_str(), &BM_grid_filter<PointT, T>, dataset)
      ->Unit(unit);
};

template <typename PointT, template <typename> class... Ts, std::size_t ArraySize>
bool
BM_registration(const std::array<std::string, ArraySize>& classes_name,
                const std::vector<typename PointCloud<PointT>::Ptr>& datasets,
                const std::vector<std::string>& datasets_name,
                const benchmark::TimeUnit unit = benchmark::kMillisecond)
{
  static_assert(sizeof...(Ts) == ArraySize,
                "Number of template classes and classes name are different.\n");

  if (datasets.size() != datasets_name.size()) {
    PCL_ERROR("[Benchmark] Number of datasets and datasets name are different.\n");
    return false;
  }

  for (std::size_t i = 0; i < datasets.size(); ++i) {
    typename std::array<std::string, ArraySize>::const_iterator name_it =
        classes_name.begin();
    const int res[] = {(BM_register<PointT, Ts>(
                            *(name_it++), datasets.at(i), datasets_name.at(i), unit),
                        0)...};
    (void)res; // suppress warning
  }

  return true;
}

int
main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr
        << "No test files given. Please download `table_scene_mug_stereo_textured.pcd` "
           ", `milk_cartoon_all_small_clorox.pcd`, `office1.pcd` and "
           "`five_people.pcd`, and pass their paths to the test."
        << std::endl;
    return (-1);
  }

  const std::vector<std::string> datasets_path(argv + 1, argv + argc);
  const std::vector<std::string> datasets_name{
      "table", "milk_cartoon", "office", "five_people"};

  std::vector<PointCloud<PointXYZ>::Ptr> datasets;
  datasets.reserve(datasets_path.size());

  PCDReader reader;
  for (const auto& path : datasets_path) {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    reader.read(path, *cloud);
    datasets.push_back(std::move(cloud));
  }

  const std::array<std::string, 3> filters_name{
      "VoxelGrid", "ApproximateVoxelGrid", "FunctorVoxelGrid"};
  if (BM_registration<PointXYZ,
                      VoxelGrid,
                      ApproximateVoxelGrid,
                      experimental::VoxelGrid>(filters_name, datasets, datasets_name)) {
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
  }

  return 0;
}
