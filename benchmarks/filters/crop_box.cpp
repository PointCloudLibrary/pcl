
#include <pcl/common/eigen.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/experimental/crop_box.h>

#include <benchmark/benchmark.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

using namespace pcl;

template <typename PointT, template <typename> class Filter>
static void
BM_crop_box(benchmark::State& state, const typename PointCloud<PointT>::Ptr cloud)
{
  Filter<PointT> filter;
  typename pcl::PointCloud<PointT> out_cloud;

  filter.setInputCloud(cloud);
  const Eigen::Vector3f r = Eigen::Vector3f::Random();
  filter.setTransform(pcl::getTransformationFromTwoUnitVectors(r, r));
  filter.setRotation(r);
  filter.setTranslation(r);

  for (auto _ : state)
    filter.filter(out_cloud);
}

template <typename PointT, template <typename> class T>
void
BM_register(const std::string& class_name,
            const typename PointCloud<PointT>::Ptr dataset,
            const std::string& dataset_name,
            const benchmark::TimeUnit unit)
{
  const std::string bm_name = dataset_name + "_" + class_name;
  benchmark::RegisterBenchmark(bm_name.c_str(), &BM_crop_box<PointT, T>, dataset)
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
  const std::vector<std::string> datasets_name = {
      "320p", "480p", "720p", "1080p", "1440p"};
  const std::vector<std::size_t> datasets_size = {
      480 * 320, 640 * 480, 1280 * 720, 1920 * 1080, 2560 * 1440};

  std::vector<PointCloud<PointXYZ>::Ptr> datasets;
  for (const std::size_t dataset_size : datasets_size) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->resize(dataset_size);
    for (auto& pt : *cloud)
      pt = pcl::PointXYZ{0., 0., 0.};
    datasets.push_back(std::move(cloud));
  }

  const std::array<std::string, 2> filters_name{"CropBox", "FunctorCropBox"};

  if (BM_registration<PointXYZ, CropBox, experimental::CropBox>(
          filters_name, datasets, datasets_name)) {
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
  }

  return 0;
}