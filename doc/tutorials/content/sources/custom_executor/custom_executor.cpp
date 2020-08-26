#include <pcl/common/generate.h>
#include <pcl/filters/functor_filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

namespace executor
{
// Forward declaration for custom executor
template <typename Blocking, typename ProtoAllocator>
struct omp_benchmark_executor;

// Mark the executor as available
template <>
struct is_executor_available<omp_benchmark_executor> : std::true_type
{
};

// Custom executor derived from OMP executor
template <typename Blocking = blocking_t::always_t, typename ProtoAllocator = allocator_t<void>>
struct omp_benchmark_executor : omp_executor<Blocking, ProtoAllocator>
{
  // Introduce base struct members
  using Base = omp_executor<Blocking, ProtoAllocator>;
  using Base::max_threads;
  using typename Base::index_type;
  using typename Base::shape_type;

  template <typename F>
  void bulk_execute(F &&f, const shape_type &n) const
  {
    // Throw static assert failure if executor is not available
    static_assert(is_executor_available_v<omp_executor>, "OpenMP benchmark executor unavailable");

    // Limit the the number of threads, to the maximum set in the executor
    const auto num_threads = n ? std::min(max_threads, n) : max_threads;

    // Measure total time taken by all threads
    auto total_t1 = std::chrono::steady_clock::now();

#pragma omp parallel num_threads(num_threads)
    {
      // Measure time taken by each thread
      auto thread_t1 = std::chrono::steady_clock::now();
      // Invoke the callable, withe current index of the execution agent
      index_type index{num_threads, omp_get_thread_num()};
      f(index);

      auto thread_t2 = std::chrono::steady_clock::now();
      auto thread_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(thread_t2 - thread_t1).count();
      std::cout<<"Time taken by Thread: "<<omp_get_thread_num()<<" is "<<thread_duration<<" ns\n";
    }

    auto total_t2 = std::chrono::steady_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(total_t2 - total_t1).count();
    std::cout<<"Total time taken: "<<total_duration<<" ns"<<std::endl;
  }
};
} // namespace executor

int main()
{
  // Fill the cloud with randomly generated points
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::UniformGenerator<float>> generator{{-20., 20., 128}};
  generator.fill(200, 200, *cloud);

  // Create a functor filter that filters point outside a fixed radius
  const auto positive_cond = [](const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::index_t idx) {
    const auto &pt = cloud[idx];
    return (pt.getArray3fMap() > 0).all();
  };

  auto positive_filter = pcl::experimental::FunctorFilter<pcl::PointXYZ, decltype(positive_cond)>(positive_cond);
  positive_filter.setInputCloud(cloud);


  // Create instance of custom executr and apply the filters with it
  auto exec = executor::omp_benchmark_executor<>{};
  exec.set_max_threads(4);
  positive_filter.filter(exec, *cloud);
}
