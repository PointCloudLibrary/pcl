#include <pcl/common/generate.h>
#include <pcl/filters/functor_filter.h>

#include <chrono>

namespace executor {
// Forward declaration for custom executor
template <typename Blocking, typename ProtoAllocator>
struct omp_benchmark_executor;

// Mark the executor as available
#ifdef _OPENMP
  template <>
  struct is_executor_available<omp_benchmark_executor> : std::true_type {};
#endif

// Custom executor derived from OMP executor
template <typename Blocking = blocking_t::always_t,
          typename ProtoAllocator = allocator_t<void>>
struct omp_benchmark_executor : public omp_executor<Blocking, ProtoAllocator> {
  // Introduce base struct members
  using Base = omp_executor<Blocking, ProtoAllocator>;
  using Base::max_threads;
  using typename Base::index_type;
  using typename Base::omp_executor;
  using typename Base::shape_type;

  template <typename F>
  void
  bulk_execute(F&& f, const shape_type& n) const
  {
    // Throw static assert failure if executor is not available
    static_assert(is_executor_available_v<omp_executor>,
                  "OpenMP benchmark executor unavailable");

    // Measure total time taken by all threads
    const auto total_t1 = std::chrono::steady_clock::now();

    #pragma omp parallel num_threads(max_threads)
    {
      // Measure time taken by each thread
      const auto thread_t1 = std::chrono::steady_clock::now();

      // Invoke the callable n times
      #pragma omp for nowait
      for (index_type index = 0; index < n; ++index)
        f(index);

      const auto thread_t2 = std::chrono::steady_clock::now();
      const auto thread_duration =
          std::chrono::duration_cast<std::chrono::nanoseconds>(thread_t2 - thread_t1)
              .count();
      std::cout << "Time taken by Thread: " << omp_get_thread_num() << " is "
                << thread_duration << " ns\n";
    }

    const auto total_t2 = std::chrono::steady_clock::now();
    const auto total_duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(total_t2 - total_t1)
            .count();
    std::cout << "Total time taken: " << total_duration << " ns" << std::endl;
  }
};
} // namespace executor

int
main()
{
  // Create empty output point clouds and fill the input cloud with randomly generated
  // points
  pcl::PointCloud<pcl::PointXYZ> out_cloud1, out_cloud2;
  const auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::UniformGenerator<float>>
      generator{{-20., 20., 128}};
  generator.fill(200, 200, *cloud);

  // Create a functor filter that filters point outside a fixed radius
  const auto positive_cond = [](const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                pcl::index_t idx) {
    return (cloud[idx].getArray3fMap() > 0).all();
  };

  auto positive_filter =
      pcl::experimental::FunctorFilter<pcl::PointXYZ, decltype(positive_cond)>(
          positive_cond);
  positive_filter.setInputCloud(cloud);

  // Create instance of custom executor and apply the filter with it
  auto exec = executor::omp_benchmark_executor<>(4);
  std::cout << "Filtering using 4 Threads" << std::endl;
  positive_filter.filter(exec, out_cloud1);

  exec.set_max_threads(1);
  std::cout << "\nFiltering using 4 Threads" << std::endl;
  positive_filter.filter(exec, out_cloud1);
}
