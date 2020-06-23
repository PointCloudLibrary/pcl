//
// Created by Shrijit Singh on 2020-06-14.
//

#pragma once

#ifdef CUDA
#include <cuda_runtime_api.h>

#include <nvfunctional>
#endif

#include <pcl/experimental/execution//executor/base_executor.hpp>
#include <pcl/experimental/execution//executor/inline_executor.hpp>

#ifdef CUDA
template <typename F>
__global__ void global_kernel(F f) {
  (*f)();
}

template <typename NVSTDFunc, typename Lambda>
__global__ void device_func_insert(NVSTDFunc *f, Lambda l) {
  *f = l;
}
#endif

template <typename Interface, typename Cardinality, typename Blocking,
          typename ProtoAllocator>
struct cuda_executor;

#ifdef CUDA
namespace experimental {
template <>
struct is_executor_available<cuda_executor> : std::true_type {};
}  // namespace experimental
#endif

template <typename Interface, typename Cardinality, typename Blocking,
          typename ProtoAllocator = std::allocator<void>>
struct cuda_executor : executor<cuda_executor, Interface, Cardinality, Blocking,
                                ProtoAllocator> {
  using shape_type = typename std::array<int, 6>;

  template <typename F>
  void execute(F &&f) {
#ifdef CUDA
    nvstd::function<void(void)> *nv_f;
    cudaMalloc(reinterpret_cast<void **>(&nv_f),
               sizeof(nvstd::function<void(void)>));
    void *device_func_insert_args[] = {static_cast<void *>(&nv_f),
                                       static_cast<void *>(&f)};

    cudaLaunchKernel(reinterpret_cast<void *>(
                         device_func_insert<nvstd::function<void(void)>, F>),
                     1, 1, device_func_insert_args, 0, nullptr);

    void *global_kernel_args[] = {static_cast<void *>(&nv_f)};
    cudaLaunchKernel(
        reinterpret_cast<void *>(global_kernel<nvstd::function<void(void)> *>),
        1, 1, global_kernel_args, 0, nullptr);
    cudaDeviceSynchronize();
#endif
  }

  // Temporary fix for unit test compilation
  template <typename F>
  void bulk_execute(F f, std::size_t n) {
    bulk_execute(f, std::array<int, 6>{1, 1, 1, static_cast<int>(n), 1, 1});
  }

  template <typename F>
  void bulk_execute(F f, shape_type shape) {
#ifdef CUDA
    nvstd::function<void(void)> *nv_f;
    cudaMalloc(reinterpret_cast<void **>(&nv_f),
               sizeof(nvstd::function<void(void)>));
    void *device_func_insert_args[] = {static_cast<void *>(&nv_f),
                                       static_cast<void *>(&f)};

    cudaLaunchKernel(reinterpret_cast<void *>(
                         device_func_insert<nvstd::function<void(void)>, F>),
                     1, 1, device_func_insert_args, 0, nullptr);

    void *global_kernel_args[] = {static_cast<void *>(&nv_f)};
    dim3 grid_size(shape[0], shape[1], shape[2]);
    dim3 block_size(shape[3], shape[4], shape[5]);
    cudaLaunchKernel(
        reinterpret_cast<void *>(global_kernel<nvstd::function<void(void)> *>),
        grid_size, block_size, global_kernel_args, 0, nullptr);
    cudaDeviceSynchronize();
#endif
  }

  //  auto decay_t() -> decltype(auto) {
  //    if constexpr (experimental::is_executor_available_v<cuda_executor>) {
  //      return *this;
  //    } else
  //      return inline_executor<oneway_t, single_t, blocking_t::always_t,
  //                             ProtoAllocator>{};
  //  }

  std::string name() { return "cuda"; }
};
