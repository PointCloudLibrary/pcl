/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#ifdef CUDA
  #include <cuda_runtime_api.h>
#endif

#include <pcl/experimental/executor/default/base_executor.hpp>
#include <pcl/experimental/executor/default/inline_executor.hpp>

namespace executor {

#ifdef CUDA
template <typename F>
__global__ void global_kernel(F f) {
  f();
}
#endif

template <typename Blocking, typename ProtoAllocator>
struct cuda_executor;

#ifdef CUDA
template <>
struct is_executor_available<cuda_executor> : std::true_type {};
#endif

template <typename Blocking = blocking_t::always_t,
          typename ProtoAllocator = std::allocator<void>>
struct cuda_executor {
  using shape_type = typename std::array<int, 6>;

  template <typename Executor, instance_of_base<cuda_executor, Executor> = 0>
  friend bool operator==(const cuda_executor& lhs,
                         const Executor& rhs) noexcept {
    return std::is_same<cuda_executor, Executor>::value;
  }

  template <typename Executor, instance_of_base<cuda_executor, Executor> = 0>
  friend bool operator!=(const cuda_executor& lhs,
                         const Executor& rhs) noexcept {
    return !operator==(lhs, rhs);
  }

  template <typename F>
  void execute(F& f) const {
#ifdef CUDA
    void* global_kernel_args[] = {static_cast<void*>(&f)};
    cudaLaunchKernel(reinterpret_cast<void*>(global_kernel<F>), 1, 1,
                     global_kernel_args, 0, nullptr);
    cudaDeviceSynchronize();
#endif
  }

  // Temporary fix for unit test compilation
  template <typename F>
  void bulk_execute(F& f, std::size_t n) const {
    bulk_execute(f, std::array<int, 6>{1, 1, 1, static_cast<int>(n), 1, 1});
  }

  // Passing rvalue reference of function doesn't currently work with CUDA for
  // some reason
  template <typename F>
  void bulk_execute(F& f, shape_type shape) const {
#ifdef CUDA
    void* global_kernel_args[] = {static_cast<void*>(&f)};
    dim3 grid_size(shape[0], shape[1], shape[2]);
    dim3 block_size(shape[3], shape[4], shape[5]);
    cudaLaunchKernel(reinterpret_cast<void*>(global_kernel<F>), grid_size,
                     block_size, global_kernel_args, 0, nullptr);
    cudaDeviceSynchronize();
#endif
  }

  static constexpr auto query(blocking_t) noexcept { return Blocking{}; }

  cuda_executor<blocking_t::always_t, ProtoAllocator> require(
      const blocking_t::always_t&) const {
    return {};
  }

  static constexpr auto name() { return "cuda"; }
};

}  // namespace executor
