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

#include <pcl/experimental/executor/property.h>
#include <pcl/experimental/executor/type_trait.h>

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
  struct cuda_dim {
    struct dim3 {
      unsigned int x, y, z;
    } grid_dim, block_dim;
  };

  using shape_type = cuda_dim;

  template <typename Executor, instance_of_base<Executor, cuda_executor> = 0>
  friend bool operator==(const cuda_executor& lhs,
                         const Executor& rhs) noexcept {
    return std::is_same<cuda_executor, Executor>::value;
  }

  template <typename Executor, instance_of_base<Executor, cuda_executor> = 0>
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

  // Passing rvalue reference of function doesn't currently work with CUDA for
  // some reason
  template <typename F>
  void bulk_execute(F& f, const shape_type shape) const {
    // TODO: Add custom shape property for CUDA executor
#ifdef CUDA
    void* global_kernel_args[] = {static_cast<void*>(&f)};
    dim3 grid_size(shape.grid_dim.x, shape.grid_dim.y, shape.grid_dim.z);
    dim3 block_size(shape.block_dim.x, shape.block_dim.y, shape.block_dim.z);
    cudaLaunchKernel(reinterpret_cast<void*>(global_kernel<F>), grid_size,
                     block_size, global_kernel_args, 0, nullptr);
    cudaDeviceSynchronize();
#endif
  }

  static constexpr auto query(const blocking_t&) noexcept { return Blocking{}; }

  cuda_executor<blocking_t::always_t, ProtoAllocator> require(
      const blocking_t::always_t&) const {
    return {};
  }

  static constexpr auto name() { return "cuda"; }
};

}  // namespace executor
