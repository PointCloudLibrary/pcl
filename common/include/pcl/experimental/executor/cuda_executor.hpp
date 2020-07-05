/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#ifdef __CUDACC__
#include <cuda_runtime_api.h>
#endif

#include <pcl/experimental/executor/property.h>
#include <pcl/experimental/executor/type_trait.h>
#include <pcl/types.h>

namespace pcl {
namespace executor {

#ifdef __CUDACC__
template <typename F>
__global__ void
global_kernel(F f)
{
  f();
}
#endif

template <typename Blocking, typename ProtoAllocator>
struct cuda_executor;

#ifdef __CUDACC__
template <>
struct is_executor_available<cuda_executor> : std::true_type {};
#endif

/** \brief Executes the CUDA kernels on the GPU
 *
 * \todo
 * 1. Add support for non-blocking behaviour
 * 2. Introducing CUDA specific property to enable streams and synchronization
 * mechanisms
 */
template <typename Blocking = blocking_t::always_t,
          typename ProtoAllocator = std::allocator<void>>
struct cuda_executor {
  struct shape_type {
    struct dim3 {
      uindex_t x, y, z;
    } grid_size, block_size;
  };

  template <typename Executor, InstanceOf<Executor, executor::cuda_executor> = 0>
  friend constexpr bool
  operator==(const cuda_executor&, const Executor&) noexcept
  {
    return std::is_same<cuda_executor, Executor>::value;
  }

  template <typename Executor, executor::InstanceOf<Executor, cuda_executor> = 0>
  friend constexpr bool
  operator!=(const cuda_executor& lhs, const Executor& rhs) noexcept
  {
    return !operator==(lhs, rhs);
  }

  /**
   * \brief  Launches the a single instance of the kernel
   *
   * \param f a CUDA kernel
   */
  template <typename Function>
  void
  execute(Function& f) const
  {
    static_assert(is_executor_available_v<cuda_executor>, "CUDA executor unavailable");
    pcl::utils::ignore(f);
#ifdef __CUDACC__
    void* global_kernel_args[] = {static_cast<void*>(&f)};
    cudaLaunchKernel(reinterpret_cast<void*>(global_kernel<F>),
                     1,
                     1,
                     global_kernel_args,
                     0,
                     nullptr);
    cudaDeviceSynchronize();
#endif
  }

  /**
   * \brief Launches the kernel with the specified \param shape
   *
   * \param f a CUDA kernel
   * \param shape dimensions and size of grid and block
   *
   * \todo Investigate why passing rvalue reference of kernel doesn't work
   */
  template <typename Function>
  void
  bulk_execute(Function& f, const shape_type& shape) const
  {
    static_assert(is_executor_available_v<cuda_executor>, "CUDA executor unavailable");
    pcl::utils::ignore(f, shape);
#ifdef __CUDACC__
    void* global_kernel_args[] = {static_cast<void*>(&f)};
    dim3 grid_size(shape.grid_size.x, shape.grid_size.y, shape.grid_size.z);
    dim3 block_size(shape.block_size.x, shape.block_size.y, shape.block_size.z);
    cudaLaunchKernel(reinterpret_cast<void*>(global_kernel<F>),
                     grid_size,
                     block_size,
                     global_kernel_args,
                     0,
                     nullptr);
    cudaDeviceSynchronize();
#endif
  }

  static constexpr decltype(auto)
  query(const blocking_t&) noexcept
  {
    return blocking_t::always;
  }

  cuda_executor<blocking_t::always_t, ProtoAllocator>
  require(const blocking_t::always_t&) const
  {
    return {};
  }
};

using default_cuda_executor = cuda_executor<>;

} // namespace executor
} // namespace pcl
