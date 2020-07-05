/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/property/base_property.hpp>

namespace pcl {
namespace executor {

/**
 * \brief A property for customizing memory allocation
 *
 * \details Currently no such use of the allocator has been determined
 * and void is set as the allocator type.
 *
 * Part of Proposal P0443R13 (2.2.13)
 *
 * \todo Look into use cases of allocators in executors and implement their
 * mechanism for using them if needed
 */
template <typename ProtoAllocator>
struct allocator_t : base_executor_property<allocator_t<ProtoAllocator>, true, true> {
  constexpr explicit allocator_t(const ProtoAllocator& alloc) : alloc_(alloc) {}

  constexpr ProtoAllocator
  value() const
  {
    return alloc_;
  }

private:
  ProtoAllocator alloc_;
};
/**
 * \brief A specialization of \ref allocator_t that indicates the executor shall use
 * the default allocator
 *
 * Part of Proposal P0443R13 (2.2.13)
 */
template <>
struct allocator_t<void> : base_executor_property<allocator_t<void>, true, true> {
  template <class ProtoAllocator>
  constexpr allocator_t<ProtoAllocator>
  operator()(const ProtoAllocator& alloc) const
  {
    return allocator_t<ProtoAllocator>{alloc};
  }
};

static constexpr allocator_t<void> allocator{};

} // namespace executor
} // namespace pcl
