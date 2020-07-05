/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/property/base_property.hpp>

namespace executor {

// Part of Proposal P0443R13: 2.2.13
template <typename ProtoAllocator>
struct allocator_t
    : basic_executor_property<allocator_t<ProtoAllocator>, true, true> {
  constexpr explicit allocator_t(const ProtoAllocator& alloc) : alloc_(alloc) {}

  constexpr ProtoAllocator value() const { return alloc_; }

 private:
  ProtoAllocator alloc_;
};

template <>
struct allocator_t<void>
    : basic_executor_property<allocator_t<void>, true, true> {
  template <class ProtoAllocator>
  constexpr allocator_t<ProtoAllocator> operator()(
      const ProtoAllocator& alloc) const {
    return allocator_t<ProtoAllocator>{alloc};
  }
};

static constexpr allocator_t<void> allocator{};

}  // namespace executor
