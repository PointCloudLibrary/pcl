//
// Created by Shrijit Singh on 14/06/20.
//

#pragma once

#include <pcl/experimental/execution//property/base_property.hpp>

template <typename ProtoAllocator>
class allocator_t
  : public basic_executor_property<allocator_t<ProtoAllocator>, true, true> {
 public:
  constexpr explicit allocator_t(const ProtoAllocator &alloc) : alloc_(alloc) {}

  constexpr ProtoAllocator value() const { return alloc_; }

 private:
  ProtoAllocator alloc_;
};

template <>
struct allocator_t<void>
  : public basic_executor_property<allocator_t<void>, true, true> {
  template <class ProtoAllocator>
  constexpr allocator_t<ProtoAllocator>
  operator()(const ProtoAllocator &alloc) const {
    return allocator_t<ProtoAllocator>{alloc};
  }
};

static constexpr allocator_t<void> allocator{};
