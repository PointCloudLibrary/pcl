//
// Created by Shrijit Singh on 2020-06-14.
//

#pragma once

#ifdef _OPENMP
#include <omp.h>
#endif

#include <pcl/experimental/execution//executor/base_executor.hpp>

template <typename Interface, typename Cardinality, typename Blocking,
          typename ProtoAllocator>
struct omp_executor;

#ifdef _OPENMP
namespace execution {
template <>
struct is_executor_available<omp_executor> : std::true_type {};
}  // namespace experimental
#endif

template <typename Interface, typename Cardinality, typename Blocking,
          typename ProtoAllocator = std::allocator<void>>
struct omp_executor
    : executor<omp_executor, Interface, Cardinality, Blocking, ProtoAllocator> {
  using shape_type = std::size_t;

  template <typename F>
  void execute(F &&f) {
    invoke_hpp::invoke(std::forward<F>(f));
  }

  template <typename F>
  void bulk_execute(F &&f, shape_type n){
#ifdef _OPENMP
#pragma omp parallel num_threads(n)
      {invoke_hpp::invoke(std::forward<F>(f), omp_get_thread_num());
}
#endif
}

//  auto decay_t() -> decltype(auto) {
//    if constexpr (experimental::is_executor_available_v<omp_executor>) {
//      return *this;
//    } else
//      return inline_executor<oneway_t, single_t, blocking_t::always_t,
//                             ProtoAllocator>{};
//  }

std::string name() { return "omp"; }
}
;
