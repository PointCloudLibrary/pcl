//
// Created by Shrijit Singh on 14/06/20.
//

#pragma once

#include <pcl/experimental/execution//property/base_property.hpp>

template <class Executor>
class executor_shape: public basic_executor_property<executor_shape<Executor>, true, true> {
//private:
//  template <class T> using helper = typename T::shape_type;

 public:
  template<unsigned _s0 = 0, unsigned... _sizes>
  using type = std::size_t;
//  using type = std::experimental::detected_or_t<size_t, helper, Executor>;
//
//  static_assert(std::is_integral_v<type>,
//                "shape type must be an integral type");
};

template <class Executor> struct executor_shape;

template <class Executor>
using executor_shape_t = typename executor_shape<Executor>::type;
