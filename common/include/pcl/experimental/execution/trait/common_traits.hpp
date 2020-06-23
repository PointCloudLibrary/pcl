//
// Created by Shrijit Singh on 18/06/20.
//

#pragma once

namespace execution {
namespace detail {

template <typename... Ts>
struct make_void {
  typedef void type;
};
template <typename... Ts>
using void_t = typename make_void<Ts...>::type;

}  // namespace detail
}  // namespace experimental
