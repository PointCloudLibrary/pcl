/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */
#include <pcl/experimental/executor/property/blocking.hpp>

namespace pcl {
namespace executor {
/**
 * \todo Can inline member variable in C++17 into blocking.hpp, eliminating
 * need for separate CPP file.
 * Workaround:
 * https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
 * inline constexpr blocking_t::possibly_t blocking_t::possibly;
 */
constexpr blocking_t::possibly_t blocking_t::possibly;
constexpr blocking_t::always_t blocking_t::always;
constexpr blocking_t::never_t blocking_t::never;
} // namespace executor
} // namespace pcl
