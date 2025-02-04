/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2019-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

/**
 * \file pcl/memory.h
 *
 * \brief Defines functions, macros and traits for allocating and using memory.
 * \ingroup common
 */

#include <pcl/type_traits.h>  // for has_custom_allocator
#include <pcl/pcl_config.h> // for PCL_USES_EIGEN_HANDMADE_ALIGNED_MALLOC

#include <Eigen/Core>  // for EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#include <memory>  // for std::allocate_shared, std::dynamic_pointer_cast, std::make_shared, std::shared_ptr, std::static_pointer_cast, std::weak_ptr
#include <type_traits>  // for std::enable_if_t, std::false_type, std::true_type
#include <utility>  // for std::forward

#if !defined(PCL_SILENCE_MALLOC_WARNING) && !defined(__NVCC__)
#if PCL_USES_EIGEN_HANDMADE_ALIGNED_MALLOC
// EIGEN_DEFAULT_ALIGN_BYTES and EIGEN_MALLOC_ALREADY_ALIGNED will be set after including Eigen/Core
// this condition is the same as in the function aligned_malloc in Memory.h in the Eigen code
#if (defined(EIGEN_DEFAULT_ALIGN_BYTES) && EIGEN_DEFAULT_ALIGN_BYTES==0) || (defined(EIGEN_MALLOC_ALREADY_ALIGNED) && EIGEN_MALLOC_ALREADY_ALIGNED)
#if defined(_MSC_VER)
#error "Potential runtime error due to aligned malloc mismatch! You likely have to compile your code with AVX enabled or define EIGEN_MAX_ALIGN_BYTES=32 (to silence this message at your own risk, define PCL_SILENCE_MALLOC_WARNING=1)"
#else // defined(_MSC_VER)
#warning "Potential runtime error due to aligned malloc mismatch! You likely have to compile your code with AVX enabled or define EIGEN_MAX_ALIGN_BYTES=32 (to silence this message at your own risk, define PCL_SILENCE_MALLOC_WARNING=1)"
#endif // defined(_MSC_VER)
#endif
#else // PCL_USES_EIGEN_HANDMADE_ALIGNED_MALLOC
#if (defined(EIGEN_DEFAULT_ALIGN_BYTES) && EIGEN_DEFAULT_ALIGN_BYTES!=0) && (defined(EIGEN_MALLOC_ALREADY_ALIGNED) && !EIGEN_MALLOC_ALREADY_ALIGNED)
#if defined(_MSC_VER)
#error "Potential runtime error due to aligned malloc mismatch! PCL was likely compiled without AVX support but you enabled AVX for your code (to silence this message at your own risk, define PCL_SILENCE_MALLOC_WARNING=1)"
#else // defined(_MSC_VER)
#warning "Potential runtime error due to aligned malloc mismatch! PCL was likely compiled without AVX support but you enabled AVX for your code (to silence this message at your own risk, define PCL_SILENCE_MALLOC_WARNING=1)"
#endif // defined(_MSC_VER)
#endif
#endif // PCL_USES_EIGEN_HANDMADE_ALIGNED_MALLOC
#endif // !defined(PCL_SILENCE_MALLOC_WARNING)

/**
 * \brief Macro to signal a class requires a custom allocator
 *
 *  It's an implementation detail to have pcl::has_custom_allocator work, a
 *  thin wrapper over Eigen's own macro
 *
 * \see pcl::has_custom_allocator, pcl::make_shared
 * \ingroup common
 */
#define PCL_MAKE_ALIGNED_OPERATOR_NEW \
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
  using _custom_allocator_type_trait = void;


namespace pcl
{
/**
 * \brief Force ADL for `shared_ptr`
 *
 * For ease of switching from boost::shared_ptr to std::shared_ptr
 *
 * \see pcl::make_shared
 */
using std::shared_ptr;

/**
 * \brief Force ADL for `weak_ptr`
 *
 * For ease of switching from boost::weak_ptr to std::weak_ptr
 */
using std::weak_ptr;

/** ADL doesn't work until C++20 for dynamic_pointer_cast since it requires an explicit Tparam */
using std::dynamic_pointer_cast;

/** ADL doesn't work until C++20 for static_pointer_cast since it requires an explicit Tparam */
using std::static_pointer_cast;

#ifdef DOXYGEN_ONLY

/**
 * \brief Returns a pcl::shared_ptr compliant with type T's allocation policy.
 *
 * std::allocate_shared or std::make_shared will be invoked in case T has or
 * doesn't have a custom allocator, respectively.
 *
 * \note In MSVC < 1915 (before version 15.8) alignment was incorrectly set at
 *   most at alignof(max_align_t). This bug was fixed in said version and is
 *   acknowledged by defining _ENABLE_EXTENDED_ALIGNED_STORAGE. See #3752.
 *
 * \see pcl::has_custom_allocator, PCL_MAKE_ALIGNED_OPERATOR_NEW
 * \tparam T Type of the object to create a pcl::shared_ptr of
 * \tparam Args Types for the arguments to pcl::make_shared
 * \param args List of arguments with which an instance of T will be constructed
 * \return pcl::shared_ptr of an instance of type T
 */
template<typename T, typename ... Args>
shared_ptr<T> make_shared(Args&&... args);

#else

template<typename T, typename ... Args>
std::enable_if_t<has_custom_allocator<T>::value, shared_ptr<T>> make_shared(Args&&... args)
{
  return std::allocate_shared<T>(Eigen::aligned_allocator<T>(), std::forward<Args> (args)...);
}

template<typename T, typename ... Args>
std::enable_if_t<!has_custom_allocator<T>::value, shared_ptr<T>> make_shared(Args&&... args)
{
  return std::make_shared<T>(std::forward<Args> (args)...);
}

#endif
}
