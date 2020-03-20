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

#include <boost/make_shared.hpp>  // for boost::allocate_shared, boost::make_shared
#include <boost/pointer_cast.hpp>  // for boost::dynamic_pointer_cast, boost::static_pointer_cast
#include <boost/smart_ptr/shared_ptr.hpp>  // for boost::shared_ptr
#include <boost/weak_ptr.hpp>  // for boost::weak_ptr

#include <Eigen/Core>  // for EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#include <type_traits>  // for std::enable_if_t, std::false_type, std::true_type
#include <utility>  // for std::forward

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
using boost::shared_ptr;

/** ADL doesn't work until C++20 for dynamic_pointer_cast since it requires an explicit Tparam */
using boost::dynamic_pointer_cast;

/** ADL doesn't work until C++20 for static_pointer_cast since it requires an explicit Tparam */
using boost::static_pointer_cast;

/**
 * \brief Force ADL for `weak_ptr`
 *
 * For ease of switching from boost::weak_ptr to std::weak_ptr
 */
using boost::weak_ptr;


#ifdef DOXYGEN_ONLY

/**
 * \brief Returns a pcl::shared_ptr compliant with type T's allocation policy.
 *
 * boost::allocate_shared or boost::make_shared will be invoked in case T has or
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
  return boost::allocate_shared<T>(Eigen::aligned_allocator<T>(), std::forward<Args> (args)...);
}

template<typename T, typename ... Args>
std::enable_if_t<!has_custom_allocator<T>::value, shared_ptr<T>> make_shared(Args&&... args)
{
  return boost::make_shared<T>(std::forward<Args> (args)...);
}

#endif
}
