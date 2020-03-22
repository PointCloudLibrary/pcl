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



#include <boost/make_shared.hpp>  // for boost::allocate_shared, boost::make_shared
#include <boost/pointer_cast.hpp>  // for boost::dynamic_pointer_cast, boost::static_pointer_cast
#include <boost/smart_ptr/shared_ptr.hpp>  // for boost::shared_ptr

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
 * \brief Alias for boost::shared_ptr
 *
 * For ease of switching from boost::shared_ptr to std::shared_ptr
 *
 * \see pcl::make_shared
 * \tparam T Type of the object stored inside the shared_ptr
 */
template <typename T>
using shared_ptr = boost::shared_ptr<T>;


template <typename ...> using void_t = void; // part of std in c++17

#ifdef DOXYGEN_ONLY

/**
 * \brief Tests at compile time if type T has a custom allocator
 *
 * \see pcl::make_shared, PCL_MAKE_ALIGNED_OPERATOR_NEW
 * \tparam T Type of the object to test
 */
template <typename T> struct has_custom_allocator;

#else

template <typename, typename = void_t<>> struct has_custom_allocator : std::false_type {};
template <typename T> struct has_custom_allocator<T, void_t<typename T::_custom_allocator_type_trait>> : std::true_type {};

#endif


#ifdef DOXYGEN_ONLY

/**
 * \brief Returns a pcl::shared_ptr compliant with type T's allocation policy.
 *
 * boost::allocate_shared or boost::make_shared will be invoked in case T has or
 * doesn't have a custom allocator, respectively.
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
