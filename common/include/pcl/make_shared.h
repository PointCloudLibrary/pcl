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
 *
 */

#pragma once


#include <type_traits>
#include <utility>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <pcl/point_traits.h>


namespace pcl
{

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

} // namespace pcl
