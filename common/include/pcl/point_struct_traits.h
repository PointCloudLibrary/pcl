/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <boost/mpl/assert.hpp>  // for BOOST_MPL_ASSERT_MSG
#include <boost/mpl/identity.hpp>  // for boost::mpl::identity

#include <boost/mpl/vector.hpp>  // for boost::mpl::vector
#include <boost/preprocessor/seq/enum.hpp>  // for BOOST_PP_SEQ_ENUM
#include <boost/preprocessor/tuple/elem.hpp>  // for BOOST_PP_TUPLE_ELEM
#include <boost/preprocessor/stringize.hpp> // for BOOST_PP_STRINGIZE

// This is required for the workaround at line 84
#ifdef _MSC_VER
#include <Eigen/Core>
#include <Eigen/src/StlSupport/details.h>
#endif

#include <cstddef>  // for std::size_t, offsetof
#include <cstdint>  // for std::int8_t, std::uint8_t, std::int16_t, std::uint16_t, std::int32_t, std::uint32_t
#include <type_traits>  // for std::is_same, std::remove_all_extents_t

namespace pcl
{
namespace traits
{

// forward declaration
template<typename T> struct asEnum;

// Metafunction to decompose a type (possibly of array of any number of dimensions) into
// its scalar type and total number of elements.
template<typename T> struct decomposeArray
{
  using type = std::remove_all_extents_t<T>;
  static const std::uint32_t value = sizeof (T) / sizeof (type);
};

// For non-POD point types, this is specialized to return the corresponding POD type.
template<typename PointT>
struct POD
{
  using type = PointT;
};

#ifdef _MSC_VER
/* Sometimes when calling functions like `copyPoint()` or `copyPointCloud`
 * without explicitly specifying point types, MSVC deduces them to be e.g.
 * `Eigen::internal::workaround_msvc_stl_support<pcl::PointXYZ>` instead of
 * plain `pcl::PointXYZ`. Subsequently these types are passed to meta-
 * functions like `has_field` or `fieldList` and make them choke. This hack
 * makes use of the fact that internally `fieldList` always applies `POD` to
 * its argument type. This specialization therefore allows to unwrap the
 * contained point type. */
template<typename PointT>
struct POD<Eigen::internal::workaround_msvc_stl_support<PointT> >
{
  using type = PointT;
};
#endif

// name
/* This really only depends on Tag, but we go through some gymnastics to avoid ODR violations.
   We template it on the point type PointT to avoid ODR violations when registering multiple
   point types with shared tags.
   The dummy parameter is so we can partially specialize name on PointT and Tag but leave it
   templated on dummy. Each specialization declares a static char array containing the tag
   name. The definition of the static member would conflict when linking multiple translation
   units that include the point type registration. But when the static member definition is
   templated (on dummy), we sidestep the ODR issue.
*/
template<class PointT, typename Tag, int dummy = 0>
struct name /** \cond NO_WARN_RECURSIVE */ : name<typename POD<PointT>::type, Tag, dummy> /** \endcond */
{ /** \cond NO_WARN_RECURSIVE */
  // Contents of specialization:
  // static const char value[];

  // Avoid infinite compile-time recursion
  BOOST_MPL_ASSERT_MSG((!std::is_same<PointT, typename POD<PointT>::type>::value),
                        POINT_TYPE_NOT_PROPERLY_REGISTERED, (PointT&));
};
} // namespace traits
} // namespace pcl

#define POINT_CLOUD_REGISTER_FIELD_NAME(r, point, elem)                   \
  template<int dummy>                                                     \
  struct name<point, pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem), dummy> \
  { /** \endcond */                                                       \
    static const char value[];                                            \
  };                                                                      \
                                                                          \
  template<int dummy>                                                     \
  const char name<point,                                                  \
                  pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem),           \
                  dummy>::value[] =                                       \
    BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(3, 2, elem));                  \


namespace pcl
{
namespace traits
{
// offset
template<class PointT, typename Tag>
struct offset /** \cond NO_WARN_RECURSIVE */ : offset<typename POD<PointT>::type, Tag> /** \endcond */
{ /** \cond NO_WARN_RECURSIVE */
  // Contents of specialization:
  // static const std::size_t value;

  // Avoid infinite compile-time recursion
  BOOST_MPL_ASSERT_MSG((!std::is_same<PointT, typename POD<PointT>::type>::value),
                        POINT_TYPE_NOT_PROPERLY_REGISTERED, (PointT&));
};
} // namespace traits
} // namespace pcl

#define POINT_CLOUD_REGISTER_FIELD_OFFSET(r, name, elem)                              \
  template<> struct offset<name, pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem)>        \
  { /** \endcond */                                                                   \
    static const std::size_t value = offsetof(name, BOOST_PP_TUPLE_ELEM(3, 1, elem)); \
  };                                                                                  \


namespace pcl
{
namespace traits
{
 // datatype
 template<class PointT, typename Tag>
 struct datatype /** \cond NO_WARN_RECURSIVE */ : datatype<typename POD<PointT>::type, Tag> /** \endcond */
 { /** \cond NO_WARN_RECURSIVE */
   // Contents of specialization:
   // using type = ...;
   // static const std::uint8_t value;
   // static const std::uint32_t size;

   // Avoid infinite compile-time recursion
   BOOST_MPL_ASSERT_MSG((!std::is_same<PointT, typename POD<PointT>::type>::value),
                        POINT_TYPE_NOT_PROPERLY_REGISTERED, (PointT&));
 };
 } // namespace traits
 } // namespace pcl

#define POINT_CLOUD_REGISTER_FIELD_DATATYPE(r, name, elem)                       \
  template<> struct datatype<name, pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem)> \
  { /** \endcond */                                                              \
    using type = boost::mpl::identity<BOOST_PP_TUPLE_ELEM(3, 0, elem)>::type;    \
    using decomposed = decomposeArray<type>;                                     \
    static const std::uint8_t value = asEnum<decomposed::type>::value;           \
    static const std::uint32_t size = decomposed::value;                         \
  };                                                                             \


namespace pcl
{
namespace traits
{
// fields
template<typename PointT>
struct fieldList /** \cond NO_WARN_RECURSIVE */ : fieldList<typename POD<PointT>::type> /** \endcond */
{ /** \cond NO_WARN_RECURSIVE */
  // Contents of specialization:
  // using type = boost::mpl::vector<...>;

  // Avoid infinite compile-time recursion
  BOOST_MPL_ASSERT_MSG((!std::is_same<PointT, typename POD<PointT>::type>::value),
                        POINT_TYPE_NOT_PROPERLY_REGISTERED, (PointT&));
};
} // namespace traits
} // namespace pcl

#define POINT_CLOUD_REGISTER_POINT_FIELD_LIST(name, seq)        \
  template<> struct fieldList<name>                             \
  { /** \endcond */                                             \
    using type = boost::mpl::vector<BOOST_PP_SEQ_ENUM(seq)>;    \
  };
