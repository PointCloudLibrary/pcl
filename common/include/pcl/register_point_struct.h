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
 *
 * $Id$
 *
 */

#pragma once

#ifdef __GNUC__
#pragma GCC system_header
#endif

#if defined _MSC_VER
  #pragma warning (push, 2)
  // 4244 : conversion from 'type1' to 'type2', possible loss of data
  #pragma warning (disable: 4244)
#endif

#include <pcl/point_struct_traits.h> // for pcl::traits::POD, POINT_CLOUD_REGISTER_FIELD_(NAME, OFFSET, DATATYPE), POINT_CLOUD_REGISTER_POINT_FIELD_LIST
#include <boost/mpl/assert.hpp>  // for BOOST_MPL_ASSERT_MSG
#include <boost/preprocessor/seq/for_each.hpp>  // for BOOST_PP_SEQ_FOR_EACH
#include <boost/preprocessor/seq/transform.hpp>  // for BOOST_PP_SEQ_TRANSFORM
#include <boost/preprocessor/tuple/elem.hpp>  // for BOOST_PP_TUPLE_ELEM
#include <boost/preprocessor/cat.hpp>  // for BOOST_PP_CAT

#include <cstdint>  // for std::uint32_t
#include <type_traits>  // for std::enable_if_t, std::is_array, std::remove_const_t, std::remove_all_extents_t

// Must be used in global namespace with name fully qualified
#define POINT_CLOUD_REGISTER_POINT_STRUCT(name, fseq)               \
  POINT_CLOUD_REGISTER_POINT_STRUCT_I(name,                         \
    BOOST_PP_CAT(POINT_CLOUD_REGISTER_POINT_STRUCT_X fseq, 0))
  /***/

#define POINT_CLOUD_REGISTER_POINT_WRAPPER(wrapper, pod)    \
  BOOST_MPL_ASSERT_MSG(sizeof(wrapper) == sizeof(pod), POINT_WRAPPER_AND_POD_TYPES_HAVE_DIFFERENT_SIZES, (wrapper&, pod&)); \
  namespace pcl {                                           \
    namespace traits {                                      \
      template<> struct POD<wrapper> { using type = pod; }; \
    }                                                       \
  }
  /***/

// These macros help transform the unusual data structure (type, name, tag)(type, name, tag)...
// into a proper preprocessor sequence of 3-tuples ((type, name, tag))((type, name, tag))...
#define POINT_CLOUD_REGISTER_POINT_STRUCT_X(type, name, tag)            \
  ((type, name, tag)) POINT_CLOUD_REGISTER_POINT_STRUCT_Y
#define POINT_CLOUD_REGISTER_POINT_STRUCT_Y(type, name, tag)            \
  ((type, name, tag)) POINT_CLOUD_REGISTER_POINT_STRUCT_X
#define POINT_CLOUD_REGISTER_POINT_STRUCT_X0
#define POINT_CLOUD_REGISTER_POINT_STRUCT_Y0

namespace pcl
{
  namespace traits
  {
    template<typename T> inline
    std::enable_if_t<!std::is_array<T>::value>
    plus (T &l, const T &r)
    {
      l += r;
    }

    template<typename T> inline
    std::enable_if_t<std::is_array<T>::value>
    plus (std::remove_const_t<T> &l, const T &r)
    {
      using type = std::remove_all_extents_t<T>;
      static const std::uint32_t count = sizeof (T) / sizeof (type);
      for (std::uint32_t i = 0; i < count; ++i)
        l[i] += r[i];
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<!std::is_array<T1>::value>
    plusscalar (T1 &p, const T2 &scalar)
    {
      p += scalar;
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<std::is_array<T1>::value>
    plusscalar (T1 &p, const T2 &scalar)
    {
      using type = std::remove_all_extents_t<T1>;
      static const std::uint32_t count = sizeof (T1) / sizeof (type);
      for (std::uint32_t i = 0; i < count; ++i)
        p[i] += scalar;
    }

    template<typename T> inline
    std::enable_if_t<!std::is_array<T>::value>
    minus (T &l, const T &r)
    {
      l -= r;
    }

    template<typename T> inline
    std::enable_if_t<std::is_array<T>::value>
    minus (std::remove_const_t<T> &l, const T &r)
    {
      using type = std::remove_all_extents_t<T>;
      static const std::uint32_t count = sizeof (T) / sizeof (type);
      for (std::uint32_t i = 0; i < count; ++i)
        l[i] -= r[i];
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<!std::is_array<T1>::value>
    minusscalar (T1 &p, const T2 &scalar)
    {
      p -= scalar;
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<std::is_array<T1>::value>
    minusscalar (T1 &p, const T2 &scalar)
    {
      using type = std::remove_all_extents_t<T1>;
      static const std::uint32_t count = sizeof (T1) / sizeof (type);
      for (std::uint32_t i = 0; i < count; ++i)
        p[i] -= scalar;
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<!std::is_array<T1>::value>
    mulscalar (T1 &p, const T2 &scalar)
    {
      p *= scalar;
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<std::is_array<T1>::value>
    mulscalar (T1 &p, const T2 &scalar)
    {
      using type = std::remove_all_extents_t<T1>;
      static const std::uint32_t count = sizeof (T1) / sizeof (type);
      for (std::uint32_t i = 0; i < count; ++i)
        p[i] *= scalar;
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<!std::is_array<T1>::value>
    divscalar (T1 &p, const T2 &scalar)
    {
      p /= scalar;
    }

    template<typename T1, typename T2> inline
    std::enable_if_t<std::is_array<T1>::value>
    divscalar (T1 &p, const T2 &scalar)
    {
      using type = std::remove_all_extents_t<T1>;
      static const std::uint32_t count = sizeof (T1) / sizeof (type);
      for (std::uint32_t i = 0; i < count; ++i)
        p[i] /= scalar;
    }

    template<typename NoArrayT, typename ScalarT> inline
    std::enable_if_t<!std::is_array<NoArrayT>::value>
    divscalar2 (NoArrayT &p, const ScalarT &scalar)
    {
      p = scalar / p;
    }

    template<typename ArrayT, typename ScalarT> inline
    std::enable_if_t<std::is_array<ArrayT>::value>
    divscalar2 (ArrayT &p, const ScalarT &scalar)
    {
      using type = std::remove_all_extents_t<ArrayT>;
      static const std::uint32_t count = sizeof (ArrayT) / sizeof (type);
      for (std::uint32_t i = 0; i < count; ++i)
        p[i] = scalar / p[i];
    }
  }
}

// Point operators
#define PCL_PLUSEQ_POINT_TAG(r, data, elem)                \
  pcl::traits::plus (lhs.BOOST_PP_TUPLE_ELEM(3, 1, elem),  \
                     rhs.BOOST_PP_TUPLE_ELEM(3, 1, elem));
  /***/

#define PCL_PLUSEQSC_POINT_TAG(r, data, elem)                 \
  pcl::traits::plusscalar (p.BOOST_PP_TUPLE_ELEM(3, 1, elem), \
                           scalar);
  /***/
   //p.BOOST_PP_TUPLE_ELEM(3, 1, elem) += scalar;

#define PCL_MINUSEQ_POINT_TAG(r, data, elem)                \
  pcl::traits::minus (lhs.BOOST_PP_TUPLE_ELEM(3, 1, elem),  \
                      rhs.BOOST_PP_TUPLE_ELEM(3, 1, elem));
  /***/

#define PCL_MINUSEQSC_POINT_TAG(r, data, elem)                 \
  pcl::traits::minusscalar (p.BOOST_PP_TUPLE_ELEM(3, 1, elem), \
                            scalar);
  /***/
   //p.BOOST_PP_TUPLE_ELEM(3, 1, elem) -= scalar;

#define PCL_MULEQSC_POINT_TAG(r, data, elem)                 \
  pcl::traits::mulscalar (p.BOOST_PP_TUPLE_ELEM(3, 1, elem), \
                            scalar);
  /***/

#define PCL_DIVEQSC_POINT_TAG(r, data, elem)   \
  pcl::traits::divscalar (p.BOOST_PP_TUPLE_ELEM(3, 1, elem), \
                            scalar);
  /***/

#define PCL_DIVEQSC2_POINT_TAG(r, data, elem)   \
  pcl::traits::divscalar2 (p.BOOST_PP_TUPLE_ELEM(3, 1, elem), \
                             scalar);
  /***/

// Construct type traits given full sequence of (type, name, tag) triples
//  BOOST_MPL_ASSERT_MSG(std::is_pod<name>::value,
//                       REGISTERED_POINT_TYPE_MUST_BE_PLAIN_OLD_DATA, (name));
#define POINT_CLOUD_REGISTER_POINT_STRUCT_I(name, seq)                           \
  namespace pcl                                                                  \
  {                                                                              \
    namespace fields                                                             \
    {                                                                            \
      BOOST_PP_SEQ_FOR_EACH(POINT_CLOUD_REGISTER_FIELD_TAG, name, seq)           \
    }                                                                            \
    namespace traits                                                             \
    {                                                                            \
      BOOST_PP_SEQ_FOR_EACH(POINT_CLOUD_REGISTER_FIELD_NAME, name, seq)          \
      BOOST_PP_SEQ_FOR_EACH(POINT_CLOUD_REGISTER_FIELD_OFFSET, name, seq)        \
      BOOST_PP_SEQ_FOR_EACH(POINT_CLOUD_REGISTER_FIELD_DATATYPE, name, seq)      \
      POINT_CLOUD_REGISTER_POINT_FIELD_LIST(name, POINT_CLOUD_EXTRACT_TAGS(seq)) \
    }                                                                            \
    namespace common                                           \
    {                                                          \
      inline const name&                                       \
      operator+= (name& lhs, const name& rhs)                  \
      {                                                        \
        BOOST_PP_SEQ_FOR_EACH(PCL_PLUSEQ_POINT_TAG, _, seq)    \
        return (lhs);                                          \
      }                                                        \
      inline const name&                                       \
      operator+= (name& p, const float& scalar)                \
      {                                                        \
        BOOST_PP_SEQ_FOR_EACH(PCL_PLUSEQSC_POINT_TAG, _, seq)  \
        return (p);                                            \
      }                                                        \
      inline const name operator+ (const name& lhs, const name& rhs)   \
      { name result = lhs; result += rhs; return (result); }           \
      inline const name operator+ (const float& scalar, const name& p) \
      { name result = p; result += scalar; return (result); }          \
      inline const name operator+ (const name& p, const float& scalar) \
      { name result = p; result += scalar; return (result); }          \
      inline const name&                                       \
      operator*= (name& p, const float& scalar)                \
      {                                                        \
        BOOST_PP_SEQ_FOR_EACH(PCL_MULEQSC_POINT_TAG, _, seq)   \
        return (p);                                            \
      }                                                        \
      inline const name operator* (const float& scalar, const name& p) \
      { name result = p; result *= scalar; return (result); }          \
      inline const name operator* (const name& p, const float& scalar) \
      { name result = p; result *= scalar; return (result); }          \
      inline const name&                                       \
      operator-= (name& lhs, const name& rhs)                  \
      {                                                        \
        BOOST_PP_SEQ_FOR_EACH(PCL_MINUSEQ_POINT_TAG, _, seq)   \
        return (lhs);                                          \
      }                                                        \
      inline const name&                                       \
      operator-= (name& p, const float& scalar)                \
      {                                                        \
        BOOST_PP_SEQ_FOR_EACH(PCL_MINUSEQSC_POINT_TAG, _, seq) \
        return (p);                                            \
      }                                                        \
      inline const name operator- (const name& lhs, const name& rhs)   \
      { name result = lhs; result -= rhs; return (result); }           \
      inline const name operator- (const float& scalar, const name& p) \
      { name result = p; result *= -1.0f; result += scalar; return (result); } \
      inline const name operator- (const name& p, const float& scalar) \
      { name result = p; result -= scalar; return (result); }          \
      inline const name&                                       \
      operator/= (name& p, const float& scalar)                \
      {                                                        \
        BOOST_PP_SEQ_FOR_EACH(PCL_DIVEQSC_POINT_TAG, _, seq)   \
        return (p);                                            \
      }                                                        \
      inline const name operator/ (const float& scalar, const name& p_in) \
      { name p = p_in; BOOST_PP_SEQ_FOR_EACH(PCL_DIVEQSC2_POINT_TAG, _, seq) \
        return (p); } \
      inline const name operator/ (const name& p, const float& scalar) \
      { name result = p; result /= scalar; return (result); }          \
    }                                                          \
  }
  /***/

#define POINT_CLOUD_REGISTER_FIELD_TAG(r, name, elem)   \
  struct BOOST_PP_TUPLE_ELEM(3, 2, elem);               \
  /***/

#define POINT_CLOUD_TAG_OP(s, data, elem) pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem)

#define POINT_CLOUD_EXTRACT_TAGS(seq) BOOST_PP_SEQ_TRANSFORM(POINT_CLOUD_TAG_OP, _, seq)

#if defined _MSC_VER
  #pragma warning (pop)
#endif
