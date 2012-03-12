/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef PCL_REGISTER_POINT_STRUCT_H_
#define PCL_REGISTER_POINT_STRUCT_H_

#include <pcl/point_traits.h>

#include <boost/mpl/vector.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/transform.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/repetition/repeat_from_to.hpp>
#include <boost/type_traits/is_pod.hpp>
#include <stddef.h> //offsetof

// Must be used in global namespace with name fully qualified
#define POINT_CLOUD_REGISTER_POINT_STRUCT(name, fseq)               \
  POINT_CLOUD_REGISTER_POINT_STRUCT_I(name,                         \
    BOOST_PP_CAT(POINT_CLOUD_REGISTER_POINT_STRUCT_X fseq, 0))      \
  /***/

#define POINT_CLOUD_REGISTER_POINT_WRAPPER(wrapper, pod)    \
  BOOST_MPL_ASSERT_MSG(sizeof(wrapper) == sizeof(pod), POINT_WRAPPER_AND_POD_TYPES_HAVE_DIFFERENT_SIZES, (wrapper&, pod&)); \
  namespace pcl {                                           \
    namespace traits {                                      \
      template<> struct POD<wrapper> { typedef pod type; }; \
    }                                                       \
  }                                                         \
  /***/

// These macros help transform the unusual data structure (type, name, tag)(type, name, tag)...
// into a proper preprocessor sequence of 3-tuples ((type, name, tag))((type, name, tag))...
#define POINT_CLOUD_REGISTER_POINT_STRUCT_X(type, name, tag)            \
  ((type, name, tag)) POINT_CLOUD_REGISTER_POINT_STRUCT_Y
#define POINT_CLOUD_REGISTER_POINT_STRUCT_Y(type, name, tag)            \
  ((type, name, tag)) POINT_CLOUD_REGISTER_POINT_STRUCT_X
#define POINT_CLOUD_REGISTER_POINT_STRUCT_X0
#define POINT_CLOUD_REGISTER_POINT_STRUCT_Y0

// Construct type traits given full sequence of (type, name, tag) triples
//  BOOST_MPL_ASSERT_MSG(boost::is_pod<name>::value,                    
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
  }                                                                              \
  /***/

#define POINT_CLOUD_REGISTER_FIELD_TAG(r, name, elem)   \
  struct BOOST_PP_TUPLE_ELEM(3, 2, elem);               \
  /***/

#define POINT_CLOUD_REGISTER_FIELD_NAME(r, point, elem)                 \
  template<int dummy>                                                   \
  struct name<point, pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem), dummy> \
  {                                                                     \
    static const char value[];                                          \
  };                                                                    \
                                                                        \
  template<int dummy>                                                   \
  const char name<point,                                                \
                  pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem),         \
                  dummy>::value[] =                                     \
    BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(3, 2, elem));                \
  /***/

#define POINT_CLOUD_REGISTER_FIELD_OFFSET(r, name, elem)                \
  template<> struct offset<name, pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem)> \
  {                                                                     \
    static const size_t value = offsetof(name, BOOST_PP_TUPLE_ELEM(3, 1, elem)); \
  };                                                                    \
  /***/

// \note: the mpl::identity weirdness is to support array types without requiring the
// user to wrap them. The basic problem is:
// typedef float[81] type; // SYNTAX ERROR!
// typedef float type[81]; // OK, can now use "type" as a synonym for float[81]
#define POINT_CLOUD_REGISTER_FIELD_DATATYPE(r, name, elem)              \
  template<> struct datatype<name, pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem)> \
  {                                                                     \
    typedef boost::mpl::identity<BOOST_PP_TUPLE_ELEM(3, 0, elem)>::type type; \
    typedef decomposeArray<type> decomposed;                            \
    static const uint8_t value = asEnum<decomposed::type>::value;       \
    static const uint32_t size = decomposed::value;                     \
  };                                                                    \
  /***/

#define POINT_CLOUD_TAG_OP(s, data, elem) pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem)

#define POINT_CLOUD_EXTRACT_TAGS(seq) BOOST_PP_SEQ_TRANSFORM(POINT_CLOUD_TAG_OP, _, seq)

#define POINT_CLOUD_REGISTER_POINT_FIELD_LIST(name, seq)        \
  template<> struct fieldList<name>                             \
  {                                                             \
    typedef boost::mpl::vector<BOOST_PP_SEQ_ENUM(seq)> type;    \
  };                                                            \
  /***/

// Disabling barely-used Fusion registration of point types for now.
#if 0
#define POINT_CLOUD_EXPAND_TAG_OP(s, data, elem)                \
  (boost::mpl::identity<BOOST_PP_TUPLE_ELEM(3, 0, elem)>::type, \
   BOOST_PP_TUPLE_ELEM(3, 1, elem),                             \
   pcl::fields::BOOST_PP_TUPLE_ELEM(3, 2, elem))                \
  /***/

#define POINT_CLOUD_EXPAND_TAGS(seq) BOOST_PP_SEQ_TRANSFORM(POINT_CLOUD_EXPAND_TAG_OP, _, seq)

#define POINT_CLOUD_REGISTER_WITH_FUSION(name, seq)                     \
  BOOST_FUSION_ADAPT_ASSOC_STRUCT_I(name, POINT_CLOUD_EXPAND_TAGS(seq)) \
  /***/
#endif

#endif  //#ifndef PCL_REGISTER_POINT_STRUCT_H_
