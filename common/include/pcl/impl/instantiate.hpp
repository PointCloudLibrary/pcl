/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 */
#ifndef PCL_IMPL_INSTANTIATE_H_
#define PCL_IMPL_INSTANTIATE_H_

#ifdef __GNUC__
#pragma GCC system_header 
#endif

#include <pcl/pcl_config.h>

//#define PCL_POINT_TYPES (bool)(int)(float)(double)
//#define PCL_TEMPLATES (Type)(Othertype)

//
// PCL_INSTANTIATE: call to instantiate template TEMPLATE for all
// POINT_TYPES
//

#ifdef PCL_NO_PRECOMPILE

#define PCL_INSTANTIATE_PRODUCT_IMPL(r, product)
#define PCL_INSTANTIATE_IMPL(r, TEMPLATE, POINT_TYPE) 
#define PCL_INSTANTIATE(TEMPLATE, POINT_TYPES)
#define PCL_INSTANTIATE_PRODUCT_IMPL(r, product)
#define PCL_INSTANTIATE_PRODUCT(TEMPLATE, PRODUCT)

#else

#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/for_each_product.hpp>
#include <boost/preprocessor/seq/to_tuple.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/expand.hpp>

#define PCL_INSTANTIATE_IMPL(r, TEMPLATE, POINT_TYPE) \
  BOOST_PP_CAT(PCL_INSTANTIATE_, TEMPLATE)(POINT_TYPE)

#define PCL_INSTANTIATE(TEMPLATE, POINT_TYPES)        \
  BOOST_PP_SEQ_FOR_EACH(PCL_INSTANTIATE_IMPL, TEMPLATE, POINT_TYPES)


//
// PCL_INSTANTIATE_PRODUCT(templatename, (seq1)(seq2)...(seqN))
//
// instantiate templates
//
// A call to PCL_INSTANTIATE_PRODUCT(T, ((a)(b)) ((d)(e)) ) results in calls
//
//   PCL_INSTANTIATE_T(a, d) 
//   PCL_INSTANTIATE_T(a, e) 
//   PCL_INSTANTIATE_T(b, d) 
//   PCL_INSTANTIATE_T(b, e) 
//
// That is, PCL_INSTANTIATE_T is called for the cartesian product of the sequences seq1 ... seqN
//
// BE CAREFUL WITH YOUR PARENTHESIS!  The argument PRODUCT is a
// sequence of sequences.  e.g. if it were three sequences of,
// 1. letters, 2. numbers, and 3. our favorite transylvanians, it
// would be
//
//    ((x)(y)(z))((1)(2)(3))((dracula)(radu))
//
#ifdef _MSC_VER
#define PCL_INSTANTIATE_PRODUCT_IMPL(r, product) \
  BOOST_PP_CAT(PCL_INSTANTIATE_, BOOST_PP_SEQ_HEAD(product)) \
          BOOST_PP_EXPAND(BOOST_PP_SEQ_TO_TUPLE(BOOST_PP_SEQ_TAIL(product))) 
#else
#define PCL_INSTANTIATE_PRODUCT_IMPL(r, product) \
  BOOST_PP_EXPAND(BOOST_PP_CAT(PCL_INSTANTIATE_, BOOST_PP_SEQ_HEAD(product)) \
		  BOOST_PP_SEQ_TO_TUPLE(BOOST_PP_SEQ_TAIL(product)))
#endif


#define PCL_INSTANTIATE_PRODUCT(TEMPLATE, PRODUCT) \
  BOOST_PP_SEQ_FOR_EACH_PRODUCT(PCL_INSTANTIATE_PRODUCT_IMPL, ((TEMPLATE))PRODUCT)

#endif

#endif
