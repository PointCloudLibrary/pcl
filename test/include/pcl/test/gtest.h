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

#include <gtest/gtest.h>

/**
 * \file pcl/test/gtest.h
 *
 * \brief Defines all the PCL test macros used
 * \ingroup test
 */

/**
 * \brief Macro choose between TYPED_TEST_CASE and TYPED_TEST_SUITE depending on the GTest version
 *
 * \ingroup test
 */
#if !defined(TYPED_TEST_SUITE)
  #define TYPED_TEST_SUITE TYPED_TEST_CASE
#endif

/**
 * \brief Macro choose between TYPED_TEST_CASE_P and TYPED_TEST_SUITE_P depending on the GTest version
 *
 * \ingroup test
 */
#if !defined(TYPED_TEST_SUITE_P)
  #define TYPED_TEST_SUITE_P TYPED_TEST_CASE_P
#endif

/**
 * \brief Macro choose between INSTANTIATE_TEST_CASE_P and INSTANTIATE_TEST_SUITE_P depending on the GTest version
 *
 * \ingroup test
 */
#if !defined(INSTANTIATE_TEST_SUITE_P)
  #define INSTANTIATE_TEST_SUITE_P INSTANTIATE_TEST_CASE_P
#endif

/**
 * \brief Macro choose between INSTANTIATE_TYPED_TEST_CASE_P and INSTANTIATE_TYPED_TEST_SUITE_P depending on the GTest version
 *
 * \ingroup test
 */
#if !defined(INSTANTIATE_TYPED_TEST_SUITE_P)
  #define INSTANTIATE_TYPED_TEST_SUITE_P INSTANTIATE_TYPED_TEST_CASE_P
#endif

/**
 * \brief Macro choose between REGISTER_TYPED_TEST_CASE_P and REGISTER_TYPED_TEST_SUITE_P depending on the GTest version
 *
 * \ingroup test
 */
#if !defined(REGISTER_TYPED_TEST_SUITE_P)
  #define REGISTER_TYPED_TEST_SUITE_P REGISTER_TYPED_TEST_CASE_P
#endif

/**
 * \brief Macro choose between compile-time and run-time tests depending on the value of PCL_RUN_TESTS_AT_COMPILE_TIME
 *
 * \ingroup test
 */
#if PCL_RUN_TESTS_AT_COMPILE_TIME == true
  #define PCL_CONSTEXPR constexpr
  #define PCL_EXPECT_TRUE(...) \
    static_assert(__VA_ARGS__, #__VA_ARGS__)
  #define PCL_EXPECT_FLOAT_EQ(val1, val2) \
    static_assert((val1) == (val2), "")
  #define PCL_EXPECT_INT_EQ(val1, val2) \
    static_assert((val1) == (val2), "")
#else
  #define PCL_CONSTEXPR
  #define PCL_EXPECT_TRUE(...) \
    EXPECT_TRUE(__VA_ARGS__) << (#__VA_ARGS__);
  #define PCL_EXPECT_FLOAT_EQ(val1, val2) \
    EXPECT_FLOAT_EQ((val1), (val2))
  #define PCL_EXPECT_INT_EQ(val1, val2) \
    EXPECT_EQ((val1), (val2))
#endif
