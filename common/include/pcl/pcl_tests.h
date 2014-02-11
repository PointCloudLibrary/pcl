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
 */


#ifndef PCL_TEST_MACROS
#define PCL_TEST_MACROS

#include <Eigen/Core>

/** \file pcl_tests.h
  * Helper macros for testing equality of various data fields in PCL points */

namespace pcl
{

  /** test_macros.h provide helper macros for testing vectors, matrices etc.
    * We took some liberty with upcasing names to make them look like googletest
    * macros names so that reader is not confused.
    *
    * This file also provides a family of googletest-style macros for asserting
    * equality or nearness of xyz, normal, and rgba fields.
    *
    * \author Nizar Sallem, Sergey Alexandrov
    */

  namespace test
  {

    template <typename V1, typename V2>
    void EXPECT_EQ_VECTORS (const V1& v1, const V2& v2)
    {
      SCOPED_TRACE("EXPECT_EQ_VECTORS failed");
      EXPECT_EQ (v1.size (), v2.size ());
      size_t length = v1.size ();
      for (size_t i = 0; i < length; ++i)
        EXPECT_EQ (v1[i], v2[i]);
    }

    template <typename V1, typename V2, typename Scalar>
    void EXPECT_NEAR_VECTORS (const V1& v1, const V2& v2, const Scalar& epsilon)
    {
      SCOPED_TRACE("EXPECT_NEAR_VECTORS failed");
      EXPECT_EQ (v1.size (), v2.size ());
      size_t length = v1.size ();
      for (size_t i = 0; i < length; ++i)
        EXPECT_NEAR (v1[i], v2[i], epsilon);
    }

    namespace internal
    {

      template <typename Point1T, typename Point2T>
      ::testing::AssertionResult XYZEQ (const char* expr1,
                                        const char* expr2,
                                        const Point1T& p1,
                                        const Point2T& p2)
      {
        if ((p1).getVector3fMap ().cwiseEqual ((p2).getVector3fMap ()).all ())
          return ::testing::AssertionSuccess ();
        return ::testing::AssertionFailure ()
               << "Value of: " << expr2 << ".getVector3fMap ()" << std::endl
               << "  Actual: " << p2.getVector3fMap ().transpose () << std::endl
               << "Expected: " << expr1 << ".getVector3fMap ()" << std::endl
               << "Which is: " << p1.getVector3fMap ().transpose ();
      }

      template <typename Point1T, typename Point2T>
      ::testing::AssertionResult XYZNear (const char* expr1,
                                          const char* expr2,
                                          const char* abs_error_expr,
                                          const Point1T& p1,
                                          const Point2T& p2,
                                          double abs_error)
      {
        const Eigen::Vector3f diff = ((p1).getVector3fMap () -
                                      (p2).getVector3fMap ()).cwiseAbs ();
        if ((diff.array () < abs_error).all ())
          return ::testing::AssertionSuccess ();
        return ::testing::AssertionFailure ()
               << "Some of the element-wise differences exceed " << abs_error_expr
               << " (which evaluates to " << abs_error << ")" << std::endl
               << "Difference: " << diff.transpose () << std::endl
               << "  Value of: " << expr2 << ".getVector3fMap ()" << std::endl
               << "    Actual: " << p2.getVector3fMap ().transpose () << std::endl
               << "  Expected: " << expr1 << ".getVector3fMap ()" << std::endl
               << "  Which is: " << p1.getVector3fMap ().transpose ();
      }

      template <typename Point1T, typename Point2T>
      ::testing::AssertionResult NormalEQ (const char* expr1,
                                           const char* expr2,
                                           const Point1T& p1,
                                           const Point2T& p2)
      {
        if ((p1).getNormalVector3fMap ().cwiseEqual ((p2).getNormalVector3fMap ()).all ())
          return ::testing::AssertionSuccess ();
        return ::testing::AssertionFailure ()
               << "Value of: " << expr2 << ".getNormalVector3fMap ()" << std::endl
               << "  Actual: " << p2.getNormalVector3fMap ().transpose () << std::endl
               << "Expected: " << expr1 << ".getNormalVector3fMap ()" << std::endl
               << "Which is: " << p1.getNormalVector3fMap ().transpose ();
      }

      template <typename Point1T, typename Point2T>
      ::testing::AssertionResult NormalNear (const char* expr1,
                                             const char* expr2,
                                             const char* abs_error_expr,
                                             const Point1T& p1,
                                             const Point2T& p2,
                                             double abs_error)
      {
        const Eigen::Vector3f diff = ((p1).getNormalVector3fMap () -
                                      (p2).getNormalVector3fMap ()).cwiseAbs ();
        if ((diff.array () < abs_error).all ())
          return ::testing::AssertionSuccess ();
        return ::testing::AssertionFailure ()
               << "Some of the element-wise differences exceed " << abs_error_expr
               << " (which evaluates to " << abs_error << ")" << std::endl
               << "Difference: " << diff.transpose () << std::endl
               << "  Value of: " << expr2 << ".getNormalVector3fMap ()" << std::endl
               << "    Actual: " << p2.getNormalVector3fMap ().transpose () << std::endl
               << "  Expected: " << expr1 << ".getNormalVector3fMap ()" << std::endl
               << "  Which is: " << p1.getNormalVector3fMap ().transpose ();
      }

      template <typename Point1T, typename Point2T>
      ::testing::AssertionResult RGBEQ (const char* expr1,
                                         const char* expr2,
                                         const Point1T& p1,
                                         const Point2T& p2)
      {
        if ((p1).getRGBVector3i ().cwiseEqual ((p2).getRGBVector3i ()).all ())
          return ::testing::AssertionSuccess ();
        return ::testing::AssertionFailure ()
               << "Value of: " << expr2 << ".getRGBVector3i ()" << std::endl
               << "  Actual: " << p2.getRGBVector3i ().transpose () << std::endl
               << "Expected: " << expr1 << ".getRGBVector3i ()" << std::endl
               << "Which is: " << p1.getRGBVector3i ().transpose ();
      }

      template <typename Point1T, typename Point2T>
      ::testing::AssertionResult RGBAEQ (const char* expr1,
                                         const char* expr2,
                                         const Point1T& p1,
                                         const Point2T& p2)
      {
        if ((p1).getRGBAVector4i ().cwiseEqual ((p2).getRGBAVector4i ()).all ())
          return ::testing::AssertionSuccess ();
        return ::testing::AssertionFailure ()
               << "Value of: " << expr2 << ".getRGBAVector4i ()" << std::endl
               << "  Actual: " << p2.getRGBAVector4i ().transpose () << std::endl
               << "Expected: " << expr1 << ".getRGBAVector4i ()" << std::endl
               << "Which is: " << p1.getRGBAVector4i ().transpose ();
      }

    }

  }

}

/// Expect that each of x, y, and z fields are equal in
/// two points.
#define EXPECT_XYZ_EQ(expected, actual)                  \
  EXPECT_PRED_FORMAT2(::pcl::test::internal::XYZEQ,      \
                      (expected), (actual))

/// Assert that each of x, y, and z fields are equal in
/// two points.
#define ASSERT_XYZ_EQ(expected, actual)                  \
  ASSERT_PRED_FORMAT2(::pcl::test::internal::XYZEQ,      \
                      (expected), (actual))

/// Expect that differences between x, y, and z fields in
/// two points are each within abs_error.
#define EXPECT_XYZ_NEAR(expected, actual, abs_error)     \
  EXPECT_PRED_FORMAT3(::pcl::test::internal::XYZNear,    \
                      (expected), (actual), abs_error)

/// Assert that differences between x, y, and z fields in
/// two points are each within abs_error.
#define ASSERT_XYZ_NEAR(expected, actual, abs_error)     \
  EXPECT_PRED_FORMAT3(::pcl::test::internal::XYZNear,    \
                      (expected), (actual), abs_error)

/// Expect that each of normal_x, normal_y, and normal_z
/// fields are equal in two points.
#define EXPECT_NORMAL_EQ(expected, actual)               \
  EXPECT_PRED_FORMAT2(::pcl::test::internal::NormalEQ,   \
                      (expected), (actual))

/// Assert that each of normal_x, normal_y, and normal_z
/// fields are equal in two points.
#define ASSERT_NORMAL_EQ(expected, actual)               \
  ASSERT_PRED_FORMAT2(::pcl::test::internal::NormalEQ,   \
                      (expected), (actual))

/// Expect that differences between normal_x, normal_y,
/// and normal_z fields in two points are each within
/// abs_error.
#define EXPECT_NORMAL_NEAR(expected, actual, abs_error)  \
  EXPECT_PRED_FORMAT3(::pcl::test::internal::NormalNear, \
                      (expected), (actual), abs_error)

/// Assert that differences between normal_x, normal_y,
/// and normal_z fields in two points are each within
/// abs_error.
#define ASSERT_NORMAL_NEAR(expected, actual, abs_error)  \
  EXPECT_PRED_FORMAT3(::pcl::test::internal::NormalNear, \
                      (expected), (actual), abs_error)

/// Expect that each of r, g, and b fields are equal in
/// two points.
#define EXPECT_RGB_EQ(expected, actual)                  \
  EXPECT_PRED_FORMAT2(::pcl::test::internal::RGBEQ,      \
                      (expected), (actual))

/// Assert that each of r, g, and b fields are equal in
/// two points.
#define ASSERT_RGB_EQ(expected, actual)                  \
  ASSERT_PRED_FORMAT2(::pcl::test::internal::RGBEQ,      \
                      (expected), (actual))

/// Expect that each of r, g, b, and a fields are equal
/// in two points.
#define EXPECT_RGBA_EQ(expected, actual)                 \
  EXPECT_PRED_FORMAT2(::pcl::test::internal::RGBAEQ,     \
                      (expected), (actual))

/// Assert that each of r, g, b, and a fields are equal
/// in two points.
#define ASSERT_RGBA_EQ(expected, actual)                 \
  ASSERT_PRED_FORMAT2(::pcl::test::internal::RGBAEQ,     \
                      (expected), (actual))

#endif
