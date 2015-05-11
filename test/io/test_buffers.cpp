/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#include <gtest/gtest.h>

#include <cmath>

#include <pcl/io/buffers.h>

using namespace pcl::io;

template <typename T>
class BuffersTest : public ::testing::Test
{

  public:

    BuffersTest ()
    {
      if (std::numeric_limits<T>::has_quiet_NaN)
        invalid_ = std::numeric_limits<T>::quiet_NaN ();
      else
        invalid_ = 0;
    }

    template <typename Buffer> void
    checkBuffer (Buffer& buffer, const T* data, const T* expected, size_t size)
    {
      const T* dptr = data;
      const T* eptr = expected;
      for (size_t i = 0; i < size; ++i)
      {
        std::vector<T> d (buffer.size ());
        memcpy (d.data (), dptr, buffer.size () * sizeof (T));
        buffer.push (d);
        for (size_t j = 0; j < buffer.size (); ++j)
          if (isnan (eptr[j]))
            EXPECT_TRUE (isnan (buffer[j]));
          else
            EXPECT_EQ (eptr[j], buffer[j]);
        dptr += buffer.size ();
        eptr += buffer.size ();
      }
    }

    T invalid_;

};

typedef ::testing::Types<char, int, float> DataTypes;
TYPED_TEST_CASE (BuffersTest, DataTypes);

TYPED_TEST (BuffersTest, SingleBuffer)
{
  SingleBuffer<TypeParam> sb (1);
  const TypeParam data[] = {5, 4, 3, 2, 1};
  this->checkBuffer (sb, data, data, sizeof (data) / sizeof (TypeParam));
}

TYPED_TEST (BuffersTest, MedianBufferWindow1)
{
  MedianBuffer<TypeParam> mb (1, 1);
  const TypeParam data[] = {5, 4, 3, 2, 1};
  this->checkBuffer (mb, data, data, sizeof (data) / sizeof (TypeParam));
}

TYPED_TEST (BuffersTest, MedianBufferWindow2)
{
  {
    MedianBuffer<TypeParam> mb (1, 2);
    const TypeParam data[] = {5, 4, 3, 2, 1};
    const TypeParam median[] = {5, 5, 4, 3, 2};
    this->checkBuffer (mb, data, median, sizeof (data) / sizeof (TypeParam));
  }
  {
    MedianBuffer<TypeParam> mb (1, 2);
    const TypeParam data[] = {3, 4, 1, 3, 4};
    const TypeParam median[] = {3, 4, 4, 3, 4};
    this->checkBuffer (mb, data, median, sizeof (data) / sizeof (TypeParam));
  }
}

TYPED_TEST (BuffersTest, MedianBufferWindow3)
{
  {
    MedianBuffer<TypeParam> mb (1, 3);
    const TypeParam data[] = {5, 4, 3, 2, 1, -1, -1};
    const TypeParam median[] = {5, 5, 4, 3, 2, 1, -1};
    this->checkBuffer (mb, data, median, sizeof (data) / sizeof (TypeParam));
  }
  {
    MedianBuffer<TypeParam> mb (1, 3);
    const TypeParam data[] = {3, 4, 1, 3, 4, -1, -1};
    const TypeParam median[] = {3, 4, 3, 3, 3, 3, -1};
    this->checkBuffer (mb, data, median, sizeof (data) / sizeof (TypeParam));
  }
  {
    MedianBuffer<TypeParam> mb (1, 3);
    const TypeParam data[] = {-4, -1, 3, -4, 1, 3, 4, -1};
    const TypeParam median[] = {-4, -1, -1, -1, 1, 1, 3, 3};
    this->checkBuffer (mb, data, median, sizeof (data) / sizeof (TypeParam));
  }
}

TYPED_TEST (BuffersTest, MedianBufferWindow4)
{
  {
    MedianBuffer<TypeParam> mb (1, 4);
    const TypeParam data[] = {5, 4, 3, 2, 1, -1, -1};
    const TypeParam median[] = {5, 5, 4, 4, 3, 2, 1};
    this->checkBuffer (mb, data, median, sizeof (data) / sizeof (TypeParam));
  }
  {
    MedianBuffer<TypeParam> mb (1, 4);
    const TypeParam data[] = {-4, -1, 3, -4, 1, 3, 4, -2};
    const TypeParam median[] = {-4, -1, -1, -1, 1, 3, 3, 3};
    this->checkBuffer (mb, data, median, sizeof (data) / sizeof (TypeParam));
  }
}

TYPED_TEST (BuffersTest, MedianBufferPushInvalid)
{
  const TypeParam& invalid = this->invalid_;
  MedianBuffer<TypeParam> mb (1, 3);
  const TypeParam data[] = {5, 4, 3, invalid, 1, invalid, invalid, invalid, 9, 3, 1};
  const TypeParam median[] = {5, 5, 4, 4, 3, 1, 1, invalid, 9, 9, 3};
  this->checkBuffer (mb, data, median, sizeof (data) / sizeof (TypeParam));
}

TYPED_TEST (BuffersTest, MedianBufferSize3Window3)
{
  {
    MedianBuffer<TypeParam> mb (3, 3);
    const TypeParam data[] = {3, 3, 3, 1, 1, 1, -1, -1, -1};
    const TypeParam median[] = {3, 3, 3, 3, 3, 3, 1, 1, 1};
    this->checkBuffer (mb, data, median, sizeof (data) / sizeof (TypeParam) / mb.size ());
  }
  {
    MedianBuffer<TypeParam> mb (3, 3);
    const TypeParam data[] = {3, 2, 1, 1, 1, 1, 3, 2, 1, 1, 2, 3};
    const TypeParam median[] = {3, 2, 1, 3, 2, 1, 3, 2, 1, 1, 2, 1};
    this->checkBuffer (mb, data, median, sizeof (data) / sizeof (TypeParam) / mb.size ());
  }
}

TYPED_TEST (BuffersTest, AverageBufferWindow1)
{
  AverageBuffer<TypeParam> ab (1, 1);
  const TypeParam data[] = {5, 4, 3, 2, 1};
  this->checkBuffer (ab, data, data, sizeof (data) / sizeof (TypeParam));
}

TYPED_TEST (BuffersTest, AverageBufferWindow2)
{
  {
    AverageBuffer<TypeParam> ab (1, 2);
    const TypeParam data[] = {5, 3, 3, 1, 1};
    const TypeParam average[] = {5, 4, 3, 2, 1};
    this->checkBuffer (ab, data, average, sizeof (data) / sizeof (TypeParam));
  }
  {
    AverageBuffer<TypeParam> ab (1, 2);
    const TypeParam data[] = {3, 5, 1, 13, 3};
    const TypeParam average[] = {3, 4, 3, 7, 8};
    this->checkBuffer (ab, data, average, sizeof (data) / sizeof (TypeParam));
  }
}

TYPED_TEST (BuffersTest, AverageBufferWindow3)
{
  {
    AverageBuffer<TypeParam> ab (1, 3);
    const TypeParam data[] = {5, 3, 1, 2, -3, 4, -7};
    const TypeParam average[] = {5, 4, 3, 2, 0, 1, -2};
    this->checkBuffer (ab, data, average, sizeof (data) / sizeof (TypeParam));
  }
  {
    AverageBuffer<TypeParam> ab (1, 3);
    const TypeParam data[] = {3, -5, 2, -3, 4, -1, -3};
    const TypeParam average[] = {3, -1, 0, -2, 1, 0, 0};
    this->checkBuffer (ab, data, average, sizeof (data) / sizeof (TypeParam));
  }
}

TYPED_TEST (BuffersTest, AverageBufferPushInvalid)
{
  const TypeParam& invalid = this->invalid_;
  AverageBuffer<TypeParam> ab (1, 3);
  const TypeParam data[] = {5, 3, 7, invalid, 1, invalid, invalid, invalid, 9, 3, -3};
  const TypeParam median[] = {5, 4, 5, 5, 4, 1, 1, invalid, 9, 6, 3};
  this->checkBuffer (ab, data, median, sizeof (data) / sizeof (TypeParam));
}

int main (int argc, char **argv)
{
  testing::InitGoogleTest (&argc, argv);
  return RUN_ALL_TESTS ();
}

