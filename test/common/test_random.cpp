/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
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
#include <pcl/common/random.h>
#include <pcl/test/gtest.h>

#include <random>

using pcl::common::RandomBase;

template <typename T = std::default_random_engine>
struct RandomDerived : public RandomBase<T> {
  using typename RandomBase<T>::SeedT;
  using RandomBase<T>::rng_;

  RandomDerived(SeedT seed = std::random_device()())
  : RandomBase<T>(seed)
  {}

  int
  operator()()
  {
    return dist(rng_);
  }

  std::uniform_int_distribution<int> dist;
};

template <typename T>
struct RandomBaseTest : public testing::Test {};

using RandomEngineTypes = testing::Types<std::default_random_engine,
                                         std::minstd_rand0,
                                         std::minstd_rand,
                                         std::mt19937,
                                         std::mt19937_64,
                                         std::ranlux24_base,
                                         std::ranlux48_base,
                                         std::ranlux48_base,
                                         std::ranlux24,
                                         std::ranlux48,
                                         std::knuth_b>;
TYPED_TEST_SUITE(RandomBaseTest, RandomEngineTypes);

TYPED_TEST(RandomBaseTest, Constructor)
{
  constexpr unsigned seed = 42u;

  // Check the seed is properly set
  ASSERT_EQ(seed, RandomBase<TypeParam>(seed).getSeed());

  // Assert the random engine's state is properly set
  ASSERT_EQ(RandomDerived<TypeParam>(seed)(), RandomDerived<TypeParam>(seed)());
}

TYPED_TEST(RandomBaseTest, GetSetSeed)
{
  constexpr unsigned seed = 24u;

  RandomDerived<TypeParam> rb;
  rb.setSeed(seed);

  // Assert seed is the same as the one set
  ASSERT_EQ(seed, rb.getSeed());

  // Assert the engine is also reset
  ASSERT_EQ(rb(), RandomDerived<TypeParam>(seed)());
}

TYPED_TEST(RandomBaseTest, ResetEngine)
{
  RandomDerived<TypeParam> rb;
  const int sample = rb();

  // If the engine is not being reset properly it would be very unlikely to
  // draw the same sample 5 times
  for (unsigned i = 0; i < 5; ++i) {
    rb.resetEngine();
    ASSERT_EQ(sample, rb());
  }
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
