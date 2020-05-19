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
 *
 */

#pragma once

#include <pcl/common/random.h>


namespace pcl
{

namespace common
{


template <typename T>
UniformGenerator<T>::UniformGenerator(T min, T max, std::uint32_t seed)
  : distribution_ (min, max)
{
  parameters_ = Parameters (min, max, seed);
  if(parameters_.seed != static_cast<std::uint32_t> (-1))
    rng_.seed (seed);
}


template <typename T>
UniformGenerator<T>::UniformGenerator(const Parameters& parameters)
  : parameters_ (parameters)
  , distribution_ (parameters_.min, parameters_.max)
{
  if(parameters_.seed != static_cast<std::uint32_t> (-1))
    rng_.seed (parameters_.seed);
}


template <typename T> void
UniformGenerator<T>::setSeed (std::uint32_t seed)
{
  if (seed != static_cast<std::uint32_t> (-1))
  {
    parameters_.seed = seed;
    rng_.seed(parameters_.seed);
  }
}


template <typename T> void
UniformGenerator<T>::setParameters (T min, T max, std::uint32_t seed)
{
  parameters_.min = min;
  parameters_.max = max;
  parameters_.seed = seed;
  typename DistributionType::param_type params (parameters_.min, parameters_.max);
  distribution_.param (params);
  distribution_.reset ();
  if (seed != static_cast<std::uint32_t> (-1))
  {
    parameters_.seed = seed;
    rng_.seed (parameters_.seed);
  }
}


template <typename T> void
UniformGenerator<T>::setParameters (const Parameters& parameters)
{
  parameters_ = parameters;
  typename DistributionType::param_type params (parameters_.min, parameters_.max);
  distribution_.param (params);
  distribution_.reset ();
  if (parameters_.seed != static_cast<std::uint32_t> (-1))
    rng_.seed (parameters_.seed);
}


template <typename T>
NormalGenerator<T>::NormalGenerator(T mean, T sigma, std::uint32_t seed)
  : distribution_ (mean, sigma)
{
  parameters_ = Parameters (mean, sigma, seed);
  if(parameters_.seed != static_cast<std::uint32_t> (-1))
    rng_.seed (seed);
}


template <typename T>
NormalGenerator<T>::NormalGenerator(const Parameters& parameters)
  : parameters_ (parameters)
  , distribution_ (parameters_.mean, parameters_.sigma)
{
  if(parameters_.seed != static_cast<std::uint32_t> (-1))
    rng_.seed (parameters_.seed);
}


template <typename T> void
NormalGenerator<T>::setSeed (std::uint32_t seed)
{
  if (seed != static_cast<std::uint32_t> (-1))
  {
    parameters_.seed = seed;
    rng_.seed(seed);
  }
}


template <typename T> void
NormalGenerator<T>::setParameters (T mean, T sigma, std::uint32_t seed)
{
  parameters_.mean = mean;
  parameters_.sigma = sigma;
  parameters_.seed = seed;
  typename DistributionType::param_type params (parameters_.mean, parameters_.sigma);
  distribution_.param (params);
  distribution_.reset ();
  if (seed != static_cast<std::uint32_t> (-1))
    rng_.seed (parameters_.seed);
}


template <typename T> void
NormalGenerator<T>::setParameters (const Parameters& parameters)
{
  parameters_ = parameters;
  typename DistributionType::param_type params (parameters_.mean, parameters_.sigma);
  distribution_.param (params);
  distribution_.reset ();
  if (parameters_.seed != static_cast<std::uint32_t> (-1))
    rng_.seed (parameters_.seed);
}

} // namespace common
} // namespace pcl

