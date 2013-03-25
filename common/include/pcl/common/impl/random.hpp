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

#ifndef PCL_COMMON_RANDOM_HPP_
#define PCL_COMMON_RANDOM_HPP_

#include <boost/version.hpp>
#include <pcl/pcl_macros.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
pcl::common::UniformGenerator<T>::UniformGenerator(T min, T max, pcl::uint32_t seed)
  : distribution_ (min, max)
  , generator_ (rng_, distribution_) 
{
  parameters_ = Parameters (min, max, seed);
  if(parameters_.seed != -1)
    rng_.seed (seed);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
pcl::common::UniformGenerator<T>::UniformGenerator(const Parameters& parameters)
  : parameters_ (parameters)
  , distribution_ (parameters_.min, parameters_.max)
  , generator_ (rng_, distribution_) 
{
  if(parameters_.seed != -1)
    rng_.seed (parameters_.seed);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> void
pcl::common::UniformGenerator<T>::setSeed (pcl::uint32_t seed)
{
  if (seed != -1)
  {
    parameters_.seed = seed;
    rng_.seed(parameters_.seed);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> void
pcl::common::UniformGenerator<T>::setParameters (T min, T max, pcl::uint32_t seed)
{
  parameters_.min = min;
  parameters_.max = max;
  parameters_.seed = seed;
#if BOOST_VERSION >= 104700
  typename DistributionType::param_type params (parameters_.min, parameters_.max);
  distribution_.param (params);
#else
  distribution_ = DistributionType (parameters_.min, parameters_.max);
#endif
  distribution_.reset ();
  generator_.distribution () = distribution_;
  if (seed != -1)
  {
    parameters_.seed = seed;
    rng_.seed (parameters_.seed);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> void
pcl::common::UniformGenerator<T>::setParameters (const Parameters& parameters)
{
  parameters_ = parameters;
#if BOOST_VERSION >= 104700
  typename DistributionType::param_type params (parameters_.min, parameters_.max);
  distribution_.param (params);
#else
  distribution_ = DistributionType (parameters_.min, parameters_.max);
#endif
  distribution_.reset ();
  generator_.distribution () = distribution_;
  if (parameters_.seed != -1)
    rng_.seed (parameters_.seed);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
pcl::common::NormalGenerator<T>::NormalGenerator(T mean, T sigma, pcl::uint32_t seed)
  : distribution_ (mean, sigma)
  , generator_ (rng_, distribution_) 
{
  parameters_ = Parameters (mean, sigma, seed);
  if(parameters_.seed != -1)
    rng_.seed (seed);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
pcl::common::NormalGenerator<T>::NormalGenerator(const Parameters& parameters)
  : parameters_ (parameters)
  , distribution_ (parameters_.mean, parameters_.sigma)
  , generator_ (rng_, distribution_) 
{
  if(parameters_.seed != -1)
    rng_.seed (parameters_.seed);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> void
pcl::common::NormalGenerator<T>::setSeed (pcl::uint32_t seed)
{	
  if (seed != -1)
  {
    parameters_.seed = seed;
    rng_.seed(seed);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> void
pcl::common::NormalGenerator<T>::setParameters (T mean, T sigma, pcl::uint32_t seed)
{
  parameters_.mean = mean;
  parameters_.sigma = sigma;
  parameters_.seed = seed;
#if BOOST_VERSION >= 104700
  typename DistributionType::param_type params (parameters_.mean, parameters_.sigma);
  distribution_.param (params);
#else
  distribution_ = DistributionType (parameters_.mean, parameters_.sigma);
#endif
  distribution_.reset ();
  generator_.distribution () = distribution_;
  if (seed != -1)
    rng_.seed (parameters_.seed);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> void
pcl::common::NormalGenerator<T>::setParameters (const Parameters& parameters)
{
  parameters_ = parameters;
#if BOOST_VERSION >= 104700
  typename DistributionType::param_type params (parameters_.mean, parameters_.sigma);
  distribution_.param (params);
#else
  distribution_ = DistributionType (parameters_.mean, parameters_.sigma);
#endif
  distribution_.reset ();
  generator_.distribution () = distribution_;
  if (parameters_.seed != -1)
    rng_.seed (parameters_.seed);
}

#endif
