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

#include <random>

#include <pcl/pcl_macros.h>

namespace pcl 
{
  namespace common 
  {
    /// uniform distribution dummy struct
    template <typename T, typename T2=void> struct uniform_distribution;
    /// uniform distribution int specialized
    template <typename T>
    struct uniform_distribution<T, std::enable_if_t<std::is_integral<T>::value>>
    {
      using type = std::uniform_int_distribution<T>;
    };
    /// uniform distribution float specialized
    template <typename T>
    struct uniform_distribution<T, std::enable_if_t<std::is_floating_point<T>::value>>
    {
      using type = std::uniform_real_distribution<T>;
    };
    ///  normal distribution
    template<typename T> 
    struct normal_distribution
    {
      using type = std::normal_distribution<T>;
    };

    /** \brief UniformGenerator class generates a random number from range [min, max] at each run picked
      * according to a uniform distribution i.e each number within [min, max] has almost the same 
      * probability of being drawn.
      *
      * \author Nizar Sallem
      */
    template<typename T>
    class UniformGenerator 
    {
      public:
        struct Parameters
        {
            Parameters (T _min = 0, T _max = 1, std::uint32_t _seed = 1)
            : min (_min)
            , max (_max)
            , seed (_seed)
          {}

          T min;
          T max;
          std::uint32_t seed;
        };

        /** Constructor
          * \param min: included lower bound
          * \param max: included higher bound
          * \param seed: seeding value
          */
        UniformGenerator(T min = 0, T max = 1, std::uint32_t seed = -1);

        /** Constructor
          * \param parameters uniform distribution parameters and generator seed
          */
        UniformGenerator(const Parameters& parameters);

        /** Change seed value
          * \param[in] seed new generator seed value
          */
        void 
        setSeed (std::uint32_t seed);

        /** Set the uniform number generator parameters
          * \param[in] min minimum allowed value
          * \param[in] max maximum allowed value
          * \param[in] seed random number generator seed (applied if != -1)
          */
        void 
        setParameters (T min, T max, std::uint32_t seed = -1);

        /** Set generator parameters
          * \param parameters uniform distribution parameters and generator seed
          */
        void
        setParameters (const Parameters& parameters);

        /// \return uniform distribution parameters and generator seed
        const Parameters&
        getParameters () { return (parameters_); }

        /// \return a randomly generated number in the interval [min, max]
        inline T 
        run () { return (distribution_ (rng_)); }

      private:
        using DistributionType = typename uniform_distribution<T>::type;
        /// parameters
        Parameters parameters_;
        /// random number generator
        std::mt19937 rng_;
        /// uniform distribution
        DistributionType distribution_;
    };

    /** \brief NormalGenerator class generates a random number from a normal distribution specified
      * by (mean, sigma).
      *
      * \author Nizar Sallem
      */
    template<typename T>
    class NormalGenerator 
    {
      public:
        struct Parameters
        {
            Parameters (T _mean = 0, T _sigma = 1, std::uint32_t _seed = 1)
            : mean (_mean)
            , sigma (_sigma)
            , seed (_seed)
          {}

          T mean;
          T sigma;
          std::uint32_t seed;
        };

        /** Constructor
          * \param[in] mean normal mean
          * \param[in] sigma normal variation
          * \param[in] seed seeding value
          */
        NormalGenerator(T mean = 0, T sigma = 1, std::uint32_t seed = -1);

        /** Constructor
          * \param parameters normal distribution parameters and seed
          */
        NormalGenerator(const Parameters& parameters);

        /** Change seed value
          * \param[in] seed new seed value
          */
        void 
        setSeed (std::uint32_t seed);

        /** Set the normal number generator parameters
          * \param[in] mean mean of the normal distribution
          * \param[in] sigma standard variation of the normal distribution
          * \param[in] seed random number generator seed (applied if != -1)
          */
        void 
        setParameters (T mean, T sigma, std::uint32_t seed = -1);

        /** Set generator parameters
          * \param parameters normal distribution parameters and seed
          */
        void
        setParameters (const Parameters& parameters);

        /// \return normal distribution parameters and generator seed
        const Parameters&
        getParameters () { return (parameters_); }

        /// \return a randomly generated number in the normal distribution (mean, sigma)
        inline T 
        run () { return (distribution_ (rng_)); }

        using DistributionType = typename normal_distribution<T>::type;
        /// parameters
        Parameters parameters_;
        /// random number generator
        std::mt19937 rng_;
        /// normal distribution
        DistributionType distribution_;
    };
  }
}

#include <pcl/common/impl/random.hpp>
