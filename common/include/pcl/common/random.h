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

#ifndef PCL_COMMON_RANDOM_H_
#define PCL_COMMON_RANDOM_H_

#ifdef __GNUC__
#pragma GCC system_header 
#endif

#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <pcl/pcl_macros.h>

namespace pcl 
{
  namespace common 
  {
    /// uniform distribution dummy struct
    template <typename T> struct uniform_distribution;
    /// uniform distribution int specialized
    template<> 
    struct uniform_distribution<int> 
    {
      typedef boost::uniform_int<int> type;
    };
    /// uniform distribution float specialized
    template<> 
    struct uniform_distribution<float> 
    {
      typedef boost::uniform_real<float> type;
    };
    ///  normal distribution
    template<typename T> 
    struct normal_distribution
    {
      typedef boost::normal_distribution<T> type;
    };

    /** \brief UniformGenerator class generates a random number from range [min, max] at each run picked
      * according to a uniform distribution i.e eaach number within [min, max] has almost the same 
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
            Parameters (T _min = 0, T _max = 1, pcl::uint32_t _seed = 1)
            : min (_min)
            , max (_max)
            , seed (_seed)
          {}

          T min;
          T max;
          pcl::uint32_t seed;
        };

        /** Constructor
          * \param min: included lower bound
          * \param max: included higher bound
          * \param seed: seeding value
          */
        UniformGenerator(T min = 0, T max = 1, pcl::uint32_t seed = -1);

        /** Constructor
          * \param parameters uniform distribution parameters and generator seed
          */
        UniformGenerator(const Parameters& parameters);

        /** Change seed value
          * \param[in] seed new generator seed value
          */
        void 
        setSeed (pcl::uint32_t seed);

        /** Set the uniform number generator parameters
          * \param[in] min minimum allowed value
          * \param[in] max maximum allowed value
          * \param[in] seed random number generator seed (applied if != -1)
          */
        void 
        setParameters (T min, T max, pcl::uint32_t seed = -1);

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
        run () { return (generator_ ()); }

      private:
        typedef boost::mt19937 EngineType;
        typedef typename uniform_distribution<T>::type DistributionType;
        /// parameters
        Parameters parameters_;
        /// uniform distribution
        DistributionType distribution_;
        /// random number generator
        EngineType rng_;
        /// generator of random number from a uniform distribution
        boost::variate_generator<EngineType&, DistributionType> generator_;
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
            Parameters (T _mean = 0, T _sigma = 1, pcl::uint32_t _seed = 1)
            : mean (_mean)
            , sigma (_sigma)
            , seed (_seed)
          {}

          T mean;
          T sigma;
          pcl::uint32_t seed;
        };

        /** Constructor
          * \param[in] mean normal mean
          * \param[in] sigma normal variation
          * \param[in] seed seeding value
          */
        NormalGenerator(T mean = 0, T sigma = 1, pcl::uint32_t seed = -1);

        /** Constructor
          * \param parameters normal distribution parameters and seed
          */
        NormalGenerator(const Parameters& parameters);

        /** Change seed value
          * \param[in] seed new seed value
          */
        void 
        setSeed (pcl::uint32_t seed);

        /** Set the normal number generator parameters
          * \param[in] mean mean of the normal distribution
          * \param[in] sigma standard variation of the normal distribution
          * \param[in] seed random number generator seed (applied if != -1)
          */
        void 
        setParameters (T mean, T sigma, pcl::uint32_t seed = -1);

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
        run () { return (generator_ ()); }

        typedef boost::mt19937 EngineType;
        typedef typename normal_distribution<T>::type DistributionType;
        /// parameters
        Parameters parameters_;
        /// normal distribution
        DistributionType distribution_;
        /// random number generator
        EngineType rng_;
        /// generator of random number from a normal distribution
        boost::variate_generator<EngineType&, DistributionType > generator_;
    };
  }
}

#include <pcl/common/impl/random.hpp>

#endif
