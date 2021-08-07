/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: extract_indices.h 1370 2011-06-19 01:06:01Z jspricke $
 *
 */

#pragma once

#include <pcl/filters/filter_indices.h>
#include <ctime>
#include <climits>

namespace pcl
{
  /** \brief @b RandomSample applies a random sampling with uniform probability.
    * Based off Algorithm A from the paper "Faster Methods for Random Sampling"
    * by Jeffrey Scott Vitter. The algorithm runs in O(N) and results in sorted
    * indices
    * http://www.ittc.ku.edu/~jsv/Papers/Vit84.sampling.pdf
    * \author Justin Rosen
    * \ingroup filters
    */
  template<typename PointT>
  class RandomSample : public FilterIndices<PointT>
  {
    using FilterIndices<PointT>::filter_name_;
    using FilterIndices<PointT>::getClassName;
    using FilterIndices<PointT>::indices_;
    using FilterIndices<PointT>::input_;
    using FilterIndices<PointT>::negative_;
    using FilterIndices<PointT>::keep_organized_;
    using FilterIndices<PointT>::user_filter_value_;
    using FilterIndices<PointT>::extract_removed_indices_;
    using FilterIndices<PointT>::removed_indices_;

    using PointCloud = typename FilterIndices<PointT>::PointCloud;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    public:

      using Ptr = shared_ptr<RandomSample<PointT> >;
      using ConstPtr = shared_ptr<const RandomSample<PointT> >;

      /** \brief Empty constructor. */
      RandomSample (bool extract_removed_indices = false) : 
        FilterIndices<PointT> (extract_removed_indices),
        sample_ (UINT_MAX), 
        seed_ (static_cast<unsigned int> (time (nullptr)))
      {
        filter_name_ = "RandomSample";
      }

      /** \brief Set number of indices to be sampled.
        * \param sample
        */
      inline void
      setSample (unsigned int sample)
      {
        sample_ = sample;
      }

      /** \brief Get the value of the internal \a sample parameter.
        */
      inline unsigned int
      getSample ()
      {
        return (sample_);
      }

      /** \brief Set seed of random function.
        * \param seed
        */
      inline void
      setSeed (unsigned int seed)
      {
        seed_ = seed;
      }

      /** \brief Get the value of the internal \a seed parameter.
        */
      inline unsigned int
      getSeed ()
      {
        return (seed_);
      }

    protected:

      /** \brief Number of indices that will be returned. */
      unsigned int sample_;
      /** \brief Random number seed. */
      unsigned int seed_;

      /** \brief Sample of point indices
        * \param indices the resultant point cloud indices
        */
      void
      applyFilter (Indices &indices) override;

      /** \brief Return a random number fast using a LCG (Linear Congruential Generator) algorithm.
        * See http://software.intel.com/en-us/articles/fast-random-number-generator-on-the-intel-pentiumr-4-processor/ for more information.
        */
      inline float
      unifRand ()
      {
        return (static_cast<float>(rand () / double (RAND_MAX)));
        //return (((214013 * seed_ + 2531011) >> 16) & 0x7FFF);
      }
  };

  /** \brief @b RandomSample applies a random sampling with uniform probability.
    * \author Justin Rosen
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS RandomSample<pcl::PCLPointCloud2> : public FilterIndices<pcl::PCLPointCloud2>
  {
    using FilterIndices<pcl::PCLPointCloud2>::filter_name_;
    using FilterIndices<pcl::PCLPointCloud2>::getClassName;

    using PCLPointCloud2 = pcl::PCLPointCloud2;
    using PCLPointCloud2Ptr = PCLPointCloud2::Ptr;
    using PCLPointCloud2ConstPtr = PCLPointCloud2::ConstPtr;

    public:
  
      using Ptr = shared_ptr<RandomSample<pcl::PCLPointCloud2> >;
      using ConstPtr = shared_ptr<const RandomSample<pcl::PCLPointCloud2> >;
  
      /** \brief Empty constructor. */
      RandomSample () : sample_ (UINT_MAX), seed_ (static_cast<unsigned int> (time (nullptr)))
      {
        filter_name_ = "RandomSample";
      }

      /** \brief Set number of indices to be sampled.
        * \param sample
        */
      inline void
      setSample (unsigned int sample)
      {
        sample_ = sample;
      }

      /** \brief Get the value of the internal \a sample parameter.
        */
      inline unsigned int
      getSample ()
      {
        return (sample_);
      }

      /** \brief Set seed of random function.
        * \param seed
        */
      inline void
      setSeed (unsigned int seed)
      {
        seed_ = seed;
      }

      /** \brief Get the value of the internal \a seed parameter.
        */
      inline unsigned int
      getSeed ()
      {
        return (seed_);
      }

    protected:

      /** \brief Number of indices that will be returned. */
      unsigned int sample_;
      /** \brief Random number seed. */
      unsigned int seed_;

      /** \brief Sample of point indices into a separate PointCloud
        * \param output the resultant point cloud
        */
      void
      applyFilter (PCLPointCloud2 &output) override;

      /** \brief Sample of point indices
        * \param indices the resultant point cloud indices
        */
      void
      applyFilter (Indices &indices) override;

      /** \brief Return a random number fast using a LCG (Linear Congruential Generator) algorithm.
        * See http://software.intel.com/en-us/articles/fast-random-number-generator-on-the-intel-pentiumr-4-processor/ for more information.
        */
      inline float
      unifRand ()
      {
        return (static_cast<float> (rand () / double (RAND_MAX)));
      }
   };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/random_sample.hpp>
#endif
