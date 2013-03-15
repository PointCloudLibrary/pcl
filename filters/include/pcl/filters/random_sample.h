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

#ifndef PCL_FILTERS_RANDOM_SUBSAMPLE_H_
#define PCL_FILTERS_RANDOM_SUBSAMPLE_H_

#include <pcl/filters/filter_indices.h>
#include <time.h>
#include <limits.h>

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

    typedef typename FilterIndices<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:

      typedef boost::shared_ptr< RandomSample<PointT> > Ptr;
      typedef boost::shared_ptr< const RandomSample<PointT> > ConstPtr;

      /** \brief Empty constructor. */
      RandomSample (bool extract_removed_indices = false)
        : FilterIndices<PointT> (extract_removed_indices)
        , sample_ (std::numeric_limits<unsigned int>::max ())
        , rng_alg_ ()
        , rng_ (new boost::uniform_01<boost::mt19937> (rng_alg_))
        , seed_ (static_cast<unsigned int> (time (NULL)))
      { filter_name_ = "RandomSample"; }

      /** \brief Set number of indices to be sampled.
        * \param sample
        */
      inline void
      setSample (unsigned int sample)
      { sample_ = sample; }

      /** \brief Get the value of the internal \a sample parameter.
        */
      inline unsigned int
      getSample ()
      { return (sample_); }

      /** \brief Set seed of random function.
        * \param seed
        */
      inline void
      setSeed (unsigned int seed)
      { seed_ = seed; }

      /** \brief Get the value of the internal \a seed parameter.
        */
      inline unsigned int
      getSeed ()
      { return (seed_); }

    protected:
      /** \brief Number of indices that will be returned. */
      unsigned int sample_;
      /** \brief Random number seed. */
      unsigned int seed_;

      /** \brief Sample of point indices into a separate PointCloud
        * \param output the resultant point cloud
        */
      void
      applyFilter (PointCloud &output);

      /** \brief Sample of point indices
        * \param indices the resultant point cloud indices
        */
      void
      applyFilter (std::vector<int> &indices);

      /** \brief Return a random number. */
      inline float
      unifRand ()
      { return ((*rng_) ()); }

    private:
      /** \brief Boost-based random number generator algorithm. */
      boost::mt19937 rng_alg_;

      /** \brief Boost-based random number generator distribution. */
      boost::shared_ptr<boost::uniform_01<boost::mt19937> > rng_;
  };

  /** \brief @b RandomSample applies a random sampling with uniform probability.
    * \author Justin Rosen
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS RandomSample<sensor_msgs::PointCloud2> : public FilterIndices<sensor_msgs::PointCloud2>
  {
    using FilterIndices<sensor_msgs::PointCloud2>::filter_name_;
    using FilterIndices<sensor_msgs::PointCloud2>::getClassName;

    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    public:
  
      typedef boost::shared_ptr<RandomSample<sensor_msgs::PointCloud2> > Ptr;
      typedef boost::shared_ptr<const RandomSample<sensor_msgs::PointCloud2> > ConstPtr;
  
      /** \brief Empty constructor. */
      RandomSample ()
        : sample_ (std::numeric_limits<unsigned int>::max ())
        , rng_alg_ ()
        , rng_ (new boost::uniform_01<boost::mt19937> (rng_alg_))
        , seed_ (static_cast<unsigned int> (time (NULL)))
      { filter_name_ = "RandomSample"; }

      /** \brief Set number of indices to be sampled.
        * \param sample
        */
      inline void
      setSample (unsigned int sample)
      { sample_ = sample; }

      /** \brief Get the value of the internal \a sample parameter.
        */
      inline unsigned int
      getSample ()
      { return (sample_); }

      /** \brief Set seed of random function.
        * \param seed
        */
      inline void
      setSeed (unsigned int seed)
      { seed_ = seed; }

      /** \brief Get the value of the internal \a seed parameter.
        */
      inline unsigned int
      getSeed ()
      { return (seed_); }

    protected:
      /** \brief Number of indices that will be returned. */
      unsigned int sample_;
      /** \brief Random number seed. */
      unsigned int seed_;

      /** \brief Sample of point indices into a separate PointCloud
        * \param output the resultant point cloud
        */
      void
      applyFilter (PointCloud2 &output);

      /** \brief Sample of point indices
        * \param indices the resultant point cloud indices
        */
      void
      applyFilter (std::vector<int> &indices);

      /** \brief Return a random number. */
      inline float
      unifRand ()
      { return ((*rng_) ()); }

    private:
      /** \brief Boost-based random number generator algorithm. */
      boost::mt19937 rng_alg_;

      /** \brief Boost-based random number generator distribution. */
      boost::shared_ptr<boost::uniform_01<boost::mt19937> > rng_;
   };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/random_sample.hpp>
#endif

#endif  //#ifndef PCL_FILTERS_RANDOM_SUBSAMPLE_H_
