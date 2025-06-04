/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/filters/filter_indices.h>

#include <climits>
#include <random>

namespace pcl
 {
    /** \brief @b FarthestPointSampling applies farthest point sampling using euclidean 
      * distance, starting with a random point, utilizing a naive method.
      * \author Haritha Jayasinghe
      * \ingroup filters
      * \todo add support to export/import distance metric
      */
    template<typename PointT>
    class FarthestPointSampling : public FilterIndices<PointT>
    {
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using FilterIndices<PointT>::keep_organized_;
      using FilterIndices<PointT>::user_filter_value_;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;
    
      using typename FilterIndices<PointT>::PointCloud;

      public:
        /** \brief Empty constructor. */
        FarthestPointSampling (bool extract_removed_indices = false) : 
          FilterIndices<PointT> (extract_removed_indices),
          sample_size_ (std::numeric_limits<int>::max ()), 
          seed_ (std::random_device()())
        {
          filter_name_ = "FarthestPointSamping";
        }

        /** \brief Set number of points to be sampled.
          * \param sample_size the number of points to sample
          */
        inline void
        setSample (std::size_t sample_size)
        {
          sample_size_ = sample_size;
        }

        /** \brief Get the value of the internal \a sample_size parameter.
          */
        inline std::size_t
        getSample () const
        {
          return (sample_size_);
        }

        /** \brief Set seed of random function.
          * \param seed for the random number generator, to choose the first sample point
          */
        inline void
        setSeed (unsigned int  seed)
        {
          seed_ = seed;
        }

        /** \brief Get the value of the internal \a seed_ parameter.
          */
        inline unsigned int 
        getSeed () const
        {
          return (seed_);
        }

        /** \brief Set the number of threads to use when operating in parallel 
          * \param nr_threads the number of threads to use
          */
        inline void
        setNumberOfThreads (unsigned int nr_threads)
        {
          #ifdef _OPENMP
          nr_threads_ = nr_threads == 0 ? omp_get_num_procs() : nr_threads;
          #else
          if (nr_threads_ != 1) 
            PCL_WARN("OpenMP is not available. Keeping number of threads unchanged at 1\n");
          #endif
        }

        /** \brief Get the value of the internal \a nr_threads_ parameter.
          */
        inline unsigned int 
        getNumberOfThreads () const
        {
          return nr_threads_;
        }

      protected:

        /** \brief Number of points that will be returned. */
        std::size_t sample_size_;
        /** \brief Random number seed. */
        unsigned int seed_;
        /** \brief Number of threads */
        unsigned int nr_threads_{1};

        /** \brief Sample of point indices
          * \param indices indices of the filtered point cloud
          */
        void
        applyFilter (pcl::Indices &indices) override;

    };
 }

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/farthest_point_sampling.hpp>
#endif
