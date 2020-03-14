/*
 * Software License Agreement (BSD License)
 *
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
      */
    template<typename PointT>
    class FarthestPointSampling : public FilterIndices<PointT>
    {
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using FilterIndices<PointT>::negative_;
      using FilterIndices<PointT>::keep_organized_;
      using FilterIndices<PointT>::user_filter_value_;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;
    
      using typename FilterIndices<PointT>::PointCloud;

      public:
        /** \brief Empty constructor. */
        FarthestPointSampling (bool extract_removed_indices = false) : 
          FilterIndices<PointT> (extract_removed_indices),
          sample_ (UINT_MAX), 
          seed_ (static_cast<unsigned int> (std::random_device()()))
        {
          filter_name_ = "FarthestPointSamping";
        }

        /** \brief Set number of points to be sampled.
          * \param sample
          */
        inline void
        setSample (std::size_t sample)
        {
          sample_ = sample;
        }

        /** \brief Get the value of the internal \a sample_ parameter.
          */
        inline std::size_t
        getSample ()
        {
          return (sample_);
        }

        /** \brief Set seed of random function.
          * \param seed
          */
        inline void
        setSeed (unsigned int  seed)
        {
          seed_ = seed;
        }

        /** \brief Get the value of the internal \a seed_ parameter.
          */
        inline unsigned int 
        getSeed ()
        {
          return (seed_);
        }

      protected:

        /** \brief Number of points that will be returned. */
        std::size_t sample_;
        /** \brief Random number seed. */
        unsigned int seed_;

        /** \brief Sample of point indices into a separate PointCloud
          * \param output the resultant point cloud
          */
        void
        applyFilter (PointCloud &output) override;

              /** \brief Sample of point indices
          * \param indices the resultant point cloud indices
          */
        void
        applyFilter (std::vector<int> &indices) override;

    };
 }
