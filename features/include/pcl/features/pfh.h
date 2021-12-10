/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <map>
#include <queue> // for std::queue

namespace pcl
{
  /** \brief PFHEstimation estimates the Point Feature Histogram (PFH) descriptor for a given point cloud dataset
    * containing points and normals.
    *
    * A commonly used type for PointOutT is pcl::PFHSignature125.
    *
    * \note If you use this code in any academic work, please cite:
    *
    *   - R.B. Rusu, N. Blodow, Z.C. Marton, M. Beetz.
    *     Aligning Point Cloud Views using Persistent Feature Histograms.
    *     In Proceedings of the 21st IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS),
    *     Nice, France, September 22-26 2008.
    *   - R.B. Rusu, Z.C. Marton, N. Blodow, M. Beetz.
    *     Learning Informative Point Classes for the Acquisition of Object Model Maps.
    *     In Proceedings of the 10th International Conference on Control, Automation, Robotics and Vision (ICARCV),
    *     Hanoi, Vietnam, December 17-20 2008.
    *
    * \attention 
    * The convention for PFH features is:
    *   - if a query point's nearest neighbors cannot be estimated, the PFH feature will be set to NaN 
    *     (not a number)
    *   - it is impossible to estimate a PFH descriptor for a point that
    *     doesn't have finite 3D coordinates. Therefore, any point that contains
    *     NaN data on x, y, or z, will have its PFH feature property set to NaN.
    *
    * \note The code is stateful as we do not expect this class to be multicore parallelized. Please look at
    * \ref FPFHEstimationOMP for examples on parallel implementations of the FPFH (Fast Point Feature Histogram).
    *
    * \author Radu B. Rusu
    * \ingroup features
    */
  template <typename PointInT, typename PointNT, typename PointOutT = pcl::PFHSignature125>
  class PFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<PFHEstimation<PointInT, PointNT, PointOutT> >;
      using ConstPtr = shared_ptr<const PFHEstimation<PointInT, PointNT, PointOutT> >;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::input_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;
      using PointCloudIn = typename Feature<PointInT, PointOutT>::PointCloudIn;

      /** \brief Empty constructor. 
        * Sets \a use_cache_ to false, \a nr_subdiv_ to 5, and the internal maximum cache size to 1GB.
        */
      PFHEstimation () : 
        nr_subdiv_ (5), 
        d_pi_ (1.0f / (2.0f * static_cast<float> (M_PI))), 
        key_list_ (),
        // Default 1GB memory size. Need to set it to something more conservative.
        max_cache_size_ ((1ul*1024ul*1024ul*1024ul) / sizeof (std::pair<std::pair<int, int>, Eigen::Vector4f>)),
        use_cache_ (false)
      {
        feature_name_ = "PFHEstimation";
      };

      /** \brief Set the maximum internal cache size. Defaults to 2GB worth of entries.
        * \param[in] cache_size maximum cache size 
        */
      inline void
      setMaximumCacheSize (unsigned int cache_size)
      {
        max_cache_size_ = cache_size;
      }

      /** \brief Get the maximum internal cache size. */
      inline unsigned int 
      getMaximumCacheSize ()
      {
        return (max_cache_size_);
      }

      /** \brief Set whether to use an internal cache mechanism for removing redundant calculations or not. 
        *
        * \note Depending on how the point cloud is ordered and how the nearest
        * neighbors are estimated, using a cache could have a positive or a
        * negative influence. Please test with and without a cache on your
        * data, and choose whatever works best!
        *
        * See \ref setMaximumCacheSize for setting the maximum cache size
        *
        * \param[in] use_cache set to true to use the internal cache, false otherwise
        */
      inline void
      setUseInternalCache (bool use_cache)
      {
        use_cache_ = use_cache;
      }

      /** \brief Get whether the internal cache is used or not for computing the PFH features. */
      inline bool
      getUseInternalCache ()
      {
        return (use_cache_);
      }

      /** \brief Compute the 4-tuple representation containing the three angles and one distance between two points
        * represented by Cartesian coordinates and normals.
        * \note For explanations about the features, please see the literature mentioned above (the order of the
        * features might be different).
        * \param[in] cloud the dataset containing the XYZ Cartesian coordinates of the two points
        * \param[in] normals the dataset containing the surface normals (assuming normalized vectors) at each point in cloud
        * \param[in] p_idx the index of the first point (source)
        * \param[in] q_idx the index of the second point (target)
        * \param[out] f1 the first angular feature (angle between the projection of nq_idx and u)
        * \param[out] f2 the second angular feature (angle between nq_idx and v)
        * \param[out] f3 the third angular feature (angle between np_idx and |p_idx - q_idx|)
        * \param[out] f4 the distance feature (p_idx - q_idx)
        * \note For efficiency reasons, we assume that the point data passed to the method is finite.
        */
      bool 
      computePairFeatures (const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals, 
                           int p_idx, int q_idx, float &f1, float &f2, float &f3, float &f4);

      /** \brief Estimate the PFH (Point Feature Histograms) individual signatures of the three angular (f1, f2, f3)
        * features for a given point based on its spatial neighborhood of 3D points with normals
        * \param[in] cloud the dataset containing the XYZ Cartesian coordinates of the two points
        * \param[in] normals the dataset containing the surface normals at each point in \a cloud
        * \param[in] indices the k-neighborhood point indices in the dataset
        * \param[in] nr_split the number of subdivisions for each angular feature interval
        * \param[out] pfh_histogram the resultant (combinatorial) PFH histogram representing the feature at the query point
        */
      void 
      computePointPFHSignature (const pcl::PointCloud<PointInT> &cloud, const pcl::PointCloud<PointNT> &normals, 
                                const pcl::Indices &indices, int nr_split, Eigen::VectorXf &pfh_histogram);

    protected:
      /** \brief Estimate the Point Feature Histograms (PFH) descriptors at a set of points given by
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () and the spatial locator in
        * setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset that contains the PFH feature estimates
        */
      void 
      computeFeature (PointCloudOut &output) override;

      /** \brief The number of subdivisions for each angular feature interval. */
      int nr_subdiv_;

      /** \brief Placeholder for a point's PFH signature. */
      Eigen::VectorXf pfh_histogram_;

      /** \brief Placeholder for a PFH 4-tuple. */
      Eigen::Vector4f pfh_tuple_;

      /** \brief Placeholder for a histogram index. */
      int f_index_[3];

      /** \brief Float constant = 1.0 / (2.0 * M_PI) */
      float d_pi_; 

      /** \brief Internal hashmap, used to optimize efficiency of redundant computations. */
      std::map<std::pair<int, int>, Eigen::Vector4f, std::less<>, Eigen::aligned_allocator<std::pair<const std::pair<int, int>, Eigen::Vector4f> > > feature_map_;

      /** \brief Queue of pairs saved, used to constrain memory usage. */
      std::queue<std::pair<int, int> > key_list_;

      /** \brief Maximum size of internal cache memory. */
      unsigned int max_cache_size_;

      /** \brief Set to true to use the internal cache for removing redundant computations. */
      bool use_cache_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/pfh.hpp>
#endif
