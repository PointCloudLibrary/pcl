/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#include <pcl/pcl_macros.h>
#include <pcl/common/utils.h>
#include <pcl/filters/filter.h>

namespace pcl
{
  /** \brief Assign weights of nearby normals used for refinement
   * \todo Currently, this function equalizes all weights to 1
   * @param cloud the point cloud data
   * @param index a \a valid index in \a cloud representing a \a valid (i.e., finite) query point
   * @param k_indices indices of neighboring points
   * @param k_sqr_distances squared distances to the neighboring points
   * @return weights
   * \ingroup filters
   */
  template <typename NormalT> inline std::vector<float>
  assignNormalWeights (const PointCloud<NormalT>& cloud,
                       index_t index,
                       const Indices& k_indices,
                       const std::vector<float>& k_sqr_distances)
  {
    pcl::utils::ignore(cloud, index);
    // Check inputs
    if (k_indices.size () != k_sqr_distances.size ())
      PCL_ERROR("[pcl::assignNormalWeights] inequal size of neighbor indices and distances!\n");
    
    // TODO: For now we use uniform weights
    return (std::vector<float> (k_indices.size (), 1.0f));
  }
  
  /** \brief Refine an indexed point based on its neighbors, this function only writes to the normal_* fields
   * 
   * \note If the indexed point has only NaNs in its neighborhood, the resulting normal will be zero.
   * 
   * @param cloud the point cloud data
   * @param index a \a valid index in \a cloud representing a \a valid (i.e., finite) query point
   * @param k_indices indices of neighboring points
   * @param k_sqr_distances squared distances to the neighboring points
   * @param point the output point, only normal_* fields are written
   * @return false if an error occurred (norm of summed normals zero or all neighbors NaN)
   * \ingroup filters
   */
  template <typename NormalT> inline bool
  refineNormal (const PointCloud<NormalT>& cloud,
                int index,
                const Indices& k_indices,
                const std::vector<float>& k_sqr_distances,
                NormalT& point)
  {
    // Start by zeroing result
    point.normal_x = 0.0f;
    point.normal_y = 0.0f;
    point.normal_z = 0.0f;
    
    // Check inputs
    if (k_indices.size () != k_sqr_distances.size ())
    {
      PCL_ERROR("[pcl::refineNormal] inequal size of neighbor indices and distances!\n");
      return (false);
    }
    
    // Get weights
    const std::vector<float> weights = assignNormalWeights (cloud, index, k_indices, k_sqr_distances);
    
    // Loop over all neighbors and accumulate sum of normal components
    float nx = 0.0f;
    float ny = 0.0f;
    float nz = 0.0f;
    for (std::size_t i = 0; i < k_indices.size (); ++i) {
      // Neighbor
      const NormalT& pointi = cloud[k_indices[i]];
      
      // Accumulate if not NaN
      if (std::isfinite (pointi.normal_x) && std::isfinite (pointi.normal_y) && std::isfinite (pointi.normal_z))
      {
        const float& weighti = weights[i];
        nx += weighti * pointi.normal_x;
        ny += weighti * pointi.normal_y;
        nz += weighti * pointi.normal_z;
      }
    }
    
    // Normalize if norm valid and non-zero
    const float norm = std::sqrt (nx * nx + ny * ny + nz * nz);
    if (std::isfinite (norm) && norm > std::numeric_limits<float>::epsilon ())
    {
      point.normal_x = nx / norm;
      point.normal_y = ny / norm;
      point.normal_z = nz / norm;

      return (true);
    }

    return (false);
  }
  
  /** \brief %Normal vector refinement class 
    *
    * This class refines a set of already estimated normals by iteratively updating each normal to the (weighted)
    * mean of all normals in its neighborhood. The intention is that you reuse the same point correspondences
    * as used when estimating the original normals in order to avoid repeating a nearest neighbor search.
    * 
    * \note This class avoids points for which a NaN is encountered in the neighborhood. In the special case
    * where a point has only NaNs in its neighborhood, the resultant refined normal will be set to zero,
    * i.e. this class only produces finite normals.
    * 
    * \details Usage example:
    * 
    * \code
    * // Input point cloud
    * pcl::PointCloud<PointT> cloud;
    * 
    * // Fill cloud...
    * 
    * // Estimated and refined normals
    * pcl::PointCloud<NormalT> normals;
    * pcl::PointCloud<NormalT> normals_refined;
    * 
    * // Search parameters
    * const int k = 5;
    * std::vector<Indices > k_indices;
    * std::vector<std::vector<float> > k_sqr_distances;
    * 
    * // Run search
    * pcl::search::KdTree<pcl::PointXYZRGB> search;
    * search.setInputCloud (cloud.makeShared ());
    * search.nearestKSearch (cloud, Indices (), k, k_indices, k_sqr_distances);
    * 
    * // Use search results for normal estimation
    * pcl::NormalEstimation<PointT, NormalT> ne;
    * for (unsigned int i = 0; i < cloud.size (); ++i)
    * {
    *   NormalT normal;
    *   ne.computePointNormal (cloud, k_indices[i]
    *                          normal.normal_x, normal.normal_y, normal.normal_z, normal.curvature);
    *   pcl::flipNormalTowardsViewpoint (cloud[i], cloud.sensor_origin_[0], cloud.sensor_origin_[1], cloud.sensor_origin_[2],
    *                                    normal.normal_x, normal.normal_y, normal.normal_z);
    *   normals.push_back (normal);
    * }
    * 
    * // Run refinement using search results
    * pcl::NormalRefinement<NormalT> nr (k_indices, k_sqr_distances);
    * nr.setInputCloud (normals.makeShared ());
    * nr.filter (normals_refined);
    * \endcode
    *
    * \author Anders Glent Buch
    * \ingroup filters
    */
  template<typename NormalT>
  class NormalRefinement : public Filter<NormalT>
  {
    using Filter<NormalT>::input_;
    using Filter<NormalT>::filter_name_;
    using Filter<NormalT>::getClassName;

    using PointCloud = typename Filter<NormalT>::PointCloud;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    public:
      /** \brief Empty constructor, sets default convergence parameters
        */
      NormalRefinement () :
        Filter<NormalT>::Filter ()
      {
        filter_name_ = "NormalRefinement";
        setMaxIterations (15);
        setConvergenceThreshold (0.00001f);
      }
      
      /** \brief Constructor for setting correspondences, sets default convergence parameters
       * @param k_indices indices of neighboring points
       * @param k_sqr_distances squared distances to the neighboring points
       */
      NormalRefinement (const std::vector< Indices >& k_indices, const std::vector< std::vector<float> >& k_sqr_distances) :
        Filter<NormalT>::Filter ()
      {
        filter_name_ = "NormalRefinement";
        setCorrespondences (k_indices, k_sqr_distances);
        setMaxIterations (15);
        setConvergenceThreshold (0.00001f);
      }
      
      /** \brief Set correspondences calculated from nearest neighbor search
       * @param k_indices indices of neighboring points
       * @param k_sqr_distances squared distances to the neighboring points
       */
      inline void
      setCorrespondences (const std::vector< Indices >& k_indices, const std::vector< std::vector<float> >& k_sqr_distances)
      {
        k_indices_ = k_indices;
        k_sqr_distances_ = k_sqr_distances;
      }
      
      /** \brief Get correspondences (copy)
       * @param k_indices indices of neighboring points
       * @param k_sqr_distances squared distances to the neighboring points
       */
      inline void
      getCorrespondences (std::vector< Indices >& k_indices, std::vector< std::vector<float> >& k_sqr_distances)
      {
        k_indices.assign (k_indices_.begin (), k_indices_.end ());
        k_sqr_distances.assign (k_sqr_distances_.begin (), k_sqr_distances_.end ());
      }
      
      /** \brief Set maximum iterations
       * @param max_iterations maximum iterations
       */
      inline void
      setMaxIterations (unsigned int max_iterations)
      {
        max_iterations_ = max_iterations;
      }
      
      /** \brief Get maximum iterations
       * @return maximum iterations
       */
      inline unsigned int
      getMaxIterations ()
      {
        return max_iterations_;
      }
      
      /** \brief Set convergence threshold
       * @param convergence_threshold convergence threshold
       */
      inline void
      setConvergenceThreshold (float convergence_threshold)
      {
        convergence_threshold_ = convergence_threshold;
      }

      /** \brief Get convergence threshold
       * @return convergence threshold
       */
      inline float
      getConvergenceThreshold ()
      {
        return convergence_threshold_;
      }

    protected:
      /** \brief Filter a Point Cloud.
        * \param output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output) override;
      
    private:
      /** \brief indices of neighboring points */
      std::vector< Indices > k_indices_;
      
      /** \brief squared distances to the neighboring points */
      std::vector< std::vector<float> > k_sqr_distances_;
      
      /** \brief Maximum allowable iterations over the whole point cloud for refinement */
      unsigned int max_iterations_;
      
      /** \brief Convergence threshold in the interval [0,1] on the mean of 1 - dot products between previous iteration and the current */
      float convergence_threshold_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/normal_refinement.hpp>
#else
#define PCL_INSTANTIATE_NormalRefinement(T) template class PCL_EXPORTS pcl::NormalRefinement<T>;
#endif
