/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#include <pcl/console/print.h> // for PCL_ERROR
#include <pcl/pcl_base.h>

#include <pcl/search/search.h> // for Search
#include <pcl/search/kdtree.h> // for KdTree

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Decompose a region of space into clusters based on the Euclidean distance between points
    * \param cloud the point cloud message
    * \param tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching
    * \note the tree has to be created as a spatial locator on \a cloud
    * \param tolerance the spatial cluster tolerance as a measure in L2 Euclidean space
    * \param clusters the resultant clusters containing point indices (as a vector of PointIndices)
    * \param min_pts_per_cluster minimum number of points that a cluster may contain (default: 1)
    * \param max_pts_per_cluster maximum number of points that a cluster may contain (default: max int)
    * \ingroup segmentation
    */
  template <typename PointT> void 
  extractEuclideanClusters (
      const PointCloud<PointT> &cloud, const typename search::Search<PointT>::Ptr &tree,
      float tolerance, std::vector<PointIndices> &clusters,
      unsigned int min_pts_per_cluster = 1, unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ());

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Decompose a region of space into clusters based on the Euclidean distance between points
    * \param cloud the point cloud message
    * \param indices a list of point indices to use from \a cloud
    * \param tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching
    * \note the tree has to be created as a spatial locator on \a cloud and \a indices
    * \param tolerance the spatial cluster tolerance as a measure in L2 Euclidean space
    * \param clusters the resultant clusters containing point indices (as a vector of PointIndices)
    * \param min_pts_per_cluster minimum number of points that a cluster may contain (default: 1)
    * \param max_pts_per_cluster maximum number of points that a cluster may contain (default: max int)
    * \ingroup segmentation
    */
  template <typename PointT> void 
  extractEuclideanClusters (
      const PointCloud<PointT> &cloud, const Indices &indices,
      const typename search::Search<PointT>::Ptr &tree, float tolerance, std::vector<PointIndices> &clusters,
      unsigned int min_pts_per_cluster = 1, unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ());

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
    * angular deviation between points. Each point added to the cluster is origin to another radius search. Each point
    * within radius range will be compared to the origin in respect to normal angle and euclidean distance. If both
    * are under their respective threshold the point will be added to the cluster. Generally speaking the cluster 
    * algorithm will not stop on smooth surfaces but on surfaces with sharp edges.
    * \param cloud the point cloud message
    * \param normals the point cloud message containing normal information
    * \param tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching
    * \note the tree has to be created as a spatial locator on \a cloud
    * \param tolerance the spatial cluster tolerance as a measure in the L2 Euclidean space
    * \param clusters the resultant clusters containing point indices (as a vector of PointIndices)
    * \param eps_angle the maximum allowed difference between normals in radians for cluster/region growing
    * \param min_pts_per_cluster minimum number of points that a cluster may contain (default: 1)
    * \param max_pts_per_cluster maximum number of points that a cluster may contain (default: max int)
    * \ingroup segmentation
    */
  template <typename PointT, typename Normal> void 
  extractEuclideanClusters (
      const PointCloud<PointT> &cloud, const PointCloud<Normal> &normals,
      float tolerance, const typename KdTree<PointT>::Ptr &tree,
      std::vector<PointIndices> &clusters, double eps_angle,
      unsigned int min_pts_per_cluster = 1,
      unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
  {
    if (tree->getInputCloud ()->size () != cloud.size ())
    {
      PCL_ERROR("[pcl::extractEuclideanClusters] Tree built for a different point "
                "cloud dataset (%zu) than the input cloud (%zu)!\n",
                static_cast<std::size_t>(tree->getInputCloud()->size()),
                static_cast<std::size_t>(cloud.size()));
      return;
    }
    if (cloud.size () != normals.size ())
    {
      PCL_ERROR("[pcl::extractEuclideanClusters] Number of points in the input point "
                "cloud (%zu) different than normals (%zu)!\n",
                static_cast<std::size_t>(cloud.size()),
                static_cast<std::size_t>(normals.size()));
      return;
    }
    const double cos_eps_angle = std::cos (eps_angle); // compute this once instead of acos many times (faster)

    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.size (), false);

    Indices nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (std::size_t i = 0; i < cloud.size (); ++i)
    {
      if (processed[i])
        continue;

      Indices seed_queue;
      int sq_idx = 0;
      seed_queue.push_back (static_cast<index_t> (i));

      processed[i] = true;

      while (sq_idx < static_cast<int> (seed_queue.size ()))
      {
        // Search for sq_idx
        if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
        {
          sq_idx++;
          continue;
        }

        for (std::size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
        {
          if (processed[nn_indices[j]])                         // Has this point been processed before ?
            continue;

          //processed[nn_indices[j]] = true;
          // [-1;1]
          double dot_p = normals[seed_queue[sq_idx]].normal[0] * normals[nn_indices[j]].normal[0] +
                         normals[seed_queue[sq_idx]].normal[1] * normals[nn_indices[j]].normal[1] +
                         normals[seed_queue[sq_idx]].normal[2] * normals[nn_indices[j]].normal[2];
          if ( std::abs (dot_p) > cos_eps_angle )
          {
            processed[nn_indices[j]] = true;
            seed_queue.push_back (nn_indices[j]);
          }
        }

        sq_idx++;
      }

      // If this queue is satisfactory, add to the clusters
      if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
      {
        pcl::PointIndices r;
        r.indices.resize (seed_queue.size ());
        for (std::size_t j = 0; j < seed_queue.size (); ++j)
          r.indices[j] = seed_queue[j];

        // These two lines should not be needed: (can anyone confirm?) -FF
        std::sort (r.indices.begin (), r.indices.end ());
        r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

        r.header = cloud.header;
        clusters.push_back (r);   // We could avoid a copy by working directly in the vector
      }
      else
      {
        PCL_DEBUG("[pcl::extractEuclideanClusters] This cluster has %zu points, which is not between %u and %u points, so it is not a final cluster\n",
                  seed_queue.size (), min_pts_per_cluster, max_pts_per_cluster);
      }
    }
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
    * angular deviation between points. Each point added to the cluster is origin to another radius search. Each point
    * within radius range will be compared to the origin in respect to normal angle and euclidean distance. If both
    * are under their respective threshold the point will be added to the cluster. Generally speaking the cluster 
    * algorithm will not stop on smooth surfaces but on surfaces with sharp edges.
    * \param cloud the point cloud message
    * \param normals the point cloud message containing normal information
    * \param indices a list of point indices to use from \a cloud
    * \param tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching
    * \note the tree has to be created as a spatial locator on \a cloud
    * \param tolerance the spatial cluster tolerance as a measure in the L2 Euclidean space
    * \param clusters the resultant clusters containing point indices (as PointIndices)
    * \param eps_angle the maximum allowed difference between normals in radians for cluster/region growing
    * \param min_pts_per_cluster minimum number of points that a cluster may contain (default: 1)
    * \param max_pts_per_cluster maximum number of points that a cluster may contain (default: max int)
    * \ingroup segmentation
    */
  template <typename PointT, typename Normal> 
  void extractEuclideanClusters (
      const PointCloud<PointT> &cloud, const PointCloud<Normal> &normals,
      const Indices &indices, const typename KdTree<PointT>::Ptr &tree,
      float tolerance, std::vector<PointIndices> &clusters, double eps_angle,
      unsigned int min_pts_per_cluster = 1,
      unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
  {
    // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
    //and indices[i]
    if (tree->getInputCloud()->size() != cloud.size()) {
      PCL_ERROR("[pcl::extractEuclideanClusters] Tree built for a different point "
                "cloud dataset (%zu) than the input cloud (%zu)!\n",
                static_cast<std::size_t>(tree->getInputCloud()->size()),
                static_cast<std::size_t>(cloud.size()));
      return;
    }
    if (tree->getIndices()->size() != indices.size()) {
      PCL_ERROR("[pcl::extractEuclideanClusters] Tree built for a different set of "
                "indices (%zu) than the input set (%zu)!\n",
                static_cast<std::size_t>(tree->getIndices()->size()),
                indices.size());
      return;
    }
    if (cloud.size() != normals.size()) {
      PCL_ERROR("[pcl::extractEuclideanClusters] Number of points in the input point "
                "cloud (%zu) different than normals (%zu)!\n",
                static_cast<std::size_t>(cloud.size()),
                static_cast<std::size_t>(normals.size()));
      return;
    }
    const double cos_eps_angle = std::cos (eps_angle); // compute this once instead of acos many times (faster)
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.size (), false);

    Indices nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (const auto& point_idx : indices)
    {
      if (processed[point_idx])
        continue;

      Indices seed_queue;
      int sq_idx = 0;
      seed_queue.push_back (point_idx);

      processed[point_idx] = true;

      while (sq_idx < static_cast<int> (seed_queue.size ()))
      {
        // Search for sq_idx
        if (!tree->radiusSearch (cloud[seed_queue[sq_idx]], tolerance, nn_indices, nn_distances))
        {
          sq_idx++;
          continue;
        }

        for (std::size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
        {
          if (processed[nn_indices[j]])                             // Has this point been processed before ?
            continue;

          //processed[nn_indices[j]] = true;
          // [-1;1]
          double dot_p = normals[seed_queue[sq_idx]].normal[0] * normals[nn_indices[j]].normal[0] +
                         normals[seed_queue[sq_idx]].normal[1] * normals[nn_indices[j]].normal[1] +
                         normals[seed_queue[sq_idx]].normal[2] * normals[nn_indices[j]].normal[2];
          if ( std::abs (dot_p) > cos_eps_angle )
          {
            processed[nn_indices[j]] = true;
            seed_queue.push_back (nn_indices[j]);
          }
        }

        sq_idx++;
      }

      // If this queue is satisfactory, add to the clusters
      if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
      {
        pcl::PointIndices r;
        r.indices.resize (seed_queue.size ());
        for (std::size_t j = 0; j < seed_queue.size (); ++j)
          r.indices[j] = seed_queue[j];

        // These two lines should not be needed: (can anyone confirm?) -FF
        std::sort (r.indices.begin (), r.indices.end ());
        r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

        r.header = cloud.header;
        clusters.push_back (r);
      }
      else
      {
        PCL_DEBUG("[pcl::extractEuclideanClusters] This cluster has %zu points, which is not between %u and %u points, so it is not a final cluster\n",
                  seed_queue.size (), min_pts_per_cluster, max_pts_per_cluster);
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b EuclideanClusterExtraction represents a segmentation class for cluster extraction in an Euclidean sense.
    * \author Radu Bogdan Rusu
    * \ingroup segmentation
    */
  template <typename PointT>
  class EuclideanClusterExtraction: public PCLBase<PointT>
  {
    using BasePCLBase = PCLBase<PointT>;

    public:
      using PointCloud = pcl::PointCloud<PointT>;
      using PointCloudPtr = typename PointCloud::Ptr;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      using KdTree = pcl::search::Search<PointT>;
      using KdTreePtr = typename KdTree::Ptr;

      using PointIndicesPtr = PointIndices::Ptr;
      using PointIndicesConstPtr = PointIndices::ConstPtr;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      EuclideanClusterExtraction () : tree_ (), 
                                      cluster_tolerance_ (0),
                                      min_pts_per_cluster_ (1), 
                                      max_pts_per_cluster_ (std::numeric_limits<pcl::uindex_t>::max ())
      {};

      /** \brief Provide a pointer to the search object.
        * \param[in] tree a pointer to the spatial search object.
        */
      inline void 
      setSearchMethod (const KdTreePtr &tree) 
      { 
        tree_ = tree; 
      }

      /** \brief Get a pointer to the search method used. 
       *  @todo fix this for a generic search tree
       */
      inline KdTreePtr 
      getSearchMethod () const 
      { 
        return (tree_); 
      }

      /** \brief Set the spatial cluster tolerance as a measure in the L2 Euclidean space
        * \param[in] tolerance the spatial cluster tolerance as a measure in the L2 Euclidean space
        */
      inline void 
      setClusterTolerance (double tolerance) 
      { 
        cluster_tolerance_ = tolerance; 
      }

      /** \brief Get the spatial cluster tolerance as a measure in the L2 Euclidean space. */
      inline double 
      getClusterTolerance () const 
      { 
        return (cluster_tolerance_); 
      }

      /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid.
        * \param[in] min_cluster_size the minimum cluster size
        */
      inline void 
      setMinClusterSize (pcl::uindex_t min_cluster_size)
      { 
        min_pts_per_cluster_ = min_cluster_size; 
      }

      /** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid. */
      inline pcl::uindex_t
      getMinClusterSize () const 
      { 
        return (min_pts_per_cluster_); 
      }

      /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid.
        * \param[in] max_cluster_size the maximum cluster size
        */
      inline void 
      setMaxClusterSize (pcl::uindex_t max_cluster_size)
      { 
        max_pts_per_cluster_ = max_cluster_size; 
      }

      /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid. */
      inline pcl::uindex_t
      getMaxClusterSize () const 
      { 
        return (max_pts_per_cluster_); 
      }

      /** \brief Cluster extraction in a PointCloud given by <setInputCloud (), setIndices ()>
        * \param[out] clusters the resultant point clusters
        */
      void 
      extract (std::vector<PointIndices> &clusters);

    protected:
      // Members derived from the base class
      using BasePCLBase::input_;
      using BasePCLBase::indices_;
      using BasePCLBase::initCompute;
      using BasePCLBase::deinitCompute;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief The spatial cluster tolerance as a measure in the L2 Euclidean space. */
      double cluster_tolerance_;

      /** \brief The minimum number of points that a cluster needs to contain in order to be considered valid (default = 1). */
      pcl::uindex_t min_pts_per_cluster_;

      /** \brief The maximum number of points that a cluster needs to contain in order to be considered valid (default = MAXINT). */
      pcl::uindex_t max_pts_per_cluster_;

      /** \brief Class getName method. */
      virtual std::string getClassName () const { return ("EuclideanClusterExtraction"); }

  };

  /** \brief Sort clusters method (for std::sort). 
    * \ingroup segmentation
    */
  inline bool 
  comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b)
  {
    return (a.indices.size () < b.indices.size ());
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/extract_clusters.hpp>
#endif
