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

#ifndef PCL_EXTRACT_CLUSTERS_H_
#define PCL_EXTRACT_CLUSTERS_H_

#include <pcl/pcl_base.h>

#include <pcl/search/pcl_search.h>

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
      const PointCloud<PointT> &cloud, const boost::shared_ptr<search::Search<PointT> > &tree, 
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
      const PointCloud<PointT> &cloud, const std::vector<int> &indices, 
      const boost::shared_ptr<search::Search<PointT> > &tree, float tolerance, std::vector<PointIndices> &clusters, 
      unsigned int min_pts_per_cluster = 1, unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ());

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
    * angular deviation
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
      float tolerance, const boost::shared_ptr<KdTree<PointT> > &tree, 
      std::vector<PointIndices> &clusters, double eps_angle, 
      unsigned int min_pts_per_cluster = 1, 
      unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
  {
    if (tree->getInputCloud ()->points.size () != cloud.points.size ())
    {
      PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
      return;
    }
    if (cloud.points.size () != normals.points.size ())
    {
      PCL_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%lu) different than normals (%lu)!\n", cloud.points.size (), normals.points.size ());
      return;
    }

    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.points.size (), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      if (processed[i])
        continue;

      std::vector<unsigned int> seed_queue;
      int sq_idx = 0;
      seed_queue.push_back (static_cast<int> (i));

      processed[i] = true;

      while (sq_idx < static_cast<int> (seed_queue.size ()))
      {
        // Search for sq_idx
        if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
        {
          sq_idx++;
          continue;
        }

        for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
        {
          if (processed[nn_indices[j]])                         // Has this point been processed before ?
            continue;

          //processed[nn_indices[j]] = true;
          // [-1;1]
          double dot_p = normals.points[i].normal[0] * normals.points[nn_indices[j]].normal[0] +
                         normals.points[i].normal[1] * normals.points[nn_indices[j]].normal[1] +
                         normals.points[i].normal[2] * normals.points[nn_indices[j]].normal[2];
          if ( fabs (acos (dot_p)) < eps_angle )
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
        for (size_t j = 0; j < seed_queue.size (); ++j)
          r.indices[j] = seed_queue[j];

        // These two lines should not be needed: (can anyone confirm?) -FF
        std::sort (r.indices.begin (), r.indices.end ());
        r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

        r.header = cloud.header;
        clusters.push_back (r);   // We could avoid a copy by working directly in the vector
      }
    }
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
    * angular deviation
    * \param cloud the point cloud message
    * \param normals the point cloud message containing normal information
    * \param indices a list of point indices to use from \a cloud
    * \param tree the spatial locator (e.g., kd-tree) used for nearest neighbors searching
    * \note the tree has to be created as a spatial locator on \a cloud
    * \param tolerance the spatial cluster tolerance as a measure in the L2 Euclidean space
    * \param clusters the resultant clusters containing point indices (as PointIndices)
    * \param eps_angle the maximum allowed difference between normals in degrees for cluster/region growing
    * \param min_pts_per_cluster minimum number of points that a cluster may contain (default: 1)
    * \param max_pts_per_cluster maximum number of points that a cluster may contain (default: max int)
    * \ingroup segmentation
    */
  template <typename PointT, typename Normal> 
  void extractEuclideanClusters (
      const PointCloud<PointT> &cloud, const PointCloud<Normal> &normals, 
      const std::vector<int> &indices, const boost::shared_ptr<KdTree<PointT> > &tree, 
      float tolerance, std::vector<PointIndices> &clusters, double eps_angle, 
      unsigned int min_pts_per_cluster = 1, 
      unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
  {
    // \note If the tree was created over <cloud, indices>, we guarantee a 1-1 mapping between what the tree returns
    //and indices[i]
    if (tree->getInputCloud ()->points.size () != cloud.points.size ())
    {
      PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
      return;
    }
    if (tree->getIndices ()->size () != indices.size ())
    {
      PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different set of indices (%lu) than the input set (%lu)!\n", tree->getIndices ()->size (), indices.size ());
      return;
    }
    if (cloud.points.size () != normals.points.size ())
    {
      PCL_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%lu) different than normals (%lu)!\n", cloud.points.size (), normals.points.size ());
      return;
    }
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.points.size (), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (size_t i = 0; i < indices.size (); ++i)
    {
      if (processed[indices[i]])
        continue;

      std::vector<int> seed_queue;
      int sq_idx = 0;
      seed_queue.push_back (indices[i]);

      processed[indices[i]] = true;

      while (sq_idx < static_cast<int> (seed_queue.size ()))
      {
        // Search for sq_idx
        if (!tree->radiusSearch (cloud.points[seed_queue[sq_idx]], tolerance, nn_indices, nn_distances))
        {
          sq_idx++;
          continue;
        }

        for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
        {
          if (processed[nn_indices[j]])                             // Has this point been processed before ?
            continue;

          //processed[nn_indices[j]] = true;
          // [-1;1]
          double dot_p =
            normals.points[indices[i]].normal[0] * normals.points[indices[nn_indices[j]]].normal[0] +
            normals.points[indices[i]].normal[1] * normals.points[indices[nn_indices[j]]].normal[1] +
            normals.points[indices[i]].normal[2] * normals.points[indices[nn_indices[j]]].normal[2];
          if ( fabs (acos (dot_p)) < eps_angle )
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
        for (size_t j = 0; j < seed_queue.size (); ++j)
          r.indices[j] = seed_queue[j];

        // These two lines should not be needed: (can anyone confirm?) -FF
        std::sort (r.indices.begin (), r.indices.end ());
        r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

        r.header = cloud.header;
        clusters.push_back (r);
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
    typedef PCLBase<PointT> BasePCLBase;

    public:
      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      typedef typename pcl::search::Search<PointT> KdTree;
      typedef typename pcl::search::Search<PointT>::Ptr KdTreePtr;

      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      EuclideanClusterExtraction () : tree_ (), 
                                      cluster_tolerance_ (0),
                                      min_pts_per_cluster_ (1), 
                                      max_pts_per_cluster_ (std::numeric_limits<int>::max ())
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
      setMinClusterSize (int min_cluster_size) 
      { 
        min_pts_per_cluster_ = min_cluster_size; 
      }

      /** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid. */
      inline int 
      getMinClusterSize () const 
      { 
        return (min_pts_per_cluster_); 
      }

      /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid.
        * \param[in] max_cluster_size the maximum cluster size
        */
      inline void 
      setMaxClusterSize (int max_cluster_size) 
      { 
        max_pts_per_cluster_ = max_cluster_size; 
      }

      /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid. */
      inline int 
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
      int min_pts_per_cluster_;

      /** \brief The maximum number of points that a cluster needs to contain in order to be considered valid (default = MAXINT). */
      int max_pts_per_cluster_;

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

#endif  //#ifndef PCL_EXTRACT_CLUSTERS_H_
