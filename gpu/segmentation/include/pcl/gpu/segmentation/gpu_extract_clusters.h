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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id:$
 * @author: Koen Buys
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/pcl_macros.h>
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>

namespace pcl
{
  namespace gpu
  {
    template <typename PointT> void
    extractEuclideanClusters (const typename pcl::PointCloud<PointT>::Ptr &host_cloud_,
                              const pcl::gpu::Octree::Ptr               &tree,
                              float                                     tolerance,
                              std::vector<PointIndices>                 &clusters,
                              unsigned int                              min_pts_per_cluster,
                              unsigned int                              max_pts_per_cluster);

   /** \brief @b EuclideanClusterExtraction represents a segmentation class for cluster extraction in an Euclidean sense, depending on pcl::gpu::octree
    * \author Koen Buys, Radu Bogdan Rusu
    * \ingroup segmentation
    */
    template <typename PointT>
    class EuclideanClusterExtraction
    {
      public:
        using PointCloudHost = pcl::PointCloud<PointT>;
        using PointCloudHostPtr = typename PointCloudHost::Ptr;
        using PointCloudHostConstPtr = typename PointCloudHost::ConstPtr;

        using PointIndicesPtr = PointIndices::Ptr;
        using PointIndicesConstPtr = PointIndices::ConstPtr;

        using GPUTree = pcl::gpu::Octree;
        using GPUTreePtr = pcl::gpu::Octree::Ptr;

        using CloudDevice = pcl::gpu::Octree::PointCloud;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief Empty constructor. */
        EuclideanClusterExtraction () = default;

        /** \brief the destructor */
/*        ~EuclideanClusterExtraction ()
        {
          tree_.clear();
        };
*/
        /** \brief Provide a pointer to the search object.
          * \param tree a pointer to the spatial search object.
          */
        inline void setSearchMethod (GPUTreePtr &tree) { tree_ = tree; }

        /** \brief Get a pointer to the search method used. 
          *  @todo fix this for a generic search tree
          */
        inline GPUTreePtr getSearchMethod () { return (tree_); }

        /** \brief Set the spatial cluster tolerance as a measure in the L2 Euclidean space
          * \param tolerance the spatial cluster tolerance as a measure in the L2 Euclidean space
          */
        inline void setClusterTolerance (double tolerance) { cluster_tolerance_ = tolerance; }

        /** \brief Get the spatial cluster tolerance as a measure in the L2 Euclidean space. */
        inline double getClusterTolerance () { return (cluster_tolerance_); }

        /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid.
          * \param min_cluster_size the minimum cluster size
          */
        inline void setMinClusterSize (int min_cluster_size) { min_pts_per_cluster_ = min_cluster_size; }

        /** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid. */
        inline int getMinClusterSize () { return (min_pts_per_cluster_); }

        /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid.
          * \param max_cluster_size the maximum cluster size
          */
        inline void setMaxClusterSize (int max_cluster_size) { max_pts_per_cluster_ = max_cluster_size; }

        /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid. */
        inline int getMaxClusterSize () { return (max_pts_per_cluster_); }

        inline void setInput (CloudDevice input) {input_ = input;}

        inline void setHostCloud (PointCloudHostPtr host_cloud) {host_cloud_ = host_cloud;}

        /** \brief Cluster extraction in a PointCloud given by <setInputCloud (), setIndices ()>
          * \param clusters the resultant point clusters
          */
        void extract (std::vector<pcl::PointIndices> &clusters);

      protected:
        /** \brief the input cloud on the GPU */
        CloudDevice input_;

        /** \brief the original cloud the Host */
        PointCloudHostPtr host_cloud_;

        /** \brief A pointer to the spatial search object. */
        GPUTreePtr tree_;

        /** \brief The spatial cluster tolerance as a measure in the L2 Euclidean space. */
        double cluster_tolerance_ {0};

        /** \brief The minimum number of points that a cluster needs to contain in order to be considered valid (default = 1). */
        int min_pts_per_cluster_ {1};

        /** \brief The maximum number of points that a cluster needs to contain in order to be considered valid (default = MAXINT). */
        int max_pts_per_cluster_ {std::numeric_limits<int>::max()};

        /** \brief Class getName method. */
        virtual std::string getClassName () const { return ("gpu::EuclideanClusterExtraction"); }
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
}
