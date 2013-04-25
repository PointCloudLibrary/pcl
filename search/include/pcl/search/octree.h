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
 */

#ifndef PCL_SEARCH_OCTREE_H
#define PCL_SEARCH_OCTREE_H

#include <pcl/search/search.h>
#include <pcl/octree/octree_search.h>

namespace pcl
{
  namespace search
  {
    /** \brief @b search::Octree is a wrapper class which implements nearest neighbor search operations based on the 
      * pcl::octree::Octree structure. 
      * 
      * The octree pointcloud class needs to be initialized with its voxel
      * resolution. Its bounding box is automatically adjusted according to the
      * pointcloud dimension or it can be predefined. Note: The tree depth
      * equates to the resolution and the bounding box dimensions of the
      * octree.
      *
      * \note typename: PointT: type of point used in pointcloud
      * \note typename: LeafT:  leaf node class (usuallt templated with integer indices values)
      * \note typename: OctreeT: octree implementation ()
      *
      * \author Julius Kammerl
      * \ingroup search
      */
    template<typename PointT,
             typename LeafTWrap = pcl::octree::OctreeContainerPointIndices,
             typename BranchTWrap = pcl::octree::OctreeContainerEmpty,
             typename OctreeT = pcl::octree::OctreeBase<LeafTWrap, BranchTWrap > >
    class Octree: public Search<PointT>
    {
      public:
        // public typedefs
        typedef boost::shared_ptr<pcl::search::Octree<PointT,LeafTWrap,BranchTWrap,OctreeT> > Ptr;
        typedef boost::shared_ptr<const pcl::search::Octree<PointT,LeafTWrap,BranchTWrap,OctreeT> > ConstPtr;

        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        // Boost shared pointers
        typedef boost::shared_ptr<pcl::octree::OctreePointCloudSearch<PointT, LeafTWrap, BranchTWrap> > OctreePointCloudSearchPtr;
        typedef boost::shared_ptr<const pcl::octree::OctreePointCloudSearch<PointT, LeafTWrap, BranchTWrap> > OctreePointCloudSearchConstPtr;
        OctreePointCloudSearchPtr tree_;

        using pcl::search::Search<PointT>::input_;
        using pcl::search::Search<PointT>::indices_;
        using pcl::search::Search<PointT>::sorted_results_;

        /** \brief Octree constructor.
          * \param[in] resolution octree resolution at lowest octree level
          */
        Octree (const double resolution)
          : Search<PointT> ("Octree")
          , tree_ (new pcl::octree::OctreePointCloudSearch<PointT, LeafTWrap, BranchTWrap> (resolution))
        {
        }

        /** \brief Empty Destructor. */
        virtual
        ~Octree ()
        {
        }

        /** \brief Provide a pointer to the input dataset.
          * \param[in] cloud the const boost shared pointer to a PointCloud message
          */
        inline void
        setInputCloud (const PointCloudConstPtr &cloud)
        {
          tree_->deleteTree ();
          tree_->setInputCloud (cloud);
          tree_->addPointsFromInputCloud ();
          input_ = cloud;
        }

        /** \brief Provide a pointer to the input dataset.
          * \param[in] cloud the const boost shared pointer to a PointCloud message
          * \param[in] indices the point indices subset that is to be used from \a cloud 
          */
        inline void
        setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr& indices)
        {
          tree_->deleteTree ();
          tree_->setInputCloud (cloud, indices);
          tree_->addPointsFromInputCloud ();
          input_ = cloud;
          indices_ = indices;
        }

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] cloud the point cloud data
          * \param[in] index the index in \a cloud representing the query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        inline int
        nearestKSearch (const PointCloud &cloud, int index, int k, std::vector<int> &k_indices,
                        std::vector<float> &k_sqr_distances) const
        {
          return (tree_->nearestKSearch (cloud, index, k, k_indices, k_sqr_distances));
        }

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] point the given query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        inline int
        nearestKSearch (const PointT &point, int k, std::vector<int> &k_indices,
                        std::vector<float> &k_sqr_distances) const
        {
          return (tree_->nearestKSearch (point, k, k_indices, k_sqr_distances));
        }

        /** \brief Search for the k-nearest neighbors for the given query point (zero-copy).
          *
          * \param[in] index the index representing the query point in the
          * dataset given by \a setInputCloud if indices were given in
          * setInputCloud, index will be the position in the indices vector
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        inline int
        nearestKSearch (int index, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) const
        {
          return (tree_->nearestKSearch (index, k, k_indices, k_sqr_distances));
        }

        /** \brief search for all neighbors of query point that are within a given radius.
         * \param cloud the point cloud data
         * \param index the index in \a cloud representing the query point
         * \param radius the radius of the sphere bounding all of p_q's neighbors
         * \param k_indices the resultant indices of the neighboring points
         * \param k_sqr_distances the resultant squared distances to the neighboring points
         * \param max_nn if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        inline int
        radiusSearch (const PointCloud &cloud, 
                      int index, 
                      double radius,
                      std::vector<int> &k_indices, 
                      std::vector<float> &k_sqr_distances, 
                      unsigned int max_nn = 0) const
        {
          tree_->radiusSearch (cloud, index, radius, k_indices, k_sqr_distances, max_nn);
          if (sorted_results_)
            this->sortResults (k_indices, k_sqr_distances);
          return (static_cast<int> (k_indices.size ()));
        }

        /** \brief search for all neighbors of query point that are within a given radius.
         * \param p_q the given query point
         * \param radius the radius of the sphere bounding all of p_q's neighbors
         * \param k_indices the resultant indices of the neighboring points
         * \param k_sqr_distances the resultant squared distances to the neighboring points
         * \param max_nn if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        inline int
        radiusSearch (const PointT &p_q, 
                      double radius, 
                      std::vector<int> &k_indices,
                      std::vector<float> &k_sqr_distances, 
                      unsigned int max_nn = 0) const
        {
          tree_->radiusSearch (p_q, radius, k_indices, k_sqr_distances, max_nn);
          if (sorted_results_)
            this->sortResults (k_indices, k_sqr_distances);
          return (static_cast<int> (k_indices.size ()));
        }

        /** \brief search for all neighbors of query point that are within a given radius.
         * \param index index representing the query point in the dataset given by \a setInputCloud.
         *        If indices were given in setInputCloud, index will be the position in the indices vector
         * \param radius radius of the sphere bounding all of p_q's neighbors
         * \param k_indices the resultant indices of the neighboring points
         * \param k_sqr_distances the resultant squared distances to the neighboring points
         * \param max_nn if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        inline int
        radiusSearch (int index, double radius, std::vector<int> &k_indices,
                      std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const
        {
          tree_->radiusSearch (index, radius, k_indices, k_sqr_distances, max_nn);
          if (sorted_results_)
            this->sortResults (k_indices, k_sqr_distances);
          return (static_cast<int> (k_indices.size ()));
        }


        /** \brief Search for approximate nearest neighbor at the query point.
          * \param[in] cloud the point cloud data
          * \param[in] query_index the index in \a cloud representing the query point
          * \param[out] result_index the resultant index of the neighbor point
          * \param[out] sqr_distance the resultant squared distance to the neighboring point
          * \return number of neighbors found
          */
        inline void
        approxNearestSearch (const PointCloudConstPtr &cloud, int query_index, int &result_index,
                             float &sqr_distance)
        {
          return (tree_->approxNearestSearch (cloud->points[query_index], result_index, sqr_distance));
        }

        /** \brief Search for approximate nearest neighbor at the query point.
          * \param[in] p_q the given query point
          * \param[out] result_index the resultant index of the neighbor point
          * \param[out] sqr_distance the resultant squared distance to the neighboring point
          */
        inline void
        approxNearestSearch (const PointT &p_q, int &result_index, float &sqr_distance)
        {
          return (tree_->approxNearestSearch (p_q, result_index, sqr_distance));
        }

        /** \brief Search for approximate nearest neighbor at the query point.
          * \param query_index index representing the query point in the dataset given by \a setInputCloud.
          *        If indices were given in setInputCloud, index will be the position in the indices vector.
          * \param result_index the resultant index of the neighbor point
          * \param sqr_distance the resultant squared distance to the neighboring point
          * \return number of neighbors found
          */
        inline void
        approxNearestSearch (int query_index, int &result_index, float &sqr_distance)
        {
          return (tree_->approxNearestSearch (query_index, result_index, sqr_distance));
        }

    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/octree/impl/octree_search.hpp>
#else
#define PCL_INSTANTIATE_Octree(T) template class PCL_EXPORTS pcl::search::Octree<T>;
#endif

#endif    // PCL_SEARCH_OCTREE_H
