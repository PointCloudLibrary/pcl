/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#ifndef OCTREE_SEARCH_H
#define OCTREE_SEARCH_H

#include "octree_pointcloud.h"

#include "octree_base.h"
#include "octree2buf_base.h"

#include "octree_nodes.h"

namespace pcl
{
  namespace octree
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud search class
     *  \note This class provides several methods for spatial neighbor search based on octree structure
     *  \note typename: PointT: type of point used in pointcloud
     *  \ingroup octree
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT = OctreeLeafDataTVector<int> , typename OctreeT = OctreeBase<int, LeafT> >
      class OctreePointCloudSearch : public OctreePointCloud<PointT, LeafT, OctreeT>
      {

      public:
        // public typedefs
        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        // public typedefs for single/double buffering
        typedef OctreePointCloudSearch<PointT, LeafT, OctreeBase<int, LeafT> > SingleBuffer;
        typedef OctreePointCloudSearch<PointT, LeafT, Octree2BufBase<int, LeafT> > DoubleBuffer;
        typedef OctreePointCloudSearch<PointT, LeafT, OctreeLowMemBase<int, LeafT> > LowMem;

        // Boost shared pointers
        typedef boost::shared_ptr<OctreePointCloudSearch<PointT, LeafT, OctreeT> > Ptr;
        typedef boost::shared_ptr<const OctreePointCloudSearch<PointT, LeafT, OctreeT> > ConstPtr;

        typedef typename OctreeT::OctreeBranch OctreeBranch;
        typedef typename OctreeT::OctreeKey OctreeKey;
        typedef typename OctreeT::OctreeLeaf OctreeLeaf;

        /** \brief Constructor.
         *  \param resolution_arg: octree resolution at lowest octree level
         * */
        OctreePointCloudSearch (const double resolution_arg) :
          OctreePointCloud<PointT, LeafT, OctreeT> (resolution_arg)
        {
        }

        /** \brief Empty class constructor. */
        virtual
        ~OctreePointCloudSearch ()
        {
        }

        /** \brief Search for neighbors within a voxel at given point
         *  \param point_arg point addressing a leaf node voxel
         *  \param pointIdx_data_arg the resultant indices of the neighboring voxel points
         *  \return "true" if leaf node exist; "false" otherwise
         * */
        bool
        voxelSearch (const PointT& point_arg, std::vector<int>& pointIdx_data_arg);

        /** \brief Search for neighbors within a voxel at given point referenced by a point index
         *  \param index_arg the index in input cloud defining the query point
         *  \param pointIdx_data_arg the resultant indices of the neighboring voxel points
         *  \return "true" if leaf node exist; "false" otherwise
         * */
        bool
        voxelSearch (const int index_arg, std::vector<int>& pointIdx_data_arg);

        /** \brief Search for k-nearest neighbors at the query point.
         * \param cloud_arg the point cloud data
         * \param index_arg the index in \a cloud representing the query point
         * \param k_arg the number of neighbors to search for
         * \param k_indices_arg the resultant indices of the neighboring points (must be resized to \a k a priori!)
         * \param k_sqr_distances_arg the resultant squared distances to the neighboring points (must be resized to \a k
         * a priori!)
         * \return number of neighbors found
         */
        int
        nearestKSearch (const PointCloudConstPtr &cloud_arg, int index_arg, int k_arg, std::vector<int> &k_indices_arg,
                        std::vector<float> &k_sqr_distances_arg);

        /** \brief Search for k-nearest neighbors at given query point.
         * @param p_q_arg the given query point
         * @param k_arg the number of neighbors to search for
         * @param k_indices_arg the resultant indices of the neighboring points (must be resized to k a priori!)
         * @param k_sqr_distances_arg  the resultant squared distances to the neighboring points (must be resized to k a priori!)
         * @return number of neighbors found
         */
        int
        nearestKSearch (const PointT &p_q_arg, int k_arg, std::vector<int> &k_indices_arg,
                        std::vector<float> &k_sqr_distances_arg);

        /** \brief Search for k-nearest neighbors at query point
         * \param index_arg index representing the query point in the dataset given by \a setInputCloud.
         *        If indices were given in setInputCloud, index will be the position in the indices vector.
         * \param k_arg the number of neighbors to search for
         * \param k_indices_arg the resultant indices of the neighboring points (must be resized to \a k a priori!)
         * \param k_sqr_distances_arg the resultant squared distances to the neighboring points (must be resized to \a k
         * a priori!)
         * \return number of neighbors found
         */
        int
        nearestKSearch (int index_arg, int k_arg, std::vector<int> &k_indices_arg,
                        std::vector<float> &k_sqr_distances_arg);

        /** \brief Search for approx. nearest neighbor at the query point.
         * \param cloud_arg the point cloud data
         * \param query_index_arg the index in \a cloud representing the query point
         * \param result_index_arg the resultant index of the neighbor point
         * \param sqr_distance_arg the resultant squared distance to the neighboring point
         * \return number of neighbors found
         */
        void
        approxNearestSearch (const PointCloudConstPtr &cloud_arg, int query_index_arg, int &result_index_arg,
                             float &sqr_distance_arg);

        /** \brief Search for approx. nearest neighbor at the query point.
         * @param p_q_arg the given query point
         * \param result_index_arg the resultant index of the neighbor point
         * \param sqr_distance_arg the resultant squared distance to the neighboring point
         */
        void
        approxNearestSearch (const PointT &p_q_arg, int &result_index_arg, float &sqr_distance_arg);

        /** \brief Search for approx. nearest neighbor at the query point.
         * \param query_index_arg index representing the query point in the dataset given by \a setInputCloud.
         *        If indices were given in setInputCloud, index will be the position in the indices vector.
         * \param result_index_arg the resultant index of the neighbor point
         * \param sqr_distance_arg the resultant squared distance to the neighboring point
         * \return number of neighbors found
         */
        void
        approxNearestSearch (int query_index_arg, int &result_index_arg, float &sqr_distance_arg);

        /** \brief Search for all neighbors of query point that are within a given radius.
         * \param cloud_arg the point cloud data
         * \param index_arg the index in \a cloud representing the query point
         * \param radius_arg the radius of the sphere bounding all of p_q's neighbors
         * \param k_indices_arg the resultant indices of the neighboring points
         * \param k_sqr_distances_arg the resultant squared distances to the neighboring points
         * \param max_nn_arg if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        int
        radiusSearch (const PointCloudConstPtr &cloud_arg, int index_arg, double radius_arg,
                      std::vector<int> &k_indices_arg, std::vector<float> &k_sqr_distances_arg,
                      int max_nn_arg = INT_MAX);

        /** \brief Search for all neighbors of query point that are within a given radius.
         * \param p_q_arg the given query point
         * \param radius_arg the radius of the sphere bounding all of p_q's neighbors
         * \param k_indices_arg the resultant indices of the neighboring points
         * \param k_sqr_distances_arg the resultant squared distances to the neighboring points
         * \param max_nn_arg if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        int
        radiusSearch (const PointT &p_q_arg, const double radius_arg, std::vector<int> &k_indices_arg,
                      std::vector<float> &k_sqr_distances_arg, int max_nn_arg = INT_MAX) const;

        /** \brief Search for all neighbors of query point that are within a given radius.
         * \param index_arg index representing the query point in the dataset given by \a setInputCloud.
         *        If indices were given in setInputCloud, index will be the position in the indices vector
         * \param radius_arg radius of the sphere bounding all of p_q's neighbors
         * \param k_indices_arg the resultant indices of the neighboring points
         * \param k_sqr_distances_arg the resultant squared distances to the neighboring points
         * \param max_nn_arg if given, bounds the maximum returned neighbors to this value
         * \return number of neighbors found in radius
         */
        int
        radiusSearch (int index_arg, const double radius_arg, std::vector<int> &k_indices_arg,
                      std::vector<float> &k_sqr_distances_arg, int max_nn_arg = INT_MAX) const;

      protected:

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Octree-based search routines & helpers
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief @b Priority queue entry for branch nodes
         *  \note This class defines priority queue entries for the nearest neighbor search.
         *  \author Julius Kammerl (julius@kammerl.de)
         */
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        class prioBranchQueueEntry
        {
        public:

          /** \brief Empty constructor  */
          prioBranchQueueEntry ()
          {
          }

          /** \brief Constructor for initializing priority queue entry.
           * \param node_arg pointer to octree node
           * \param key_arg octree key addressing voxel in octree structure
           * \param pointDistance_arg distance of query point to voxel center
           * */
          prioBranchQueueEntry (OctreeNode* node_arg, OctreeKey& key_arg, double pointDistance_arg)
          {
            node = node_arg;
            pointDistance = pointDistance_arg;
            key = key_arg;
          }

          /** \brief Operator< for comparing priority queue entries with each other.  */
          bool
          operator< (const prioBranchQueueEntry rhs_arg) const
          {
            return (this->pointDistance > rhs_arg.pointDistance);
          }

          // pointer to octree node
          const OctreeNode* node;

          // distance to query point
          double pointDistance;

          // octree key
          OctreeKey key;

        };

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief @b Priority queue entry for point candidates
         *  \note This class defines priority queue entries for the nearest neighbor point candidates.
         *  \author Julius Kammerl (julius@kammerl.de)
         */
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        class prioPointQueueEntry
        {
        public:

          /** \brief Empty constructor  */
          prioPointQueueEntry ()
          {
          }

          /** \brief Constructor for initializing priority queue entry.
           * \param pointIdx_arg an index representing a point in the dataset given by \a setInputCloud
           * \param pointDistance_arg distance of query point to voxel center
           * */
          prioPointQueueEntry (unsigned int& pointIdx_arg, double pointDistance_arg)
          {
            pointIdx_ = pointIdx_arg;
            pointDistance_ = pointDistance_arg;
          }

          /** \brief Operator< for comparing priority queue entries with each other.  */
          bool
          operator< (const prioPointQueueEntry& rhs_arg) const
          {
            return (this->pointDistance_ < rhs_arg.pointDistance_);
          }

          // index representing a point in the dataset given by \a setInputCloud
          int pointIdx_;

          // distance to query point
          double pointDistance_;

        };

        /** \brief Helper function to calculate the squared distance between two points
         * \param pointA_arg point A
         * \param pointB_arg point B
         * \return squared distance between point A and point B
         */
        double
        pointSquaredDist (const PointT & pointA_arg, const PointT & pointB_arg) const;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Recursive search routine methods
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        /** \brief Recursive search method that explores the octree and finds neighbors within a given radius
         * \param point_arg query point
         * \param radiusSquared_arg squared search radius
         * \param node_arg current octree node to be explored
         * \param key_arg octree key addressing a leaf node.
         * \param treeDepth_arg current depth/level in the octree
         * \param k_indices_arg vector of indices found to be neighbors of query point
         * \param k_sqr_distances_arg squared distances of neighbors to query point
         * \param max_nn_arg maximum of neighbors to be found
         */
        void
        getNeighborsWithinRadiusRecursive (const PointT & point_arg, const double radiusSquared_arg,
                                           const OctreeBranch* node_arg, const OctreeKey& key_arg,
                                           unsigned int treeDepth_arg, std::vector<int>& k_indices_arg,
                                           std::vector<float>& k_sqr_distances_arg, int max_nn_arg) const;

        /** \brief Recursive search method that explores the octree and finds the K nearest neighbors
         * \param point_arg query point
         * \param K_arg amount of nearest neighbors to be found
         * \param node_arg current octree node to be explored
         * \param key_arg octree key addressing a leaf node.
         * \param treeDepth_arg current depth/level in the octree
         * \param squaredSearchRadius_arg squared search radius distance
         * \param pointCandidates_arg priority queue of nearest neigbor point candidates
         * \return squared search radius based on current point candidate set found
         */
        double
        getKNearestNeighborRecursive (const PointT & point_arg, unsigned int K_arg, const OctreeBranch* node_arg,
                                      const OctreeKey& key_arg, unsigned int treeDepth_arg,
                                      const double squaredSearchRadius_arg,
                                      std::vector<prioPointQueueEntry>& pointCandidates_arg) const;

        /** \brief Recursive search method that explores the octree and finds the approximate nearest neighbor
         * \param point_arg query point
         * \param node_arg current octree node to be explored
         * \param key_arg octree key addressing a leaf node.
         * \param treeDepth_arg current depth/level in the octree
         * \param result_index_arg result index is written to this reference
         * \param sqr_distance_arg squared distance to search
         */
        void
        approxNearestSearchRecursive (const PointT & point_arg, const OctreeBranch* node_arg, const OctreeKey& key_arg,
                                      unsigned int treeDepth_arg, int& result_index_arg, float& sqr_distance_arg);

      };
  }
}

#define PCL_INSTANTIATE_OctreePointCloudSearch(T) template class PCL_EXPORTS pcl::octree::OctreePointCloudSearch<T>;

#endif
