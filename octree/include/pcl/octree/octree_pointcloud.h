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

#ifndef OCTREE_POINTCLOUD_H
#define OCTREE_POINTCLOUD_H

#include "octree_base.h"
#include "octree2buf_base.h"
#include "octree_lowmemory_base.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "octree_nodes.h"

#include <queue>
#include <vector>
#include <algorithm>
#include <iostream>

namespace pcl
{
  namespace octree
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud class
     *  \note Octree implementation for pointclouds. Only indices are stored by the octree leaf nodes (zero-copy).
     *  \note The octree pointcloud class needs to be initialized with its voxel resolution. Its bounding box is automatically adjusted
     *  \note according to the pointcloud dimension or it can be predefined.
     *  \note Note: The tree depth equates to the resolution and the bounding box dimensions of the octree.
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \note typename: LeafT:  leaf node class (usuallz templated with integer indices values)
     *  \note typename: OctreeT: octree implementation ()
     *  \ingroup octree
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT = OctreeLeafDataTVector<int> , typename OctreeT = OctreeBase<int, LeafT> >
      class OctreePointCloud : public OctreeT
      {

      public:

        /** \brief Octree pointcloud constructor.
         *  \param resolution_arg: octree resolution at lowest octree level
         * */
        OctreePointCloud (const double resolution_arg);

        /** \brief Empty deconstructor. */
        virtual
        ~OctreePointCloud ();

        // public typedefs
        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        // public typedefs for single/double buffering
        typedef OctreePointCloud<PointT, LeafT, OctreeBase<int, LeafT> > SingleBuffer;
        typedef OctreePointCloud<PointT, LeafT, Octree2BufBase<int, LeafT> > DoubleBuffer;
        typedef OctreePointCloud<PointT, LeafT, OctreeLowMemBase<int, LeafT> > LowMem;



        // Boost shared pointers
        typedef boost::shared_ptr<OctreePointCloud<PointT, LeafT, OctreeT> > Ptr;
        typedef boost::shared_ptr<const OctreePointCloud<PointT, LeafT, OctreeT> > ConstPtr;

        /** \brief Provide a pointer to the input data set.
         *  \param cloud_arg the const boost shared pointer to a PointCloud message
         *  \param indices_arg the point indices subset that is to be used from \a cloud - if 0 the whole point cloud is used
         */
        inline void
        setInputCloud (const PointCloudConstPtr &cloud_arg, const IndicesConstPtr &indices_arg = IndicesConstPtr ())
        {
          assert (this->leafCount_==0);

          if ((input_ != cloud_arg) && (this->leafCount_ == 0))
          {
            input_ = cloud_arg;
            indices_ = indices_arg;
          }
        }

        /** \brief Get a pointer to the vector of indices used.
         * \return pointer to vector of indices used.
         * */
        inline IndicesConstPtr const
        getIndices ()
        {
          return (indices_);
        }

        /** \brief Get a pointer to the input point cloud dataset.
         *  \return pointer to pointcloud input class.
         * */
        inline PointCloudConstPtr
        getInputCloud ()
        {
          return (input_);
        }

        /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
         * \param eps precision (error bound) for nearest neighbors searches
         */
        inline void
        setEpsilon (double eps)
        {
          epsilon_ = eps;
        }

        /** \brief Get the search epsilon precision (error bound) for nearest neighbors searches. */
        inline double
        getEpsilon ()
        {
          return (epsilon_);
        }

        /** \brief Set/change the octree voxel resolution
         * \param resolution_arg side length of voxels at lowest tree level
         */
        inline void
        setResolution (double resolution_arg)
        {
          // octree needs to be empty to change its resolution
          assert( this->leafCount_ == 0 );

          resolution_ = resolution_arg;
        }

        /** \brief Get octree voxel resolution
         * \return voxel resolution at lowest tree level*/
        inline double
        getResolution ()
        {
          return (resolution_);
        }

        /** \brief Add points from input point cloud to octree. */
        void
        addPointsFromInputCloud ();

        /** \brief Add point at given index from input point cloud to octree. Index will be also added to indices vector.
         *  \param pointIdx_arg index of point to be added
         *  \param indices_arg pointer to indices vector of the dataset (given by \a setInputCloud)
         * */
        void
        addPointFromCloud (const int pointIdx_arg, IndicesPtr indices_arg);

        /** \brief Add point simultaneously to octree and input point cloud.
         *  \param point_arg point to be added
         *  \param cloud_arg pointer to input point cloud dataset (given by \a setInputCloud)
         * */
        void
        addPointToCloud (const PointT& point_arg, PointCloudPtr cloud_arg);

        /** \brief Add point simultaneously to octree and input point cloud. A corresponding index will be added to the indices vector.
         *  \param point_arg point to be added
         *  \param cloud_arg pointer to input point cloud dataset (given by \a setInputCloud)
         *  \param indices_arg pointer to indices vector of the dataset (given by \a setInputCloud)
         * */
        void
        addPointToCloud (const PointT& point_arg, PointCloudPtr cloud_arg, IndicesPtr indices_arg);

        /** \brief Check if voxel at given point exist.
         *  \param point_arg point to be checked
         *  \return "true" if voxel exist; "false" otherwise
         * */
        bool
        isVoxelOccupiedAtPoint (const PointT& point_arg) const;

        /** \brief Delete the octree structure and its leaf nodes. */
        void
        deleteTree()
        {
          // reset bounding box
          minX_ = minY_ = maxY_ = minZ_ = maxZ_ = 0;
          maxKeys_ = 1;
          this->boundingBoxDefined_ = false;

          OctreeT::deleteTree();
        }

        /** \brief Check if voxel at given point coordinates exist.
         *  \param pointX_arg X coordinate of point to be checked
         *  \param pointY_arg Y coordinate of point to be checked
         *  \param pointZ_arg Z coordinate of point to be checked
         *  \return "true" if voxel exist; "false" otherwise
         * */
        bool
        isVoxelOccupiedAtPoint (const double pointX_arg, const double pointY_arg, const double pointZ_arg) const;

        /** \brief Check if voxel at given point from input cloud exist.
         *  \param pointIdx_arg point to be checked
         *  \return "true" if voxel exist; "false" otherwise
         * */
        bool
        isVoxelOccupiedAtPoint (const int& pointIdx_arg) const;

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

        /** \brief Get a PointT vector of centers of all occupied voxels.
         * \param voxelCenterList_arg results are written to this vector of PointT elements
         * \return number of occupied voxels
         */
        int
        getOccupiedVoxelCenters (std::vector<PointT, Eigen::aligned_allocator<PointT> > &voxelCenterList_arg) const;

        /** \brief Delete leaf node / voxel at given point
         *  \param point_arg point addressing the voxel to be deleted.
         * */
        void
        deleteVoxelAtPoint (const PointT& point_arg);

        /** \brief Delete leaf node / voxel at given point from input cloud
         *  \param pointIdx_arg index of point addressing the voxel to be deleted.
         * */
        void
        deleteVoxelAtPoint (const int& pointIdx_arg);


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Bounding box methods
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        /** \brief Investigate dimensions of pointcloud data set and define corresponding bounding box for octree
         * */
        void
        defineBoundingBox ();

        /** \brief Define bounding box for octree
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param minX_arg X coordinate of lower bounding box corner
         * \param minY_arg Y coordinate of lower bounding box corner
         * \param minZ_arg Z coordinate of lower bounding box corner
         * \param maxX_arg X coordinate of upper bounding box corner
         * \param maxY_arg Y coordinate of upper bounding box corner
         * \param maxZ_arg Z coordinate of upper bounding box corner
         */
        void
        defineBoundingBox (const double minX_arg, const double minY_arg, const double minZ_arg, const double maxX_arg,
                           const double maxY_arg, const double maxZ_arg);

        /** \brief Define bounding box for octree
         * \note Lower bounding box point is set to (0, 0, 0)
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param maxX_arg X coordinate of upper bounding box corner
         * \param maxY_arg Y coordinate of upper bounding box corner
         * \param maxZ_arg Z coordinate of upper bounding box corner
         */
        void
        defineBoundingBox (const double maxX_arg, const double maxY_arg, const double maxZ_arg);

        /** \brief Define bounding box cube for octree
         * \note Lower bounding box corner is set to (0, 0, 0)
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param cubeLen_arg side length of bounding box cube.
         */
        void
        defineBoundingBox (const double cubeLen_arg);

        /** \brief Get bounding box for octree
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param minX_arg X coordinate of lower bounding box corner
         * \param minY_arg Y coordinate of lower bounding box corner
         * \param minZ_arg Z coordinate of lower bounding box corner
         * \param maxX_arg X coordinate of upper bounding box corner
         * \param maxY_arg Y coordinate of upper bounding box corner
         * \param maxZ_arg Z coordinate of upper bounding box corner
         */
        void
        getBoundingBox (double& minX_arg, double& minY_arg, double& minZ_arg, double& maxX_arg, double& maxY_arg,
                        double& maxZ_arg) const ;

        /** \brief Calculates the squared diameter of a voxel at given tree depth
         * \param treeDepth_arg depth/level in octree
         * \return squared diameter
         */
        double
        getVoxelSquaredDiameter (unsigned int treeDepth_arg) const ;

        /** \brief Calculates the squared diameter of a voxel at leaf depth
         * \return squared diameter
         */
        inline double
        getVoxelSquaredDiameter ( ) const
        {
          return getVoxelSquaredDiameter( this->octreeDepth_ );
        }

        /** \brief Calculates the squared voxel cube side length at given tree depth
         * \param treeDepth_arg depth/level in octree
         * \return squared voxel cube side length
         */
        double
        getVoxelSquaredSideLen (unsigned int treeDepth_arg) const ;

        /** \brief Calculates the squared voxel cube side length at leaf level
         * \return squared voxel cube side length
         */
        inline double
        getVoxelSquaredSideLen () const
        {
          return getVoxelSquaredSideLen( this->octreeDepth_ );
        }

        typedef typename OctreeT::OctreeLeaf OctreeLeaf;

      protected:

        /** \brief Add point at index from input pointcloud dataset to octree
         * \param pointIdx_arg the index representing the point in the dataset given by \a setInputCloud to be added
         */
        void
        addPointIdx (const int pointIdx_arg);

        /** \brief Get point at index from input pointcloud dataset
         * \param index_arg index representing the point in the dataset given by \a setInputCloud
         * \return PointT from input pointcloud dataset
         */
        const PointT&
        getPointByIndex (const unsigned int index_arg) const;

        /** \brief Find octree leaf node at a given point
         * \param point_arg query point
         * \return pointer to leaf node. If leaf node does not exist, pointer is 0.
         */
        LeafT*
        findLeafAtPoint (const PointT& point_arg) const ;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Protected octree methods based on octree keys
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        typedef typename OctreeT::OctreeKey OctreeKey;
        typedef typename OctreeT::OctreeBranch OctreeBranch;

        /** \brief Define octree key setting and octree depth based on defined bounding box.
         */
        void
        getKeyBitSize ();

        /** \brief Grow the bounding box/octree until point fits
         * \param pointIdx_arg point that should be within bounding box;
         */
        void
        adoptBoundingBoxToPoint (const PointT& pointIdx_arg);

        /** \brief Generate octree key for voxel at a given point
         * \param point_arg the point addressing a voxel
         * \param key_arg write octree key to this reference
         */
        void
        genOctreeKeyforPoint (const PointT & point_arg, OctreeKey & key_arg) const ;

        /** \brief Generate octree key for voxel at a given point
         * \param pointX_arg X coordinate of point addressing a voxel
         * \param pointY_arg Y coordinate of point addressing a voxel
         * \param pointZ_arg Z coordinate of point addressing a voxel
         * \param key_arg write octree key to this reference
         */
        void
        genOctreeKeyforPoint (const double pointX_arg, const double pointY_arg, const double pointZ_arg,
                              OctreeKey & key_arg) const;

        /** \brief Virtual method for generating octree key for a given point index.
         * \note This method enables to assign indices to leaf nodes during octree deserialization.
         * \param data_arg index value representing a point in the dataset given by \a setInputCloud
         * \param key_arg write octree key to this reference
         * \return "true" - octree keys are assignable
         */
        virtual bool
        genOctreeKeyForDataT (const int& data_arg, OctreeKey & key_arg) const;

        /** \brief Generate a point at center of leaf node voxel
         * \param key_arg octree key addressing a leaf node.
         * \param point_arg write leaf node voxel center to this point reference
         */
        void
        genLeafNodeCenterFromOctreeKey (const OctreeKey & key_arg, PointT& point_arg) const ;

        /** \brief Generate a point at center of octree voxel at given tree level
         * \param key_arg octree key addressing an octree node.
         * \param treeDepth_arg octree depth of query voxel
         * \param point_arg write leaf node center point to this reference
         */
        void
        genVoxelCenterFromOctreeKey (const OctreeKey & key_arg, unsigned int treeDepth_arg, PointT& point_arg) const ;

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
                                      unsigned int treeDepth_arg, int& result_index_arg,
                                      float& sqr_distance_arg);

        /** \brief Recursively search the tree for all leaf nodes and return a vector of voxel centers.
         * \param node_arg current octree node to be explored
         * \param key_arg octree key addressing a leaf node.
         * \param voxelCenterList_arg results are written to this vector of PointT elements
         * \return number of voxels found
         */
        int
        getOccupiedVoxelCentersRecursive (const OctreeBranch* node_arg, const OctreeKey& key_arg,
                                          std::vector<PointT, Eigen::aligned_allocator<PointT> > &voxelCenterList_arg) const;



        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Globals
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Pointer to input point cloud dataset. */
        PointCloudConstPtr input_;

        /** \brief A pointer to the vector of point indices to use. */
        IndicesConstPtr indices_;

        /** \brief Epsilon precision (error bound) for nearest neighbors searches. */
        double epsilon_;

        /** \brief Octree resolution. */
        double resolution_;

        // Octree bounding box coordinates
        double minX_;
        double maxX_;

        double minY_;
        double maxY_;

        double minZ_;
        double maxZ_;

        /** \brief Maximum amount of keys available in octree. */
        unsigned int maxKeys_;

        /** \brief Flag indicating if octree has defined bounding box. */
        bool boundingBoxDefined_;

      };

  }
}

#endif

