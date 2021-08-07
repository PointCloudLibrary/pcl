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
 * $Id$
 */

#pragma once

#include <pcl/octree/octree_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace pcl {
namespace octree {

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Octree pointcloud class
 *  \note Octree implementation for pointclouds. Only indices are stored by the octree
 * leaf nodes (zero-copy).
 * \note The octree pointcloud class needs to be initialized with its voxel resolution.
 * Its bounding box is automatically adjusted
 * \note according to the pointcloud dimension or it can be predefined.
 * \note Note: The tree depth equates to the resolution and the bounding box dimensions
 * of the octree.
 * \tparam PointT: type of point used in pointcloud
 * \tparam LeafContainerT: leaf node container
 * \tparam BranchContainerT:  branch node container
 * \tparam OctreeT: octree implementation
 * \ingroup octree
 * \author Julius Kammerl * (julius@kammerl.de)
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT,
          typename LeafContainerT = OctreeContainerPointIndices,
          typename BranchContainerT = OctreeContainerEmpty,
          typename OctreeT = OctreeBase<LeafContainerT, BranchContainerT>>

class OctreePointCloud : public OctreeT {
public:
  using Base = OctreeT;

  using LeafNode = typename OctreeT::LeafNode;
  using BranchNode = typename OctreeT::BranchNode;

  /** \brief Octree pointcloud constructor.
   * \param[in] resolution_arg octree resolution at lowest octree level
   */
  OctreePointCloud(const double resolution_arg);

  // public typedefs
  using IndicesPtr = shared_ptr<Indices>;
  using IndicesConstPtr = shared_ptr<const Indices>;

  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

  // public typedefs for single/double buffering
  using SingleBuffer = OctreePointCloud<PointT,
                                        LeafContainerT,
                                        BranchContainerT,
                                        OctreeBase<LeafContainerT>>;
  // using DoubleBuffer = OctreePointCloud<PointT, LeafContainerT, BranchContainerT,
  // Octree2BufBase<LeafContainerT> >;

  // Boost shared pointers
  using Ptr =
      shared_ptr<OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>>;
  using ConstPtr = shared_ptr<
      const OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>>;

  // Eigen aligned allocator
  using AlignedPointTVector = std::vector<PointT, Eigen::aligned_allocator<PointT>>;
  using AlignedPointXYZVector =
      std::vector<PointXYZ, Eigen::aligned_allocator<PointXYZ>>;

  /** \brief Provide a pointer to the input data set.
   * \param[in] cloud_arg the const boost shared pointer to a PointCloud message
   * \param[in] indices_arg the point indices subset that is to be used from \a cloud -
   * if 0 the whole point cloud is used
   */
  inline void
  setInputCloud(const PointCloudConstPtr& cloud_arg,
                const IndicesConstPtr& indices_arg = IndicesConstPtr())
  {
    input_ = cloud_arg;
    indices_ = indices_arg;
  }

  /** \brief Get a pointer to the vector of indices used.
   * \return pointer to vector of indices used.
   */
  inline IndicesConstPtr const
  getIndices() const
  {
    return (indices_);
  }

  /** \brief Get a pointer to the input point cloud dataset.
   * \return pointer to pointcloud input class.
   */
  inline PointCloudConstPtr
  getInputCloud() const
  {
    return (input_);
  }

  /** \brief Set the search epsilon precision (error bound) for nearest neighbors
   * searches.
   * \param[in] eps precision (error bound) for nearest neighbors searches
   */
  inline void
  setEpsilon(double eps)
  {
    epsilon_ = eps;
  }

  /** \brief Get the search epsilon precision (error bound) for nearest neighbors
   * searches. */
  inline double
  getEpsilon() const
  {
    return (epsilon_);
  }

  /** \brief Set/change the octree voxel resolution
   * \param[in] resolution_arg side length of voxels at lowest tree level
   */
  inline void
  setResolution(double resolution_arg)
  {
    // octree needs to be empty to change its resolution
    assert(this->leaf_count_ == 0);

    resolution_ = resolution_arg;

    getKeyBitSize();
  }

  /** \brief Get octree voxel resolution
   * \return voxel resolution at lowest tree level
   */
  inline double
  getResolution() const
  {
    return (resolution_);
  }

  /** \brief Get the maximum depth of the octree.
   *  \return depth_arg: maximum depth of octree
   * */
  inline uindex_t
  getTreeDepth() const
  {
    return this->octree_depth_;
  }

  /** \brief Add points from input point cloud to octree. */
  void
  addPointsFromInputCloud();

  /** \brief Add point at given index from input point cloud to octree. Index will be
   * also added to indices vector.
   * \param[in] point_idx_arg index of point to be added
   * \param[in] indices_arg pointer to indices vector of the dataset (given by \a
   * setInputCloud)
   */
  void
  addPointFromCloud(uindex_t point_idx_arg, IndicesPtr indices_arg);

  /** \brief Add point simultaneously to octree and input point cloud.
   *  \param[in] point_arg point to be added
   *  \param[in] cloud_arg pointer to input point cloud dataset (given by \a
   * setInputCloud)
   */
  void
  addPointToCloud(const PointT& point_arg, PointCloudPtr cloud_arg);

  /** \brief Add point simultaneously to octree and input point cloud. A corresponding
   * index will be added to the indices vector.
   * \param[in] point_arg point to be added
   * \param[in] cloud_arg pointer to input point cloud dataset (given by \a
   * setInputCloud)
   * \param[in] indices_arg pointer to indices vector of the dataset (given by \a
   * setInputCloud)
   */
  void
  addPointToCloud(const PointT& point_arg,
                  PointCloudPtr cloud_arg,
                  IndicesPtr indices_arg);

  /** \brief Check if voxel at given point exist.
   * \param[in] point_arg point to be checked
   * \return "true" if voxel exist; "false" otherwise
   */
  bool
  isVoxelOccupiedAtPoint(const PointT& point_arg) const;

  /** \brief Delete the octree structure and its leaf nodes.
   * */
  void
  deleteTree()
  {
    // reset bounding box
    min_x_ = min_y_ = max_y_ = min_z_ = max_z_ = 0;
    this->bounding_box_defined_ = false;

    OctreeT::deleteTree();
  }

  /** \brief Check if voxel at given point coordinates exist.
   * \param[in] point_x_arg X coordinate of point to be checked
   * \param[in] point_y_arg Y coordinate of point to be checked
   * \param[in] point_z_arg Z coordinate of point to be checked
   * \return "true" if voxel exist; "false" otherwise
   */
  bool
  isVoxelOccupiedAtPoint(const double point_x_arg,
                         const double point_y_arg,
                         const double point_z_arg) const;

  /** \brief Check if voxel at given point from input cloud exist.
   * \param[in] point_idx_arg point to be checked
   * \return "true" if voxel exist; "false" otherwise
   */
  bool
  isVoxelOccupiedAtPoint(const index_t& point_idx_arg) const;

  /** \brief Get a PointT vector of centers of all occupied voxels.
   * \param[out] voxel_center_list_arg results are written to this vector of PointT
   * elements
   * \return number of occupied voxels
   */
  uindex_t
  getOccupiedVoxelCenters(AlignedPointTVector& voxel_center_list_arg) const;

  /** \brief Get a PointT vector of centers of voxels intersected by a line segment.
   * This returns a approximation of the actual intersected voxels by walking
   * along the line with small steps. Voxels are ordered, from closest to
   * furthest w.r.t. the origin.
   * \param[in] origin origin of the line segment
   * \param[in] end end of the line segment
   * \param[out] voxel_center_list results are written to this vector of PointT elements
   * \param[in] precision determines the size of the steps: step_size =
   * octree_resolution x precision
   * \return number of intersected voxels
   */
  uindex_t
  getApproxIntersectedVoxelCentersBySegment(const Eigen::Vector3f& origin,
                                            const Eigen::Vector3f& end,
                                            AlignedPointTVector& voxel_center_list,
                                            float precision = 0.2);

  /** \brief Delete leaf node / voxel at given point
   * \param[in] point_arg point addressing the voxel to be deleted.
   */
  void
  deleteVoxelAtPoint(const PointT& point_arg);

  /** \brief Delete leaf node / voxel at given point from input cloud
   *  \param[in] point_idx_arg index of point addressing the voxel to be deleted.
   */
  void
  deleteVoxelAtPoint(const index_t& point_idx_arg);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Bounding box methods
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Investigate dimensions of pointcloud data set and define corresponding
   * bounding box for octree. */
  void
  defineBoundingBox();

  /** \brief Define bounding box for octree
   * \note Bounding box cannot be changed once the octree contains elements.
   * \param[in] min_x_arg X coordinate of lower bounding box corner
   * \param[in] min_y_arg Y coordinate of lower bounding box corner
   * \param[in] min_z_arg Z coordinate of lower bounding box corner
   * \param[in] max_x_arg X coordinate of upper bounding box corner
   * \param[in] max_y_arg Y coordinate of upper bounding box corner
   * \param[in] max_z_arg Z coordinate of upper bounding box corner
   */
  void
  defineBoundingBox(const double min_x_arg,
                    const double min_y_arg,
                    const double min_z_arg,
                    const double max_x_arg,
                    const double max_y_arg,
                    const double max_z_arg);

  /** \brief Define bounding box for octree
   * \note Lower bounding box point is set to (0, 0, 0)
   * \note Bounding box cannot be changed once the octree contains elements.
   * \param[in] max_x_arg X coordinate of upper bounding box corner
   * \param[in] max_y_arg Y coordinate of upper bounding box corner
   * \param[in] max_z_arg Z coordinate of upper bounding box corner
   */
  void
  defineBoundingBox(const double max_x_arg,
                    const double max_y_arg,
                    const double max_z_arg);

  /** \brief Define bounding box cube for octree
   * \note Lower bounding box corner is set to (0, 0, 0)
   * \note Bounding box cannot be changed once the octree contains elements.
   * \param[in] cubeLen_arg side length of bounding box cube.
   */
  void
  defineBoundingBox(const double cubeLen_arg);

  /** \brief Get bounding box for octree
   * \note Bounding box cannot be changed once the octree contains elements.
   * \param[in] min_x_arg X coordinate of lower bounding box corner
   * \param[in] min_y_arg Y coordinate of lower bounding box corner
   * \param[in] min_z_arg Z coordinate of lower bounding box corner
   * \param[in] max_x_arg X coordinate of upper bounding box corner
   * \param[in] max_y_arg Y coordinate of upper bounding box corner
   * \param[in] max_z_arg Z coordinate of upper bounding box corner
   */
  void
  getBoundingBox(double& min_x_arg,
                 double& min_y_arg,
                 double& min_z_arg,
                 double& max_x_arg,
                 double& max_y_arg,
                 double& max_z_arg) const;

  /** \brief Calculates the squared diameter of a voxel at given tree depth
   * \param[in] tree_depth_arg depth/level in octree
   * \return squared diameter
   */
  double
  getVoxelSquaredDiameter(uindex_t tree_depth_arg) const;

  /** \brief Calculates the squared diameter of a voxel at leaf depth
   * \return squared diameter
   */
  inline double
  getVoxelSquaredDiameter() const
  {
    return getVoxelSquaredDiameter(this->octree_depth_);
  }

  /** \brief Calculates the squared voxel cube side length at given tree depth
   * \param[in] tree_depth_arg depth/level in octree
   * \return squared voxel cube side length
   */
  double
  getVoxelSquaredSideLen(uindex_t tree_depth_arg) const;

  /** \brief Calculates the squared voxel cube side length at leaf level
   * \return squared voxel cube side length
   */
  inline double
  getVoxelSquaredSideLen() const
  {
    return getVoxelSquaredSideLen(this->octree_depth_);
  }

  /** \brief Generate bounds of the current voxel of an octree iterator
   * \param[in] iterator: octree iterator
   * \param[out] min_pt lower bound of voxel
   * \param[out] max_pt upper bound of voxel
   */
  inline void
  getVoxelBounds(const OctreeIteratorBase<OctreeT>& iterator,
                 Eigen::Vector3f& min_pt,
                 Eigen::Vector3f& max_pt) const
  {
    this->genVoxelBoundsFromOctreeKey(iterator.getCurrentOctreeKey(),
                                      iterator.getCurrentOctreeDepth(),
                                      min_pt,
                                      max_pt);
  }

  /** \brief Enable dynamic octree structure
   *  \note Leaf nodes are kept as close to the root as possible and are only expanded
   * if the number of DataT objects within a leaf node exceeds a fixed limit.
   * \param maxObjsPerLeaf: maximum number of DataT objects per leaf
   * */
  inline void
  enableDynamicDepth(std::size_t maxObjsPerLeaf)
  {
    assert(this->leaf_count_ == 0);
    max_objs_per_leaf_ = maxObjsPerLeaf;

    this->dynamic_depth_enabled_ = max_objs_per_leaf_ > 0;
  }

protected:
  /** \brief Add point at index from input pointcloud dataset to octree
   * \param[in] point_idx_arg the index representing the point in the dataset given by
   * \a setInputCloud to be added
   */
  virtual void
  addPointIdx(uindex_t point_idx_arg);

  /** \brief Add point at index from input pointcloud dataset to octree
   * \param[in] leaf_node to be expanded
   * \param[in] parent_branch parent of leaf node to be expanded
   * \param[in] child_idx child index of leaf node (in parent branch)
   * \param[in] depth_mask of leaf node to be expanded
   */
  void
  expandLeafNode(LeafNode* leaf_node,
                 BranchNode* parent_branch,
                 unsigned char child_idx,
                 uindex_t depth_mask);

  /** \brief Get point at index from input pointcloud dataset
   * \param[in] index_arg index representing the point in the dataset given by \a
   * setInputCloud
   * \return PointT from input pointcloud dataset
   */
  const PointT&
  getPointByIndex(uindex_t index_arg) const;

  /** \brief Find octree leaf node at a given point
   * \param[in] point_arg query point
   * \return pointer to leaf node. If leaf node does not exist, pointer is 0.
   */
  LeafContainerT*
  findLeafAtPoint(const PointT& point_arg) const
  {
    OctreeKey key;

    // generate key for point
    this->genOctreeKeyforPoint(point_arg, key);

    return (this->findLeaf(key));
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Protected octree methods based on octree keys
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Define octree key setting and octree depth based on defined bounding box.
   */
  void
  getKeyBitSize();

  /** \brief Grow the bounding box/octree until point fits
   * \param[in] point_idx_arg point that should be within bounding box;
   */
  void
  adoptBoundingBoxToPoint(const PointT& point_idx_arg);

  /** \brief Checks if given point is within the bounding box of the octree
   * \param[in] point_idx_arg point to be checked for bounding box violations
   * \return "true" - no bound violation
   */
  inline bool
  isPointWithinBoundingBox(const PointT& point_idx_arg) const
  {
    return (!((point_idx_arg.x < min_x_) || (point_idx_arg.y < min_y_) ||
              (point_idx_arg.z < min_z_) || (point_idx_arg.x >= max_x_) ||
              (point_idx_arg.y >= max_y_) || (point_idx_arg.z >= max_z_)));
  }

  /** \brief Generate octree key for voxel at a given point
   * \param[in] point_arg the point addressing a voxel
   * \param[out] key_arg write octree key to this reference
   */
  void
  genOctreeKeyforPoint(const PointT& point_arg, OctreeKey& key_arg) const;

  /** \brief Generate octree key for voxel at a given point
   * \param[in] point_x_arg X coordinate of point addressing a voxel
   * \param[in] point_y_arg Y coordinate of point addressing a voxel
   * \param[in] point_z_arg Z coordinate of point addressing a voxel
   * \param[out] key_arg write octree key to this reference
   */
  void
  genOctreeKeyforPoint(const double point_x_arg,
                       const double point_y_arg,
                       const double point_z_arg,
                       OctreeKey& key_arg) const;

  /** \brief Virtual method for generating octree key for a given point index.
   * \note This method enables to assign indices to leaf nodes during octree
   * deserialization.
   * \param[in] data_arg index value representing a point in the dataset given by \a
   * setInputCloud
   * \param[out] key_arg write octree key to this reference \return "true" - octree keys
   * are assignable
   */
  virtual bool
  genOctreeKeyForDataT(const index_t& data_arg, OctreeKey& key_arg) const;

  /** \brief Generate a point at center of leaf node voxel
   * \param[in] key_arg octree key addressing a leaf node.
   * \param[out] point_arg write leaf node voxel center to this point reference
   */
  void
  genLeafNodeCenterFromOctreeKey(const OctreeKey& key_arg, PointT& point_arg) const;

  /** \brief Generate a point at center of octree voxel at given tree level
   * \param[in] key_arg octree key addressing an octree node.
   * \param[in] tree_depth_arg octree depth of query voxel
   * \param[out] point_arg write leaf node center point to this reference
   */
  void
  genVoxelCenterFromOctreeKey(const OctreeKey& key_arg,
                              uindex_t tree_depth_arg,
                              PointT& point_arg) const;

  /** \brief Generate bounds of an octree voxel using octree key and tree depth
   * arguments
   * \param[in] key_arg octree key addressing an octree node.
   * \param[in] tree_depth_arg octree depth of query voxel
   * \param[out] min_pt lower bound of voxel
   * \param[out] max_pt upper bound of voxel
   */
  void
  genVoxelBoundsFromOctreeKey(const OctreeKey& key_arg,
                              uindex_t tree_depth_arg,
                              Eigen::Vector3f& min_pt,
                              Eigen::Vector3f& max_pt) const;

  /** \brief Recursively search the tree for all leaf nodes and return a vector of voxel
   * centers.
   * \param[in] node_arg current octree node to be explored
   * \param[in] key_arg octree key addressing a leaf node.
   * \param[out] voxel_center_list_arg results are written to this vector of PointT
   * elements
   * \return number of voxels found
   */
  uindex_t
  getOccupiedVoxelCentersRecursive(const BranchNode* node_arg,
                                   const OctreeKey& key_arg,
                                   AlignedPointTVector& voxel_center_list_arg) const;

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
  double min_x_;
  double max_x_;

  double min_y_;
  double max_y_;

  double min_z_;
  double max_z_;

  /** \brief Flag indicating if octree has defined bounding box. */
  bool bounding_box_defined_;

  /** \brief Amount of DataT objects per leafNode before expanding branch
   *  \note zero indicates a fixed/maximum depth octree structure
   * **/
  std::size_t max_objs_per_leaf_;
};

} // namespace octree
} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/octree/impl/octree_pointcloud.hpp>
#endif
