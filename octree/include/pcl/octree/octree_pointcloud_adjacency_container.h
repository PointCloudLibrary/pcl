/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Jeremie Papon
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
 *  Author : jpapon@gmail.com
 *  Email  : jpapon@gmail.com
 */

#pragma once

#include <list> // for std::list

namespace pcl {

namespace octree {
/** \brief @b Octree adjacency leaf container class- stores a list of pointers to
 * neighbors, number of points added, and a DataT value \note This class implements a
 * leaf node that stores pointers to neighboring leaves \note This class also has a
 * virtual computeData function, which is called by
 * octreePointCloudAdjacency::addPointsFromInputCloud. \note You should make explicit
 * instantiations of it for your pointtype/datatype combo (if needed) see
 * supervoxel_clustering.hpp for an example of this
 */
template <typename PointInT, typename DataT = PointInT>
class OctreePointCloudAdjacencyContainer : public OctreeContainerBase {
  template <typename T, typename U, typename V>
  friend class OctreePointCloudAdjacency;

public:
  using NeighborListT = std::list<OctreePointCloudAdjacencyContainer<PointInT, DataT>*>;
  using const_iterator = typename NeighborListT::const_iterator;
  // const iterators to neighbors
  inline const_iterator
  cbegin() const
  {
    return (neighbors_.begin());
  }
  inline const_iterator
  cend() const
  {
    return (neighbors_.end());
  }
  // size of neighbors
  inline std::size_t
  size() const
  {
    return neighbors_.size();
  }

  /** \brief Class initialization. */
  OctreePointCloudAdjacencyContainer() : OctreeContainerBase() { this->reset(); }

  /** \brief Empty class deconstructor. */
  ~OctreePointCloudAdjacencyContainer() override = default;

  /** \brief Returns the number of neighbors this leaf has
   *  \returns number of neighbors
   */
  std::size_t
  getNumNeighbors() const
  {
    return neighbors_.size();
  }

  /** \brief Gets the number of points contributing to this leaf */
  uindex_t
  getPointCounter() const
  {
    return num_points_;
  }

  /** \brief Returns a reference to the data member to access it without copying */
  DataT&
  getData()
  {
    return data_;
  }

  /** \brief Sets the data member
   *  \param[in] data_arg New value for data
   */
  void
  setData(const DataT& data_arg)
  {
    data_ = data_arg;
  }

  /** \brief  virtual method to get size of container
   * \return number of points added to leaf node container.
   */
  uindex_t
  getSize() const override
  {
    return num_points_;
  }

protected:
  // iterators to neighbors
  using iterator = typename NeighborListT::iterator;
  inline iterator
  begin()
  {
    return (neighbors_.begin());
  }
  inline iterator
  end()
  {
    return (neighbors_.end());
  }

  /** \brief deep copy function */
  virtual OctreePointCloudAdjacencyContainer*
  deepCopy() const
  {
    auto* new_container = new OctreePointCloudAdjacencyContainer;
    new_container->setNeighbors(this->neighbors_);
    new_container->setPointCounter(this->num_points_);
    return new_container;
  }

  /** \brief Add new point to container- this just counts points
   * \note To actually store data in the leaves, need to specialize this
   * for your point and data type as in supervoxel_clustering.hpp
   */
  // param[in] new_point the new point to add
  void
  addPoint(const PointInT& /*new_point*/)
  {
    using namespace pcl::common;
    ++num_points_;
  }

  /** \brief Function for working on data added. Base implementation does nothing
   * */
  void
  computeData()
  {}

  /** \brief Sets the number of points contributing to this leaf */
  void
  setPointCounter(uindex_t points_arg)
  {
    num_points_ = points_arg;
  }

  /** \brief Clear the voxel centroid */
  void
  reset() override
  {
    neighbors_.clear();
    num_points_ = 0;
    data_ = DataT();
  }

  /** \brief Add new neighbor to voxel.
   * \param[in] neighbor the new neighbor to add
   */
  void
  addNeighbor(OctreePointCloudAdjacencyContainer* neighbor)
  {
    neighbors_.push_back(neighbor);
  }

  /** \brief Remove neighbor from neighbor set.
   * \param[in] neighbor the neighbor to remove
   */
  void
  removeNeighbor(OctreePointCloudAdjacencyContainer* neighbor)
  {
    for (auto neighb_it = neighbors_.begin(); neighb_it != neighbors_.end();
         ++neighb_it) {
      if (*neighb_it == neighbor) {
        neighbors_.erase(neighb_it);
        return;
      }
    }
  }

  /** \brief Sets the whole neighbor set
   * \param[in] neighbor_arg the new set
   */
  void
  setNeighbors(const NeighborListT& neighbor_arg)
  {
    neighbors_ = neighbor_arg;
  }

private:
  uindex_t num_points_;
  NeighborListT neighbors_;
  DataT data_;
};
} // namespace octree
} // namespace pcl
