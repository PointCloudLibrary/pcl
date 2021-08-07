/*
 * simple_octree.hpp
 *
 *  Created on: Mar 12, 2013
 *      Author: papazov
 */

#ifndef SIMPLE_OCTREE_HPP_
#define SIMPLE_OCTREE_HPP_

#include <cmath>


namespace pcl
{

namespace recognition
{

template<typename NodeData, typename NodeDataCreator, typename Scalar> inline
SimpleOctree<NodeData, NodeDataCreator, Scalar>::Node::Node ()
: data_ (nullptr),
  parent_ (nullptr),
  children_(nullptr)
{}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline
SimpleOctree<NodeData, NodeDataCreator, Scalar>::Node::~Node ()
{
  this->deleteChildren ();
  this->deleteData ();
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline void
SimpleOctree<NodeData, NodeDataCreator, Scalar>::Node::setCenter (const Scalar *c)
{
  center_[0] = c[0];
  center_[1] = c[1];
  center_[2] = c[2];
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline void
SimpleOctree<NodeData, NodeDataCreator, Scalar>::Node::setBounds (const Scalar *b)
{
  bounds_[0] = b[0];
  bounds_[1] = b[1];
  bounds_[2] = b[2];
  bounds_[3] = b[3];
  bounds_[4] = b[4];
  bounds_[5] = b[5];
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline void
SimpleOctree<NodeData, NodeDataCreator, Scalar>::Node::computeRadius ()
{
  Scalar v[3] = {static_cast<Scalar> (0.5)*(bounds_[1]-bounds_[0]),
                 static_cast<Scalar> (0.5)*(bounds_[3]-bounds_[2]),
                 static_cast<Scalar> (0.5)*(bounds_[5]-bounds_[4])};

  radius_ = static_cast<Scalar> (std::sqrt (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]));
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline bool
SimpleOctree<NodeData, NodeDataCreator, Scalar>::Node::createChildren ()
{
  if ( children_ )
    return (false);

  Scalar bounds[6], center[3], childside = static_cast<Scalar> (0.5)*(bounds_[1]-bounds_[0]);
  children_ = new Node[8];

  // Compute bounds and center for child 0, i.e., for (0,0,0)
  bounds[0] = bounds_[0]; bounds[1] = center_[0];
  bounds[2] = bounds_[2]; bounds[3] = center_[1];
  bounds[4] = bounds_[4]; bounds[5] = center_[2];
  // Compute the center of the new child
  center[0] = 0.5f*(bounds[0] + bounds[1]);
  center[1] = 0.5f*(bounds[2] + bounds[3]);
  center[2] = 0.5f*(bounds[4] + bounds[5]);
  // Save the results
  children_[0].setBounds(bounds);
  children_[0].setCenter(center);

  // Compute bounds and center for child 1, i.e., for (0,0,1)
  bounds[4] = center_[2]; bounds[5] = bounds_[5];
  // Update the center
  center[2] += childside;
  // Save the results
  children_[1].setBounds(bounds);
  children_[1].setCenter(center);

  // Compute bounds and center for child 3, i.e., for (0,1,1)
  bounds[2] = center_[1]; bounds[3] = bounds_[3];
  // Update the center
  center[1] += childside;
  // Save the results
  children_[3].setBounds(bounds);
  children_[3].setCenter(center);

  // Compute bounds and center for child 2, i.e., for (0,1,0)
  bounds[4] = bounds_[4]; bounds[5] = center_[2];
  // Update the center
  center[2] -= childside;
  // Save the results
  children_[2].setBounds(bounds);
  children_[2].setCenter(center);

  // Compute bounds and center for child 6, i.e., for (1,1,0)
  bounds[0] = center_[0]; bounds[1] = bounds_[1];
  // Update the center
  center[0] += childside;
  // Save the results
  children_[6].setBounds(bounds);
  children_[6].setCenter(center);

  // Compute bounds and center for child 7, i.e., for (1,1,1)
  bounds[4] = center_[2]; bounds[5] = bounds_[5];
  // Update the center
  center[2] += childside;
  // Save the results
  children_[7].setBounds(bounds);
  children_[7].setCenter(center);

  // Compute bounds and center for child 5, i.e., for (1,0,1)
  bounds[2] = bounds_[2]; bounds[3] = center_[1];
  // Update the center
  center[1] -= childside;
  // Save the results
  children_[5].setBounds(bounds);
  children_[5].setCenter(center);

  // Compute bounds and center for child 4, i.e., for (1,0,0)
  bounds[4] = bounds_[4]; bounds[5] = center_[2];
  // Update the center
  center[2] -= childside;
  // Save the results
  children_[4].setBounds(bounds);
  children_[4].setCenter(center);

  for ( int i = 0 ; i < 8 ; ++i )
  {
    children_[i].computeRadius();
    children_[i].setParent(this);
  }

  return (true);
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline void
SimpleOctree<NodeData, NodeDataCreator, Scalar>::Node::deleteChildren ()
{
  delete[] children_;
  children_ = nullptr;
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline void
SimpleOctree<NodeData, NodeDataCreator, Scalar>::Node::deleteData ()
{
  delete data_;
  data_ = nullptr;
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline void
SimpleOctree<NodeData, NodeDataCreator, Scalar>::Node::makeNeighbors (Node* node)
{
  if ( !this->hasData () || !node->hasData () )
    return;

  this->full_leaf_neighbors_.insert (node);
  node->full_leaf_neighbors_.insert (this);
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline
SimpleOctree<NodeData, NodeDataCreator, Scalar>::SimpleOctree ()
: tree_levels_ (0),
  root_ (nullptr)
{
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline
SimpleOctree<NodeData, NodeDataCreator, Scalar>::~SimpleOctree ()
{
  this->clear ();
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline void
SimpleOctree<NodeData, NodeDataCreator, Scalar>::clear ()
{
  delete root_;
  root_ = nullptr;

  full_leaves_.clear();
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline void
SimpleOctree<NodeData, NodeDataCreator, Scalar>::build (const Scalar* bounds, Scalar voxel_size,
    NodeDataCreator* node_data_creator)
{
  if ( voxel_size <= 0 )
    return;

  this->clear();

  voxel_size_ = voxel_size;
  node_data_creator_ = node_data_creator;

  Scalar extent = std::max (std::max (bounds[1]-bounds[0], bounds[3]-bounds[2]), bounds[5]-bounds[4]);
  Scalar center[3] = {static_cast<Scalar> (0.5)*(bounds[0]+bounds[1]),
                      static_cast<Scalar> (0.5)*(bounds[2]+bounds[3]),
                      static_cast<Scalar> (0.5)*(bounds[4]+bounds[5])};

  Scalar arg = extent/voxel_size;

  // Compute the number of tree levels
  if ( arg > 1 )
    tree_levels_ = static_cast<int> (std::ceil (std::log (arg)/std::log (2.0)) + 0.5);
  else
    tree_levels_ = 0;

  // Compute the number of octree levels and the bounds of the root
  Scalar half_root_side = static_cast<Scalar> (0.5f*pow (2.0, tree_levels_)*voxel_size);

  // Determine the bounding box of the octree
  bounds_[0] = center[0] - half_root_side;
  bounds_[1] = center[0] + half_root_side;
  bounds_[2] = center[1] - half_root_side;
  bounds_[3] = center[1] + half_root_side;
  bounds_[4] = center[2] - half_root_side;
  bounds_[5] = center[2] + half_root_side;

  // Create and initialize the root
  root_ = new Node ();
  root_->setCenter (center);
  root_->setBounds (bounds_);
  root_->setParent (nullptr);
  root_->computeRadius ();
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline
typename SimpleOctree<NodeData, NodeDataCreator, Scalar>::Node*
SimpleOctree<NodeData, NodeDataCreator, Scalar>::createLeaf (Scalar x, Scalar y, Scalar z)
{
  // Make sure that the input point is within the octree bounds
  if ( x < bounds_[0] || x > bounds_[1] ||
       y < bounds_[2] || y > bounds_[3] ||
       z < bounds_[4] || z > bounds_[5] )
  {
    return (nullptr);
  }

  Node* node = root_;

  // Go down to the right leaf
  for ( int l = 0 ; l < tree_levels_ ; ++l )
  {
    node->createChildren ();
    const Scalar *c = node->getCenter ();
    int id = 0;

    if ( x >= c[0] ) id |= 4;
    if ( y >= c[1] ) id |= 2;
    if ( z >= c[2] ) id |= 1;

    node = node->getChild (id);
  }

  if ( !node->hasData () )
  {
    node->setData (node_data_creator_->create (node));
    this->insertNeighbors (node);
    full_leaves_.push_back (node);
  }

  return (node);
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline
typename SimpleOctree<NodeData, NodeDataCreator, Scalar>::Node*
SimpleOctree<NodeData, NodeDataCreator, Scalar>::getFullLeaf (int i, int j, int k)
{
  Scalar offset = 0.5f*voxel_size_;
  Scalar p[3] = {bounds_[0] + offset + static_cast<Scalar> (i)*voxel_size_,
                 bounds_[2] + offset + static_cast<Scalar> (j)*voxel_size_,
                 bounds_[4] + offset + static_cast<Scalar> (k)*voxel_size_};

  return (this->getFullLeaf (p[0], p[1], p[2]));
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline
typename SimpleOctree<NodeData, NodeDataCreator, Scalar>::Node*
SimpleOctree<NodeData, NodeDataCreator, Scalar>::getFullLeaf (Scalar x, Scalar y, Scalar z)
{
  // Make sure that the input point is within the octree bounds
  if ( x < bounds_[0] || x > bounds_[1] ||
       y < bounds_[2] || y > bounds_[3] ||
       z < bounds_[4] || z > bounds_[5] )
  {
    return (nullptr);
  }

  Node* node = root_;

  // Go down to the right leaf
  for ( int l = 0 ; l < tree_levels_ ; ++l )
  {
    if ( !node->hasChildren () )
      return (nullptr);

    const Scalar *c = node->getCenter ();
    int id = 0;

    if ( x >= c[0] ) id |= 4;
    if ( y >= c[1] ) id |= 2;
    if ( z >= c[2] ) id |= 1;

    node = node->getChild (id);
  }

  if ( !node->hasData () )
    return (nullptr);

  return (node);
}


template<typename NodeData, typename NodeDataCreator, typename Scalar> inline void
SimpleOctree<NodeData, NodeDataCreator, Scalar>::insertNeighbors (Node* node)
{
  const Scalar* c = node->getCenter ();
  Scalar s = static_cast<Scalar> (0.5)*voxel_size_;
  Node *neigh;

  neigh = this->getFullLeaf (c[0]+s, c[1]+s, c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]+s, c[1]+s, c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]+s, c[1]+s, c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]+s, c[1]  , c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]+s, c[1]  , c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]+s, c[1]  , c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]+s, c[1]-s, c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]+s, c[1]-s, c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]+s, c[1]-s, c[2]-s); if ( neigh ) node->makeNeighbors (neigh);

  neigh = this->getFullLeaf (c[0]  , c[1]+s, c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]  , c[1]+s, c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]  , c[1]+s, c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]  , c[1]  , c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
//neigh = this->getFullLeaf (c[0]  , c[1]  , c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]  , c[1]  , c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]  , c[1]-s, c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]  , c[1]-s, c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]  , c[1]-s, c[2]-s); if ( neigh ) node->makeNeighbors (neigh);

  neigh = this->getFullLeaf (c[0]-s, c[1]+s, c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]-s, c[1]+s, c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]-s, c[1]+s, c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]-s, c[1]  , c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]-s, c[1]  , c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]-s, c[1]  , c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]-s, c[1]-s, c[2]+s); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]-s, c[1]-s, c[2]  ); if ( neigh ) node->makeNeighbors (neigh);
  neigh = this->getFullLeaf (c[0]-s, c[1]-s, c[2]-s); if ( neigh ) node->makeNeighbors (neigh);
}

} // namespace recognition
} // namespace pcl

#endif /* SIMPLE_OCTREE_HPP_ */

