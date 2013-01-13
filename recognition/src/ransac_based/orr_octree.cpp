/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *
 */

#include <pcl/recognition/ransac_based/orr_octree.h>
#include <pcl/common/common.h>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <list>

using namespace std;
using namespace pcl::recognition;

pcl::recognition::ORROctree::ORROctree ()
: voxel_size_ (-1.0),
  tree_levels_ (-1),
  root_ (NULL),
  randgen_ (0, 1, static_cast<uint32_t> (time (NULL)))
{
//  mInputPoints = NULL;
//  // Set illegal bounds
//  mBoundsOfFullLeafs[0] = mBounds[0] = 1.0; mBoundsOfFullLeafs[1] = mBounds[1] = -1.0;
//  mBoundsOfFullLeafs[2] = mBounds[2] = 1.0; mBoundsOfFullLeafs[3] = mBounds[3] = -1.0;
//  mBoundsOfFullLeafs[4] = mBounds[4] = 1.0; mBoundsOfFullLeafs[5] = mBounds[5] = -1.0;
}

//================================================================================================================================================================

void
pcl::recognition::ORROctree::clear ()
{
  if ( root_ )
  {
    delete root_;
    root_ = NULL;
  }

  full_leaves_.clear();
  full_leaf_counter_ = 0;
}

//================================================================================================================================================================

void
pcl::recognition::ORROctree::build (const PointCloudIn& points, float voxelsize, const PointCloudN* normals)
{
  if ( voxelsize <= 0 )
    return;

  this->clear();

  points_ = &points;
  voxel_size_ = voxelsize;

  // Get the bounds of the input point set
  PointXYZ min, max;
  getMinMax3D(points, min, max);

#ifdef PCL_REC_ORR_OCTREE_VERBOSE
  printf("ORROctree::%s(): start\n", __func__);
  printf("point set bounds =\n"
         "[%f, %f]\n"
         "[%f, %f]\n"
         "[%f, %f]\n", min.x, max.x, min.y, max.y, min.z, max.z);
#endif

  // Get the largest side of the bounding box of the input point set
  float point_set_extent = std::max (std::max (max.x-min.x, max.y-min.y), max.z-min.z);

  // Enlarge the extent a little bit in order to avoid points lying exact on the boundaries of the octree
  point_set_extent += point_set_extent*0.00001f;

  // Compute the center of the octree
  float center[3] = {0.5f*(min.x+max.x), 0.5f*(min.y+max.y), 0.5f*(min.z+max.z)};
  float arg = point_set_extent/voxelsize;

  // Compute the number of tree levels
  if ( arg > 1.0f )
    tree_levels_ = static_cast<int> (ceil (log (arg)/log (2.0)) + 0.5);
  else
    tree_levels_ = 0;

#ifdef PCL_REC_ORR_OCTREE_VERBOSE
  printf("tree levels = %i\n", tree_levels_);
#endif

  // Compute the number of octree levels and the bounds of the root
  float half_root_side = static_cast<float> (0.5f*pow (2.0, tree_levels_)*voxelsize);

  // Determine the bounding box of the octree
  bounds_[0] = center[0] - half_root_side;
  bounds_[1] = center[0] + half_root_side;
  bounds_[2] = center[1] - half_root_side;
  bounds_[3] = center[1] + half_root_side;
  bounds_[4] = center[2] - half_root_side;
  bounds_[5] = center[2] + half_root_side;

  // Create and initialize the root
  root_ = new ORROctree::Node();
  root_->setCenter(center);
  root_->setBounds(bounds_);
  root_->setParent(NULL);
  root_->computeRadius();

  const float *c;
  int i, l, id, num_points = static_cast<int> (points_->size ());
  ORROctree::Node* node;

  // Fill the leaves with the points
  for ( i = 0 ; i < num_points ; ++i )
  {
    // Some initialization
    node = root_;

    // Go down to the right leaf
    for ( l = 0 ; l < tree_levels_ ; ++l )
    {
      node->createChildren (); // If this node already has children -> nothing will happen
      c = node->getCenter ();
      id = 0;

      if ( points[i].x >= c[0] ) id |= 4;
      if ( points[i].y >= c[1] ) id |= 2;
      if ( points[i].z >= c[2] ) id |= 1;

      node = node->getChild (id);
    }

    // Now, that we have the right leaf -> fill it
    if ( node->getData() == NULL )
    {
      Node::Data* data = new Node::Data ();
      // Compute the 3d integer id of the leaf
      data->set3dId(
        static_cast<int> ((node->getCenter ()[0] - bounds_[0])/voxel_size_),
        static_cast<int> ((node->getCenter ()[1] - bounds_[2])/voxel_size_),
        static_cast<int> ((node->getCenter ()[2] - bounds_[4])/voxel_size_));
      // Save the data
      node->setData (data);
      full_leaves_.push_back (node);
    }

    node->getData ()->addToPoint (points[i].x, points[i].y, points[i].z);
    node->getData ()->addPointId (i);
    if ( normals )
      node->getData ()->addToNormal (normals->at(i).normal_x, normals->at(i).normal_y, normals->at(i).normal_z);
  }

  // Now iterate over all full leaves and compute the normals and average points
  for ( vector<ORROctree::Node*>::iterator it = full_leaves_.begin() ; it != full_leaves_.end() ; ++it )
  {
    aux::vecMult3 ((*it)->getData ()->getPoint (), 1.0f/static_cast<float> ((*it)->getData ()->getNumberOfPoints ()));
    if ( normals )
      aux::vecNormalize3 ((*it)->getData ()->getNormal ());
  }

#ifdef PCL_REC_ORR_OCTREE_VERBOSE
  printf("ORROctree::%s(): end\n", __func__);
#endif
}

//================================================================================================================================================================

void
pcl::recognition::ORROctree::Node::createChildren()
{
  if ( children_ )
    return;

  float bounds[6], center[3], childside = 0.5f*(bounds_[1]-bounds_[0]);
  children_ = new ORROctree::Node[8];

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
    children_[i].child_nr_ = i;
    children_[i].computeRadius();
    children_[i].setParent(this);
  }
}

//====================================================================================================

void
pcl::recognition::ORROctree::getFullLeavesIntersectedBySphere (const float* p, float radius, std::list<ORROctree::Node*>& out) const
{
  list<ORROctree::Node*> nodes;
  nodes.push_back (root_);

  ORROctree::Node *node, *child;

  int i;

  while ( !nodes.empty () )
  {
    // Get the last element in the list
    node = nodes.back ();
    // Remove the last element from the list
    nodes.pop_back ();

    // Check if the sphere intersects the current node
    if ( fabs (radius - aux::vecDistance3 (p, node->getCenter ())) <= node->getRadius () )
    {
      // We have an intersection -> push back the children of the current node
      if ( node->hasChildren () )
      {
        for ( i = 0 ; i < 8 ; ++i )
        {
          child = node->getChild (i);
          // We do not want to push all children -> only leaves or children with children
          if ( child->getData () || child->hasChildren () )
            nodes.push_back (child);
        }
      }
      else if ( node->getData () )
        out.push_back (node); // We got a full leaf
    }
  }
}

//================================================================================================================================================================

ORROctree::Node*
pcl::recognition::ORROctree::getRandomFullLeafOnSphere (const float* p, float radius)
{
  int i, rand_id;
  vector<int> tmp_ids;
  tmp_ids.reserve (8);

  list<ORROctree::Node*> nodes;
  nodes.push_back (root_);

  while ( !nodes.empty () )
  {
    // Get the last element in the list
    ORROctree::Node* node = nodes.back ();
    // Remove the last element from the list
    nodes.pop_back ();

    // Check if the sphere intersects the current node
    if ( fabs (radius - aux::vecDistance3<float> (p, node->getCenter ())) <= node->getRadius () )
    {
      // We have an intersection -> push back the children of the current node
      if ( node->hasChildren () )
      {
        // Prepare the tmp id vector
        for ( i = 0 ; i < 8 ; ++i )
          tmp_ids.push_back (i);

        // Push back the children in random order
        for ( i = 0 ; i < 8 ; ++i )
        {
          randgen_.setParameters (0, static_cast<int> (tmp_ids.size ()));
          rand_id = randgen_.run ();
          nodes.push_back (node->getChild (tmp_ids[rand_id]));
          // Remove the randomly selected id
          tmp_ids.erase (tmp_ids.begin () + rand_id);
        }
      }
      else if ( node->getData () )
        return node;
    }
  }

  return NULL;
}

//================================================================================================================================================================

void
pcl::recognition::ORROctree::deleteBranch (Node* node)
{
  node->deleteChildren ();
  node->deleteData ();

  Node *children, *parent = node->getParent ();
  int i;

  // Go up until you reach a node which has other non-empty children (i.e., children with other children or with data)
  while ( parent )
  {
    children = parent->getChildren ();
    // Check the children
	for ( i = 0 ; i < 8 ; ++i )
    {
      if ( children[i].hasData () || children[i].hasChildren () )
        break;
    }

	// There are no children with other children or with data -> delete them all!
	if ( i == 8 )
    {
      parent->deleteChildren ();
      parent->deleteData ();
      // Go one level up
      parent = parent->getParent ();
    }
	else
      // Terminate the deleting process
      break;
  }
}

//================================================================================================================================================================

void
pcl::recognition::ORROctree::getFullLeafPoints (PointCloudOut& out)
{
  out.resize(full_leaves_.size ());
  size_t i = 0;

  // Now iterate over all full leaves and compute the normals and average points
  for ( vector<ORROctree::Node*>::iterator it = full_leaves_.begin() ; it != full_leaves_.end() ; ++it, ++i )
  {
    out[i].x = (*it)->getData ()->getPoint ()[0];
    out[i].y = (*it)->getData ()->getPoint ()[1];
    out[i].z = (*it)->getData ()->getPoint ()[2];
  }
}

//================================================================================================================================================================

void
pcl::recognition::ORROctree::getNormalsOfFullLeaves (PointCloudN& out)
{
  out.resize(full_leaves_.size ());
  size_t i = 0;

  // Now iterate over all full leaves and compute the normals and average points
  for ( vector<ORROctree::Node*>::iterator it = full_leaves_.begin() ; it != full_leaves_.end() ; ++it, ++i )
  {
    out[i].normal_x = (*it)->getData ()->getNormal ()[0];
    out[i].normal_y = (*it)->getData ()->getNormal ()[1];
    out[i].normal_z = (*it)->getData ()->getNormal ()[2];
  }
}

//================================================================================================================================================================
