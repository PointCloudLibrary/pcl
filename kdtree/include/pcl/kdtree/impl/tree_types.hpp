/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: tree_types.hpp 34692 2010-12-12 06:12:22Z rusu $
 *
 */

#ifndef PCL_KDTREE_IMPL_TREE_TYPES_H_
#define PCL_KDTREE_IMPL_TREE_TYPES_H_

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/organized_data.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::initTree (const int &spatial_locator, boost::shared_ptr<pcl::KdTree<PointT> > &tree, int k)
{
  // This should be a bit more dynamic, but since dynamic_reconfigure's API needs similar entries, we're fine for now
  switch (spatial_locator)
  {
    case KDTREE_FLANN:
    {
      tree.reset (new KdTreeFLANN<PointT> ());
      break;
    }
    case KDTREE_ORGANIZED_INDEX:
    {
      tree.reset (new OrganizedDataIndex<PointT> ());
      if (k != 0)
      {
        boost::shared_ptr<OrganizedDataIndex<PointT> > tree_organized = boost::static_pointer_cast<OrganizedDataIndex<PointT> > (tree);
        // Special case: use k as a rectangular window.
        tree_organized->setSearchWindowAsK (k);
        ROS_DEBUG ("[pcl::initTree] Setting the horizontal/vertical window to %d/%d, given k = %d.",
                   tree_organized->getHorizontalSearchWindow (), tree_organized->getVerticalSearchWindow (), k);

        // Default parameters
        tree_organized->setMinPts (3);           // Need at least 3 nearest neighbors to estimate a viable feature
        tree_organized->setMaxDistance (0.1);
      }
      break;
    }
    default:
    {
      ROS_WARN ("[pcl::initTree] No spatial locator or wrong spatial locator given (%d)!", spatial_locator);
      tree.reset ();
      break;
    }
  }
}

#define PCL_INSTANTIATE_initTree(T) template void pcl::initTree<T> (const int &, boost::shared_ptr<pcl::KdTree<T> > &, int);

#endif  //#ifndef PCL_KDTREE_IMPL_TREE_TYPES_H_
