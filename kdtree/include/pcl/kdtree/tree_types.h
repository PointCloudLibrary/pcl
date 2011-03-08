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
 * $Id: tree_types.h 34692 2010-12-12 06:12:22Z rusu $
 *
 */

#ifndef PCL_KDTREE_TREE_TYPES_H_
#define PCL_KDTREE_TREE_TYPES_H_

#include <pcl/kdtree/kdtree.h>

namespace pcl
{
  const static int KDTREE_FLANN           = 0;
  const static int KDTREE_ORGANIZED_INDEX = 1;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Initialize the spatial locator used for nearest neighbor search.
    * \param spatial_locator the type of spatial locator to construct (0 = FLANN, 1 = organized)
    * \param tree the resultant tree as a boost shared pointer
    * \param k optional parameter (default 0) applied only if the spatial locator is set to organized (1)
    */
  template <typename PointT> void initTree (const int &spatial_locator, boost::shared_ptr<pcl::KdTree<PointT> > &tree, int k = 0);
}

#endif  //#ifndef PCL_KDTREE_TREE_TYPES_H_
