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
 *
 */

/*
 * orr_octree_zprojection.cpp
 *
 *  Created on: Nov 17, 2012
 *      Author: papazov
 */

#include <pcl/recognition/ransac_based/orr_octree_zprojection.h>
#include <array>
#include <vector>


//=========================================================================================================================================

void
pcl::recognition::ORROctreeZProjection::clear ()
{
  if ( pixels_ )
  {
    for ( int i = 0 ; i < num_pixels_x_ ; ++i )
    {
      // Delete pixel by pixel
      for ( int j = 0 ; j < num_pixels_y_ ; ++j )
          delete pixels_[i][j];

      // Delete the whole row
      delete[] pixels_[i];
    }

    delete[] pixels_;
    pixels_ = nullptr;
  }

  if ( sets_ )
  {
    for ( int i = 0 ; i < num_pixels_x_ ; ++i )
    {
      for ( int j = 0 ; j < num_pixels_y_ ; ++j )
          delete sets_[i][j];

      delete[] sets_[i];
    }

    delete[] sets_;
    sets_ = nullptr;
  }

  full_sets_.clear ();
  full_pixels_.clear ();
}

//=========================================================================================================================================

void
pcl::recognition::ORROctreeZProjection::build (const ORROctree& input, float eps_front, float eps_back)
{
  this->clear();

  // Compute the bounding box of the full leaves
  const std::vector<ORROctree::Node*>& full_leaves = input.getFullLeaves ();
  std::array<float, 4> full_leaves_bounds;

  if ( full_leaves.empty() )
    return;

  // The initialization run
  full_leaves_bounds[0] = std::numeric_limits<float>::infinity();
  full_leaves_bounds[1] = -std::numeric_limits<float>::infinity();
  full_leaves_bounds[2] = std::numeric_limits<float>::infinity();
  full_leaves_bounds[3] = -std::numeric_limits<float>::infinity();

  for (const auto &leaf : full_leaves)
  {
    const auto bounds = leaf->getBounds ();
    if ( bounds[0] < full_leaves_bounds[0] ) full_leaves_bounds[0] = bounds[0];
    if ( bounds[1] > full_leaves_bounds[1] ) full_leaves_bounds[1] = bounds[1];
    if ( bounds[2] < full_leaves_bounds[2] ) full_leaves_bounds[2] = bounds[2];
    if ( bounds[3] > full_leaves_bounds[3] ) full_leaves_bounds[3] = bounds[3];
  }

  // Make some initializations
  pixel_size_ = input.getVoxelSize();
  inv_pixel_size_ = 1.0f/pixel_size_;

  bounds_[0] = full_leaves_bounds[0]; bounds_[1] = full_leaves_bounds[1];
  bounds_[2] = full_leaves_bounds[2]; bounds_[3] = full_leaves_bounds[3];

  extent_x_ = full_leaves_bounds[1] - full_leaves_bounds[0];
  extent_y_ = full_leaves_bounds[3] - full_leaves_bounds[2];

  num_pixels_x_ = static_cast<int> (extent_x_/pixel_size_ + 0.5f); // we do not need to round, but it's safer due to numerical errors
  num_pixels_y_ = static_cast<int> (extent_y_/pixel_size_ + 0.5f);
  num_pixels_   = num_pixels_x_*num_pixels_y_;

  // Allocate and initialize memory for the pixels and the sets
  pixels_ = new Pixel**[num_pixels_x_];
  sets_ = new Set**[num_pixels_x_];

  for ( int i = 0 ; i < num_pixels_x_ ; ++i )
  {
    pixels_[i] = new Pixel*[num_pixels_y_];
    sets_[i] = new Set*[num_pixels_y_];

    for ( int j = 0 ; j < num_pixels_y_ ; ++j )
    {
      pixels_[i][j] = nullptr;
      sets_[i][j] = nullptr;
    }
  }

  int pixel_id = 0;

  // Project the octree full leaves onto the xy-plane
  for (const auto &full_leaf : full_leaves)
  {
    this->getPixelCoordinates (full_leaf->getCenter(), num_pixels_x_, num_pixels_y_);
    // If there is no set/pixel and at this position -> create one
    if ( sets_[num_pixels_x_][num_pixels_y_] == nullptr )
    {
      pixels_[num_pixels_x_][num_pixels_y_] = new Pixel (pixel_id++);
      sets_[num_pixels_x_][num_pixels_y_] = new Set (num_pixels_x_, num_pixels_y_);
      full_pixels_.push_back (pixels_[num_pixels_x_][num_pixels_y_]);
      full_sets_.push_back (sets_[num_pixels_x_][num_pixels_y_]);
    }

    // Insert the full octree leaf at the right position in the set
    sets_[num_pixels_x_][num_pixels_y_]->insert (full_leaf);
  }

  // Now, at each occupied (i, j) position, get the longest connected component consisting of neighboring full leaves
  for (const auto &full_set : full_sets_)
  {
    // Get the first node in the set
    auto node = full_set->get_nodes ().begin ();
    // Initialize
    float best_min = (*node)->getBounds ()[4];
    float best_max = (*node)->getBounds ()[5];
    float cur_min = best_min;
    int id_z1 = (*node)->getData ()->get3dIdZ ();
    int maxlen = 1;
    int len = 1;

    // Find the longest 1D "connected component" at the current (i, j) position
    for ( ++node ; node != full_set->get_nodes ().end () ; ++node )
    {
      int id_z2 = (*node)->getData ()->get3dIdZ ();
      float cur_max = (*node)->getBounds()[5];

      if ( id_z2 - id_z1 > 1 ) // This connected component is over
      {
        // Start a new connected component
        cur_min = (*node)->getBounds ()[4];
        len = 1;
      }
      else // This connected component is still ongoing
      {
        ++len;
        if ( len > maxlen )
        {
          // This connected component is the longest one
          maxlen = len;
          best_min = cur_min;
          best_max = cur_max;
        }
      }
      id_z1 = id_z2;
    }

    int i = full_set->get_x ();
    int j = full_set->get_y ();

    pixels_[i][j]->set_z1 (best_min - eps_front);
    pixels_[i][j]->set_z2 (best_max  + eps_back);
  }
}
