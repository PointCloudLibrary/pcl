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
 * $id $
 */

#ifndef PCL_SEGMENTATION_IMPL_SEEDED_HUE_SEGMENTATION_H_
#define PCL_SEGMENTATION_IMPL_SEEDED_HUE_SEGMENTATION_H_

#include <pcl/segmentation/seeded_hue_segmentation.h>

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::seededHueSegmentation (const PointCloud<PointXYZRGB>                         &cloud, 
                            const boost::shared_ptr<search::Search<PointXYZRGB> > &tree,
                            float                                                 tolerance, 
                            PointIndices                                          &indices_in,
                            PointIndices                                          &indices_out,
                            float                                                 delta_hue)
{
  if (tree->getInputCloud ()->points.size () != cloud.points.size ())
  {
    PCL_ERROR ("[pcl::seededHueSegmentation] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
    return;
  }
  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.points.size (), false);

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;

  // Process all points in the indices vector
  for (size_t k = 0; k < indices_in.indices.size (); ++k)
  {
    int i = indices_in.indices[k];
    if (processed[i])
      continue;

    processed[i] = true;

    std::vector<int> seed_queue;
    int sq_idx = 0;
    seed_queue.push_back (i);

    PointXYZRGB  p;
    p = cloud.points[i];
    PointXYZHSV h;
    PointXYZRGBtoXYZHSV(p, h);

    while (sq_idx < static_cast<int> (seed_queue.size ()))
    {
      int ret = tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances, std::numeric_limits<int>::max());
      if(ret == -1)
        PCL_ERROR("[pcl::seededHueSegmentation] radiusSearch returned error code -1");
      // Search for sq_idx
      if (!ret)
      {
        sq_idx++;
        continue;
      }

      for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
      {
        if (processed[nn_indices[j]])                             // Has this point been processed before ?
          continue;

        PointXYZRGB  p_l;
        p_l = cloud.points[nn_indices[j]];
        PointXYZHSV h_l;
        PointXYZRGBtoXYZHSV(p_l, h_l);

        if (fabs(h_l.h - h.h) < delta_hue)
        {
          seed_queue.push_back (nn_indices[j]);
          processed[nn_indices[j]] = true;
        }
      }

      sq_idx++;
    }
    // Copy the seed queue into the output indices
    for (size_t l = 0; l < seed_queue.size (); ++l)
      indices_out.indices.push_back(seed_queue[l]);
  }
  // This is purely esthetical, can be removed for speed purposes
  std::sort (indices_out.indices.begin (), indices_out.indices.end ());
}
//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::seededHueSegmentation (const PointCloud<PointXYZRGB>                           &cloud, 
                            const boost::shared_ptr<search::Search<PointXYZRGBL> >  &tree,
                            float                                                   tolerance, 
                            PointIndices                                            &indices_in,
                            PointIndices                                            &indices_out,
                            float                                                   delta_hue)
{
  if (tree->getInputCloud ()->points.size () != cloud.points.size ())
  {
    PCL_ERROR ("[pcl::seededHueSegmentation] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
    return;
  }
  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.points.size (), false);

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;

  // Process all points in the indices vector
  for (size_t k = 0; k < indices_in.indices.size (); ++k)
  {
    int i = indices_in.indices[k];
    if (processed[i])
      continue;

    processed[i] = true;

    std::vector<int> seed_queue;
    int sq_idx = 0;
    seed_queue.push_back (i);

    PointXYZRGB  p;
    p = cloud.points[i];
    PointXYZHSV h;
    PointXYZRGBtoXYZHSV(p, h);

    while (sq_idx < static_cast<int> (seed_queue.size ()))
    {
      int ret = tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances, std::numeric_limits<int>::max());
      if(ret == -1)
        PCL_ERROR("[pcl::seededHueSegmentation] radiusSearch returned error code -1");
      // Search for sq_idx
      if (!ret)
      {
        sq_idx++;
        continue;
      }
      for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
      {
        if (processed[nn_indices[j]])                             // Has this point been processed before ?
          continue;

        PointXYZRGB  p_l;
        p_l = cloud.points[nn_indices[j]];
        PointXYZHSV h_l;
        PointXYZRGBtoXYZHSV(p_l, h_l);

        if (fabs(h_l.h - h.h) < delta_hue)
        {
          seed_queue.push_back (nn_indices[j]);
          processed[nn_indices[j]] = true;
        }
      }

      sq_idx++;
    }
    // Copy the seed queue into the output indices
    for (size_t l = 0; l < seed_queue.size (); ++l)
      indices_out.indices.push_back(seed_queue[l]);
  }
  // This is purely esthetical, can be removed for speed purposes
  std::sort (indices_out.indices.begin (), indices_out.indices.end ());
}
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

void 
pcl::SeededHueSegmentation::segment (PointIndices &indices_in, PointIndices &indices_out)
{
  if (!initCompute () || 
      (input_ != 0   && input_->points.empty ()) ||
      (indices_ != 0 && indices_->empty ()))
  {
    indices_out.indices.clear ();
    return;
  }

  // Initialize the spatial locator
  if (!tree_)
  {
    if (input_->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointXYZRGB> ());
    else
      tree_.reset (new pcl::search::KdTree<PointXYZRGB> (false));
  }

  // Send the input dataset to the spatial locator
  tree_->setInputCloud (input_);
  seededHueSegmentation (*input_, tree_, static_cast<float> (cluster_tolerance_), indices_in, indices_out, delta_hue_);
  deinitCompute ();
}

#endif        // PCL_EXTRACT_CLUSTERS_IMPL_H_
