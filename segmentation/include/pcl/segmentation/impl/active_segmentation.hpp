/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ACTIVE_SEGMENTATION_HPP_
#define ACTIVE_SEGMENTATION_HPP_

#include <pcl/segmentation/active_segmentation.h>

/*
 * \brief stand alone method for doing active segmentation
 * \param[1] input_cloud
 * \param[2] boundary map
 * \param[3] fixation point
 */
template <class PointT> void 
activeSegmentation (
    const typename pcl::PointCloud<PointT>::Ptr &cloud_in, 
    const pcl::PointIndices::Ptr indices_in,
    const pcl::PointCloud<pcl::Boundary>& b, 
    const pcl::PointCloud<pcl::Normal>,
    const typename pcl::search::Search<PointT>::Ptr tree, 
    pcl::PointIndices::Ptr &indices_out,
    int fp_indice)
{
  //TODO implement a standalone function
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void 
pcl::ActiveSegmentation<PointT, NormalT>::setFixationPoint (const PointT &p)
{
  if (tree_->getInputCloud ()->points.size () != input_->points.size ())
  {
    PCL_ERROR (
        "[pcl::setFixationPoint] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n", tree_->getInputCloud ()->points.size (), input_->points.size ());
    return;
  }
  int K = 1;
  std::vector<int> pointIdxNKNSearch (K);
  std::vector<float> pointNKNSquaredDistance (K);
  if (tree_->nearestKSearch (p, K, pointIdxNKNSearch, pointNKNSquaredDistance) != 0)
  {
    fixation_point_ = input_->points[pointIdxNKNSearch[0]];
    fp_indice_ = pointIdxNKNSearch[0];
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void 
pcl::ActiveSegmentation<PointT, NormalT>::segment (PointIndices &indices_out)
{
  std::queue<int> seed_queue;
  std::vector<bool> processed (input_->size(), false);
  seed_queue.push (fp_indice_);
  indices_out.indices.push_back (fp_indice_);

  processed[fp_indice_] = true;
  int num_pts_in_segment = 1;

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;

  while (!seed_queue.empty ())
  {
    int curr_seed;
    curr_seed = seed_queue.front ();
    seed_queue.pop();
    if (!tree_->radiusSearch (curr_seed, search_radius_, nn_indices, nn_distances))
      continue;
    for (unsigned int i = 0; i < nn_indices.size (); ++i)
    {
      if (processed[nn_indices[i]])
      {
        continue;
      }
      bool is_seed, is_boundary;
      bool is_valid = isPointValid (nn_indices[i], curr_seed, is_seed, is_boundary);
      if ((is_valid && is_seed) || is_boundary)
      {
        indices_out.indices.push_back (nn_indices[i]);
        num_pts_in_segment++;
        seed_queue.push (nn_indices[i]);
        processed[nn_indices[i]] = true;
        if (is_boundary)
          break;
      }

      /* {
       indices_out.indices.push_back(nn_indices[i]);
       num_pts_in_segment++;
       processed[nn_indices[i]] = true;

       }*/

    } //new neighbor

  } //new seed point
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool 
pcl::ActiveSegmentation<PointT, NormalT>::isPointValid (
    int v_point, int, bool &is_seed, bool &is_boundary)
{
  double dot_p1 = normals_->points[v_point].normal[0] * normals_->points[fp_indice_].normal[0]
      + normals_->points[v_point].normal[1] * normals_->points[fp_indice_].normal[1]
      + normals_->points[v_point].normal[2] * normals_->points[fp_indice_].normal[2];

  PointT temp;
  temp.x = input_->points[fp_indice_].x - input_->points[v_point].x;
  temp.y = input_->points[fp_indice_].y - input_->points[v_point].y;
  temp.z = input_->points[fp_indice_].z - input_->points[v_point].z;

  double dot_p2 = normals_->points[v_point].normal[0] * temp.x
      + normals_->points[v_point].normal[1] * temp.y
      + normals_->points[v_point].normal[2] * temp.z;

  if (fabs (acos (dot_p1)) < 45 * M_PI / 180)
  {
    if (boundary_->points[v_point].boundary_point != 0)
    {
      is_boundary = true;
      is_seed = false;
      return (true);
    }
    else
    {
      is_boundary = false;
      is_seed = true;
      return (true);
    }
  }
  else if (fabs (acos (dot_p2)) > 90 * M_PI / 180)
  {
    if (boundary_->points[v_point].boundary_point != 0)
    {
      is_boundary = true;
      is_seed = false;
      return (true);
    }
    else
    {
      is_boundary = false;
      is_seed = true;
      return (true);
    }

  }
  else
    return (false);
}

#endif /* ACTIVE_SEGMENTATION_HPP_ */
