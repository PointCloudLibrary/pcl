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

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::activeSegmentation (const pcl::PointCloud<PointT> &cloud_in,
    const pcl::PointCloud<pcl::Boundary> &boundary,
    const pcl::PointCloud<pcl::Normal> &normals,
    const boost::shared_ptr<pcl::search::Search<PointT> > &tree,
    int fp_index,
    float search_radius,
    pcl::PointIndices &indices_out)
{
  if (fp_index > cloud_in.points.size () || fp_index <0)
  {
    PCL_ERROR ("[pcl::activeSegmentation] Fixation point given is invalid\n");
    return;
  }
  if (boundary.points.size () != cloud_in.points.size ())
  {
    PCL_ERROR ("[pcl::activeSegmentation] Boundary map given was built for a different dataset (%zu) than the input cloud (%zu)!\n",
        boundary.points.size () , cloud_in.points.size ());
    return;
  }
  if (tree->getInputCloud ()->points.size () != cloud_in.points.size ())
  {
    PCL_ERROR ("[pcl::activeSegmentation] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n",
        tree->getInputCloud ()->points.size (), cloud_in.points.size ());
    return;
  }
  if (normals.points.size () != cloud_in.points.size ())
  {
    PCL_ERROR ("[pcl::activeSegmentation] Input Normals are for a different point cloud dataset (%zu) than the input cloud (%zu)!\n",
        normals.points.size (), cloud_in.points.size ());
    return;
  }

  std::queue<int> seed_queue;
  std::vector<bool> processed (cloud_in.size(), false);
  seed_queue.push (fp_index);
  indices_out.indices.push_back (fp_index);

  processed[fp_index] = true;
  int num_pts_in_segment = 1;

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;

  //process while there are valid seed points
  while (!seed_queue.empty ())
  {
    int curr_seed;
    curr_seed = seed_queue.front ();
    seed_queue.pop();
    // Search for seeds
    if (!tree->radiusSearch (curr_seed, search_radius, nn_indices, nn_distances))
      continue;
    //process all points found in the neighborhood
    for (unsigned int i = 0; i < nn_indices.size (); ++i)
    {
      if (processed[nn_indices[i]])
      {
        continue;
      }
      bool is_seed (false), is_boundary (false), is_valid (false);
      double dot_p1 = normals.points[nn_indices[i]].normal[0] * normals.points[fp_index].normal[0] +
          normals.points[nn_indices[i]].normal[1] * normals.points[fp_index].normal[1] +
          normals.points[nn_indices[i]].normal[2] * normals.points[fp_index].normal[2];

      PointT temp;
      temp.x = cloud_in.points[fp_index].x - cloud_in.points[nn_indices[i]].x;
      temp.y = cloud_in.points[fp_index].y - cloud_in.points[nn_indices[i]].y;
      temp.z = cloud_in.points[fp_index].z - cloud_in.points[nn_indices[i]].z;

      double dot_p2 = normals.points[nn_indices[i]].normal[0] * temp.x
          + normals.points[nn_indices[i]].normal[1] * temp.y
          + normals.points[nn_indices[i]].normal[2] * temp.z;

      if ((fabs (acos (dot_p1)) < 45 * M_PI / 180) || (fabs (acos (dot_p2)) > 90 * M_PI / 180))
      {
        if (boundary.points[nn_indices[i]].boundary_point != 0)
          is_boundary = true;
        else
          is_seed = true;
        is_valid = true;
      }
      if ((is_valid && is_seed) || is_boundary)
      {
        indices_out.indices.push_back (nn_indices[i]);
        num_pts_in_segment++;
        seed_queue.push (nn_indices[i]);
        processed[nn_indices[i]] = true;
        if (is_boundary)
          break;
      }

    } //new neighbor

  } //new seed point
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void 
pcl::ActiveSegmentation<PointT, NormalT>::setFixationPoint (const PointT &p)
{
  if (tree_->getInputCloud ()->points.size () != input_->points.size ())
  {
    PCL_ERROR (
        "[pcl::setFixationPoint] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n",
        tree_->getInputCloud ()->points.size (), input_->points.size ());
    return;
  }
  int K = 1;
  std::vector<int> pointIdxNKNSearch (K);
  std::vector<float> pointNKNSquaredDistance (K);
  if (tree_->nearestKSearch (p, K, pointIdxNKNSearch, pointNKNSquaredDistance) != 0)
  {
    fixation_point_ = input_->points[pointIdxNKNSearch[0]];
    fp_index_ = pointIdxNKNSearch[0];
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void 
pcl::ActiveSegmentation<PointT, NormalT>::segment (PointIndices &indices_out)
{
  if (fp_index_ > input_->points.size () || fp_index_ <0)
  {
    PCL_ERROR ("[pcl::segment] Fixation point given is invalid\n");
    return;
  }
  if (boundary_->points.size () != input_->points.size ())
  {
    PCL_ERROR ("[pcl::segment] Boundary map given was built for a different dataset (%zu) than the input cloud (%zu)!\n",
        boundary_->points.size () , input_->points.size ());
    return;
  }
  if (tree_->getInputCloud ()->points.size () != input_->points.size ())
  {
    PCL_ERROR ("[pcl::segment] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n",
        tree_->getInputCloud ()->points.size (), input_->points.size ());
    return;
  }
  if (normals_->points.size () != input_->points.size ())
  {
    PCL_ERROR ("[pcl::segment] Input Normals are for a different point cloud dataset (%zu) than the input cloud (%zu)!\n",
        normals_->points.size (), input_->points.size ());
    return;
  }
  std::queue<int> seed_queue;
  std::vector<bool> processed (input_->size(), false);
  seed_queue.push (fp_index_);
  indices_out.indices.push_back (fp_index_);

  processed[fp_index_] = true;
  int num_pts_in_segment = 1;

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;

  //process while there are valid seed points
  while (!seed_queue.empty ())
  {
    int curr_seed;
    curr_seed = seed_queue.front ();
    seed_queue.pop();

    // Search for seeds
    if (!tree_->radiusSearch (curr_seed, search_radius_, nn_indices, nn_distances))
      continue;

    //process all points found in the neighborhood
    for (unsigned int i = 0; i < nn_indices.size (); ++i)
    {
      if (processed[nn_indices[i]])
        continue;
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

    } //new neighbor

  } //new seed point
}

//////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool 
pcl::ActiveSegmentation<PointT, NormalT>::isPointValid (
    int v_point,
    int, bool &is_seed,
    bool &is_boundary
)
{
  double dot_p1 = normals_->points[v_point].normal[0] * normals_->points[fp_index_].normal[0] +
      normals_->points[v_point].normal[1] * normals_->points[fp_index_].normal[1] +
      normals_->points[v_point].normal[2] * normals_->points[fp_index_].normal[2];

  PointT temp;
  temp.x = input_->points[fp_index_].x - input_->points[v_point].x;
  temp.y = input_->points[fp_index_].y - input_->points[v_point].y;
  temp.z = input_->points[fp_index_].z - input_->points[v_point].z;

  double dot_p2 = normals_->points[v_point].normal[0] * temp.x
      + normals_->points[v_point].normal[1] * temp.y
      + normals_->points[v_point].normal[2] * temp.z;

  if ((fabs (acos (dot_p1)) < 45 * M_PI / 180) || (fabs (acos (dot_p2)) > 90 * M_PI / 180))
  {
    if (boundary_->points[v_point].boundary_point != 0)
    {
      is_boundary = true;
      is_seed = false;
    }
    else
    {
      is_boundary = false;
      is_seed = true;
    }
    return (true);
  }
  else
    return (false);
}

#define PCL_INSTANTIATE_ActiveSegmentation(T,NT) template class PCL_EXPORTS pcl::ActiveSegmentation<T,NT>;
#define PCL_INSTANTIATE_activeSegmentation(T) template PCL_EXPORTS void pcl::activeSegmentation<T>(const pcl::PointCloud<T> &,const pcl::PointCloud<pcl::Boundary> &, const pcl::PointCloud<pcl::Normal> &, const boost::shared_ptr<pcl::search::Search<T> > &, int, float, pcl::PointIndices &);

#endif /* ACTIVE_SEGMENTATION_HPP_ */
