/*
 * Software License Agreement (BSD License)
 * 
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_FILTERS_IMPL_LOCAL_MIN_MAX_H_
#define PCL_FILTERS_IMPL_LOCAL_MIN_MAX_H_

#include <pcl/common/io.h>
#include <pcl/filters/local_min_max.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/octree/octree.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::LocalMinMax<PointT>::applyFilter (PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  std::vector<int> indices;

  output.is_dense = true;
  applyFilterIndices (indices);
  pcl::copyPointCloud<PointT> (*input_, indices, output);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::LocalMinMax<PointT>::applyFilterIndices (std::vector<int> &indices)
{
  if (locality_type_ == LT_GRID)
    applyGridFilter (indices);
  else
    applyLocalFilter (indices);
}

template <typename PointT> void
pcl::LocalMinMax<PointT>::applyGridFilter (std::vector<int> &indices)
{
  indices.reserve (indices_->size ());
  removed_indices_->reserve (indices_->size ());

  // Get the minimum and maximum dimensions
  Eigen::Vector4f min_p, max_p;
  getMinMax3D<PointT> (*input_, *indices_, min_p, max_p);

  // setup the dimension indices
  int dimIdx1, dimIdx2;
  if (projection_normal_ == PN_X)
  {
    dimIdx1 = 1;
    dimIdx2 = 2;
  }
  else if (projection_normal_ == PN_Y)
  {
    dimIdx1 = 0;
    dimIdx2 = 2;
  }
  else if (projection_normal_ == PN_Z)
  {
    dimIdx1 = 0;
    dimIdx2 = 1;
  }

  // Check that the resolution is not too small, given the size of the data
  int64_t dx = static_cast<int64_t> ((max_p[dimIdx1] - min_p[dimIdx1]) * inverse_resolution_)+1;
  int64_t dy = static_cast<int64_t> ((max_p[dimIdx2] - min_p[dimIdx2]) * inverse_resolution_)+1;

  if ((dx*dy) > static_cast<int64_t> (std::numeric_limits<int32_t>::max ()))
  {
    PCL_WARN ("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName ().c_str ());
    return;
  }

  Eigen::Vector2i min_b, max_b, div_b, divb_mul;
  int num_grid_sections;

  // Compute the minimum and maximum bounding box values
  min_b[0] = static_cast<int> (floor (min_p[dimIdx1] * inverse_resolution_));
  max_b[0] = static_cast<int> (floor (max_p[dimIdx1] * inverse_resolution_));
  min_b[1] = static_cast<int> (floor (min_p[dimIdx2] * inverse_resolution_));
  max_b[1] = static_cast<int> (floor (max_p[dimIdx2] * inverse_resolution_));

  // Compute the size of each division
  div_b = max_b - min_b + Eigen::Vector2i::Ones ();

  // Set up the division multiplier and number of grid sections
  divb_mul = Eigen::Vector2i (1, div_b[1]);
  num_grid_sections = div_b[0]*div_b[1];

  std::vector<int> index_vector (num_grid_sections);
  std::vector<int> index_saved (num_grid_sections, 0);

  // Go over all points and insert them into the index_vector vector with 
  // calculated idx if they are locally minimal/maximal. Points with the 
  // same idx value will contribute to the same point of resulting CloudPoint
  for (std::vector<int>::const_iterator it = indices_->begin (); it != indices_->end (); ++it)
  {
    if (!input_->is_dense)
      // Check if the point is invalid
      if (!pcl_isfinite (input_->points[*it].x) ||
          !pcl_isfinite (input_->points[*it].y) ||
          !pcl_isfinite (input_->points[*it].z))
        continue;

    float new_value;
    float val1, val2;
    int min1, min2;

    if (projection_normal_ == PN_X)
    {
      new_value = input_->points[*it].x;
      val1 = input_->points[*it].y;
      val2 = input_->points[*it].z;
    }
    else if (projection_normal_ == PN_Y)
    {
      new_value = input_->points[*it].y;
      val1 = input_->points[*it].x;
      val2 = input_->points[*it].z;
    }
    else if (projection_normal_ == PN_Z)
    {
      new_value = input_->points[*it].z;
      val1 = input_->points[*it].x;
      val2 = input_->points[*it].y;
    }

    min1 = min_b[0];
    min2 = min_b[1];

    int ijk0 = static_cast<int> (floor (val1 * inverse_resolution_) - static_cast<float> (min1));
    int ijk1 = static_cast<int> (floor (val2 * inverse_resolution_) - static_cast<float> (min2));

    // Compute the grid cell index
    int idx = ijk0 * divb_mul[0] + ijk1 * divb_mul[1];

    // if this is the first point we've seen in this grid section, save the index
    if (index_saved[idx] == 0)
    {
      index_saved[idx]++;
      index_vector[idx] = *it;
    }
    else 
    {
      index_saved[idx]++;

      float current_value;
      if (projection_normal_ == PN_X)
        current_value = input_->points[index_vector[idx]].x;
      else if (projection_normal_ == PN_Y)
        current_value = input_->points[index_vector[idx]].y;
      else if (projection_normal_ == PN_Z)
        current_value = input_->points[index_vector[idx]].z;
      
      if (stat_type_ == ST_MIN && new_value < current_value)
      {
        if (!negative_)
          indices.push_back (index_vector[idx]);
        if (negative_ && extract_removed_indices_)
          removed_indices_->push_back (index_vector[idx]);

        index_vector[idx] = *it;
      }
      else if (stat_type_ == ST_MAX && new_value > current_value)
      {
        if (!negative_)
          indices.push_back (index_vector[idx]);
        if (negative_ && extract_removed_indices_)
          removed_indices_->push_back (index_vector[idx]);

        index_vector[idx] = *it;
      }
      else
      {
        if (!negative_)
          indices.push_back (*it);
        if (negative_ && extract_removed_indices_)
          removed_indices_->push_back (*it);
      }
    }
  }
  
  // only save the selected points if:
  //   negative_ == false and extract_removed_indices_ == true OR
  //   negative_ == true
  if (!negative_ && extract_removed_indices_ || negative_)
  {
    for (int i = 0; i < index_saved.size (); i++)
    {
      // don't eliminate a point if it is the 
      // only point within it's grid division
      if (index_saved[i] > 1)
      {
        if (!negative_)
          removed_indices_->push_back (index_vector[i]);
        else
          indices.push_back (index_vector[i]);
      }
    }
  }
}

template <typename PointT> void
pcl::LocalMinMax<PointT>::applyLocalFilter (std::vector<int> &indices)
{
  typename PointCloud::Ptr cloud_projected (new PointCloud);

  Eigen::Vector4f min_p, max_p;
  Eigen::Vector4i min_b, max_b, div_b, divb_mul;

  // Create a set of planar coefficients
  std::vector<float> coefficients (4);
  pcl::ModelCoefficients::Ptr model_coefficients (new pcl::ModelCoefficients ());
  coefficients[0] = (projection_normal_ == PN_X) ? 1.0 : 0.0;
  coefficients[1] = (projection_normal_ == PN_Y) ? 1.0 : 0.0;
  coefficients[2] = (projection_normal_ == PN_Z) ? 1.0 : 0.0;
  coefficients[3] = 0.0;
  model_coefficients->values.insert(model_coefficients->values.begin (), coefficients.begin (), coefficients.end ());
  
  // Create the filtering object and project input
  pcl::ProjectInliers<PointT> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (input_);
  proj.setModelCoefficients (model_coefficients);
  proj.filter (*cloud_projected);

  // Initialize the search class
  if (locality_type_ == LT_BOX)
  {
    if (!octree_)
      octree_.reset (new pcl::octree::OctreePointCloudSearch<PointT> (resolution_));

    octree_->setInputCloud (cloud_projected);
    octree_->addPointsFromInputCloud ();
  }
  else if (locality_type_ == LT_RADIUS || locality_type_ == LT_KNN)
  {
    if (!searcher_)
    {
      if (input_->isOrganized ())
        searcher_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
      else
        searcher_.reset (new pcl::search::KdTree<PointT> (false));
    }
    searcher_->setInputCloud (cloud_projected);
  }

  // The arrays to be used
  indices.resize (indices_->size ());
  removed_indices_->resize (indices_->size ());
  int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator

  std::vector<bool> point_meets_criteria (indices_->size (), false);
  std::vector<bool> point_is_visited (indices_->size (), false);

  float half_res = resolution_ / 2.0f;

  // Find all points within bounds (e.g. radius, box, KNN) of the query
  // point, removing those that are locally minimal/maximal
  for (int iii = 0; iii < static_cast<int> (indices_->size ()); ++iii)
  {
    // This simply checks to make sure that the point is valid
    if (!isFinite (input_->points[(*indices_)[iii]]))
    {
      continue;
    }

    // Points in the neighborhood of a previously identified point which meets
    // the criteria, will not be minimal/maximal in their own neighborhood for
    // radius and box search.  This does not apply for k-nearest neighbors.
    if (point_is_visited[(*indices_)[iii]] && !point_meets_criteria[(*indices_)[iii]])
    {
      continue;
    }

    // Assume the current query point meets the criteria, mark as visited
    point_meets_criteria[(*indices_)[iii]] = true;
    point_is_visited[(*indices_)[iii]] = true;

    // Perform the search in the projected cloud
    std::vector<int> result_indices;
    std::vector<float> result_dists;
    PointT p = cloud_projected->points[(*indices_)[iii]];
    if (locality_type_ == LT_BOX)
    {
      Eigen::Vector3f bbox_min, bbox_max;
      float minx = (projection_normal_ == PN_X) ? -std::numeric_limits<float>::max () : p.x - half_res;
      float miny = (projection_normal_ == PN_Y) ? -std::numeric_limits<float>::max () : p.y - half_res;
      float minz = (projection_normal_ == PN_Z) ? -std::numeric_limits<float>::max () : p.z - half_res;
      float maxx = (projection_normal_ == PN_X) ? std::numeric_limits<float>::max () : p.x + half_res;
      float maxy = (projection_normal_ == PN_Y) ? std::numeric_limits<float>::max () : p.y + half_res;
      float maxz = (projection_normal_ == PN_Z) ? std::numeric_limits<float>::max () : p.z + half_res;
      bbox_min = Eigen::Vector3f (minx, miny, minz);
      bbox_max = Eigen::Vector3f (maxx, maxy, maxz);

      if (octree_->boxSearch (bbox_min, bbox_max, result_indices) == 0)
      {
        PCL_WARN ("[pcl::%s::applyFilter] Searching for neighbors with resolution %f failed.\n", getClassName ().c_str (), resolution_);
        continue;
      }
    }
    else if (locality_type_ == LT_RADIUS)
    {
      if (searcher_->radiusSearch (p, radius_, result_indices, result_dists) == 0)
      {
        PCL_WARN ("[pcl::%s::applyFilter] Searching for neighbors within radius %f failed.\n", getClassName ().c_str (), radius_);
        continue;
      }
    }
    else if (locality_type_ == LT_KNN)
    {
      if (searcher_->nearestKSearch (p, num_neighbors_+1, result_indices, result_dists) == 0)
      {
        PCL_WARN ("[pcl::%s::applyFilter] Searching for %d nearest neighbors failed.\n", getClassName ().c_str (), num_neighbors_);
        continue;
      }
    }

    // If query point is alone, we retain it regardless
    if (result_indices.size () == 1)
    {
        point_meets_criteria[(*indices_)[iii]] = false;
    }

    // Check to see if a current point meets the criteria
    // (i.e. current point is local min/max)
    float query_value = coefficients[0]*input_->points[(*indices_)[iii]].x + coefficients[1]*input_->points[(*indices_)[iii]].y + coefficients[2]*input_->points[(*indices_)[iii]].z;
    for (size_t k = 1; k < result_indices.size (); ++k)  // k = 1 is the first neighbor
    {
      float point_value = coefficients[0]*input_->points[result_indices[k]].x + coefficients[1]*input_->points[result_indices[k]].y + coefficients[2]*input_->points[result_indices[k]].z;
      if (stat_type_ == ST_MAX && point_value > query_value || stat_type_ == ST_MIN && point_value < query_value)
      {
        // Query point does not meet the criteria, no need to check others
        point_meets_criteria[(*indices_)[iii]] = false;
        break;
      }
    }

    // If the query point met the criteria, all neighbors can be marked as
    // visited, excluding them from future consideration unless we are 
    // using K-nearest neighbors as our search criteria
    if (point_meets_criteria[(*indices_)[iii]] && locality_type_ != LT_KNN)
    {
      for (size_t k = 1; k < result_indices.size (); ++k)  // k = 1 is the first neighbor
      {
        if (point_is_visited[result_indices[k]] != true)
        {
          point_is_visited[result_indices[k]] = true;

          if (!negative_)
            indices[oii++] = result_indices[k];
          if (negative_ && extract_removed_indices_)
            (*removed_indices_)[rii++] = result_indices[k];
        }
      }
    }

    // Points that meet the criteria are passed to removed indices
    // Unless negative was set, then it's the opposite condition
    if ((!negative_ && point_meets_criteria[(*indices_)[iii]]) || (negative_ && !point_meets_criteria[(*indices_)[iii]]))
    {
      if (extract_removed_indices_)
      {
        (*removed_indices_)[rii++] = (*indices_)[iii];
      }

      continue;
    }

    // Otherwise it was a normal point for output (inlier)
    indices[oii++] = (*indices_)[iii];
  }

  // Resize the output arrays
  indices.resize (oii);
  removed_indices_->resize (rii);
}

#define PCL_INSTANTIATE_LocalMinMax(T) template class PCL_EXPORTS pcl::LocalMinMax<T>;

#endif    // PCL_FILTERS_IMPL_LOCAL_MIN_MAX_H_

