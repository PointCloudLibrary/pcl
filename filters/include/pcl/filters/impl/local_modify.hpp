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

#ifndef PCL_FILTERS_IMPL_LOCAL_MODIFY_H_
#define PCL_FILTERS_IMPL_LOCAL_MODIFY_H_

#include <pcl/common/io.h>
#include <pcl/filters/local_modify.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/octree/octree.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::LocalModify<PointT>::applyFilter (PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  output.is_dense = true;
  if (locality_type_ == LT_GRID)
    applyGridFilter (output);
  else
    applyLocalFilter (output);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename PointT> void
pcl::LocalModify<PointT>::applyGridFilter (PointCloud &output)
{
  // start by copying input to output
  pcl::copyPointCloud<PointT> (*input_, *indices_, output);

  // Get the minimum and maximum dimensions
  Eigen::Vector4f min_p, max_p;
  getMinMax3D<PointT> (*input_, *indices_, min_p, max_p);

  // Check that the resolution is not too small, given the size of the data
  int64_t dx = static_cast<int64_t> ((max_p[0] - min_p[0]) * inverse_resolution_)+1;
  int64_t dy = static_cast<int64_t> ((max_p[1] - min_p[1]) * inverse_resolution_)+1;

  if ((dx*dy) > static_cast<int64_t> (std::numeric_limits<int32_t>::max ()))
  {
    PCL_WARN ("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName ().c_str ());
    return;
  }

  Eigen::Vector2i min_b, max_b, div_b, divb_mul;
  int num_grid_sections;

  // Compute the minimum and maximum bounding box values
  min_b[0] = static_cast<int> (floor (min_p[0] * inverse_resolution_));
  max_b[0] = static_cast<int> (floor (max_p[0] * inverse_resolution_));
  min_b[1] = static_cast<int> (floor (min_p[1] * inverse_resolution_));
  max_b[1] = static_cast<int> (floor (max_p[1] * inverse_resolution_));

  // Compute the size of each division
  div_b = max_b - min_b + Eigen::Vector2i::Ones ();

  // Set up the division multiplier and number of grid sections
  divb_mul = Eigen::Vector2i (1, div_b[1]);
  num_grid_sections = div_b[0]*div_b[1];

  std::vector<float> assignment_vector (num_grid_sections);
  std::vector<int> sum_count_vector (num_grid_sections, 0);
  std::vector< std::vector<float> > median_vector (num_grid_sections);

  #ifdef _OPENMP
  std::vector<omp_lock_t> write_locks (num_grid_sections);
  for (int i = 0; i < write_locks.size (); ++i)
    omp_init_lock(&write_locks[i]);
  #endif

  // Go over all points and update the vector containers depending upon the stat type
  #ifdef _OPENMP
  #pragma omp parallel for num_threads (threads_) schedule (dynamic)
  #endif
  for (int i = 0; i < indices_->size (); ++i)
  {
    if (!input_->is_dense)
      // Check if the point is invalid
      if (!pcl_isfinite (input_->points[(*indices_)[i]].x) ||
          !pcl_isfinite (input_->points[(*indices_)[i]].y) ||
          !pcl_isfinite (input_->points[(*indices_)[i]].z))
        continue;

    float new_value = input_->points[(*indices_)[i]].z;

    int ijk0 = static_cast<int> (floor (input_->points[(*indices_)[i]].x * inverse_resolution_) - static_cast<float> (min_b[0]));
    int ijk1 = static_cast<int> (floor (input_->points[(*indices_)[i]].y * inverse_resolution_) - static_cast<float> (min_b[1]));

    // Compute the grid cell index
    int idx = ijk0 * divb_mul[0] + ijk1 * divb_mul[1];
    
    #ifdef _OPENMP
    omp_set_lock(&write_locks[idx]);
    #endif

    float current_value = assignment_vector[idx];

    // stat type is min and the new value is smaller than our current 
    // value, or is the first value encountered for this grid index
    if (stat_type_ == ST_MIN && (new_value < current_value || sum_count_vector[idx] == 0))
    {
      // save the new min value
      assignment_vector[idx] = new_value;
      sum_count_vector[idx]++;
    }
    // stat type is max and the new value is greater than our current 
    // value, or is the first value encountered for this grid index
    else if (stat_type_ == ST_MAX && (new_value > current_value || sum_count_vector[idx] == 0))
    {
      // save the new max value
      assignment_vector[idx] = new_value;
      sum_count_vector[idx]++;
    }
    // stat type is mean, so we increment the running rum for this grid index
    else if (stat_type_ == ST_MEAN)
    {
      // increment the sum & count
      assignment_vector[idx] += new_value;
      sum_count_vector[idx]++;
    }
    // stat type is median, so we store this point in the vector associated with this grid index
    else if (stat_type_ == ST_MEDIAN)
    {
      // save the z value so we can compute the median later
      median_vector[idx].push_back (new_value);
    }

    #ifdef _OPENMP
    omp_unset_lock(&write_locks[idx]);
    #endif
  }

  // calculate the mean/median for each grid index
  if (stat_type_ == ST_MEDIAN || stat_type_ == ST_MEAN)
  {
    #ifdef _OPENMP
    #pragma omp parallel for num_threads (threads_) schedule (dynamic)
    #endif
    for (int i = 0; i < assignment_vector.size (); ++i)
    {
      if (stat_type_ == ST_MEDIAN && median_vector[i].size () > 0)
      {
        // sort the values & save the median
        std::sort (median_vector[i].begin (), median_vector[i].end ());

        // assign the median z value
        if (median_vector[i].size () % 2 == 1)
          assignment_vector[i] = median_vector[i][median_vector[i].size () / 2];
        else{
          assignment_vector[i] = (median_vector[i][(median_vector[i].size () / 2) - 1] + median_vector[i][median_vector[i].size () / 2]) / 2.0;
        }
      }
      else if (stat_type_ == ST_MEAN && sum_count_vector[i] > 0)
      {
        // calculate and save the mean
        assignment_vector[i] /= sum_count_vector[i];
      }
    }
  }

  // Assign the Min/Max/Mean/Median value to the z field of each cloud point 
  #ifdef _OPENMP
  #pragma omp parallel for num_threads (threads_)
  #endif
  for (int i = 0; i < indices_->size (); ++i)
  {
    if (!input_->is_dense)
      // Check if the point is invalid
      if (!pcl_isfinite (input_->points[(*indices_)[i]].x) ||
          !pcl_isfinite (input_->points[(*indices_)[i]].y) ||
          !pcl_isfinite (input_->points[(*indices_)[i]].z))
        continue;

    int ijk0 = static_cast<int> (floor (input_->points[(*indices_)[i]].x * inverse_resolution_) - static_cast<float> (min_b[0]));
    int ijk1 = static_cast<int> (floor (input_->points[(*indices_)[i]].y * inverse_resolution_) - static_cast<float> (min_b[1]));

    // Compute the grid cell index
    int idx = ijk0 * divb_mul[0] + ijk1 * divb_mul[1];

    // update the z value for this point
    output.points[(*indices_)[i]].z = assignment_vector[idx];
  }
}

template <typename PointT> void
pcl::LocalModify<PointT>::applyLocalFilter (PointCloud &output)
{

  typename PointCloud::Ptr cloud_projected (new PointCloud);

  // copy the input data to a temporary cloud
  pcl::copyPointCloud<PointT> (*input_, *indices_, *cloud_projected);

  // zero each point's z value
  #ifdef _OPENMP
  #pragma omp parallel for num_threads (threads_)
  #endif
  for (int i = 0; i < indices_->size (); ++i)
    cloud_projected->points[(*indices_)[i]].z = 0.0;

  // copy the input data to the output point cloud
  pcl::copyPointCloud<PointT> (*input_, *indices_, output);

  // Initialize the search class
  if (locality_type_ == LT_BOX)
  {
    octree_.reset (new pcl::octree::OctreePointCloudSearch<PointT> (resolution_));

    octree_->setInputCloud (cloud_projected);
    octree_->addPointsFromInputCloud ();
  }
  else if (locality_type_ == LT_RADIUS || locality_type_ == LT_KNN)
  {
    if (!searcher_)
    {
      if (cloud_projected->isOrganized ())
        searcher_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
      else
        searcher_.reset (new pcl::search::KdTree<PointT> (false));
    }
    searcher_->setInputCloud (cloud_projected);
  }

  float half_res = resolution_ / 2.0f;

  // Find all points within bounds (e.g. radius, box, KNN) of the query
  // point, and determine the min/max/mean/median
  #ifdef _OPENMP
  #pragma omp parallel for num_threads (threads_) schedule (dynamic)
  #endif
  for (int i = 0; i < indices_->size (); ++i)
  {
    // This simply checks to make sure that the point is valid
    if (!isFinite (cloud_projected->points[(*indices_)[i]]))
    {
      continue;
    }

    // Perform the search in the projected cloud
    std::vector<int> result_indices;
    std::vector<float> result_dists;
    PointT p = cloud_projected->points[(*indices_)[i]];
    if (locality_type_ == LT_BOX)
    {
      Eigen::Vector3f bbox_min, bbox_max;
      float minx = p.x - half_res;
      float miny = p.y - half_res;
      float minz = -std::numeric_limits<float>::max ();
      float maxx = p.x + half_res;
      float maxy = p.y + half_res;
      float maxz = std::numeric_limits<float>::max ();
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

    // now that we've found our neighbors, find the min/max/mean/median z value
    if (stat_type_ == ST_MIN || stat_type_ == ST_MAX)
    {
      Eigen::Vector4f min_p, max_p;

      // get the min/max points for the results
      getMinMax3D<PointT> (*input_, result_indices, min_p, max_p);

      // assign either min or max to the current point
      if (stat_type_ == ST_MIN)
        output.points[(*indices_)[i]].z = min_p.z ();
      else 
        output.points[(*indices_)[i]].z = max_p.z ();
    }
    else if (stat_type_ == ST_MEAN)
    {
      float sum = 0.0;

      // calculate the sum of the results
      for (int j = 0; j < result_indices.size (); ++j)
        sum += input_->points[result_indices[j]].z;

      // assign the sum divided by the number of points
      output.points[(*indices_)[i]].z = sum / (float)result_indices.size ();
    }
    else if (stat_type_ == ST_MEDIAN)
    {
      std::vector<float> z_values (result_indices.size (), 0.0);

      // copy the z values for the results
      for (int j = 0; j < result_indices.size (); ++j)
        z_values[j] = input_->points[result_indices[j]].z;
      
      // sort the z values
      std::sort (z_values.begin (), z_values.end ());

      // assign the median z value
      if (z_values.size () % 2 == 1)
        output.points[(*indices_)[i]].z = z_values[z_values.size () / 2];
      else
        output.points[(*indices_)[i]].z = (z_values[(z_values.size () / 2) - 1] + z_values[z_values.size () / 2]) / 2.0;
    }
  }
}

#define PCL_INSTANTIATE_LocalModify(T) template class PCL_EXPORTS pcl::LocalModify<T>;

#endif    // PCL_FILTERS_IMPL_LOCAL_MODIFY_H_

