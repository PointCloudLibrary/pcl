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
 * $Id: organized.hpp 3220 2011-11-21 15:50:53Z koenbuys $
 *
 */

#ifndef PCL_SEARCH_IMPL_ORGANIZED_NEIGHBOR_SEARCH_OMP_H_
#define PCL_SEARCH_IMPL_ORGANIZED_NEIGHBOR_SEARCH_OMP_H_

#include "pcl/search/organized_omp.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighborOMP<PointT>::radiusSearch (const PointCloudConstPtr    &cloud, 
                                                         int                         index,
                                                         double                      radius, 
                                                         std::vector<int>            &k_indices,
                                                         std::vector<float>          &k_sqr_distances, 
                                                         int                         max_nn)
{
  this->setInputCloud (cloud);
  return (radiusSearch (index, radius, k_indices, k_sqr_distances, max_nn));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighborOMP<PointT>::radiusSearch (int                 index, 
                                                         const double        radius,
                                                         std::vector<int>    &k_indices,
                                                         std::vector<float>  &k_sqr_distances, 
                                                         int                 max_nn) const
{
  const PointT searchPoint = getPointByIndex (index);
  return (radiusSearch (searchPoint, radius, k_indices, k_sqr_distances, max_nn));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighborOMP<PointT>::radiusSearch (const               PointT &p_q, 
                                                         const double        radius,
                                                         std::vector<int>    &k_indices,
                                                         std::vector<float>  &k_sqr_distances, 
                                                         int                 max_nn) const
{
  if (input_->height == 1 || input_->width == 1)
  {
    PCL_ERROR ("[pcl::%s::radiusSearch] Input dataset is not organized!\n", getName ().c_str ());
    return (-1);
  }

  // NAN test
  if(!pcl_isfinite (p_q.x) || !pcl_isfinite (p_q.y) || !pcl_isfinite (p_q.z) )
    return 0;

  // search window
  int leftX, rightX, leftY, rightY;
  //int x, y, idx;
  //double squared_distance, squared_radius;
  bool abort = false;

  k_indices.clear ();
  k_sqr_distances.clear ();

  const double squared_radius = radius * radius;

  this->getProjectedRadiusSearchBox (p_q, squared_radius, leftX, rightX, leftY, rightY);

  // iterate over search box
  #pragma omp parallel for schedule (dynamic, threads_)
  for (int x = leftX; x <= rightX; x++)
  {
    #pragma omp flush (abort)
    if (!abort)
    {
      for (int y = leftY; y <= rightY; y++)
      {
        int idx = y * input_->width + x;
        const PointT& point = input_->points[idx];
 
        const double point_dist_x = point.x - p_q.x;
        const double point_dist_y = point.y - p_q.y;
        const double point_dist_z = point.z - p_q.z;
 
        // calculate squared distance
        double squared_distance = (point_dist_x * point_dist_x + point_dist_y * point_dist_y + point_dist_z * point_dist_z);
 
        // check distance and add to results
        if (squared_distance <= squared_radius)
        {
          #pragma omp critical
          {
            k_indices.push_back (idx);
            k_sqr_distances.push_back (squared_distance);
          }
          // return if we are enough indices
          if(k_indices.size() >= max_nn)
            abort = true;                 /// Thanks to Michael Suess for this OpenMP 'hack'
            #pragma omp flush (abort)
        }
      }
    }
  }
  return (k_indices.size());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighborOMP<PointT>::exactNearestKSearch ( int                 index, 
                                                              int                 k, 
                                                              std::vector<int>    &k_indices,
                                                              std::vector<float>  &k_sqr_distances)
{
  const PointT searchPoint = getPointByIndex (index);
  return (exactNearestKSearch (searchPoint, k, k_indices, k_sqr_distances));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighborOMP<PointT>::nearestKSearch (  const PointCloud        &cloud, 
                                                          int                     index,
                                                          int                     k, 
                                                          std::vector<int>        &k_indices,
                                                          std::vector<float>      &k_sqr_distances)
{
  if(!exactFocalLength_)
  {
    estimateFocalLengthFromInputCloud (cloud);
    generateRadiusLookupTable (cloud.width, cloud.height);
    exactFocalLength_ = 1;
  }
  return (exactNearestKSearch (index, k, k_indices, k_sqr_distances));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighborOMP<PointT>::exactNearestKSearch ( const PointT &p_q, 
                                                              int k,
                                                              std::vector<int> &k_indices,
                                                              std::vector<float> &k_sqr_distances)
{
  PCL_ERROR ("[pcl::search::OrganizedNeighborOMP::exactNearestKSearch] Method not implemented!\n");
  return -1;
/** This needs to be rewritten :(
  int x_pos, y_pos, x, y, idx;
  std::size_t i;
  int leftX, rightX, leftY, rightY;
  int radiusSearchPointCount;

  double squaredMaxSearchRadius;

  assert (k > 0);

  if (input_->height == 1)
  {
    PCL_ERROR ("[pcl::%s::exactNearestKSearch] Input dataset is not organized!", getName ().c_str ());
    return (0);
  }

  squaredMaxSearchRadius = max_distance_ * max_distance_;

  // vector for nearest neighbor candidates
  std::vector<nearestNeighborCandidate> nearestNeighbors;

  // iterator for radius seach lookup table
  typename std::vector<radiusSearchLoopkupEntry>::const_iterator radiusSearchLookup_Iterator;
  radiusSearchLookup_Iterator = radiusSearchLookup_.begin ();

  nearestNeighbors.reserve (k * 2);

  // project search point to plane
  pointPlaneProjection (p_q, x_pos, y_pos);
  x_pos += (int)input_->width / 2;
  y_pos += (int)input_->height / 2;

  // initialize result vectors
  k_indices.clear ();
  k_sqr_distances.clear ();

  radiusSearchPointCount = 0;
  // search for k nearest neighbor candidates using the radius lookup table
  while ((radiusSearchLookup_Iterator != radiusSearchLookup_.end ()) && ((int)nearestNeighbors.size () < k))
  {
    // select point from organized pointcloud
    x = x_pos + (*radiusSearchLookup_Iterator).x_diff_;
    y = y_pos + (*radiusSearchLookup_Iterator).y_diff_;

    radiusSearchLookup_Iterator++;
    radiusSearchPointCount++;

    if ((x >= 0) && (y >= 0) && (x < (int)input_->width) && (y < (int)input_->height))
    {
      idx = y * (int)input_->width + x;
      const PointT& point = input_->points[idx];

<<<<<<< .mine
      if ((point.x == point.x) && // check for NaNs
          (point.y == point.y) && 
          (point.z == point.z))
      {
        double squared_distance;
=======
  if (!pcl_isfinite (input_->points[index].x) ||
      !pcl_isfinite (input_->points[index].y) ||
      !pcl_isfinite (input_->points[index].z))
    return (0);
>>>>>>> .r3214

<<<<<<< .mine
        const double point_dist_x = point.x - p_q.x;
        const double point_dist_y = point.y - p_q.y;
        const double point_dist_z = point.z - p_q.z;
=======
  // Put point itself into results. The vector capacity should be unchanged.
  k_indices.push_back (index);
  k_distances.push_back (0.0f);
>>>>>>> .r3214

        // calculate squared distance
        squared_distance = (point_dist_x * point_dist_x + point_dist_y * point_dist_y + point_dist_z * point_dist_z);

        if (squared_distance > squaredMaxSearchRadius)
          continue;

        // we have a new candidate -> add it
        nearestNeighborCandidate newCandidate;
        newCandidate.index_ = idx;
        newCandidate.squared_distance_ = squared_distance;

        nearestNeighbors.push_back (newCandidate);
      }
    }
  }

  // sort candidate list
  std::sort (nearestNeighbors.begin (), nearestNeighbors.end ());

  // we found k candidates -> do radius search
  if ((int)nearestNeighbors.size () == k)
  {
    double squared_radius;

    squared_radius = std::min<double> (nearestNeighbors.back ().squared_distance_, squaredMaxSearchRadius);

    std::vector<int> k_radius_indices;
    std::vector<float> k_radius_distances;

    nearestNeighbors.clear ();

    this->getProjectedRadiusSearchBox (p_q, squared_radius, leftX, rightX, leftY, rightY);

    // iterate over all children
    for (x = leftX; x <= rightX; x++)
    {
<<<<<<< .mine
      for (y = leftY; y <= rightY; y++)
=======
      if (i <= 2 * radius)
        ++x2;
      else if (i <= 4 * radius)
        ++y2;
      else if (i <= 6 * radius)
        --x2;
      else
        --y2;
      if (x2 < 0 || x2 >= width || y2 < 0 || y2 >= height)
        continue;
      int neighbor_index = y2 * width + x2;

      if (!pcl_isfinite (input_->points[neighbor_index].x) ||
          !pcl_isfinite (input_->points[neighbor_index].y) ||
          !pcl_isfinite (input_->points[neighbor_index].z))
        continue;

      float distance_squared = squaredEuclideanDistance (input_->points[index], input_->points[neighbor_index]);
      if (distance_squared > max_dist_squared)
        continue;

      still_in_range = true;
      // The capacity should change here if the new neighborhood has more points than the previous one
      k_indices.push_back (neighbor_index);
      k_distances.push_back (distance_squared);
      if ((int)k_indices.size () >= max_nn)
>>>>>>> .r3214
      {
        double squared_distance;
        idx = y * input_->width + x;
        const PointT& point = input_->points[idx];

        const double point_dist_x = point.x - p_q.x;
        const double point_dist_y = point.y - p_q.y;
        const double point_dist_z = point.z - p_q.z;

        // calculate squared distance
        squared_distance = (point_dist_x * point_dist_x + point_dist_y * point_dist_y + point_dist_z * point_dist_z);

        // check distance and add to results
        if (squared_distance <= squared_radius)
        {
          nearestNeighborCandidate newCandidate;
          newCandidate.index_ = idx;
          newCandidate.squared_distance_ = squared_distance;

          nearestNeighbors.push_back (newCandidate);
        }
      }
    }
    std::sort (nearestNeighbors.begin (), nearestNeighbors.end ());

    // truncate sorted nearest neighbor vector if we found more than k candidates
    if (nearestNeighbors.size () > (size_t)k)
      nearestNeighbors.resize (k);
  }

  // copy results from nearestNeighbors vector to separate indices and distance vector
  k_indices.resize (nearestNeighbors.size ());
  k_sqr_distances.resize (nearestNeighbors.size ());

  for (i = 0; i < nearestNeighbors.size (); i++)
  {
    k_indices[i] = nearestNeighbors[i].index_;
    k_sqr_distances[i] = nearestNeighbors[i].squared_distance_;
  }

  return (k_indices.size ());
*/
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pcl::search::OrganizedNeighborOMP<PointT>::nearestKSearch (  int index, 
                                                          int k,
                                                          std::vector<int> &k_indices,
                                                          std::vector<float> &k_distances)
{
  if (!input_)
  {
    PCL_ERROR ("[pcl::%s::approxNearestKSearch] Input dataset was not set! use setInputCloud before continuing.\n", getName ().c_str ());
    return -1;
  }

  k_indices.resize (k);
  if (!input_->isOrganized ())
  {
    PCL_ERROR ("[pcl::%s::approxNearestKSearch] Input dataset is not organized!\n", getName ().c_str ());
    return -1;
  }
  int data_size = input_->points.size ();
  if (index >= data_size)
    return 0;

  // Get the cloud width
  int width = input_->width;

  // Obtain the <u,v> pixel values
  int u = index / width;
  int v = index % width;

  int l = -1, idx, uwv = u * width + v, uwvx;

  // Save the query point as the first neighbor (*ANN compatibility)
  k_indices[++l] = index;

  if (horizontal_window_ == 0 || vertical_window_)
    setSearchWindowAsK (k);

  // Get all point neighbors in a H x V window

  for (int x = -horizontal_window_; x != horizontal_window_; ++x)
  {
    uwvx = uwv + x * width; // Get the correct index

    for (int y = -vertical_window_; y != vertical_window_; ++y)
    {
      // idx = (u+x) * cloud.width + (v+y);
      idx = uwvx + y;

      // If the index is not in the point cloud, continue
      if (idx == index || idx < 0 || idx >= data_size)
        continue;

      if (max_distance_ != 0)
      {
        if (fabs (input_->points[index].z - input_->points[index].z) < max_distance_)
          k_indices[++l] = idx;
      }
      else
        k_indices[++l] = idx;
    }
  }
  // We need at least min_pts_ nearest neighbors to do something useful with them
  if (l < min_pts_)
    return (0);
  return (k);
}


////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::search::OrganizedNeighborOMP<PointT>::getProjectedRadiusSearchBox ( const PointT& point,
                                                                      double squared_radius, 
                                                                      int &minX,
                                                                      int &maxX, 
                                                                      int &minY, 
                                                                      int & maxY) const
{
  double r_sqr, r_quadr, z_sqr;
  double sqrt_term_y, sqrt_term_x, norm;
  double x_times_z, y_times_z;
  double x1, x2, y1, y2;
  double term_x, term_y;

  // see http://www.wolframalpha.com/input/?i=solve+%5By%2Fsqrt%28f^2%2By^2%29*c-f%2Fsqrt%28f^2%2By^2%29*b%2Br%3D%3D0%2C+f%3D1%2C+y%5D
  // where b = p_q.y, c = p_q.z, r = radius, f = oneOverFocalLength_

  r_sqr = squared_radius;
  r_quadr = r_sqr * r_sqr;
  z_sqr = point.z * point.z;
  norm = 1.0 / (z_sqr - r_sqr);

  // radius sphere projection on X axis
  term_x = point.x * point.x * r_sqr + z_sqr * r_sqr - r_quadr;

  // radius sphere projection on Y axis
  term_y = point.y * point.y * r_sqr + z_sqr * r_sqr - r_quadr;

  if ((term_x > 0) && (term_y > 0))
  {
    sqrt_term_x = sqrt (term_x);

    x_times_z = point.x * point.z;

    x1 = (x_times_z - sqrt_term_x) * norm;
    x2 = (x_times_z + sqrt_term_x) * norm;

    // determine 2-D search window
    minX = (int)floor ((double)input_->width / 2 + (x1 / oneOverFocalLength_));
    maxX = (int)ceil ((double)input_->width / 2 + (x2 / oneOverFocalLength_));

    // make sure the coordinates fit to point cloud resolution
    minX = std::max<int> (0, minX);
    maxX = std::min<int> ((int)input_->width - 1, maxX);

    sqrt_term_y = sqrt (term_y);

    y_times_z = point.y * point.z;

    y1 = (y_times_z - sqrt_term_y) * norm;
    y2 = (y_times_z + sqrt_term_y) * norm;

    // determine 2-D search window
    minY = (int)floor ((double)input_->height / 2 + (y1 / oneOverFocalLength_));
    maxY = (int)ceil ((double)input_->height / 2 + (y2 / oneOverFocalLength_));

    // make sure the coordinates fit to point cloud resolution
    minY = std::max<int> (0, minY);
    maxY = std::min<int> ((int)input_->height - 1, maxY);
  }
  else
  {
    // search point lies within search sphere
    minX = 0;
    maxX = (int)input_->width - 1;

    minY = 0;
    maxY = (int)input_->height - 1;
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> double
pcl::search::OrganizedNeighborOMP<PointT>::estimateFocalLengthFromInputCloud (const pcl::PointCloud<PointT> &cloud)
{
  if (input_->height == 1 || input_->width == 1)
  {
    PCL_ERROR ("[pcl::%s::estimateFocalLenghtFromInputCloud] Input dataset is not organized!\n", getName ().c_str ());
    return 0.0;
  }
  size_t i, count;
  int x, y;

  oneOverFocalLength_ = 0;

  count = 0;
  for (y = 0; y < (int)input_->height; y++)
    for (x = 0; x < (int)input_->width; x++)
    {
      i = y * input_->width + x;
      if ((cloud.points[i].x == cloud.points[i].x) && // check for NaNs
          (cloud.points[i].y == cloud.points[i].y) && 
          (cloud.points[i].z == cloud.points[i].z))
      {
        const PointT& point = cloud.points[i];
        if ((double)(x - cloud.width / 2) * (double)(y - cloud.height / 2) * point.z != 0)
        {
          // estimate the focal length for point.x and point.y
          oneOverFocalLength_ += point.x / ((double)(x - (int)cloud.width / 2) * point.z);
          oneOverFocalLength_ += point.y / ((double)(y - (int)cloud.height / 2) * point.z);
          count += 2;
        }
      }
    }
  // calculate an average of the focalLength
  oneOverFocalLength_ /= (double)count;
  if(pcl_isfinite(oneOverFocalLength_))
    return oneOverFocalLength_;
  else
  {
    PCL_ERROR ("[pcl::%s::estimateFocalLenghtFromInputCloud] Input dataset is not projectable!\n", getName ().c_str ());
    return 0.0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::search::OrganizedNeighborOMP<PointT>::generateRadiusLookupTable ( unsigned int width, 
                                                                    unsigned int height)
{
  int x, y, c;

  //check if point cloud dimensions changed
  if ((radiusLookupTableWidth_ != (int)width) || (radiusLookupTableHeight_ != (int)height))
  {
    radiusLookupTableWidth_ = (int)width;
    radiusLookupTableHeight_ = (int)height;

    radiusSearchLookup_.clear ();
    radiusSearchLookup_.resize ((2 * width + 1) * (2 * height + 1));

    c = 0;
    for (x = -(int)width; x < (int)width + 1; x++)
      for (y = -(int)height; y < (int)height + 1; y++)
        radiusSearchLookup_[c++].defineShiftedSearchPoint (x, y);

    std::sort (radiusSearchLookup_.begin (), radiusSearchLookup_.end ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> const PointT&
pcl::search::OrganizedNeighborOMP<PointT>::getPointByIndex (const unsigned int index) const
{
  // retrieve point from input cloud
  return (this->input_->points[index]);
}
//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::search::OrganizedNeighborOMP<PointT>::setSearchWindowAsK (int k)
{
  int hw = 0, vw = 0;
  while ((2 * hw + 1) * (2 * vw + 1) < k)
  {
    ++hw;
    ++vw;
  }
  horizontal_window_ = hw - 1;
  vertical_window_ = vw - 1;
}

#define PCL_INSTANTIATE_OrganizedNeighborOMP(T) template class PCL_EXPORTS pcl::search::OrganizedNeighborOMP<T>;

#endif
