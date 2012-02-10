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
 */

#ifndef PCL_SEARCH_IMPL_BRUTE_FORCE_SEARCH_H_
#define PCL_SEARCH_IMPL_BRUTE_FORCE_SEARCH_H_

#include "pcl/search/brute_force.h"
#include <algorithm>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::search::BruteForce<PointT>::getDistSqr (
    const PointT& point1, const PointT& point2) const
{
  return ((point1.x - point2.x) * (point1.x - point2.x) +
          (point1.y - point2.y) * (point1.y - point2.y) +
          (point1.z - point2.z) * (point1.z - point2.z) ) ;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::BruteForce<PointT>::nearestKSearch (
    const PointT& point, int k, std::vector<int>& k_indices, std::vector<float>& k_distances) const
{
  if (input_->is_dense)
    return denseKSearch (point, k, k_indices, k_distances);
  else
    return sparseKSearch (point, k, k_indices, k_distances);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::BruteForce<PointT>::denseKSearch (
    const PointT &point, int k, std::vector<int> &k_indices, std::vector<float> &k_distances) const
{
  k_indices.clear ();
  k_indices.reserve (k);
  k_distances.clear ();
  k_distances.reserve (k);

  std::vector<Entry> result;
  result.reserve (k + 1);
  const PointCloud& cloud = *input_;
  if (indices_ != NULL)
  {
    const std::vector<int>& indices = *indices_;
    Entry entry;
    // fill up queue with k elements
    for (entry.index = 0; entry.index < std::min ((unsigned) k, (unsigned) indices.size ()); ++entry.index)
    {
      result.push_back (Entry (indices[entry.index], getDistSqr (cloud[indices[entry.index]], point)));
    }
    // sort them
    std::sort (result.begin (), result.end ());

    // add the rest
    for (; entry.index < indices.size (); ++entry.index)
    {
      entry.distance = getDistSqr (cloud[indices[entry.index]], point);
      if (entry.distance >= result.back ().distance)
        continue;
      typename std::vector<Entry>::iterator it = std::upper_bound (result.begin (), result.end (), entry);
      if (it != result.end ())
      {
        result.insert ( it, entry );
        result.pop_back (); // remove the largest element
      }
    }
  }
  else
  {
    Entry entry;
    for (entry.index = 0; entry.index < std::min ((unsigned) k, (unsigned) cloud.size ()); ++entry.index)
    {
      entry.distance = getDistSqr (cloud[entry.index], point);
      result.push_back (entry);
    }
    // sort them
    std::sort (result.begin (), result.end ());

    // add the rest
    for (; entry.index < cloud.size (); ++entry.index)
    {
      entry.distance = getDistSqr (cloud[entry.index], point);
      if (entry.distance >= result.back ().distance)
        continue;

      typename std::vector<Entry>::iterator it = std::upper_bound (result.begin (), result.end (), entry);
      if (it != result.end ())
      {
        result.insert ( it, entry );
        result.pop_back (); // remove the largest element
      }
    }
  }

  for (typename std::vector<Entry>::const_iterator rIt = result.begin (); rIt != result.end (); ++rIt)
  {
    k_indices.push_back (rIt->index);
    k_distances.push_back (rIt->distance);
  }
  return result.size ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::BruteForce<PointT>::sparseKSearch (
    const PointT &point, int k, std::vector<int> &k_indices, std::vector<float> &k_distances) const
{
  k_indices.clear ();
  k_indices.reserve (k);
  k_distances.clear ();
  k_distances.reserve (k);

  std::vector<Entry> result;
  result.reserve (k + 1);
  const PointCloud& cloud = *input_;
  if (indices_ != NULL)
  {
    const std::vector<int>& indices = *indices_;
    Entry entry;
    // fill up queue with k elements
    for (entry.index = 0; entry.index < indices.size (); ++entry.index)
    {
      if (isFinite (cloud[indices[entry.index]]))
      {
        result.push_back (Entry (indices[entry.index], getDistSqr (cloud[indices[entry.index]], point)));
        if (result.size () == (unsigned) k)
          break;
      }
    }
    // sort them
    std::sort (result.begin (), result.end ());

    // either we have k elements, or there are none left to iterate >in either case we're fine
    // add the rest
    for (; entry.index < indices.size (); ++entry.index)
    {
      if (!isFinite (cloud[indices[entry.index]]))
        continue;

      entry.distance = getDistSqr (cloud[indices[entry.index]], point);
      typename std::vector<Entry>::iterator it = std::upper_bound (result.begin (), result.end (), entry);
      if (it != result.end ())
      {
        result.insert ( it, entry );
        result.pop_back (); // remove the largest element
      }
    }
  }
  else
  {
    Entry entry;
    for (entry.index = 0; entry.index < cloud.size (); ++entry.index)
    {
      if (isFinite (cloud[entry.index]))
      {
        entry.distance = getDistSqr (cloud[entry.index], point);
        result.push_back (entry);
        if (result.size () == (unsigned) k)
          break;
      }
    }
    // sort them
    std::sort (result.begin (), result.end ());

    // add the rest
    for (; entry.index < cloud.size (); ++entry.index)
    {
      if (!isFinite (cloud[entry.index]))
        continue;

      entry.distance = getDistSqr (cloud[entry.index], point);
      if (entry.distance >= result.back ().distance)
        continue;

      typename std::vector<Entry>::iterator it = std::upper_bound (result.begin (), result.end (), entry);
      if (it != result.end ())
      {
        result.insert ( it, entry );
        result.pop_back (); // remove the largest element
      }
    }
  }

  for (typename std::vector<Entry>::const_iterator rIt = result.begin (); rIt != result.end (); ++rIt)
  {
    k_indices.push_back (rIt->index);
    k_distances.push_back (rIt->distance);
  }
  return result.size ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::BruteForce<PointT>::denseRadiusSearch (
    const PointT& point, double radius,
    std::vector<int> &k_indices, std::vector<float> &k_sqr_distances,
    unsigned int max_nn) const
{
  k_indices.clear ();
  k_sqr_distances.clear ();

  radius *= radius;

  int reserve = max_nn;
  if (reserve == 0)
  {
    if (indices_ != NULL)
      reserve = std::min (indices_->size (), input_->size ());
    else
      reserve = input_->size ();
  }
  k_indices.reserve (reserve);
  k_sqr_distances.reserve (reserve);

  std::vector<Entry> result;
  result.reserve (reserve);
  const PointCloud& cloud = *input_;
  if (indices_ != NULL)
  {
    const std::vector<int>& indices = *indices_;
    Entry entry;

    // add the rest
    for (entry.index = 0; entry.index < indices.size (); ++entry.index)
    {
      entry.distance = getDistSqr (cloud[indices[entry.index]], point);
      if (entry.distance <= radius)
      {
        result.push_back (entry);
        if (result.size () == max_nn) // never true if max_nn = -1
          break;
      }
    }
    std::sort (result.begin (), result.end ());
  }
  else
  {
    Entry entry;

    for (entry.index = 0; entry.index < cloud.size (); ++entry.index)
    {
      entry.distance = getDistSqr (cloud[entry.index], point);
      if (entry.distance < radius)
      {
        result.push_back (entry);
        if (result.size () == max_nn) // never true if max_nn = -1
          break;
      }
    }
    std::sort (result.begin (), result.end ());
  }

  for (typename std::vector<Entry>::const_iterator rIt = result.begin (); rIt != result.end (); ++rIt)
  {
    k_indices.push_back (rIt->index);
    k_sqr_distances.push_back (rIt->distance);
  }
  return result.size ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::BruteForce<PointT>::sparseRadiusSearch (
    const PointT& point, double radius,
    std::vector<int> &k_indices, std::vector<float> &k_sqr_distances,
    unsigned int max_nn) const
{
  k_indices.clear ();
  k_sqr_distances.clear ();

  radius *= radius;

  int reserve = max_nn;
  if (reserve == 0)
  {
    if (indices_ != NULL)
      reserve = std::min (indices_->size (), input_->size ());
    else
      reserve = input_->size ();
  }
  k_indices.reserve (reserve);
  k_sqr_distances.reserve (reserve);

  std::vector<Entry> result;
  result.reserve (reserve);
  const PointCloud& cloud = *input_;
  if (indices_ != NULL)
  {
    const std::vector<int>& indices = *indices_;
    Entry entry;

    for (entry.index = 0; entry.index < indices.size (); ++entry.index)
    {
      if (!isFinite (cloud[indices[entry.index]]))
        continue;

      entry.distance = getDistSqr (cloud[indices[entry.index]], point);
      if (entry.distance <= radius)
      {
        result.push_back (entry);
        if (result.size () == max_nn) // never true if max_nn = -1
          break;
      }
    }
    std::sort (result.begin (), result.end ());
  }
  else
  {
    Entry entry;

    for (entry.index = 0; entry.index < cloud.size (); ++entry.index)
    {
      if (!isFinite (cloud[entry.index]))
        continue;
      entry.distance = getDistSqr (cloud[entry.index], point);
      if (entry.distance < radius)
      {
        result.push_back (entry);
        if (result.size () == max_nn) // never true if max_nn = -1
          break;
      }
    }
    std::sort (result.begin (), result.end ());
  }

  for (typename std::vector<Entry>::const_iterator rIt = result.begin (); rIt != result.end (); ++rIt)
  {
    k_indices.push_back (rIt->index);
    k_sqr_distances.push_back (rIt->distance);
  }
  return result.size ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::BruteForce<PointT>::radiusSearch (
    const PointT& point, double radius, std::vector<int> &k_indices,
    std::vector<float> &k_sqr_distances, unsigned int max_nn) const
{
  if (input_->is_dense)
    return denseRadiusSearch (point, radius, k_indices, k_sqr_distances, max_nn);
  else
    return sparseRadiusSearch (point, radius, k_indices, k_sqr_distances, max_nn);
}

#define PCL_INSTANTIATE_BruteForce(T) template class PCL_EXPORTS pcl::search::BruteForce<T>;

#endif //PCL_SEARCH_IMPL_BRUTE_FORCE_SEARCH_H_
