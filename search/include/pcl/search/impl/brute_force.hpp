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
 */

#pragma once

#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/search/brute_force.h>
#include <queue>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::search::BruteForce<PointT>::getDistSqr (
    const PointT& point1, const PointT& point2) const
{
  return (point1.getVector3fMap () - point2.getVector3fMap ()).squaredNorm ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::BruteForce<PointT>::nearestKSearch (
    const PointT& point, int k, Indices& k_indices, std::vector<float>& k_distances) const
{
  assert (isFinite (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
  
  k_indices.clear ();
  k_distances.clear ();
  if (k < 1)
    return 0;

  if (input_->is_dense)
    return denseKSearch (point, k, k_indices, k_distances);
  return sparseKSearch (point, k, k_indices, k_distances);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::BruteForce<PointT>::denseKSearch (
    const PointT &point, int k, Indices &k_indices, std::vector<float> &k_distances) const
{
  // container for first k elements -> O(1) for insertion, since order not required here
  std::vector<Entry> result;
  result.reserve (k);
  std::priority_queue<Entry> queue;
  if (indices_)
  {
    auto iIt = indices_->cbegin ();
    auto iEnd = indices_->cbegin () + std::min (static_cast<unsigned> (k), static_cast<unsigned> (indices_->size ()));
    for (; iIt != iEnd; ++iIt)
      result.push_back (Entry (*iIt, getDistSqr ((*input_)[*iIt], point)));

    queue = std::priority_queue<Entry> (result.begin (), result.end ());

    // add the rest
    Entry entry;
    for (; iIt != indices_->cend (); ++iIt)
    {
      entry.distance = getDistSqr ((*input_)[*iIt], point);
      if (queue.top ().distance > entry.distance)
      {
        entry.index = *iIt;
        queue.pop ();
        queue.push (entry);
      }
    }
  }
  else
  {
    Entry entry;
    for (entry.index = 0; entry.index < std::min<pcl::index_t> (k, input_->size ()); ++entry.index)
    {
      entry.distance = getDistSqr ((*input_)[entry.index], point);
      result.push_back (entry);
    }

    queue = std::priority_queue<Entry> (result.begin (), result.end ());

    // add the rest
    for (; entry.index < static_cast<pcl::index_t>(input_->size ()); ++entry.index)
    {
      entry.distance = getDistSqr ((*input_)[entry.index], point);
      if (queue.top ().distance > entry.distance)
      {
        queue.pop ();
        queue.push (entry);
      }      
    }
  }

  k_indices.resize (queue.size ());
  k_distances.resize (queue.size ());
  std::size_t idx = queue.size () - 1;
  while (!queue.empty ())
  {
    k_indices [idx] = queue.top ().index;
    k_distances [idx] = queue.top ().distance;
    queue.pop ();
    --idx;
  }
  
  return (static_cast<int> (k_indices.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::BruteForce<PointT>::sparseKSearch (
    const PointT &point, int k, Indices &k_indices, std::vector<float> &k_distances) const
{
  // result used to collect the first k neighbors -> unordered
  std::vector<Entry> result;
  result.reserve (k);
  
  std::priority_queue<Entry> queue;
  if (indices_)
  {
    auto iIt =indices_->cbegin ();
    for (; iIt != indices_->cend () && result.size () < static_cast<unsigned> (k); ++iIt)
    {
      if (std::isfinite ((*input_)[*iIt].x))
        result.push_back (Entry (*iIt, getDistSqr ((*input_)[*iIt], point)));
    }
    
    queue = std::priority_queue<Entry> (result.begin (), result.end ());

    // either we have k elements, or there are none left to iterate >in either case we're fine
    // add the rest
    Entry entry;
    for (; iIt != indices_->cend (); ++iIt)
    {
      if (!std::isfinite ((*input_)[*iIt].x))
        continue;

      entry.distance = getDistSqr ((*input_)[*iIt], point);
      if (queue.top ().distance > entry.distance)
      {
        entry.index = *iIt;
        queue.pop ();
        queue.push (entry);
      }
    }
  }
  else
  {
    Entry entry;
    for (entry.index = 0; (entry.index < static_cast<pcl::index_t>(input_->size ())) && (result.size () < static_cast<std::size_t> (k)); ++entry.index)
    {
      if (std::isfinite ((*input_)[entry.index].x))
      {
        entry.distance = getDistSqr ((*input_)[entry.index], point);
        result.push_back (entry);
      }
    }
    queue = std::priority_queue<Entry> (result.begin (), result.end ());
    
    // add the rest
    for (; entry.index < static_cast<pcl::index_t>(input_->size ()); ++entry.index)
    {
      if (!std::isfinite ((*input_)[entry.index].x))
        continue;

      entry.distance = getDistSqr ((*input_)[entry.index], point);
      if (queue.top ().distance > entry.distance)
      {
        queue.pop ();
        queue.push (entry);
      }
    }
  }
  
  k_indices.resize (queue.size ());
  k_distances.resize (queue.size ());
  std::size_t idx = queue.size () - 1;
  while (!queue.empty ())
  {
    k_indices [idx] = queue.top ().index;
    k_distances [idx] = queue.top ().distance;
    queue.pop ();
    --idx;
  }
  return (static_cast<int> (k_indices.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::BruteForce<PointT>::denseRadiusSearch (
    const PointT& point, double radius,
    Indices &k_indices, std::vector<float> &k_sqr_distances,
    unsigned int max_nn) const
{  
  radius *= radius;

  std::size_t reserve = max_nn;
  if (reserve == 0)
  {
    if (indices_)
      reserve = std::min (indices_->size (), input_->size ());
    else
      reserve = input_->size ();
  }
  k_indices.reserve (reserve);
  k_sqr_distances.reserve (reserve);
  float distance;
  if (indices_)
  {
    for (const auto& idx : *indices_)
    {
      distance = getDistSqr ((*input_)[idx], point);
      if (distance <= radius)
      {
        k_indices.push_back (idx);
        k_sqr_distances.push_back (distance);
        if (k_indices.size () == max_nn) // max_nn = 0 -> never true
          break;
      }
    }
  }
  else
  {
    for (std::size_t index = 0; index < input_->size (); ++index)
    {
      distance = getDistSqr ((*input_)[index], point);
      if (distance <= radius)
      {
        k_indices.push_back (index);
        k_sqr_distances.push_back (distance);
        if (k_indices.size () == max_nn) // never true if max_nn = 0
          break;
      }
    }
  }

  if (sorted_results_)
    this->sortResults (k_indices, k_sqr_distances);
  
  return (static_cast<int> (k_indices.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::BruteForce<PointT>::sparseRadiusSearch (
    const PointT& point, double radius,
    Indices &k_indices, std::vector<float> &k_sqr_distances,
    unsigned int max_nn) const
{
  radius *= radius;

  std::size_t reserve = max_nn;
  if (reserve == 0)
  {
    if (indices_)
      reserve = std::min (indices_->size (), input_->size ());
    else
      reserve = input_->size ();
  }
  k_indices.reserve (reserve);
  k_sqr_distances.reserve (reserve);

  float distance;
  if (indices_)
  {
    for (const auto& idx : *indices_)
    {
      if (!std::isfinite ((*input_)[idx].x))
        continue;

      distance = getDistSqr ((*input_)[idx], point);
      if (distance <= radius)
      {
        k_indices.push_back (idx);
        k_sqr_distances.push_back (distance);
        if (k_indices.size () == max_nn) // never true if max_nn = 0
          break;
      }
    }
  }
  else
  {
    for (std::size_t index = 0; index < input_->size (); ++index)
    {
      if (!std::isfinite ((*input_)[index].x))
        continue;
      distance = getDistSqr ((*input_)[index], point);
      if (distance <= radius)
      {
        k_indices.push_back (index);
        k_sqr_distances.push_back (distance);
        if (k_indices.size () == max_nn) // never true if max_nn = 0
          break;
      }
    }
  }

  if (sorted_results_)
    this->sortResults (k_indices, k_sqr_distances);

  return (static_cast<int> (k_indices.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::BruteForce<PointT>::radiusSearch (
    const PointT& point, double radius, Indices &k_indices,
    std::vector<float> &k_sqr_distances, unsigned int max_nn) const
{
  assert (isFinite (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
  
  k_indices.clear ();
  k_sqr_distances.clear ();
  if (radius <= 0)
    return 0;

  if (input_->is_dense)
    return denseRadiusSearch (point, radius, k_indices, k_sqr_distances, max_nn);
  return sparseRadiusSearch (point, radius, k_indices, k_sqr_distances, max_nn);
}

#define PCL_INSTANTIATE_BruteForce(T) template class PCL_EXPORTS pcl::search::BruteForce<T>;
