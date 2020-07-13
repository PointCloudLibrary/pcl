/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#ifndef PCL_SEARCH_SEARCH_IMPL_HPP_
#define PCL_SEARCH_SEARCH_IMPL_HPP_

#include <pcl/search/search.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::search::Search<PointT>::Search (const std::string& name, bool sorted)
  : input_ () 
  , sorted_results_ (sorted)
  , name_ (name)
{
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> const std::string& 
pcl::search::Search<PointT>::getName () const
{
  return (name_);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::search::Search<PointT>::setSortedResults (bool sorted)
{
  sorted_results_ = sorted;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::search::Search<PointT>::getSortedResults ()
{
  return (sorted_results_);
}
 
///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::search::Search<PointT>::setInputCloud (
    const PointCloudConstPtr& cloud, const IndicesConstPtr &indices)
{
  input_ = cloud;
  indices_ = indices;
}


///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::Search<PointT>::nearestKSearch (
    const PointCloud &cloud, index_t index, int k,
    Indices &k_indices, std::vector<float> &k_sqr_distances) const
{
  assert (index >= 0 && index < static_cast<index_t> (cloud.size ()) && "Out-of-bounds error in nearestKSearch!");
  return (nearestKSearch (cloud[index], k, k_indices, k_sqr_distances));
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::Search<PointT>::nearestKSearch (
    index_t index, int k,
    Indices &k_indices,
    std::vector<float> &k_sqr_distances) const
{
  if (!indices_)
  {
    assert (index >= 0 && index < static_cast<index_t> (input_->size ()) && "Out-of-bounds error in nearestKSearch!");
    return (nearestKSearch ((*input_)[index], k, k_indices, k_sqr_distances));
  }
  assert (index >= 0 && index < static_cast<index_t> (indices_->size ()) && "Out-of-bounds error in nearestKSearch!");
  if (index >= static_cast<index_t> (indices_->size ()) || index < 0)
    return (0);
  return (nearestKSearch ((*input_)[(*indices_)[index]], k, k_indices, k_sqr_distances));
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::search::Search<PointT>::nearestKSearch (
    const PointCloud& cloud, const Indices& indices,
    int k, std::vector<Indices>& k_indices,
    std::vector< std::vector<float> >& k_sqr_distances) const
{
  if (indices.empty ())
  {
    k_indices.resize (cloud.size ());
    k_sqr_distances.resize (cloud.size ());
    for (std::size_t i = 0; i < cloud.size (); i++)
      nearestKSearch (cloud, static_cast<index_t> (i), k, k_indices[i], k_sqr_distances[i]);
  }
  else
  {
    k_indices.resize (indices.size ());
    k_sqr_distances.resize (indices.size ());
    for (std::size_t i = 0; i < indices.size (); i++)
      nearestKSearch (cloud, indices[i], k, k_indices[i], k_sqr_distances[i]);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::Search<PointT>::radiusSearch (
    const PointCloud &cloud, index_t index, double radius,
    Indices &k_indices, std::vector<float> &k_sqr_distances,
    unsigned int max_nn) const
{
  assert (index >= 0 && index < static_cast<index_t> (cloud.size ()) && "Out-of-bounds error in radiusSearch!");
  return (radiusSearch(cloud[index], radius, k_indices, k_sqr_distances, max_nn));
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::search::Search<PointT>::radiusSearch (
    index_t index, double radius, Indices &k_indices,
    std::vector<float> &k_sqr_distances, unsigned int max_nn ) const
{
  if (!indices_)
  {
    assert (index >= 0 && index < static_cast<index_t> (input_->size ()) && "Out-of-bounds error in radiusSearch!");
    return (radiusSearch ((*input_)[index], radius, k_indices, k_sqr_distances, max_nn));
  }
  assert (index >= 0 && index < static_cast<index_t> (indices_->size ()) && "Out-of-bounds error in radiusSearch!");
  return (radiusSearch ((*input_)[(*indices_)[index]], radius, k_indices, k_sqr_distances, max_nn));
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::search::Search<PointT>::radiusSearch (
    const PointCloud& cloud,
    const Indices& indices,
    double radius,
    std::vector<Indices>& k_indices,
    std::vector< std::vector<float> > &k_sqr_distances,
    unsigned int max_nn) const
{
  if (indices.empty ())
  {
    k_indices.resize (cloud.size ());
    k_sqr_distances.resize (cloud.size ());
    for (std::size_t i = 0; i < cloud.size (); i++)
      radiusSearch (cloud, static_cast<index_t> (i), radius,k_indices[i], k_sqr_distances[i], max_nn);
  }
  else
  {
    k_indices.resize (indices.size ());
    k_sqr_distances.resize (indices.size ());
    for (std::size_t i = 0; i < indices.size (); i++)
      radiusSearch (cloud,indices[i],radius,k_indices[i],k_sqr_distances[i], max_nn);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::search::Search<PointT>::sortResults (
    Indices& indices, std::vector<float>& distances) const
{
  Indices order (indices.size ());
  for (std::size_t idx = 0; idx < order.size (); ++idx)
    order [idx] = static_cast<index_t> (idx);

  Compare compare (distances);
  sort (order.begin (), order.end (), compare);

  Indices sorted (indices.size ());
  for (std::size_t idx = 0; idx < order.size (); ++idx)
    sorted [idx] = indices[order [idx]];

  indices = sorted;

  // sort  the according distances.
  sort (distances.begin (), distances.end ());
}

#define PCL_INSTANTIATE_Search(T) template class PCL_EXPORTS pcl::search::Search<T>;

#endif  //#ifndef _PCL_SEARCH_SEARCH_IMPL_HPP_


