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
 */

#ifndef PCL_SEGMENTATION_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_
#define PCL_SEGMENTATION_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_

#include <pcl/common/common.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::ProgressiveMorphologicalFilter<PointT>::ProgressiveMorphologicalFilter () :
  max_window_size_ (33),
  slope_ (0.7f),
  max_distance_ (10.0f),
  initial_distance_ (0.15f),
  ground_ (0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::ProgressiveMorphologicalFilter<PointT>::~ProgressiveMorphologicalFilter ()
{
  ground_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::ProgressiveMorphologicalFilter<PointT>::getMaxWindowSize ()
{
  return (max_window_size_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveMorphologicalFilter<PointT>::setMaxWindowSize (int max_window_size)
{
  max_window_size_ = max_window_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::ProgressiveMorphologicalFilter<PointT>::getSlope () const
{
  return (slope_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveMorphologicalFilter<PointT>::setSlope (float slope)
{
  slope_ = slope;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::ProgressiveMorphologicalFilter<PointT>::getMaxDistance () const
{
  return (max_distance_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveMorphologicalFilter<PointT>::setMaxDistance (float max_distance)
{
  max_distance_ = max_distance;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::ProgressiveMorphologicalFilter<PointT>::getInitialDistance () const
{
  return (initial_distance_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveMorphologicalFilter<PointT>::setInitialDistance (float initial_distance)
{
  initial_distance_ = initial_distance;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<int>
pcl::ProgressiveMorphologicalFilter<PointT>::firstIteration (const typename pcl::PointCloud<PointT>::ConstPtr &source, float c, float b, int k, float s, float dh_0, float dh_max, float dh, float w, float w_max, bool exponential)
{
  PCL_DEBUG ("      Iteration %d", k);
  PCL_DEBUG (" (dh = %f, w = %f)", dh, w);
  PCL_DEBUG ("...");

  typename pcl::PointCloud<PointT>::Ptr ground (new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr object (new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);

  pcl::copyPointCloud<PointT,PointT> (*source, *cloud);
  pcl::copyPointCloud<PointT,PointT> (*source, *cloud_f);

  float resolution = w; // also divided in morphologicalOpen, need to clear this up

  pcl::morphologicalOpen<PointT> (*cloud, resolution, *cloud_f);

  for (boost::int32_t p_idx = 0; p_idx < source->points.size (); ++p_idx)
    cloud->points[p_idx].z = source->points[p_idx].z - cloud_f->points[p_idx].z;

  std::vector<int> pt_indices;

  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-std::numeric_limits<float>::max (), dh);
  pass.filter (pt_indices);

  pcl::PointIndicesPtr foo (new pcl::PointIndices);
  foo->indices = pt_indices;

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (source);
  extract.setIndices (foo);
  extract.filter (*ground);

  PCL_DEBUG ("ground now has %d points\n", ground->points.size ());

  float w_prev = w;

  if (exponential)
    w = c * (2.0f * std::pow(b, k++) + 1.0f);
  else
    w = c * (2.0f * k++ * b + 1.0f);

  dh = s * (w - w_prev) * c + dh_0;

  if (dh > dh_max)
    dh = dh_max;

  std::vector<int>
  ground_pts = pmfIteration (*ground, c, b, k, s, dh_0, dh_max, dh, w, w_max, pt_indices, exponential);

  return (ground_pts);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::vector<int>
pcl::ProgressiveMorphologicalFilter<PointT>::pmfIteration (const pcl::PointCloud<PointT> source, float c, float b, int k, float s, float dh_0, float dh_max, float dh, float w, float w_max, std::vector<int> indices, bool exponential)
{
  if (w > w_max) 
  {
    PCL_DEBUG ("      Reached max window size %f\n", w);
    return (indices);
  }

  PCL_DEBUG ("      Iteration %d", k);
  PCL_DEBUG (" (dh = %f, w = %f)", dh, w);
  PCL_DEBUG ("...");

  if (indices.empty ())
    PCL_WARN ("indices are empty!\n");

  typename pcl::PointCloud<PointT>::Ptr ground (new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);

  pcl::copyPointCloud<PointT,PointT> (source, *cloud);
  pcl::copyPointCloud<PointT,PointT> (source, *cloud_f);

  float resolution = w; // divide by two occurs in morphological filter

  pcl::morphologicalOpen<PointT> (*cloud, resolution, *cloud_f);

  for (boost::int32_t p_idx = 0; p_idx < source.points.size (); ++p_idx)
    cloud->points[p_idx].z = source.points[p_idx].z - cloud_f->points[p_idx].z;

  std::vector<int> pt_indices;

  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-std::numeric_limits<float>::max (), dh);
  pass.filter (pt_indices);

  pcl::PointIndicesPtr foo(new pcl::PointIndices);
  foo->indices = pt_indices;

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (source.makeShared ());
  extract.setIndices (foo);
  extract.filter (*ground);

  PCL_DEBUG ("ground now has %d points\n", ground->points.size ());

  float w_prev = w;

  if (exponential)
    w = c * (2.0f * std::pow(b, k++) + 1.0f);
  else
    w = c * (2.0f * k++ * b + 1.0f);

  dh = s * (w - w_prev) * c + dh_0;

  if (dh > dh_max)
    dh = dh_max;

  std::vector<int>
  ground_pts = pmfIteration (*ground, c, b, k, s, dh_0, dh_max, dh, w, w_max, pt_indices, exponential);

  std::vector<int> output_vec;

  for (size_t i = 0; i < ground_pts.size (); ++i)
    output_vec.push_back (indices[ground_pts[i]]);

  return (output_vec);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ProgressiveMorphologicalFilter<PointT>::extract (std::vector <int>& ground)
{
  ground_.clear ();
  ground.clear ();

  bool segmentation_is_possible = initCompute ();
  if (!segmentation_is_possible)
  {
    deinitCompute ();
    return;
  }

  typename pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);

  float dh_0 = 0.15f;
  float c = 1.0f;
  float s = 0.7f;
  float dh_max = 2.5f;
  float w_max = 33.0f;
  bool exponential = true;

  float b = 2.0f;          // w_k = 2 * b^k + 1
  float dh = dh_0;         // elevation differnce threshold
  std::vector<float> w_k;  // series of window sizes
  float w = 0.0f;
  int k = 0;

  if (exponential)
    w = c * ( 2.0f * std::pow(b, k++) + 1.0f);
  else
  {
    k = 1;
    w = c * ( 2.0f * k++ * b + 1.0f );
  }

  ground = firstIteration (input_, c, b, k, s, dh_0, dh_max, dh, w, w_max, exponential);

  deinitCompute ();
}

#define PCL_INSTANTIATE_ProgressiveMorphologicalFilter(T) template class pcl::ProgressiveMorphologicalFilter<T>;

#endif    // PCL_SEGMENTATION_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_

