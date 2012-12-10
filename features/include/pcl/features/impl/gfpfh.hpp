/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: gfpfh.hpp 2218 2011-08-25 20:27:15Z rusu $
 *
 */

#ifndef PCL_FEATURES_IMPL_GFPFH_H_
#define PCL_FEATURES_IMPL_GFPFH_H_

#include <pcl/features/gfpfh.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/eigen.h>

#include <algorithm>
#include <fstream>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::GFPFHEstimation<PointInT, PointNT, PointOutT>::compute (PointCloudOut &output)
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  // Copy the header
  output.header = input_->header;

  // Resize the output dataset
  // Important! We should only allocate precisely how many elements we will need, otherwise
  // we risk at pre-allocating too much memory which could lead to bad_alloc 
  // (see http://dev.pointclouds.org/issues/657)
  output.width = output.height = 1;
  output.is_dense = input_->is_dense;
  output.points.resize (1);

  // Perform the actual feature computation
  computeFeature (output);

  Feature<PointInT, PointOutT>::deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GFPFHEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  pcl::octree::OctreePointCloudSearch<PointInT> octree (octree_leaf_size_);
  octree.setInputCloud (input_);
  octree.addPointsFromInputCloud ();

  typename pcl::PointCloud<PointInT>::VectorType occupied_cells;
  octree.getOccupiedVoxelCenters (occupied_cells);

  // Determine the voxels crosses along the line segments
  // formed by every pair of occupied cells.
  std::vector< std::vector<int> > line_histograms;
  for (size_t i = 0; i < occupied_cells.size (); ++i)
  {
    Eigen::Vector3f origin = occupied_cells[i].getVector3fMap ();

    for (size_t j = i+1; j < occupied_cells.size (); ++j)
    {
      typename pcl::PointCloud<PointInT>::VectorType intersected_cells;
      Eigen::Vector3f end = occupied_cells[j].getVector3fMap ();
      octree.getApproxIntersectedVoxelCentersBySegment (origin, end, intersected_cells, 0.5f);

      // Intersected cells are ordered from closest to furthest w.r.t. the origin.
      std::vector<int> histogram;
      for (size_t k = 0; k < intersected_cells.size (); ++k)
      {
        std::vector<int> indices;
        octree.voxelSearch (intersected_cells[k], indices);
        int label = emptyLabel ();
        if (indices.size () != 0)
        {
          label = getDominantLabel (indices);
        }
        histogram.push_back (label);
      }

      line_histograms.push_back(histogram);
    }
  }

  std::vector< std::vector<int> > transition_histograms;
  computeTransitionHistograms (line_histograms, transition_histograms);

  std::vector<float> distances;
  computeDistancesToMean (transition_histograms, distances);

  std::vector<float> gfpfh_histogram;
  computeDistanceHistogram (distances, gfpfh_histogram);

  output.clear ();
  output.width = 1;
  output.height = 1;
  output.points.resize (1);
  std::copy (gfpfh_histogram.begin (), gfpfh_histogram.end (), output.points[0].histogram);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GFPFHEstimation<PointInT, PointNT, PointOutT>::computeTransitionHistograms (const std::vector< std::vector<int> >& label_histograms,
                                                                                 std::vector< std::vector<int> >& transition_histograms)
{
  transition_histograms.resize (label_histograms.size ());

  for (size_t i = 0; i < label_histograms.size (); ++i)
  {
    transition_histograms[i].resize ((getNumberOfClasses () + 2) * (getNumberOfClasses () + 1) / 2, 0);

    std::vector< std::vector <int> > transitions (getNumberOfClasses () + 1);
    for (size_t k = 0; k < transitions.size (); ++k)
    {
      transitions[k].resize (getNumberOfClasses () + 1, 0);
    }

    for (size_t k = 1; k < label_histograms[i].size (); ++k)
    {
      uint32_t first_class = label_histograms[i][k-1];
      uint32_t second_class = label_histograms[i][k];
      // Order has no influence.
      if (second_class < first_class)
        std::swap (first_class, second_class);

      transitions[first_class][second_class] += 1;
    }

    // Build a one-dimension histogram out of it.
    int flat_index = 0;
    for (int m = 0; m < static_cast<int> (transitions.size ()); ++m)
      for (int n = m; n < static_cast<int> (transitions[m].size ()); ++n)
      {
        transition_histograms[i][flat_index] = transitions[m][n];
        ++flat_index;
      }

    assert (flat_index == static_cast<int> (transition_histograms[i].size ()));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GFPFHEstimation<PointInT, PointNT, PointOutT>::computeDistancesToMean (const std::vector< std::vector<int> >& transition_histograms,
                                                                            std::vector<float>& distances)
{
  distances.resize (transition_histograms.size ());

  std::vector<float> mean_histogram;
  computeMeanHistogram (transition_histograms, mean_histogram);

  for (size_t i = 0; i < transition_histograms.size (); ++i)
  {
    float d = computeHIKDistance (transition_histograms[i], mean_histogram);
    distances[i] = d;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GFPFHEstimation<PointInT, PointNT, PointOutT>::computeDistanceHistogram (const std::vector<float>& distances,
                                                                              std::vector<float>& histogram)
{
  std::vector<float>::const_iterator min_it = std::min_element (distances.begin (), distances.end ());
  assert (min_it != distances.end ());
  const float min_value = *min_it;

  std::vector<float>::const_iterator max_it = std::max_element (distances.begin (), distances.end ());
  assert (max_it != distances.end());
  const float max_value = *max_it;

  histogram.resize (descriptorSize (), 0);

  const float range = max_value - min_value;
  const int max_bin = descriptorSize () - 1;
  for (size_t i = 0; i < distances.size (); ++i)
  {
    const float raw_bin = static_cast<float> (descriptorSize ()) * (distances[i] - min_value) / range;
    int bin = std::min (max_bin, static_cast<int> (floor (raw_bin)));
    histogram[bin] += 1;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GFPFHEstimation<PointInT, PointNT, PointOutT>::computeMeanHistogram (const std::vector< std::vector<int> >& histograms,
                                                                          std::vector<float>& mean_histogram)
{
  assert (histograms.size () > 0);

  mean_histogram.resize (histograms[0].size (), 0);
  for (size_t i = 0; i < histograms.size (); ++i)
    for (size_t j = 0; j < histograms[i].size (); ++j)
      mean_histogram[j] += static_cast<float> (histograms[i][j]);

  for (size_t i = 0; i < mean_histogram.size (); ++i)
    mean_histogram[i] /= static_cast<float> (histograms.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> float
pcl::GFPFHEstimation<PointInT, PointNT, PointOutT>::computeHIKDistance (const std::vector<int>& histogram,
                                                                        const std::vector<float>& mean_histogram)
{
  assert (histogram.size () == mean_histogram.size ());

  float norm = 0.f;
  for (size_t i = 0; i < histogram.size (); ++i)
    norm += std::min (static_cast<float> (histogram[i]), mean_histogram[i]);

  norm /= static_cast<float> (histogram.size ());
  return (norm);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> boost::uint32_t
pcl::GFPFHEstimation<PointInT, PointNT, PointOutT>::getDominantLabel (const std::vector<int>& indices)
{
  std::vector<uint32_t> counts (getNumberOfClasses () + 1, 0);
  for (size_t i = 0; i < indices.size (); ++i)
  {
    uint32_t label = labels_->points[indices[i]].label;
    counts[label] += 1;
  }

  std::vector<uint32_t>::const_iterator max_it;
  max_it = std::max_element (counts.begin (), counts.end ());
  if (max_it == counts.end ())
    return (emptyLabel ());

  return (static_cast<uint32_t> (max_it - counts.begin ()));
}

#define PCL_INSTANTIATE_GFPFHEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::GFPFHEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_GFPFH_H_
