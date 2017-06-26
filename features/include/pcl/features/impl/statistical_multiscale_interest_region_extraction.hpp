/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
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
 *  $Id$
 */

#ifndef PCL_FEATURES_IMPL_STATISTICAL_MULTISCALE_INTEREST_REGION_EXTRACTION_H_
#define PCL_FEATURES_IMPL_STATISTICAL_MULTISCALE_INTEREST_REGION_EXTRACTION_H_

#include <pcl/features/statistical_multiscale_interest_region_extraction.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/distances.h>
#include <pcl/features/boost.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::StatisticalMultiscaleInterestRegionExtraction<PointT>::generateCloudGraph ()
{
  // generate a K-NNG (K-nearest neighbors graph)
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud (input_);

  using namespace boost;
  typedef property<edge_weight_t, float> Weight;
  typedef adjacency_list<vecS, vecS, undirectedS, no_property, Weight> Graph;
  Graph cloud_graph;

  for (size_t point_i = 0; point_i < input_->points.size (); ++point_i)
  {
    std::vector<int> k_indices (16);
    std::vector<float> k_distances (16);
    kdtree.nearestKSearch (static_cast<int> (point_i), 16, k_indices, k_distances);

    for (int k_i = 0; k_i < static_cast<int> (k_indices.size ()); ++k_i)
      add_edge (point_i, k_indices[k_i], Weight (std::sqrt (k_distances[k_i])), cloud_graph);
  }

  const size_t E = num_edges (cloud_graph),
      V = num_vertices (cloud_graph);
  PCL_INFO ("The graph has %lu vertices and %lu edges.\n", V, E);
  geodesic_distances_.clear ();
  for (size_t i = 0; i < V; ++i)
  {
    std::vector<float> aux (V);
    geodesic_distances_.push_back (aux);
  }
  johnson_all_pairs_shortest_paths (cloud_graph, geodesic_distances_);

  PCL_INFO ("Done generating the graph\n");
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::StatisticalMultiscaleInterestRegionExtraction<PointT>::initCompute ()
{
  if (!PCLBase<PointT>::initCompute ())
  {
    PCL_ERROR ("[pcl::StatisticalMultiscaleInterestRegionExtraction::initCompute] PCLBase::initCompute () failed - no input cloud was given.\n");
    return (false);
  }
  if (scale_values_.empty ())
  {
    PCL_ERROR ("[pcl::StatisticalMultiscaleInterestRegionExtraction::initCompute] No scale values were given\n");
    return (false);
  }

  return (true);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::StatisticalMultiscaleInterestRegionExtraction<PointT>::geodesicFixedRadiusSearch (size_t &query_index,
                                                                                       float &radius,
                                                                                       std::vector<int> &result_indices)
{
  for (size_t i = 0; i < geodesic_distances_[query_index].size (); ++i)
    if (i != query_index && geodesic_distances_[query_index][i] < radius)
      result_indices.push_back (static_cast<int> (i));
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::StatisticalMultiscaleInterestRegionExtraction<PointT>::computeRegionsOfInterest (std::list<IndicesPtr> &rois)
{
  if (!initCompute ())
  {
    PCL_ERROR ("StatisticalMultiscaleInterestRegionExtraction: not completely initialized\n");
    return;
  }

  generateCloudGraph ();

  computeF ();

  extractExtrema (rois);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::StatisticalMultiscaleInterestRegionExtraction<PointT>::computeF ()
{
  PCL_INFO ("Calculating statistical information\n");

  // declare and initialize data structure
  F_scales_.resize (scale_values_.size ());
  std::vector<float> point_density (input_->points.size ()),
          F (input_->points.size ());
  std::vector<std::vector<float> > phi (input_->points.size ());
  std::vector<float> phi_row (input_->points.size ());

  for (size_t scale_i = 0; scale_i < scale_values_.size (); ++scale_i)
  {
    float scale_squared = scale_values_[scale_i] * scale_values_[scale_i];

    // calculate point density for each point x_i
    for (size_t point_i = 0; point_i < input_->points.size (); ++point_i)
    {
      float point_density_i = 0.0;
      for (size_t point_j = 0; point_j < input_->points.size (); ++point_j)
      {
        float d_g = geodesic_distances_[point_i][point_j];
        float phi_i_j = 1.0f / std::sqrt (2.0f * static_cast<float> (M_PI) * scale_squared) * expf ( (-1) * d_g*d_g / (2.0f * scale_squared));

        point_density_i += phi_i_j;
        phi_row[point_j] = phi_i_j;
      }
      point_density[point_i] = point_density_i;
      phi[point_i] = phi_row;
    }

    // compute weights for each pair (x_i, x_j), evaluate the operator A_hat
    for (size_t point_i = 0; point_i < input_->points.size (); ++point_i)
    {
      float A_hat_normalization = 0.0;
      PointT A_hat; A_hat.x = A_hat.y = A_hat.z = 0.0;
      for (size_t point_j = 0; point_j < input_->points.size (); ++point_j)
      {
        float phi_hat_i_j = phi[point_i][point_j] / (point_density[point_i] * point_density[point_j]);
        A_hat_normalization += phi_hat_i_j;

        PointT aux = input_->points[point_j];
        aux.x *= phi_hat_i_j; aux.y *= phi_hat_i_j; aux.z *= phi_hat_i_j;

        A_hat.x += aux.x; A_hat.y += aux.y; A_hat.z += aux.z;
      }
      A_hat.x /= A_hat_normalization; A_hat.y /= A_hat_normalization; A_hat.z /= A_hat_normalization;

      // compute the invariant F
      float aux = 2.0f / scale_values_[scale_i] * euclideanDistance<PointT, PointT> (A_hat, input_->points[point_i]);
      F[point_i] = aux * expf (-aux);
    }

    F_scales_[scale_i] = F;
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::StatisticalMultiscaleInterestRegionExtraction<PointT>::extractExtrema (std::list<IndicesPtr> &rois)
{
  std::vector<std::vector<bool> > is_min (scale_values_.size ()),
      is_max (scale_values_.size ());

  // for each point, check if it is a local extrema on each scale
  for (size_t scale_i = 0; scale_i < scale_values_.size (); ++scale_i)
  {
    std::vector<bool> is_min_scale (input_->points.size ()),
        is_max_scale (input_->points.size ());
    for (size_t point_i = 0; point_i < input_->points.size (); ++point_i)
    {
      std::vector<int> nn_indices;
      geodesicFixedRadiusSearch (point_i, scale_values_[scale_i], nn_indices);
      bool is_max_point = true, is_min_point = true;
      for (std::vector<int>::iterator nn_it = nn_indices.begin (); nn_it != nn_indices.end (); ++nn_it)
        if (F_scales_[scale_i][point_i] < F_scales_[scale_i][*nn_it])
          is_max_point = false;
        else
          is_min_point = false;

      is_min_scale[point_i] = is_min_point;
      is_max_scale[point_i] = is_max_point;
    }

    is_min[scale_i] = is_min_scale;
    is_max[scale_i] = is_max_scale;
  }

  // look for points that are min/max over three consecutive scales
  for (size_t scale_i = 1; scale_i < scale_values_.size () - 1; ++scale_i)
  {
    for (size_t point_i = 0; point_i < input_->points.size (); ++point_i)
      if ((is_min[scale_i - 1][point_i] && is_min[scale_i][point_i] && is_min[scale_i + 1][point_i]) ||
          (is_max[scale_i - 1][point_i] && is_max[scale_i][point_i] && is_max[scale_i + 1][point_i]))
        {
        // add the point to the result vector
        IndicesPtr region (new std::vector<int>);
        region->push_back (static_cast<int> (point_i));

        // and also add its scale-sized geodesic neighborhood
        std::vector<int> nn_indices;
        geodesicFixedRadiusSearch (point_i, scale_values_[scale_i], nn_indices);
        region->insert (region->end (), nn_indices.begin (), nn_indices.end ());
        rois.push_back (region);
      }
  }
}


#define PCL_INSTANTIATE_StatisticalMultiscaleInterestRegionExtraction(T) template class PCL_EXPORTS pcl::StatisticalMultiscaleInterestRegionExtraction<T>;

#endif /* PCL_FEATURES_IMPL_STATISTICAL_MULTISCALE_INTEREST_REGION_EXTRACTION_H_ */

