/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexandru-Eugen Ichim
 *                      Willow Garage, Inc
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
 *  $Id$
 */

#ifndef PCL_FEATURES_IMPL_STATISTICAL_MULTISCALE_INTEREST_REGION_EXTRACTION_H_
#define PCL_FEATURES_IMPL_STATISTICAL_MULTISCALE_INTEREST_REGION_EXTRACTION_H_

#include "pcl/features/statistical_multiscale_interest_region_extraction.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
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
    kdtree.nearestKSearch (point_i, 16, k_indices, k_distances);

    for (size_t k_i = 0; k_i < k_indices.size (); ++k_i)
      add_edge (point_i, k_indices[k_i], Weight (k_distances[k_i]), cloud_graph);
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

  PCL_INFO("some distances: %f %f %f\n", geodesic_distances_[1][3], geodesic_distances_[100][200], geodesic_distances_[400][401]);


  PCL_INFO ("Done generating the graph\n");
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::StatisticalMultiscaleInterestRegionExtraction<PointT>::initCompute ()
{
  if (!PCLBase<PointT>::initCompute ())
  {
    PCL_ERROR ("[pcl::StatisticalMultiscaleInterestRegionExtraction::initCompute] PCLBase::initCompute () failed - no input cloud was given.\n");
    return false;
  }
  if (scale_values_.empty ())
  {
    PCL_ERROR ("[pcl::StatisticalMultiscaleInterestRegionExtraction::initCompute] No scale values were given\n");
    return false;
  }

  return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::StatisticalMultiscaleInterestRegionExtraction<PointT>::computeRegionsOfInterest (typename pcl::PointCloud<PointT>::Ptr &output)
{
  if (!initCompute ())
  {
    PCL_ERROR ("StatisticalMultiscaleInterestRegionExtraction: not completely initialized\n");
    return;
  }

  generateCloudGraph ();

  PCL_INFO ("Calculating statistical information\n");
  output = typename pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT> ());

  for (std::vector<float>::iterator scale_it = scale_values_.begin (); scale_it != scale_values_.end (); ++scale_it)
  {
    float scale_squared = (*scale_it) * (*scale_it);

    // calculate point density for each point x_i
    std::vector<float> point_density (input_->points.size ()),
        F (input_->points.size ());
    std::vector<std::vector<float> > phi;
    for (size_t point_i = 0; point_i < input_->points.size (); ++point_i)
    {
      float point_density_i = 0.0;
      std::vector<float> phi_row;
      for (size_t point_j = 0; point_j < input_->points.size (); ++point_j)
      {
        float d_g = geodesic_distances_[point_i][point_j];
        float phi_i_j = 1.0 / sqrt(2.0*M_PI*scale_squared) * exp( (-1) * d_g*d_g / (2.0*scale_squared));

        point_density_i += phi_i_j;
        phi_row.push_back (phi_i_j);
      }
      point_density[point_i] = point_density_i;
      phi.push_back (phi_row);
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
      float aux = 2.0 / (*scale_it) * euclideanDistance<PointT, PointT> (A_hat, input_->points[point_i]);
      F[point_i] = aux * exp (-aux);
    }


    /// just test out - find maximum F
    size_t max_f_i = 0;
    for (size_t f_i = 0; f_i < F.size (); ++f_i)
      if (F[f_i] > F[max_f_i])
        max_f_i = f_i;
    output->points.push_back (input_->points[max_f_i]);
  }
}


#define PCL_INSTANTIATE_StatisticalMultiscaleInterestRegionExtraction(T) template class PCL_EXPORTS pcl::StatisticalMultiscaleInterestRegionExtraction<T>;

#endif /* PCL_FEATURES_IMPL_STATISTICAL_MULTISCALE_INTEREST_REGION_EXTRACTION_H_ */

