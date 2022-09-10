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
 * $Id$
 *
 */

#ifndef PCL_FEATURES_IMPL_RSD_H_
#define PCL_FEATURES_IMPL_RSD_H_

#include <limits>
#include <pcl/features/rsd.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> Eigen::MatrixXf
pcl::computeRSD (const pcl::PointCloud<PointInT> &surface, const pcl::PointCloud<PointNT> &normals,
		 const pcl::Indices &indices, double max_dist,
		 int nr_subdiv, double plane_radius, PointOutT &radii, bool compute_histogram)
{
  // Check if the full histogram has to be saved or not
  Eigen::MatrixXf histogram;
  if (compute_histogram)
    histogram = Eigen::MatrixXf::Zero (nr_subdiv, nr_subdiv);

  // Check if enough points are provided or not
  if (indices.size () < 2)
  {
    radii.r_max = 0;
    radii.r_min = 0;
    return histogram;
  }
  
  // Initialize minimum and maximum angle values in each distance bin
  std::vector<std::vector<double> > min_max_angle_by_dist (nr_subdiv);
  for (auto& minmax: min_max_angle_by_dist)
  {
    minmax.resize (2);
    minmax[0] = std::numeric_limits<double>::max();
    minmax[1] = -std::numeric_limits<double>::max();
  }
  min_max_angle_by_dist[0][0] = min_max_angle_by_dist[0][1] = 0.0;

  // Compute distance by normal angle distribution for points
  pcl::Indices::const_iterator i, begin (indices.begin()), end (indices.end());
  for (i = begin+1; i != end; ++i)
  {
    // compute angle between the two lines going through normals (disregard orientation!)
    double cosine = normals[*i].normal[0] * normals[*begin].normal[0] +
                    normals[*i].normal[1] * normals[*begin].normal[1] +
                    normals[*i].normal[2] * normals[*begin].normal[2];
    if (cosine > 1) cosine = 1;
    if (cosine < -1) cosine = -1;
    double angle  = std::acos (cosine);
    if (angle > M_PI/2) angle = M_PI - angle; /// \note: orientation is neglected!

    // Compute point to point distance
    double dist = sqrt ((surface[*i].x - surface[*begin].x) * (surface[*i].x - surface[*begin].x) +
                        (surface[*i].y - surface[*begin].y) * (surface[*i].y - surface[*begin].y) +
                        (surface[*i].z - surface[*begin].z) * (surface[*i].z - surface[*begin].z));

    if (dist > max_dist)
      continue; /// \note: we neglect points that are outside the specified interval!

    // compute bins and increase
    int bin_d = static_cast<int> (std::floor (nr_subdiv * dist / max_dist));
    if (compute_histogram)
    {
      int bin_a = std::min (nr_subdiv-1, static_cast<int> (std::floor (nr_subdiv * angle / (M_PI/2))));
      histogram(bin_a, bin_d)++;
    }

    // update min-max values for distance bins
    min_max_angle_by_dist[bin_d][0] = std::min(angle, min_max_angle_by_dist[bin_d][0]);
    min_max_angle_by_dist[bin_d][1] = std::max(angle, min_max_angle_by_dist[bin_d][1]);
  }

  // Estimate radius from min and max lines
  double Amint_Amin = 0, Amint_d = 0;
  double Amaxt_Amax = 0, Amaxt_d = 0;
  for (int di=0; di<nr_subdiv; di++)
  {
    // combute the members of A'*A*r = A'*D
    if (min_max_angle_by_dist[di][1] >= 0)
    {
      double p_min = min_max_angle_by_dist[di][0];
      double p_max = min_max_angle_by_dist[di][1];
      double f = (di+0.5)*max_dist/nr_subdiv;
      Amint_Amin += p_min * p_min;
      Amint_d += p_min * f;
      Amaxt_Amax += p_max * p_max;
      Amaxt_d += p_max * f;
    }
  }
  float min_radius = Amint_Amin == 0.0f ? float (plane_radius) : float (std::min (Amint_d/Amint_Amin, plane_radius));
  float max_radius = Amaxt_Amax == 0.0f ? float (plane_radius) : float (std::min (Amaxt_d/Amaxt_Amax, plane_radius));

  // Small correction of the systematic error of the estimation (based on analysis with nr_subdiv_ = 5)
  min_radius *= 1.1f;
  max_radius *= 0.9f;
  if (min_radius < max_radius)
  {
    radii.r_min = min_radius;
    radii.r_max = max_radius;
  }
  else
  {
    radii.r_max = min_radius;
    radii.r_min = max_radius;
  }
  
  return histogram;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT, typename PointOutT> Eigen::MatrixXf
pcl::computeRSD (const pcl::PointCloud<PointNT> &normals,
		 const pcl::Indices &indices, const std::vector<float> &sqr_dists, double max_dist,
		 int nr_subdiv, double plane_radius, PointOutT &radii, bool compute_histogram)
{
  // Check if the full histogram has to be saved or not
  Eigen::MatrixXf histogram;
  if (compute_histogram)
    histogram = Eigen::MatrixXf::Zero (nr_subdiv, nr_subdiv);
  
  // Check if enough points are provided or not
  if (indices.size () < 2)
  {
    radii.r_max = 0;
    radii.r_min = 0;
    return histogram;
  }
  
  // Initialize minimum and maximum angle values in each distance bin
  std::vector<std::vector<double> > min_max_angle_by_dist (nr_subdiv);
  min_max_angle_by_dist[0].resize (2);
  min_max_angle_by_dist[0][0] = min_max_angle_by_dist[0][1] = 0.0;
  for (int di=1; di<nr_subdiv; di++)
  {
    min_max_angle_by_dist[di].resize (2);
    min_max_angle_by_dist[di][0] = std::numeric_limits<double>::max();
    min_max_angle_by_dist[di][1] = std::numeric_limits<double>::lowest();
  }
  
  // Compute distance by normal angle distribution for points
  pcl::Indices::const_iterator i, begin (indices.begin()), end (indices.end());
  for (i = begin+1; i != end; ++i)
  {
    // compute angle between the two lines going through normals (disregard orientation!)
    double cosine = normals[*i].normal[0] * normals[*begin].normal[0] +
                    normals[*i].normal[1] * normals[*begin].normal[1] +
                    normals[*i].normal[2] * normals[*begin].normal[2];
    if (cosine > 1) cosine = 1;
    if (cosine < -1) cosine = -1;
    double angle  = std::acos (cosine);
    if (angle > M_PI/2) angle = M_PI - angle; /// \note: orientation is neglected!

    // Compute point to point distance
    double dist = sqrt (sqr_dists[i-begin]);

    if (dist > max_dist)
      continue; /// \note: we neglect points that are outside the specified interval!

    // compute bins and increase
    int bin_d = static_cast<int> (std::floor (nr_subdiv * dist / max_dist));
    if (compute_histogram)
    {
      int bin_a = std::min (nr_subdiv-1, static_cast<int> (std::floor (nr_subdiv * angle / (M_PI/2))));
      histogram(bin_a, bin_d)++;
    }

    // update min-max values for distance bins
    if (min_max_angle_by_dist[bin_d][0] > angle) min_max_angle_by_dist[bin_d][0] = angle;
    if (min_max_angle_by_dist[bin_d][1] < angle) min_max_angle_by_dist[bin_d][1] = angle;
  }

  // Estimate radius from min and max lines
  double Amint_Amin = 0, Amint_d = 0;
  double Amaxt_Amax = 0, Amaxt_d = 0;
  for (int di=0; di<nr_subdiv; di++)
  {
    // combute the members of A'*A*r = A'*D
    if (min_max_angle_by_dist[di][1] >= 0)
    {
      double p_min = min_max_angle_by_dist[di][0];
      double p_max = min_max_angle_by_dist[di][1];
      double f = (di+0.5)*max_dist/nr_subdiv;
      Amint_Amin += p_min * p_min;
      Amint_d += p_min * f;
      Amaxt_Amax += p_max * p_max;
      Amaxt_d += p_max * f;
    }
  }
  float min_radius = Amint_Amin == 0.0f ? float (plane_radius) : float (std::min (Amint_d/Amint_Amin, plane_radius));
  float max_radius = Amaxt_Amax == 0.0f ? float (plane_radius) : float (std::min (Amaxt_d/Amaxt_Amax, plane_radius));

  // Small correction of the systematic error of the estimation (based on analysis with nr_subdiv_ = 5)
  min_radius *= 1.1f;
  max_radius *= 0.9f;
  if (min_radius < max_radius)
  {
    radii.r_min = min_radius;
    radii.r_max = max_radius;
  }
  else
  {
    radii.r_max = min_radius;
    radii.r_min = max_radius;
  }
  
  return histogram;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::RSDEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Check if search_radius_ was set
  if (search_radius_ < 0)
  {
    PCL_ERROR ("[pcl::%s::computeFeature] A search radius needs to be set!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.clear ();
    return;
  }

  // List of indices and corresponding squared distances for a neighborhood
  // \note resize is irrelevant for a radiusSearch ().
  pcl::Indices nn_indices;
  std::vector<float> nn_sqr_dists;

  // Check if the full histogram has to be saved or not
  if (save_histograms_)
  {
    // Reserve space for the output histogram dataset
    histograms_.reset (new std::vector<Eigen::MatrixXf, Eigen::aligned_allocator<Eigen::MatrixXf> >);
    histograms_->reserve (output.size ());
    
    // Iterating over the entire index vector
    for (std::size_t idx = 0; idx < indices_->size (); ++idx)
    {
      // Compute and store r_min and r_max in the output cloud
      this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_sqr_dists);
      //histograms_->push_back (computeRSD (*surface_, *normals_, nn_indices, search_radius_, nr_subdiv_, plane_radius_, output[idx], true));
      histograms_->push_back (computeRSD (*normals_, nn_indices, nn_sqr_dists, search_radius_, nr_subdiv_, plane_radius_, output[idx], true));
    }
  }
  else
  {
    // Iterating over the entire index vector
    for (std::size_t idx = 0; idx < indices_->size (); ++idx)
    {
      // Compute and store r_min and r_max in the output cloud
      this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_sqr_dists);
      //computeRSD (*surface_, *normals_, nn_indices, search_radius_, nr_subdiv_, plane_radius_, output[idx], false);
      computeRSD (*normals_, nn_indices, nn_sqr_dists, search_radius_, nr_subdiv_, plane_radius_, output[idx], false);
    }
  }
}

#define PCL_INSTANTIATE_RSDEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::RSDEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_RSD_H_ 
