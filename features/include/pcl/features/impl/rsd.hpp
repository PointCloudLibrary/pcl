/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: rsd.hpp 35475 2011-01-25 18:27:05Z marton $
 *
 */

#ifndef PCL_FEATURES_IMPL_RSD_H_
#define PCL_FEATURES_IMPL_RSD_H_

#include <cfloat>
#include "pcl/features/rsd.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> inline void
pcl::computeRSD (const pcl::PointCloud<PointInT> &surface, const pcl::PointCloud<PointNT> &normals,
		 const std::vector<int> &indices, double max_dist,
		 int nr_subdiv, double plane_radius, PointOutT &radii)
{
  // Initialize minimum and maximum angle values in each distance bin
  std::vector<std::vector<double> > min_max_angle_by_dist (nr_subdiv);
  min_max_angle_by_dist[0].resize (2);
  min_max_angle_by_dist[0][0] = min_max_angle_by_dist[0][1] = 0.0;
  for (int di=1; di<nr_subdiv; di++)
  {
    min_max_angle_by_dist[di].resize (2);
    min_max_angle_by_dist[di][0] = +DBL_MAX;
    min_max_angle_by_dist[di][1] = -DBL_MAX;
  }

  // Compute distance by normal angle distribution for points
  std::vector<int>::const_iterator i, begin (indices.begin()), end (indices.end());
  for(i = begin+1; i != end; ++i)
  {
    // compute angle between the two lines going through normals (disregard orientation!)
    double cosine = normals.points[*i].normal[0] * normals.points[*begin].normal[0] +
                    normals.points[*i].normal[1] * normals.points[*begin].normal[1] +
                    normals.points[*i].normal[2] * normals.points[*begin].normal[2];
    if (cosine > 1) cosine = 1;
    if (cosine < -1) cosine = -1;
    double angle  = acos (cosine);
    if (angle > M_PI/2) angle = M_PI - angle; /// \note: orientation is neglected!

    // Compute point to point distance
    double dist = sqrt ((surface.points[*i].x - surface.points[*begin].x) * (surface.points[*i].x - surface.points[*begin].x) +
                        (surface.points[*i].y - surface.points[*begin].y) * (surface.points[*i].y - surface.points[*begin].y) +
                        (surface.points[*i].z - surface.points[*begin].z) * (surface.points[*i].z - surface.points[*begin].z));

    if (dist > max_dist)
      continue; /// \note: we neglect points that are outside the specified interval!

    // compute bins and increase
    int bin_d = (int) floor (nr_subdiv * dist / max_dist);

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
      //cerr << p_min << " " << p_max << endl;
      double f = (di+0.5)*max_dist/nr_subdiv;
      //cerr << f << endl;
      Amint_Amin += p_min * p_min;
      Amint_d += p_min * f;
      Amaxt_Amax += p_max * p_max;
      Amaxt_d += p_max * f;
    }
  }
  radii.r_max = Amint_Amin == 0 ? plane_radius : std::min (Amint_d/Amint_Amin, plane_radius);
  radii.r_min = Amaxt_Amax == 0 ? plane_radius : std::min (Amaxt_d/Amaxt_Amax, plane_radius);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::RSDEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Check if input was set
  if (!normals_)
  {
    ROS_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }
  if (normals_->points.size () != surface_->points.size ())
  {
    ROS_ERROR ("[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Check if search_radius_ was set
  if (search_radius_ < 0)
  {
    ROS_ERROR ("[pcl::%s::computeFeature] A search radius needs to be set!", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Allocate enough space to hold the results
  // \note resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices;
  std::vector<float> nn_sqr_dists;

  // Iterating over the entire index vector
  for (size_t idx = 0; idx < indices_->size (); ++idx)
  {
    // Compute and store r_min and r_max in the output cloud
    searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_sqr_dists);
    computeRSD (*surface_, *normals_, nn_indices, search_radius_, nr_subdiv_, plane_radius_, output.points[idx]);
  }
}

#define PCL_INSTANTIATE_RSDEstimation(T,NT,OutT) template class pcl::RSDEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_VFH_H_ 
