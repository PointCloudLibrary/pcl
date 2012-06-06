/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include "pcl/recognition/ransac_based/model_library.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

using namespace std;

//============================================================================================================================================

bool
pcl::recognition::ModelLibrary::addModel(const PointCloudIn& model, const PointCloudN& /*normals*/, const std::string& object_name)
{
  // Try to insert a new model entry
  pair<map<string,Entry*>::iterator, bool> result = model_entries_.insert(pair<string,Entry*>(object_name, static_cast<Entry*> (NULL)));

  // Check if 'object_name' is unique
  if ( !result.second )
    return false;

  vector<std::pair<int,int> > point_pairs;
  vector<int> point_ids;
  vector<float> sqr_dist;
  KdTreeFLANN<pcl::PointXYZ> kd_tree;
  kd_tree.setInputCloud(KdTree<pcl::PointXYZ>::PointCloudConstPtr(&model));
  // The two radii
  double min_sqr_radius = pair_width_ - pair_width_eps_, max_radius = pair_width_ + pair_width_eps_;
  int i, k, num_found_points, num_model_points = static_cast<int>(model.points.size());

  min_sqr_radius *= min_sqr_radius;

  // For each model point get the model points lying between the spheres with radii min_radius and max_radius
  for ( i = 0 ; i < num_model_points ; ++i )
  {
    point_ids.clear();
    sqr_dist.clear();
    num_found_points = kd_tree.radiusSearch(model.points[i], max_radius, point_ids, sqr_dist);

    for ( k = 0 ; k < num_found_points ; ++k )
      // Should we take that point?
      if ( sqr_dist[k] >= min_sqr_radius )
        point_pairs.push_back(pair<int,int>(i,k));
      else // Break since the points are sorted based on their distance to the query point
        break;
  }

//  Entry* new_entry = new Entry(object_name);

  return true;
}

//============================================================================================================================================
