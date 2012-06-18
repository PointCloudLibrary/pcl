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

#include <pcl/recognition/ransac_based/model_library.h>
#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/recognition/impl/ransac_based/voxel_structure.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <vector>

using namespace std;
using namespace pcl;
using namespace recognition;

//============================================================================================================================================

ModelLibrary::ModelLibrary(double pair_width)
: pair_width_(pair_width), pair_width_eps_(0.1*pair_width)
{
//  hash_table_.build();
}

//============================================================================================================================================

bool
ModelLibrary::addModel(const PointCloudIn& points, const PointCloudN& normals, const std::string& object_name)
{
  // Try to insert a new model entry
  pair<map<string,Model*>::iterator, bool> result = model_entries_.insert(pair<string,Model*>(object_name, static_cast<Model*> (NULL)));

  // Check if 'object_name' is unique
  if ( !result.second )
    return false;

  // It is unique -> create a new library model
  Model* new_model = new Model(points, normals);
  result.first->second = new_model;

  vector<std::pair<int,int> > point_pairs;
  vector<int> point_ids;
  vector<float> sqr_dist;
  KdTreeFLANN<Eigen::Vector3d> kd_tree;
  kd_tree.setInputCloud(KdTree<Eigen::Vector3d>::PointCloudConstPtr (&points));
  // The two radii
  double min_sqr_radius = pair_width_ - pair_width_eps_, max_radius = pair_width_ + pair_width_eps_;
  int i, k, num_found_points, num_model_points = static_cast<int> (points.points.size());

  min_sqr_radius *= min_sqr_radius;

  // For each model point get the model points lying between the spheres with radii min_radius and max_radius
  for ( i = 0 ; i < num_model_points ; ++i )
  {
    point_ids.clear();
    sqr_dist.clear();
    num_found_points = kd_tree.radiusSearch (points.points[i], max_radius, point_ids, sqr_dist);

    for ( k = 0 ; k < num_found_points ; ++k )
      // Should we take that point?
      if ( sqr_dist[k] >= min_sqr_radius )
        this->addToHashTable(new_model, i, point_ids[k]);
      else // Break since the points are sorted based on their distance to the query point
        break;
  }

  return true;
}

//============================================================================================================================================

void
ModelLibrary::addToHashTable(const ModelLibrary::Model* model, int i, int j)
{
  double key[3];

  // Compute the descriptor signature for the oriented point pair (i, j)
  ObjRecRANSAC::compute_oriented_point_pair_signature (
      model->points_.points[i], model->normals_.points[i],
      model->points_.points[j], model->normals_.points[j], key);

  // Use the key to insert the oriented point pair at the right place in the hash table
  HashTableCell* cell = hash_table_.getVoxel(key);

  if ( cell )
  {
    // to be continued
  }
}

//============================================================================================================================================
