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
#include <pcl/console/print.h>
#include <cmath>

using namespace pcl;
using namespace console;
using namespace pcl::recognition;

//============================================================================================================================================

ModelLibrary::ModelLibrary (float pair_width, float voxel_size, float max_coplanarity_angle)
: pair_width_ (pair_width),
  voxel_size_ (voxel_size),
  max_coplanarity_angle_ (max_coplanarity_angle),
  ignore_coplanar_opps_ (true)
{
  num_of_cells_[0] = 60;
  num_of_cells_[1] = 60;
  num_of_cells_[2] = 60;

  // Compute the bounds of the hash table
  float eps = 0.000001f; // To be sure that an angle of 0 or PI will not be excluded because it lies on the boundary of the voxel structure
  float bounds[6] = {-eps, static_cast<float> (M_PI) + eps,
                     -eps, static_cast<float> (M_PI) + eps,
                     -eps, static_cast<float> (M_PI) + eps};
 
  hash_table_.build (bounds, num_of_cells_);
}

//============================================================================================================================================

void
ModelLibrary::clear ()
{
  this->removeAllModels ();

  num_of_cells_[0] = num_of_cells_[1] = num_of_cells_[2] = 0;
  hash_table_.clear ();
}

//============================================================================================================================================

void
ModelLibrary::removeAllModels ()
{
  // Delete the model entries
  for (auto &model : models_)
    delete model.second;
  models_.clear();

  // Clear the hash table
  HashTableCell* cells = hash_table_.getVoxels();
  int num_bins = num_of_cells_[0]*num_of_cells_[1]*num_of_cells_[2];

  // Clear each cell
  for ( int i = 0 ; i < num_bins ; ++i )
    cells[i].clear();
}

//============================================================================================================================================

bool
ModelLibrary::addModel (const PointCloudIn& points, const PointCloudN& normals, const std::string& object_name,
                        float frac_of_points_for_registration, void* user_data)
{
#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("ModelLibrary::%s(): begin [%s]\n", __func__, object_name.c_str ());
#endif

  // Try to insert a new model entry
  std::pair<std::map<std::string,Model*>::iterator, bool> result = models_.insert (std::pair<std::string,Model*> (object_name, static_cast<Model*> (nullptr)));

  // Check if 'object_name' is unique
  if (!result.second)
  {
    print_error ("'%s' already exists in the model library.\n", object_name.c_str ());
    return (false);
  }

  // It is unique -> create a new library model and save it
  Model* new_model = new Model (points, normals, voxel_size_, object_name, frac_of_points_for_registration, user_data);
  result.first->second = new_model;

  const ORROctree& octree = new_model->getOctree ();
  const std::vector<ORROctree::Node*> &full_leaves = octree.getFullLeaves ();
  std::list<ORROctree::Node*> inter_leaves;
  int num_of_pairs = 0;

  // Run through all full leaves
  for (const auto &full_leaf : full_leaves)
  {
    const ORROctree::Node::Data* node_data1 = full_leaf->getData ();

    // Get all full leaves at the right distance to the current leaf
    inter_leaves.clear ();
    octree.getFullLeavesIntersectedBySphere (node_data1->getPoint (), pair_width_, inter_leaves);

    for (const auto &inter_leaf : inter_leaves)
    {
      // Compute the hash table key
      if ( this->addToHashTable(new_model, node_data1, inter_leaf->getData ()) )
        ++num_of_pairs;
    }
  }

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("ModelLibrary::%s(): end [%i oriented point pairs]\n", __func__, num_of_pairs);
#endif

  return (true);
}

//============================================================================================================================================

bool
ModelLibrary::addToHashTable (Model* model, const ORROctree::Node::Data* data1, const ORROctree::Node::Data* data2)
{
  float key[3];

  // Compute the descriptor signature for the oriented point pair (i, j)
  ObjRecRANSAC::compute_oriented_point_pair_signature (
    data1->getPoint (), data1->getNormal (),
    data2->getPoint (), data2->getNormal (), key);

  if ( ignore_coplanar_opps_ )
  {
    // If the angle between one of the normals and the connecting vector is about 90° and
    // the angle between both normals is about 0° then the points are co-planar ignore them!
    if ( std::fabs (key[0]-AUX_HALF_PI) < max_coplanarity_angle_ && key[2] < max_coplanarity_angle_ )
      return (false);
  }

  // Get the hash table cell containing 'key' (there is for sure such a cell since the hash table bounds are large enough)
  HashTableCell* cell = hash_table_.getVoxel (key);

  // Insert the pair (data1,data2) belonging to 'model'
  (*cell)[model].push_back (std::pair<const ORROctree::Node::Data*, const ORROctree::Node::Data*> (data1, data2));

  return (true);
}

//============================================================================================================================================
