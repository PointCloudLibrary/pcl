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

#include <pcl/common/random.h>
#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/recognition/ransac_based/orr_graph.h>
#include <ctime>

using namespace std;
using namespace pcl::common;

pcl::recognition::ObjRecRANSAC::ObjRecRANSAC (float pair_width, float voxel_size, float fraction_of_pairs_in_hash_table)
: pair_width_ (pair_width),
  voxel_size_ (voxel_size),
  fraction_of_pairs_in_hash_table_ (fraction_of_pairs_in_hash_table),
  relative_obj_size_ (0.05f),
  visibility_ (0.06f),
  relative_num_of_illegal_pts_ (0.02f),
  intersection_fraction_ (0.03f),
  model_library_ (pair_width, voxel_size)
{
  abs_zdist_thresh_ = 1.5f*voxel_size_;
}

//=========================================================================================================================================================================

void
pcl::recognition::ObjRecRANSAC::recognize (const PointCloudIn* scene, const PointCloudN* normals, list<ObjRecRANSAC::Output>& recognized_objects, double success_probability)
{
  // First, build the scene octree
  scene_octree_.build(*scene, voxel_size_, normals);
  // Project it on the xy-plane (which roughly corresponds to the projection plane of the scanning device)
  scene_octree_proj_.build(scene_octree_, abs_zdist_thresh_, abs_zdist_thresh_);

  if ( success_probability >= 1.0 )
    success_probability = 0.99;

  // Compute the number of iterations
  vector<ORROctree::Node*>& full_leaves = scene_octree_.getFullLeaves();
  int num_iterations = this->computeNumberOfIterations(success_probability), num_full_leaves = static_cast<int> (full_leaves.size ());

  if ( num_full_leaves < num_iterations )
    num_iterations = num_full_leaves;

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("ObjRecRANSAC::%s(): recognizing objects [%i iteration(s)]\n", __func__, num_iterations);
#endif

  list<OrientedPointPair> point_pairs;
  list<Hypothesis> hypotheses;
  vector<Hypothesis> accepted_hypotheses;
  ORRGraph graph;

  this->sampleOrientedPointPairs (num_iterations, full_leaves, point_pairs);
  this->generateHypotheses (point_pairs, hypotheses);
  point_pairs.clear (); // Free memory

  this->testHypotheses (hypotheses, accepted_hypotheses);
  hypotheses.clear (); // Free memory

  this->buildConflictGraph (accepted_hypotheses, graph);
  accepted_hypotheses.clear (); // Free memory

  this->filterWeakHypotheses (graph, recognized_objects);

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("ObjRecRANSAC::%s(): done [%i object(s)]\n", __func__, static_cast<int> (recognized_objects.size ()));
#endif
}

//=========================================================================================================================================================================

int
pcl::recognition::ObjRecRANSAC::computeNumberOfIterations (double success_probability)
{
	// 'p_obj' is the probability that given that the first sample point belongs to an object,
	// the second sample point will belong to the same object
	double p, p_obj = 0.25f;

	p = p_obj*relative_obj_size_*fraction_of_pairs_in_hash_table_;

	if ( 1.0 - p <= 0.0 )
		return 1;

	return static_cast<int> (log (1.0-success_probability)/log (1.0-p) + 1.0);
}

//=========================================================================================================================================================================

void
pcl::recognition::ObjRecRANSAC::sampleOrientedPointPairs (int num_iterations, vector<ORROctree::Node*>& full_scene_leaves, std::list<OrientedPointPair>& output)
{
#ifdef OBJ_REC_RANSAC_VERBOSE
  printf ("ObjRecRANSAC::%s(): sampling oriented point pairs ... ", __func__); fflush (stdout);
#endif

  int i, num_full_leaves = static_cast<int> (full_scene_leaves.size ());
  ORROctree::Node *leaf1, *leaf2;
  const float *p1, *p2, *n1, *n2;
  // The random generator
  UniformGenerator<int> randgen (0, num_full_leaves, static_cast<uint32_t> (time (NULL)));

  // Init the vector with the ids
  vector<int> ids (num_full_leaves);
  for ( i = 0 ; i < num_full_leaves ; ++i )
    ids[i] = i;

  // Sample 'num_iterations' number of oriented point pairs
  for ( i = 0 ; i < num_iterations ; ++i )
  {
    // Choose a random position within the array of ids
    randgen.setParameters (0, static_cast<int> (ids.size ()));
    int rand_pos = randgen.run ();

    // Get the leaf at that random position
    leaf1 = full_scene_leaves[ids[rand_pos]];
    // Delete the selected id
    ids.erase (ids.begin() + rand_pos);

    // Get the leaf's point and normal
    p1 = leaf1->getData ()->getPoint ();
    n1 = leaf1->getData ()->getNormal ();

    // Randomly select a leaf at the right distance from 'leaves[i]'
    leaf2 = scene_octree_.getRandomFullLeafOnSphere (p1, pair_width_);
    if ( !leaf2 )
      continue;

    // Get the leaf's point and normal
    p2 = leaf2->getData ()->getPoint ();
    n2 = leaf2->getData ()->getNormal ();

    // Save the sampled point pair
    output.push_back (OrientedPointPair (p1, n1, p2, n2));
  }

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf ("done.\n"); fflush (stdout);
#endif
}

//=========================================================================================================================================================================

int
pcl::recognition::ObjRecRANSAC::generateHypotheses (const list<OrientedPointPair>& pairs, list<Hypothesis>& out)
{
  // Only for 3D hash tables: this is the max number of neighbors a 3D hash table cell can have!
  ModelLibrary::HashTableCell *neigh_cells[27];
  ModelLibrary::HashTableCell::iterator cell_it;
  float hash_table_key[3];
  const float *model_p1, *model_n1, *model_p2, *model_n2;
  const float *scene_p1, *scene_n1, *scene_p2, *scene_n2;
  list<OrientedPointPair>::const_iterator pair;
  int i, num_hypotheses = 0;
  ModelLibrary::Model* obj_model;

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("ObjRecRANSAC::%s(): generating hypotheses ... ", __func__); fflush (stdout);
#endif

  for ( pair = pairs.begin () ; pair != pairs.end () ; ++pair )
  {
    // Just to make the code more readable
    scene_p1 = (*pair).p1_;
    scene_n1 = (*pair).n1_;
    scene_p2 = (*pair).p2_;
    scene_n2 = (*pair).n2_;

    // Use normals and points to compute a hash table key
    this->compute_oriented_point_pair_signature (scene_p1, scene_n1, scene_p2, scene_n2, hash_table_key);
    // Get the cell and its neighbors based on 'key'
    int num_neigh_cells = model_library_.getHashTable ()->getNeighbors (hash_table_key, neigh_cells);

    for ( i = 0 ; i < num_neigh_cells ; ++i )
    {
      // Check for all models in the current cell
      for ( cell_it = neigh_cells[i]->begin () ; cell_it != neigh_cells[i]->end () ; ++cell_it )
      {
        obj_model = (*cell_it).first;
        ModelLibrary::node_data_pair_list& model_pairs = (*cell_it).second;

        // Check for all pairs which belong to the current model
        for ( ModelLibrary::node_data_pair_list::iterator model_pair_it = model_pairs.begin () ; model_pair_it != model_pairs.end () ; ++model_pair_it )
        {
          // Get the points and normals
          model_p1 = (*model_pair_it).first->getPoint ();
          model_n1 = (*model_pair_it).first->getNormal ();
          model_p2 = (*model_pair_it).second->getPoint ();
          model_n2 = (*model_pair_it).second->getNormal ();

          Hypothesis hypothesis (obj_model);
          // Get the rigid transform from model to scene
          this->computeRigidTransform(model_p1, model_n1, model_p2, model_n2, scene_p1, scene_n1, scene_p2, scene_n2, hypothesis.rigid_transform_);

          ++num_hypotheses;
          // Save the current object hypothesis
          out.push_back(hypothesis);
        }
      }
    }
  }
#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("%i hypotheses\n", num_hypotheses);
#endif

  return (num_hypotheses);
}

//=========================================================================================================================================================================

void
pcl::recognition::ObjRecRANSAC::testHypotheses (list<Hypothesis>& hypotheses, vector<Hypothesis>& accepted_hypotheses)
{
  float transformed_point[3];
  int match, penalty, match_thresh, penalty_thresh;
  ORROctreeZProjection::Pixel* pixel;
  const float *rigid_transform;

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("ObjRecRANSAC::%s(): checking the hypotheses ... ", __func__); fflush(stdout);
#endif

  for ( list<Hypothesis>::iterator hypo_it = hypotheses.begin () ; hypo_it != hypotheses.end () ; ++hypo_it )
  {
    // Todo: Perform an ICP iteration

    // Some initializations for the second loop (the match/penalty loop)
    match = penalty = 0;

    // For better code readability
    vector<ORROctree::Node*>& full_model_leaves = (*hypo_it).obj_model_->getOctree ().getFullLeaves ();
    rigid_transform = (*hypo_it).rigid_transform_;

	match_thresh = static_cast<int> (static_cast<float> (full_model_leaves.size ())*visibility_ + 0.5f);
	penalty_thresh = static_cast<int> (static_cast<float> (full_model_leaves.size ())*relative_num_of_illegal_pts_ + 0.5f);

    // The match/penalty loop
    for ( vector<ORROctree::Node*>::iterator leaf_it = full_model_leaves.begin () ; leaf_it != full_model_leaves.end () ; ++leaf_it )
    {
      // Transform the model point with the current rigid transform
      aux::transform_point (rigid_transform, (*leaf_it)->getData ()->getPoint (), transformed_point);

      // Get the pixel 'transformed_point' lies in
      pixel = scene_octree_proj_.getPixel (transformed_point);
      // Check if we have a valid pixel
      if ( pixel == NULL )
        continue;

      if ( transformed_point[2] < pixel->z1_ ) // The transformed model point overshadows a pixel -> penalize the hypothesis
        ++penalty;
      else if ( transformed_point[2] <= pixel->z2_ ) // The point is OK
      {
        ++match;
        // 'hypo_it' explains 'pixel' => insert the pixel's id in the id set of 'hypo_it'
        (*hypo_it).explained_pixels_.insert (pixel->id_);
      }
    }

    // Check if we should accept this hypothesis
    if ( match >= match_thresh && penalty <= penalty_thresh )
      accepted_hypotheses.push_back (*hypo_it);
  }

#ifdef OBJ_REC_RANSAC_VERBOSE
	printf("%i accepted.\n", static_cast<int> (accepted_hypotheses.size ())); fflush (stdout);
#endif
}

//=========================================================================================================================================================================

void
pcl::recognition::ObjRecRANSAC::buildConflictGraph (vector<Hypothesis>& hypotheses, ORRGraph& graph)
{
  int hypothesis_id = 0, score;
  ORROctreeZProjection::Pixel* pixel;
  const float *rigid_transform;
  float transformed_point[3];

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf ("ObjRecRANSAC::%s(): building the conflict graph ... ", __func__); fflush (stdout);
#endif

  // There are as many graph nodes as hypotheses
  graph.resize (static_cast<int> (hypotheses.size ()));

  // Project the hypotheses onto the "range image" and store in each pixel the corresponding hypothesis id
  for ( vector<Hypothesis>::iterator hypo_it = hypotheses.begin () ; hypo_it != hypotheses.end () ; ++hypo_it, ++hypothesis_id )
  {
	// For better code readability
	vector<ORROctree::Node*>& full_leaves = (*hypo_it).obj_model_->getOctree ().getFullLeaves ();
	rigid_transform = (*hypo_it).rigid_transform_;

	// The i-th node corresponds to the i-th hypothesis and has id i
	graph.getNodes ()[hypothesis_id]->hypothesis_ = &(*hypo_it);
	graph.getNodes ()[hypothesis_id]->id_ = hypothesis_id;

	// At the end of the next loop this will be the number of model points ending up in a range image pixel
	score = 0;

    for ( vector<ORROctree::Node*>::iterator leaf_it = full_leaves.begin () ; leaf_it != full_leaves.end () ; ++leaf_it )
    {
      // Transform the model point with the current rigid transform
      aux::transform_point (rigid_transform, (*leaf_it)->getData ()->getPoint (), transformed_point);

      // Get the pixel containing 'transformed_point'
      pixel = scene_octree_proj_.getPixel (transformed_point);
      // Check if we have a valid pixel
      if ( pixel == NULL )
        continue;

      if ( pixel->z1_ <= transformed_point[2] && transformed_point[2] <= pixel->z2_ )
      {
        ++score;
        pixel->hypotheses_ids_.insert (hypothesis_id); // 'hypothesis_id' is the position of the hypothesis in the vector
      }
    }

    // Save the match confidence which is the number of model points falling within a range image pixel
    // divided by the total number of model points
    (*hypo_it).match_confidence_ = static_cast<float> (score)/static_cast<float> (full_leaves.size ());
  }

  list<ORROctreeZProjection::Pixel*>& full_pixels = scene_octree_proj_.getFullPixels ();
  set<int>::iterator id1, id2, last_id;

  // Now, iterate through all full pixels and build the conflict graph, i.e., create its connectivity
  for ( list<ORROctreeZProjection::Pixel*>::iterator it = full_pixels.begin () ; it != full_pixels.end () ; ++it )
  {
    // For better code readability
    pixel = *it;

    if ( pixel->hypotheses_ids_.empty () )
      continue;

    // Get the last id in the set
    last_id = pixel->hypotheses_ids_.end ();
    --last_id;

    // All hypotheses which explain the same pixel are conflicting
    for ( id1 = pixel->hypotheses_ids_.begin () ; id1 != last_id ; ++id1 )
    {
      id2 = id1;
      for ( ++id2 ; id2 != pixel->hypotheses_ids_.end () ; ++id2 )
        graph.insertEdge (*id1, *id2);
    }
  }

  vector<ORRGraph::Node*> graph_nodes = graph.getNodes ();
  ORRGraph::Node *node;
  set<int> id_intersection;
  float frac_1, frac_2;

  // Now, that we have the graph connectivity, we want to check if each two neighbors are
  // really in conflict. This requires set intersection operations which are expensive,
  // that's why we are performing them now, and not prior to computing the graph connectivity
  for ( vector<ORRGraph::Node*>::iterator it = graph_nodes.begin () ; it != graph_nodes.end () ; ++it )
  {
    node = *it;
    for ( set<ORRGraph::Node*>::iterator neigh = node->neighbors_.begin () ; neigh != node->neighbors_.end () ; ++neigh )
    {
      id_intersection.clear ();
      // Compute the ids intersection of 'node' and its neighbor
      std::set_intersection (node->hypothesis_->explained_pixels_.begin (),
                             node->hypothesis_->explained_pixels_.end (),
                             (*neigh)->hypothesis_->explained_pixels_.begin (),
                             (*neigh)->hypothesis_->explained_pixels_.end (),
                             std::inserter (id_intersection, id_intersection.begin ()));

      frac_1 = static_cast<float> (id_intersection.size ())/static_cast <float> (node->hypothesis_->explained_pixels_.size ());
      frac_2 = static_cast<float> (id_intersection.size ())/static_cast <float> ((*neigh)->hypothesis_->explained_pixels_.size ());
      // Check if the intersection set is large enough, i.e., is there a conflict
      if ( frac_1 <= intersection_fraction_ && frac_2 <= intersection_fraction_ )
        // The intersection set is too small => no conflict, detach these tqwo neighboring nodes
        graph.deleteEdge (node->id_, (*neigh)->id_);
    }
  }

#ifdef OBJ_REC_RANSAC_VERBOSE
	printf("%i accepted.\n", static_cast<int> (hypotheses.size ())); fflush (stdout);
#endif
}

//=========================================================================================================================================================================

bool
compare_orr_graph_nodes (pcl::recognition::ORRGraph::Node* n1, pcl::recognition::ORRGraph::Node* n2)
{
  return static_cast<bool> (n1->penalty_ < n2->penalty_);
}

//=========================================================================================================================================================================

void
pcl::recognition::ObjRecRANSAC::filterWeakHypotheses (ORRGraph& graph, list<ObjRecRANSAC::Output>& recognized_objects)
{
  vector<ORRGraph::Node*> &nodes = graph.getNodes (), sorted_nodes (graph.getNodes ().size ());
  size_t i = 0, num_of_explained;

  // Compute the penalty for each graph node
  for ( vector<ORRGraph::Node*>::iterator it = nodes.begin () ; it != nodes.end () ; ++it, ++i )
  {
    num_of_explained = 0;
    // Accumulate the number of pixels the neighbors are explaining
    for ( set<ORRGraph::Node*>::iterator neigh = (*it)->neighbors_.begin () ; neigh != (*it)->neighbors_.end () ; ++neigh )
      num_of_explained += (*neigh)->hypothesis_->explained_pixels_.size ();

    // Now compute the penalty for the node
    (*it)->penalty_ = static_cast<int> (num_of_explained) - static_cast<int> ((*it)->hypothesis_->explained_pixels_.size ());

    // Save the current node
    sorted_nodes[i] = *it;
  }

  // Now sort the nodes according to the penalty
  std::sort (sorted_nodes.begin (), sorted_nodes.end (), compare_orr_graph_nodes);

  // Now run through the array and start switching nodes on and off
  for ( vector<ORRGraph::Node*>::iterator it = sorted_nodes.begin () ; it != sorted_nodes.end () ; ++it )
  {
    // Ignore graph nodes which are already OFF
    if ( (*it)->state_ == ORRGraph::Node::OFF )
      continue;

    // Set the node to ON
    (*it)->state_ = ORRGraph::Node::ON;
    // save the hypothesis as an accepted solution
    recognized_objects.push_back (
      ObjRecRANSAC::Output ((*it)->hypothesis_->obj_model_->obj_name_, (*it)->hypothesis_->rigid_transform_, (*it)->hypothesis_->match_confidence_)
    );

    // and set all its neighbors to OFF
    for ( set<ORRGraph::Node*>::iterator neigh = (*it)->neighbors_.begin () ; neigh != (*it)->neighbors_.end () ; ++neigh )
      (*neigh)->state_ = ORRGraph::Node::OFF;
  }
}

//=========================================================================================================================================================================

void
pcl::recognition::ObjRecRANSAC::computeRigidTransform (
  const float *a1, const float *a1_n, const float *b1, const float* b1_n,
  const float *a2, const float *a2_n, const float *b2, const float* b2_n,
  float* rigid_transform) const
{
	// Some local variables
	float o1[3], o2[3], x1[3], x2[3], y1[3], y2[3], z1[3], z2[3], tmp1[3], tmp2[3], Ro1[3];
	float invFrame1[3][3], rot[3][3];

	// Compute the origins
	o1[0] = 0.5f*(a1[0] + b1[0]);
	o1[1] = 0.5f*(a1[1] + b1[1]);
	o1[2] = 0.5f*(a1[2] + b1[2]);

	o2[0] = 0.5f*(a2[0] + b2[0]);
	o2[1] = 0.5f*(a2[1] + b2[1]);
	o2[2] = 0.5f*(a2[2] + b2[2]);

	// Compute the x-axes
	aux::vecDiff3 (b1, a1, x1); aux::vecNormalize3 (x1);
	aux::vecDiff3 (b2, a2, x2); aux::vecNormalize3 (x2);
	// Compute the y-axes. First y-axis
	aux::projectOnPlane3 (a1_n, x1, tmp1); aux::vecNormalize3 (tmp1);
	aux::projectOnPlane3 (b1_n, x1, tmp2); aux::vecNormalize3 (tmp2);
	aux::vecSum3 (tmp1, tmp2, y1); aux::vecNormalize3 (y1);
	// Second y-axis
	aux::projectOnPlane3 (a2_n, x2, tmp1); aux::vecNormalize3 (tmp1);
	aux::projectOnPlane3 (b2_n, x2, tmp2); aux::vecNormalize3 (tmp2);
	aux::vecSum3 (tmp1, tmp2, y2); aux::vecNormalize3 (y2);
	// Compute the z-axes
	aux::vecCross3 (x1, y1, z1);
	aux::vecCross3 (x2, y2, z2);

	// 1. Invert [x1 y1 z1]
	invFrame1[0][0] = x1[0]; invFrame1[0][1] = x1[1]; invFrame1[0][2] = x1[2];
	invFrame1[1][0] = y1[0]; invFrame1[1][1] = y1[1]; invFrame1[1][2] = y1[2];
	invFrame1[2][0] = z1[0]; invFrame1[2][1] = z1[1]; invFrame1[2][2] = z1[2];
	// 2. Multiply
	aux::mult3x3 (x2, y2, z2, invFrame1, rot);

	// Construct the rotational part
	aux::copy3x3 (rot, rigid_transform);
	// Construct the translation
	aux::mult3x3 (rot, o1, Ro1);
	rigid_transform[9]  = o2[0] - Ro1[0];
	rigid_transform[10] = o2[1] - Ro1[1];
	rigid_transform[11] = o2[2] - Ro1[2];
}

//=========================================================================================================================================================================
