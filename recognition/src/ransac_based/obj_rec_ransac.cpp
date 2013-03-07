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

#include <pcl/recognition/ransac_based/obj_rec_ransac.h>

using namespace std;
using namespace pcl::common;

pcl::recognition::ObjRecRANSAC::ObjRecRANSAC (float pair_width, float voxel_size)
: pair_width_ (pair_width),
  voxel_size_ (voxel_size),
  position_discretization_ (5.0f*voxel_size_),
  rotation_discretization_ (5.0f*AUX_DEG_TO_RADIANS),
  abs_zdist_thresh_ (1.5f*voxel_size_),
  relative_obj_size_ (0.05f),
  visibility_ (0.06f),
  relative_num_of_illegal_pts_ (0.02f),
  intersection_fraction_ (0.03f),
  max_coplanarity_angle_ (3.0f*AUX_DEG_TO_RADIANS),
  scene_bounds_enlargement_factor_ (0.25f), // 25% enlargement
  ignore_coplanar_opps_ (true),
  model_library_ (pair_width, voxel_size, max_coplanarity_angle_),
  rec_mode_ (ObjRecRANSAC::FULL_RECOGNITION)
{
}

//===============================================================================================================================================

void
pcl::recognition::ObjRecRANSAC::recognize (const PointCloudIn& scene, const PointCloudN& normals, list<ObjRecRANSAC::Output>& recognized_objects, double success_probability)
{
  // Clear some stuff
  this->clearTestData ();

  // Build the scene octree
  scene_octree_.build(scene, voxel_size_, &normals);
  // Project it on the xy-plane (which roughly corresponds to the projection plane of the scanning device)
  scene_octree_proj_.build(scene_octree_, abs_zdist_thresh_, abs_zdist_thresh_);

  if ( success_probability >= 1.0 )
    success_probability = 0.99;

  // Compute the number of iterations
  vector<ORROctree::Node*>& full_leaves = scene_octree_.getFullLeaves();
  int num_iterations = this->computeNumberOfIterations(success_probability), num_full_leaves = static_cast<int> (full_leaves.size ());

  // Make sure that there are not more iterations than full leaves
  if ( num_iterations > num_full_leaves )
    num_iterations = num_full_leaves;

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("ObjRecRANSAC::%s(): recognizing objects [%i iteration(s)]\n", __func__, num_iterations);
#endif

  // First, sample oriented point pairs (opps)
  this->sampleOrientedPointPairs (num_iterations, full_leaves, sampled_oriented_point_pairs_);

  // Leave if we are in the SAMPLE_OPP test mode
  if ( rec_mode_ == ObjRecRANSAC::SAMPLE_OPP )
    return;

  // Generate hypotheses from the sampled opps
  list<HypothesisBase> pre_hypotheses;
  int num_hypotheses = this->generateHypotheses (sampled_oriented_point_pairs_, pre_hypotheses);

  // Cluster the hypotheses
  vector<RotationSpace<Hypothesis>*> rot_spaces;
  num_hypotheses = this->groupHypotheses (pre_hypotheses, num_hypotheses, transform_space_, rot_spaces);
  pre_hypotheses.clear ();

  // The last graph-based steps in the algorithm
  ORRGraph<Hypothesis> graph_of_close_hypotheses;
  this->buildGraphOfCloseHypotheses (transform_space_, rot_spaces, graph_of_close_hypotheses);
  this->filterGraphOfCloseHypotheses (graph_of_close_hypotheses, accepted_hypotheses_);
  graph_of_close_hypotheses.clear ();

  // Leave if we are in the TEST_HYPOTHESES mode
  if ( rec_mode_ == ObjRecRANSAC::TEST_HYPOTHESES )
    return;

  // Create and initialize a vector of bounded objects needed for the bounding volume hierarchy (BVH)
  vector<BVHH::BoundedObject*> bounded_objects (accepted_hypotheses_.size ());
  int i = 0;

  // Initialize the vector with bounded objects
  for ( vector<Hypothesis>::iterator hypo = accepted_hypotheses_.begin () ; hypo != accepted_hypotheses_.end () ; ++hypo, ++i )
  {
    // Create, initialize and save a bounded object based on the hypothesis
    BVHH::BoundedObject *bounded_object = new BVHH::BoundedObject (&(*hypo));
    hypo->computeCenterOfMass (bounded_object->getCentroid ());
    hypo->computeBounds (bounded_object->getBounds ());
    bounded_objects[i] = bounded_object;
  }

  // Build a bounding volume hierarchy (BVH) which is used to accelerate the hypothesis intersection tests
  BVHH bvh;
  bvh.build (bounded_objects);

  ORRGraph<Hypothesis*> conflict_graph;
  this->buildGraphOfConflictingHypotheses (bvh, conflict_graph);
  this->filterGraphOfConflictingHypotheses (conflict_graph, recognized_objects);

  // Cleanup
  this->clearTestData ();

#ifdef OBJ_REC_RANSAC_VERBOSE
    printf("ObjRecRANSAC::%s(): done [%i object(s)]\n", __func__, static_cast<int> (recognized_objects.size ()));
#endif
}

//===============================================================================================================================================

void
pcl::recognition::ObjRecRANSAC::sampleOrientedPointPairs (int num_iterations, const vector<ORROctree::Node*>& full_scene_leaves,
    list<OrientedPointPair>& output) const
{
#ifdef OBJ_REC_RANSAC_VERBOSE
  printf ("ObjRecRANSAC::%s(): sampling oriented point pairs (opps) ... ", __func__); fflush (stdout);
#endif

  int i, num_full_leaves = static_cast<int> (full_scene_leaves.size ());
  ORROctree::Node *leaf1, *leaf2;
  const float *p1, *p2, *n1, *n2;

#ifdef OBJ_REC_RANSAC_VERBOSE
  int num_of_opps = 0;
#endif

  // The random generator
//  UniformGenerator<int> randgen (0, num_full_leaves, static_cast<uint32_t> (time (NULL)));

  // Init the vector with the ids
  vector<int> ids (num_full_leaves);
  for ( i = 0 ; i < num_full_leaves ; ++i )
    ids[i] = i;

  // Sample 'num_iterations' number of oriented point pairs
  for ( i = 0 ; i < num_iterations ; ++i )
  {
    // Choose a random position within the array of ids
//    randgen.setParameters (0, static_cast<int> (ids.size ()));
//    int rand_pos = randgen.run ();
    int rand_pos = aux::getRandomInteger (0, static_cast<int> (ids.size ()) - 1);

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

    if ( aux::pointsAreCoplanar (p1, n1, p2, n2, max_coplanarity_angle_) )
      continue;

    // Save the sampled point pair
    output.push_back (OrientedPointPair (p1, n1, p2, n2));

#ifdef OBJ_REC_RANSAC_VERBOSE
    ++num_of_opps;
#endif
  }

#ifdef OBJ_REC_RANSAC_VERBOSE
  cout << "done [" << num_of_opps << " opps].\n";
#endif
}

//===============================================================================================================================================

int
pcl::recognition::ObjRecRANSAC::generateHypotheses (const list<OrientedPointPair>& pairs, list<HypothesisBase>& out) const
{
  // Only for 3D hash tables: this is the max number of neighbors a 3D hash table cell can have!
  ModelLibrary::HashTableCell *neigh_cells[27];
  ModelLibrary::HashTableCell::iterator cell_it;
  float hash_table_key[3];
  const float *model_p1, *model_n1, *model_p2, *model_n2;
  const float *scene_p1, *scene_n1, *scene_p2, *scene_n2;
  int i, num_hypotheses = 0;
  const ModelLibrary::Model* obj_model;

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("ObjRecRANSAC::%s(): generating hypotheses ... ", __func__); fflush (stdout);
#endif

  for ( list<OrientedPointPair>::const_iterator pair = pairs.begin () ; pair != pairs.end () ; ++pair )
  {
    // Just to make the code more readable
    scene_p1 = (*pair).p1_;
    scene_n1 = (*pair).n1_;
    scene_p2 = (*pair).p2_;
    scene_n2 = (*pair).n2_;

    // Use normals and points to compute a hash table key
    this->compute_oriented_point_pair_signature (scene_p1, scene_n1, scene_p2, scene_n2, hash_table_key);
    // Get the cell and its neighbors based on 'key'
    int num_neigh_cells = model_library_.getHashTable ().getNeighbors (hash_table_key, neigh_cells);

    for ( i = 0 ; i < num_neigh_cells ; ++i )
    {
      // Check for all models in the current cell
      for ( cell_it = neigh_cells[i]->begin () ; cell_it != neigh_cells[i]->end () ; ++cell_it )
      {
        // For better code readability
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

          HypothesisBase hypothesis(obj_model);
          // Get the rigid transform from model to scene
          this->computeRigidTransform(model_p1, model_n1, model_p2, model_n2, scene_p1, scene_n1, scene_p2, scene_n2, hypothesis.rigid_transform_);
          // Save the current object hypothesis
          out.push_back(hypothesis);
          ++num_hypotheses;
        }
      }
    }
  }
#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("%i hypotheses\n", num_hypotheses);
#endif

  return (num_hypotheses);
}

//===============================================================================================================================================

int
pcl::recognition::ObjRecRANSAC::groupHypotheses(const list<HypothesisBase>& hypotheses, int num_hypotheses,
    RigidTransformSpace<Hypothesis>& transform_space, vector<RotationSpace<Hypothesis>*>& rot_spaces) const
{
#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("ObjRecRANSAC::%s():\n  grouping %i hypotheses ... ", __func__, num_hypotheses); fflush (stdout);
#endif

  // Compute the bounds for the positional discretization
  float b[6]; scene_octree_.getBounds (b);
  float enlr = scene_bounds_enlargement_factor_*std::max (std::max (b[1]-b[0], b[3]-b[2]), b[5]-b[4]);
  b[0] -= enlr; b[1] += enlr;
  b[2] -= enlr; b[3] += enlr;
  b[4] -= enlr; b[5] += enlr;

  // Build the rigid transform space
  transform_space.build (b, position_discretization_, rotation_discretization_);
  float transformed_point[3];

  // First, add all rigid transforms to the discrete rigid transform space
  for ( list<HypothesisBase>::const_iterator hypo_it = hypotheses.begin () ; hypo_it != hypotheses.end () ; ++hypo_it )
  {
    // First transform the center of mass of the model
    aux::transform (hypo_it->rigid_transform_, hypo_it->obj_model_->getOctreeCenterOfMass (), transformed_point);
    // Now add the rigid transform at the right place
    transform_space.addRigidTransform (hypo_it->obj_model_, transformed_point, hypo_it->rigid_transform_);
  }

  list<RotationSpace<Hypothesis>*>& rotation_spaces = transform_space.getRotationSpaces ();
  ObjRecRANSAC::Hypothesis hypothesis;
  int penalty, penalty_thresh, num_accepted = 0;
  float match, match_thresh, num_full_leaves;

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("done\n  testing the cluster representatives ...\n", __func__); fflush (stdout);
  // These are some variables needed when printing the recognition progress
  float progress_factor = 100.0f/static_cast<float> (transform_space.getNumberOfOccupiedRotationSpaces ());
  int num_done = 0;
#endif

  // Now take the best hypothesis from each rotation space
  for ( list<RotationSpace<Hypothesis>*>::iterator rs_it = rotation_spaces.begin () ; rs_it != rotation_spaces.end () ; ++rs_it )
  {
    list<Cell*>& cells = (*rs_it)->getFullCells ();
    ObjRecRANSAC::Hypothesis *best_hypothesis = NULL;

    // Run through the cells and take the best hypothesis
    for ( list<Cell*>::iterator cell_it = cells.begin () ; cell_it != cells.end () ; ++cell_it )
    {
      // For better code readability
      map<const ModelLibrary::Model*, Cell::Entry>& entries = (*cell_it)->getEntries ();
      map<const ModelLibrary::Model*, Cell::Entry>::iterator entry_it;

      // Setup the ids
      hypothesis.setPositionId ((*cell_it)->getPositionId ());
      hypothesis.setRotationId ((*cell_it)->getRotationId ());

      // Run through all entries
      for ( entry_it = entries.begin () ; entry_it != entries.end () ; ++entry_it )
      {
        // Construct a hypothesis
        entry_it->second.computeAverageRigidTransform (hypothesis.rigid_transform_);
        hypothesis.obj_model_ = entry_it->first;
        hypothesis.explained_pixels_.clear ();

        // Now that we constructed a hypothesis -> test it
        this->testHypothesis (&hypothesis, match, penalty);

        // For better code readability
        num_full_leaves = static_cast<float> (hypothesis.obj_model_->getOctree ().getFullLeaves ().size ());
        match_thresh = num_full_leaves*visibility_;
        penalty_thresh = static_cast<int> (num_full_leaves*relative_num_of_illegal_pts_ + 0.5f);

        // Check if this hypothesis is OK
        if ( match >= match_thresh && penalty <= penalty_thresh )
        {
          if ( best_hypothesis == NULL )
          {
            best_hypothesis = new ObjRecRANSAC::Hypothesis (hypothesis);
            best_hypothesis->match_confidence_ = static_cast<float> (match)/num_full_leaves;
          }
          else if ( hypothesis.explained_pixels_.size () > best_hypothesis->explained_pixels_.size () )
          {
            *best_hypothesis = hypothesis;
            best_hypothesis->match_confidence_ = static_cast<float> (match)/num_full_leaves;
          }
        }
      }
    }

    // Save the best hypothesis in the rotation space and in the output list
    if ( best_hypothesis )
    {
      (*rs_it)->setData (*best_hypothesis);
      rot_spaces.push_back (*rs_it);
      ++num_accepted;
      delete best_hypothesis;
    }

#ifdef OBJ_REC_RANSAC_VERBOSE
    // Update the progress
    printf ("\r  %.1f%% ", (static_cast<float> (++num_done))*progress_factor); fflush (stdout);
#endif
  }

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf("done\n  %i accepted.\n", num_accepted);
#endif

  return (num_accepted);
}

//===============================================================================================================================================

void
pcl::recognition::ObjRecRANSAC::buildGraphOfCloseHypotheses (const RigidTransformSpace<Hypothesis>& transform_space,
    const vector<RotationSpace<Hypothesis>*> rotation_spaces, ORRGraph<Hypothesis>& graph) const
{
#ifdef OBJ_REC_RANSAC_VERBOSE
  printf ("ObjRecRANSAC::%s(): building the graph ... ", __func__); fflush (stdout);
#endif

  int i = 0;

  graph.resize (static_cast<int> (rotation_spaces.size ()));

  for ( vector<RotationSpace<Hypothesis>*>::const_iterator rs = rotation_spaces.begin () ; rs != rotation_spaces.end () ; ++rs, ++i )
    (*rs)->setLinearId (i);

  i = 0;

  // Now create the graph connectivity such that each two neighboring rotation spaces are neighbors in the graph
  for ( vector<RotationSpace<Hypothesis>*>::const_iterator rs = rotation_spaces.begin () ; rs != rotation_spaces.end () ; ++rs, ++i )
  {
    // Compute the fitness of the graph node
    graph.getNodes ()[i]->setFitness (static_cast<int> ((*rs)->getData ().explained_pixels_.size ()));
    graph.getNodes ()[i]->setData ((*rs)->getData ());

    // Get the neighbors of the current rotation space
    list<RotationSpace<Hypothesis>*> neighbors;
    transform_space.getNeighbors ((*rs)->getPositionId (), neighbors);

    for ( list<RotationSpace<Hypothesis>*>::iterator n = neighbors.begin() ; n != neighbors.end() ; ++n )
    {
      // Check if that neighbor has a valid hypothesis
      if ( (*n)->getData ().match_confidence_ < 0.0 )
        continue;

      graph.insertDirectedEdge ((*rs)->getLinearId (), (*n)->getLinearId ());
    }
  }

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf ("done\n");
#endif
}

//===============================================================================================================================================

void
pcl::recognition::ObjRecRANSAC::filterGraphOfCloseHypotheses (ORRGraph<Hypothesis>& graph, std::vector<Hypothesis>& out) const
{
#ifdef OBJ_REC_RANSAC_VERBOSE
  printf ("ObjRecRANSAC::%s(): building the graph ... ", __func__); fflush (stdout);
#endif

  list<ORRGraph<Hypothesis>::Node*> on_nodes, off_nodes;
  graph.computeMaximalOnOffPartition (on_nodes, off_nodes);

  // Copy the data from the on_nodes to the list 'out'
  for ( list<ORRGraph<Hypothesis>::Node*>::iterator it = on_nodes.begin () ; it != on_nodes.end () ; ++it )
    out.push_back ((*it)->getData ());

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf ("done [%i remaining hypotheses]\n", static_cast<int> (out.size ()));
#endif
}

//===============================================================================================================================================

void
pcl::recognition::ObjRecRANSAC::buildGraphOfConflictingHypotheses (const BVHH& bvh, ORRGraph<Hypothesis*>& graph) const
{
//  int score;
//  ORROctreeZProjection::Pixel* pixel;
//  const float *rigid_transform;
//  float transformed_point[3];

#ifdef OBJ_REC_RANSAC_VERBOSE
  printf ("ObjRecRANSAC::%s(): building the conflict graph ... ", __func__); fflush (stdout);
#endif

  // todo: remove pixel->hypotheses_ids_ ZProj::Pixel

  const vector<BVHH::BoundedObject*>& bounded_objects = bvh.getInputObjects ();
  int lin_id = 0;

  for ( vector<BVHH::BoundedObject*>::const_iterator obj = bounded_objects.begin () ; obj != bounded_objects.end () ; ++obj, ++lin_id )
    (*obj)->getData ()->setLinearId (lin_id);

  // There are as many graph nodes as hypotheses
  graph.resize (static_cast<int> (bounded_objects.size ()));
//  vector<ORRGraph<Hypothesis*>::Node*>& graph_nodes = graph.getNodes ();

  // Project the hypotheses onto the "range image" and store in each pixel the corresponding hypothesis id
  for ( vector<BVHH::BoundedObject*>::const_iterator obj = bounded_objects.begin () ; obj != bounded_objects.end () ; ++obj )
  {
    // Get the bounds of the current hypothesis
    float bounds[6];
    (*obj)->getData ()->computeBounds (bounds);

    list<BVHH::BoundedObject*> intersected_objects;

    // Check if these bounds intersect other hypotheses' bounds
    bvh.intersect (bounds, intersected_objects);

    for ( list<BVHH::BoundedObject*>::iterator it = intersected_objects.begin () ; it != intersected_objects.end () ; ++it )
      ;
#if 0
    const vector<ORROctree::Node*>& full_model_leaves = hypo_it->obj_model_->getOctree ().getFullLeaves ();
    rigid_transform = hypo_it->rigid_transform_;

    // The i-th node corresponds to the i-th hypothesis and has id 'i'
    graph_nodes[hypothesis_id]->setData (*hypo_it);
    graph_nodes[hypothesis_id]->setId (hypothesis_id);

    // At the end of the next loop this will be the number of model points ending up in some range image pixel
    score = 0;

    for ( vector<ORROctree::Node*>::const_iterator leaf_it = full_model_leaves.begin () ; leaf_it != full_model_leaves.end () ; ++leaf_it )
    {
      // Transform the model point with the current rigid transform
      aux::transform (rigid_transform, (*leaf_it)->getData ()->getPoint (), transformed_point);

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
#endif
  }

#if 0
  list<ORROctreeZProjection::Pixel*>& pixels = scene_octree_proj_.getFullPixels ();
  set<int>::iterator id1, id2, last_id;

  // Now, iterate through all pixels and build the conflict graph, i.e., create its connectivity
  for ( list<ORROctreeZProjection::Pixel*>::iterator it = pixels.begin () ; it != pixels.end () ; ++it )
  {
    // For better code readability
    pixel = *it;

    // Go to the next pixel if the current one has no hypotheses in it
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
        graph.insertUndirectedEdge (*id1, *id2);
    }
  }

  ORRGraph<Hypothesis>::Node *node;
  set<int> id_intersection;
  float frac_1, frac_2;

  // Now, that we have the graph connectivity, we want to check if each two neighbors are
  // really in conflict. This requires set intersection operations which are expensive,
  // that's why we are performing them now, and not during the computation of the graph connectivity
  for ( vector<ORRGraph<Hypothesis>::Node*>::iterator it = graph_nodes.begin () ; it != graph_nodes.end () ; ++it )
  {
    node = *it;
    for ( set<ORRGraph<Hypothesis>::Node*>::const_iterator neigh = node->getNeighbors ().begin () ; neigh != node->getNeighbors ().end () ; ++neigh )
    {
      id_intersection.clear ();
      // Compute the ids intersection of 'node' and its neighbor
      std::set_intersection (node->getData ().explained_pixels_.begin (),
                             node->getData ().explained_pixels_.end (),
                             (*neigh)->getData ().explained_pixels_.begin (),
                             (*neigh)->getData ().explained_pixels_.end (),
                             std::inserter (id_intersection, id_intersection.begin ()));

      frac_1 = static_cast<float> (id_intersection.size ())/static_cast <float> (node->getData ().explained_pixels_.size ());
      frac_2 = static_cast<float> (id_intersection.size ())/static_cast <float> ((*neigh)->getData ().explained_pixels_.size ());
      // Check if the intersection set is large enough, i.e., if there is a conflict
      if ( frac_1 <= intersection_fraction_ && frac_2 <= intersection_fraction_ )
        // The intersection set is too small => no conflict, detach these two neighboring nodes
        graph.deleteUndirectedEdge (node->getId (), (*neigh)->getId ());
    }
  }
#endif

#ifdef OBJ_REC_RANSAC_VERBOSE
	printf("done\n");
#endif
}

//===============================================================================================================================================

void
pcl::recognition::ObjRecRANSAC::filterGraphOfConflictingHypotheses (ORRGraph<Hypothesis*>& graph, list<ObjRecRANSAC::Output>& recognized_objects) const
{
  vector<ORRGraph<Hypothesis*>::Node*> &nodes = graph.getNodes ();
  size_t num_of_explained;

  // Compute the penalty for each graph node
  for ( vector<ORRGraph<Hypothesis*>::Node*>::iterator it = nodes.begin () ; it != nodes.end () ; ++it )
  {
    num_of_explained = 0;

    // Accumulate the number of pixels the neighbors are explaining
    for ( set<ORRGraph<Hypothesis*>::Node*>::const_iterator neigh = (*it)->getNeighbors ().begin () ; neigh != (*it)->getNeighbors ().end () ; ++neigh )
      num_of_explained += (*neigh)->getData ()->explained_pixels_.size ();

    // Now compute the fitness for the node
    (*it)->setFitness (static_cast<int> ((*it)->getData ()->explained_pixels_.size ()) - static_cast<int> (num_of_explained));
  }

  // Leave the fitest leaves on, such that there are no neighboring ON nodes
  list<ORRGraph<Hypothesis*>::Node*> on_nodes, off_nodes;
  graph.computeMaximalOnOffPartition (on_nodes, off_nodes);

  // The ON nodes correspond to accepted solutions
  for ( list<ORRGraph<Hypothesis*>::Node*>::iterator it = on_nodes.begin () ; it != on_nodes.end () ; ++it )
  {
    recognized_objects.push_back (
      ObjRecRANSAC::Output ((*it)->getData ()->obj_model_->getObjectName (),
                            (*it)->getData ()->rigid_transform_,
                            (*it)->getData ()->match_confidence_,
                            (*it)->getData ()->obj_model_->getUserData ())
    );
  }
}

//===============================================================================================================================================
