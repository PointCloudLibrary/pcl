/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012 Aitor Aldoma, Federico Tombari
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
 */

#ifndef PCL_RECOGNITION_IMPL_HV_GO_HPP_
#define PCL_RECOGNITION_IMPL_HV_GO_HPP_

#include <pcl/recognition/hv/hv_go.h>
#include <numeric>
#include <pcl/common/time.h>
#include <pcl/point_types.h>

template<typename PointT, typename NormalT>
inline void extractEuclideanClustersSmooth(const typename pcl::PointCloud<PointT> &cloud, const typename pcl::PointCloud<NormalT> &normals, float tolerance,
    const typename pcl::search::Search<PointT>::Ptr &tree, std::vector<pcl::PointIndices> &clusters, double eps_angle, float curvature_threshold,
    unsigned int min_pts_per_cluster, unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
{

  if (tree->getInputCloud ()->points.size () != cloud.points.size ())
  {
    PCL_ERROR("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset\n");
    return;
  }
  if (cloud.points.size () != normals.points.size ())
  {
    PCL_ERROR("[pcl::extractEuclideanClusters] Number of points in the input point cloud different than normals!\n");
    return;
  }

  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.points.size (), false);

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  // Process all points in the indices vector
  int size = static_cast<int> (cloud.points.size ());
  for (int i = 0; i < size; ++i)
  {
    if (processed[i])
      continue;

    std::vector<unsigned int> seed_queue;
    int sq_idx = 0;
    seed_queue.push_back (i);

    processed[i] = true;

    while (sq_idx < static_cast<int> (seed_queue.size ()))
    {

      if (normals.points[seed_queue[sq_idx]].curvature > curvature_threshold)
      {
        sq_idx++;
        continue;
      }

      // Search for sq_idx
      if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
      {
        sq_idx++;
        continue;
      }

      for (size_t j = 1; j < nn_indices.size (); ++j) // nn_indices[0] should be sq_idx
      {
        if (processed[nn_indices[j]]) // Has this point been processed before ?
          continue;

        if (normals.points[nn_indices[j]].curvature > curvature_threshold)
        {
          continue;
        }

        //processed[nn_indices[j]] = true;
        // [-1;1]

        double dot_p = normals.points[seed_queue[sq_idx]].normal[0] * normals.points[nn_indices[j]].normal[0]
            + normals.points[seed_queue[sq_idx]].normal[1] * normals.points[nn_indices[j]].normal[1]
            + normals.points[seed_queue[sq_idx]].normal[2] * normals.points[nn_indices[j]].normal[2];

        if (fabs (acos (dot_p)) < eps_angle)
        {
          processed[nn_indices[j]] = true;
          seed_queue.push_back (nn_indices[j]);
        }
      }

      sq_idx++;
    }

    // If this queue is satisfactory, add to the clusters
    if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
    {
      pcl::PointIndices r;
      r.indices.resize (seed_queue.size ());
      for (size_t j = 0; j < seed_queue.size (); ++j)
        r.indices[j] = seed_queue[j];

      std::sort (r.indices.begin (), r.indices.end ());
      r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

      r.header = cloud.header;
      clusters.push_back (r); // We could avoid a copy by working directly in the vector
    }
  }
}

template<typename ModelT, typename SceneT>
mets::gol_type pcl::GlobalHypothesesVerification<ModelT, SceneT>::evaluateSolution(const std::vector<bool> & active, int changed)
{
  float sign = 1.f;
  //update explained_by_RM
  if (active[changed])
  {
    //it has been activated
    updateExplainedVector (recognition_models_[changed]->explained_, recognition_models_[changed]->explained_distances_, explained_by_RM_,
        explained_by_RM_distance_weighted, 1.f);
    updateUnexplainedVector (recognition_models_[changed]->unexplained_in_neighborhood, recognition_models_[changed]->unexplained_in_neighborhood_weights,
        unexplained_by_RM_neighboorhods, recognition_models_[changed]->explained_, explained_by_RM_, 1.f);
    updateCMDuplicity(recognition_models_[changed]->complete_cloud_occupancy_indices_, complete_cloud_occupancy_by_RM_, 1.f);
  } else
  {
    //it has been deactivated
    updateExplainedVector (recognition_models_[changed]->explained_, recognition_models_[changed]->explained_distances_, explained_by_RM_,
        explained_by_RM_distance_weighted, -1.f);
    updateUnexplainedVector (recognition_models_[changed]->unexplained_in_neighborhood, recognition_models_[changed]->unexplained_in_neighborhood_weights,
        unexplained_by_RM_neighboorhods, recognition_models_[changed]->explained_, explained_by_RM_, -1.f);
    updateCMDuplicity(recognition_models_[changed]->complete_cloud_occupancy_indices_, complete_cloud_occupancy_by_RM_, -1.f);
    sign = -1.f;
  }

  int duplicity = getDuplicity ();
  float good_info = getExplainedValue ();

  float unexplained_info = getPreviousUnexplainedValue ();
  float bad_info = static_cast<float> (getPreviousBadInfo ())
      + (recognition_models_[changed]->outliers_weight_ * static_cast<float> (recognition_models_[changed]->bad_information_)) * sign;

  setPreviousBadInfo (bad_info);

  int n_active_hyp = 0;
  for(size_t i=0; i < active.size(); i++) {
    if(active[i])
      n_active_hyp++;
  }

  float duplicity_cm = static_cast<float> (getDuplicityCM ()) * w_occupied_multiple_cm_;
  return static_cast<mets::gol_type> ((good_info - bad_info - static_cast<float> (duplicity) - unexplained_info - duplicity_cm - static_cast<float> (n_active_hyp)) * -1.f); //return the dual to our max problem
}

///////////////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
void pcl::GlobalHypothesesVerification<ModelT, SceneT>::initialize()
{
  //clear stuff
  recognition_models_.clear ();
  unexplained_by_RM_neighboorhods.clear ();
  explained_by_RM_distance_weighted.clear ();
  explained_by_RM_.clear ();
  mask_.clear ();
  indices_.clear (),
  complete_cloud_occupancy_by_RM_.clear ();

  // initialize mask to false
  mask_.resize (complete_models_.size ());
  for (size_t i = 0; i < complete_models_.size (); i++)
    mask_[i] = false;

  indices_.resize (complete_models_.size ());

  NormalEstimator_ n3d;
  scene_normals_.reset (new pcl::PointCloud<pcl::Normal> ());

  typename pcl::search::KdTree<SceneT>::Ptr normals_tree (new pcl::search::KdTree<SceneT>);
  normals_tree->setInputCloud (scene_cloud_downsampled_);

  n3d.setRadiusSearch (radius_normals_);
  n3d.setSearchMethod (normals_tree);
  n3d.setInputCloud (scene_cloud_downsampled_);
  n3d.compute (*scene_normals_);

  //check nans...
  int j = 0;
  for (size_t i = 0; i < scene_normals_->points.size (); ++i)
  {
    if (!pcl_isfinite (scene_normals_->points[i].normal_x) || !pcl_isfinite (scene_normals_->points[i].normal_y)
        || !pcl_isfinite (scene_normals_->points[i].normal_z))
      continue;

    scene_normals_->points[j] = scene_normals_->points[i];
    scene_cloud_downsampled_->points[j] = scene_cloud_downsampled_->points[i];

    j++;
  }

  scene_normals_->points.resize (j);
  scene_normals_->width = j;
  scene_normals_->height = 1;

  scene_cloud_downsampled_->points.resize (j);
  scene_cloud_downsampled_->width = j;
  scene_cloud_downsampled_->height = 1;

  explained_by_RM_.resize (scene_cloud_downsampled_->points.size (), 0);
  explained_by_RM_distance_weighted.resize (scene_cloud_downsampled_->points.size (), 0.f);
  unexplained_by_RM_neighboorhods.resize (scene_cloud_downsampled_->points.size (), 0.f);

  //compute segmentation of the scene if detect_clutter_
  if (detect_clutter_)
  {
    //initialize kdtree for search
    scene_downsampled_tree_.reset (new pcl::search::KdTree<SceneT>);
    scene_downsampled_tree_->setInputCloud (scene_cloud_downsampled_);

    std::vector<pcl::PointIndices> clusters;
    double eps_angle_threshold = 0.2;
    int min_points = 20;
    float curvature_threshold = 0.045f;

    extractEuclideanClustersSmooth<SceneT, pcl::Normal> (*scene_cloud_downsampled_, *scene_normals_, inliers_threshold_ * 2.f, scene_downsampled_tree_,
        clusters, eps_angle_threshold, curvature_threshold, min_points);

    clusters_cloud_.reset (new pcl::PointCloud<pcl::PointXYZI>);
    clusters_cloud_->points.resize (scene_cloud_downsampled_->points.size ());
    clusters_cloud_->width = scene_cloud_downsampled_->width;
    clusters_cloud_->height = 1;

    for (size_t i = 0; i < scene_cloud_downsampled_->points.size (); i++)
    {
      pcl::PointXYZI p;
      p.getVector3fMap () = scene_cloud_downsampled_->points[i].getVector3fMap ();
      p.intensity = 0.f;
      clusters_cloud_->points[i] = p;
    }

    float intens_incr = 100.f / static_cast<float> (clusters.size ());
    float intens = intens_incr;
    for (size_t i = 0; i < clusters.size (); i++)
    {
      for (size_t j = 0; j < clusters[i].indices.size (); j++)
      {
        clusters_cloud_->points[clusters[i].indices[j]].intensity = intens;
      }

      intens += intens_incr;
    }
  }

  //compute cues
  {
    pcl::ScopeTime tcues ("Computing cues");
    recognition_models_.resize (complete_models_.size ());
    int valid = 0;
    for (int i = 0; i < static_cast<int> (complete_models_.size ()); i++)
    {
      //create recognition model
      recognition_models_[valid].reset (new RecognitionModel ());
      if(addModel (visible_models_[i], complete_models_[i], recognition_models_[valid])) {
        indices_[valid] = i;
        valid++;
      }
    }

    recognition_models_.resize(valid);
    indices_.resize(valid);
  }

  //compute the bounding boxes for the models
  ModelT min_pt_all, max_pt_all;
  min_pt_all.x = min_pt_all.y = min_pt_all.z = std::numeric_limits<float>::max ();
  max_pt_all.x = max_pt_all.y = max_pt_all.z = (std::numeric_limits<float>::max () - 0.001f) * -1;

  for (size_t i = 0; i < recognition_models_.size (); i++)
  {
    ModelT min_pt, max_pt;
    pcl::getMinMax3D (*complete_models_[indices_[i]], min_pt, max_pt);
    if (min_pt.x < min_pt_all.x)
      min_pt_all.x = min_pt.x;

    if (min_pt.y < min_pt_all.y)
      min_pt_all.y = min_pt.y;

    if (min_pt.z < min_pt_all.z)
      min_pt_all.z = min_pt.z;

    if (max_pt.x > max_pt_all.x)
      max_pt_all.x = max_pt.x;

    if (max_pt.y > max_pt_all.y)
      max_pt_all.y = max_pt.y;

    if (max_pt.z > max_pt_all.z)
      max_pt_all.z = max_pt.z;
  }

  int size_x, size_y, size_z;
  size_x = static_cast<int> (std::ceil (std::abs (max_pt_all.x - min_pt_all.x) / res_occupancy_grid_)) + 1;
  size_y = static_cast<int> (std::ceil (std::abs (max_pt_all.y - min_pt_all.y) / res_occupancy_grid_)) + 1;
  size_z = static_cast<int> (std::ceil (std::abs (max_pt_all.z - min_pt_all.z) / res_occupancy_grid_)) + 1;

  complete_cloud_occupancy_by_RM_.resize (size_x * size_y * size_z, 0);

  for (size_t i = 0; i < recognition_models_.size (); i++)
  {

    std::map<int, bool> banned;
    std::map<int, bool>::iterator banned_it;

    for (size_t j = 0; j < complete_models_[indices_[i]]->points.size (); j++)
    {
      int pos_x, pos_y, pos_z;
      pos_x = static_cast<int> (std::floor ((complete_models_[indices_[i]]->points[j].x - min_pt_all.x) / res_occupancy_grid_));
      pos_y = static_cast<int> (std::floor ((complete_models_[indices_[i]]->points[j].y - min_pt_all.y) / res_occupancy_grid_));
      pos_z = static_cast<int> (std::floor ((complete_models_[indices_[i]]->points[j].z - min_pt_all.z) / res_occupancy_grid_));

      int idx = pos_z * size_x * size_y + pos_y * size_x + pos_x;
      banned_it = banned.find (idx);
      if (banned_it == banned.end ())
      {
        complete_cloud_occupancy_by_RM_[idx]++;
        recognition_models_[i]->complete_cloud_occupancy_indices_.push_back (idx);
        banned[idx] = true;
      }
    }
  }

  {
    pcl::ScopeTime tcues ("Computing clutter cues");
#pragma omp parallel for schedule(dynamic, 4) num_threads(omp_get_num_procs())
    for (int j = 0; j < static_cast<int> (recognition_models_.size ()); j++)
      computeClutterCue (recognition_models_[j]);
  }

  cc_.clear ();
  n_cc_ = 1;
  cc_.resize (n_cc_);
  for (size_t i = 0; i < recognition_models_.size (); i++)
    cc_[0].push_back (static_cast<int> (i));

}

template<typename ModelT, typename SceneT>
void pcl::GlobalHypothesesVerification<ModelT, SceneT>::SAOptimize(std::vector<int> & cc_indices, std::vector<bool> & initial_solution)
{

  //temporal copy of recogniton_models_
  std::vector < boost::shared_ptr<RecognitionModel> > recognition_models_copy;
  recognition_models_copy = recognition_models_;

  recognition_models_.clear ();

  for (size_t j = 0; j < cc_indices.size (); j++)
  {
    recognition_models_.push_back (recognition_models_copy[cc_indices[j]]);
  }

  for (size_t j = 0; j < recognition_models_.size (); j++)
  {
    boost::shared_ptr < RecognitionModel > recog_model = recognition_models_[j];
    for (size_t i = 0; i < recog_model->explained_.size (); i++)
    {
      explained_by_RM_[recog_model->explained_[i]]++;
      explained_by_RM_distance_weighted[recog_model->explained_[i]] += recog_model->explained_distances_[i];
    }

    if (detect_clutter_)
    {
      for (size_t i = 0; i < recog_model->unexplained_in_neighborhood.size (); i++)
      {
        unexplained_by_RM_neighboorhods[recog_model->unexplained_in_neighborhood[i]] += recog_model->unexplained_in_neighborhood_weights[i];
      }
    }
  }

  int occupied_multiple = 0;
  for(size_t i=0; i < complete_cloud_occupancy_by_RM_.size(); i++) {
    if(complete_cloud_occupancy_by_RM_[i] > 1) {
      occupied_multiple+=complete_cloud_occupancy_by_RM_[i];
    }
  }

  setPreviousDuplicityCM(occupied_multiple);
  //do optimization
  //Define model SAModel, initial solution is all models activated

  int duplicity;
  float good_information_ = getTotalExplainedInformation (explained_by_RM_, explained_by_RM_distance_weighted, &duplicity);
  float bad_information_ = 0;
  float unexplained_in_neighboorhod = getUnexplainedInformationInNeighborhood (unexplained_by_RM_neighboorhods, explained_by_RM_);

  for (size_t i = 0; i < initial_solution.size (); i++)
  {
    if (initial_solution[i])
      bad_information_ += recognition_models_[i]->outliers_weight_ * static_cast<float> (recognition_models_[i]->bad_information_);
  }

  setPreviousExplainedValue (good_information_);
  setPreviousDuplicity (duplicity);
  setPreviousBadInfo (bad_information_);
  setPreviousUnexplainedValue (unexplained_in_neighboorhod);

  SAModel model;
  model.cost_ = static_cast<mets::gol_type> ((good_information_ - bad_information_
                                               - static_cast<float> (duplicity)
                                               - static_cast<float> (occupied_multiple) * w_occupied_multiple_cm_
                                               - static_cast<float> (recognition_models_.size ())
                                               - unexplained_in_neighboorhod) * -1.f);

  model.setSolution (initial_solution);
  model.setOptimizer (this);
  SAModel best (model);

  move_manager neigh (static_cast<int> (cc_indices.size ()));

  mets::best_ever_solution best_recorder (best);
  mets::noimprove_termination_criteria noimprove (max_iterations_);
  mets::linear_cooling linear_cooling;
  mets::simulated_annealing<move_manager> sa (model, best_recorder, neigh, noimprove, linear_cooling, initial_temp_, 1e-7, 2);
  sa.setApplyAndEvaluate(true);

  {
    pcl::ScopeTime t ("SA search...");
    sa.search ();
  }

  best_seen_ = static_cast<const SAModel&> (best_recorder.best_seen ());
  for (size_t i = 0; i < best_seen_.solution_.size (); i++)
  {
    initial_solution[i] = best_seen_.solution_[i];
  }

  recognition_models_ = recognition_models_copy;

}

///////////////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
void pcl::GlobalHypothesesVerification<ModelT, SceneT>::verify()
{
  initialize ();

  //for each connected component, find the optimal solution
  for (int c = 0; c < n_cc_; c++)
  {
    //TODO: Check for trivial case...
    //TODO: Check also the number of hypotheses and use exhaustive enumeration if smaller than 10
    std::vector<bool> subsolution (cc_[c].size (), true);
    SAOptimize (cc_[c], subsolution);
    for (size_t i = 0; i < subsolution.size (); i++)
    {
      mask_[indices_[cc_[c][i]]] = (subsolution[i]);
    }
  }
}

template<typename ModelT, typename SceneT>
bool pcl::GlobalHypothesesVerification<ModelT, SceneT>::addModel(typename pcl::PointCloud<ModelT>::ConstPtr & model,
    typename pcl::PointCloud<ModelT>::ConstPtr & complete_model, boost::shared_ptr<RecognitionModel> & recog_model)
{
  //voxelize model cloud
  recog_model->cloud_.reset (new pcl::PointCloud<ModelT> ());
  recog_model->complete_cloud_.reset (new pcl::PointCloud<ModelT> ());

  float size_model = resolution_;
  pcl::VoxelGrid<ModelT> voxel_grid;
  voxel_grid.setInputCloud (model);
  voxel_grid.setLeafSize (size_model, size_model, size_model);
  voxel_grid.filter (*(recog_model->cloud_));

  pcl::VoxelGrid<ModelT> voxel_grid2;
  voxel_grid2.setInputCloud (complete_model);
  voxel_grid2.setLeafSize (size_model, size_model, size_model);
  voxel_grid2.filter (*(recog_model->complete_cloud_));

  {
    //check nans...
    int j = 0;
    for (size_t i = 0; i < recog_model->cloud_->points.size (); ++i)
    {
      if (!pcl_isfinite (recog_model->cloud_->points[i].x) || !pcl_isfinite (recog_model->cloud_->points[i].y)
          || !pcl_isfinite (recog_model->cloud_->points[i].z))
        continue;

      recog_model->cloud_->points[j] = recog_model->cloud_->points[i];
      j++;
    }

    recog_model->cloud_->points.resize (j);
    recog_model->cloud_->width = j;
    recog_model->cloud_->height = 1;
  }

  if (recog_model->cloud_->points.size () <= 0)
  {
    PCL_WARN("The model cloud has no points..\n");
    return false;
  }

  //compute normals unless given (now do it always...)
  typename pcl::search::KdTree<ModelT>::Ptr normals_tree (new pcl::search::KdTree<ModelT>);
  typedef typename pcl::NormalEstimation<ModelT, pcl::Normal> NormalEstimator_;
  NormalEstimator_ n3d;
  recog_model->normals_.reset (new pcl::PointCloud<pcl::Normal> ());
  normals_tree->setInputCloud (recog_model->cloud_);
  n3d.setRadiusSearch (radius_normals_);
  n3d.setSearchMethod (normals_tree);
  n3d.setInputCloud ((recog_model->cloud_));
  n3d.compute (*(recog_model->normals_));

  //check nans...
  int j = 0;
  for (size_t i = 0; i < recog_model->normals_->points.size (); ++i)
  {
    if (!pcl_isfinite (recog_model->normals_->points[i].normal_x) || !pcl_isfinite (recog_model->normals_->points[i].normal_y)
        || !pcl_isfinite (recog_model->normals_->points[i].normal_z))
      continue;

    recog_model->normals_->points[j] = recog_model->normals_->points[i];
    recog_model->cloud_->points[j] = recog_model->cloud_->points[i];
    j++;
  }

  recog_model->normals_->points.resize (j);
  recog_model->normals_->width = j;
  recog_model->normals_->height = 1;

  recog_model->cloud_->points.resize (j);
  recog_model->cloud_->width = j;
  recog_model->cloud_->height = 1;

  std::vector<int> explained_indices;
  std::vector<float> outliers_weight;
  std::vector<float> explained_indices_distances;
  std::vector<float> unexplained_indices_weights;

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;

  std::map<int, boost::shared_ptr<std::vector<std::pair<int, float> > > > model_explains_scene_points; //which point i from the scene is explained by a points j_k with dist d_k from the model
  std::map<int, boost::shared_ptr<std::vector<std::pair<int, float> > > >::iterator it;

  outliers_weight.resize (recog_model->cloud_->points.size ());
  recog_model->outlier_indices_.resize (recog_model->cloud_->points.size ());

  size_t o = 0;
  for (size_t i = 0; i < recog_model->cloud_->points.size (); i++)
  {
    if (!scene_downsampled_tree_->radiusSearch (recog_model->cloud_->points[i], inliers_threshold_, nn_indices, nn_distances, std::numeric_limits<int>::max ()))
    {
      //outlier
      outliers_weight[o] = regularizer_;
      recog_model->outlier_indices_[o] = static_cast<int> (i);
      o++;
    } else
    {
      for (size_t k = 0; k < nn_distances.size (); k++)
      {
        std::pair<int, float> pair = std::make_pair (i, nn_distances[k]); //i is a index to a model point and then distance
        it = model_explains_scene_points.find (nn_indices[k]);
        if (it == model_explains_scene_points.end ())
        {
          boost::shared_ptr < std::vector<std::pair<int, float> > > vec (new std::vector<std::pair<int, float> > ());
          vec->push_back (pair);
          model_explains_scene_points[nn_indices[k]] = vec;
        } else
        {
          it->second->push_back (pair);
        }
      }
    }
  }

  outliers_weight.resize (o);
  recog_model->outlier_indices_.resize (o);

  recog_model->outliers_weight_ = (std::accumulate (outliers_weight.begin (), outliers_weight.end (), 0.f) / static_cast<float> (outliers_weight.size ()));
  if (outliers_weight.size () == 0)
    recog_model->outliers_weight_ = 1.f;

  pcl::IndicesPtr indices_scene (new std::vector<int>);
  //go through the map and keep the closest model point in case that several model points explain a scene point

  int p = 0;

  for (it = model_explains_scene_points.begin (); it != model_explains_scene_points.end (); it++, p++)
  {
    size_t closest = 0;
    float min_d = std::numeric_limits<float>::min ();
    for (size_t i = 0; i < it->second->size (); i++)
    {
      if (it->second->at (i).second > min_d)
      {
        min_d = it->second->at (i).second;
        closest = i;
      }
    }

    float d = it->second->at (closest).second;
    float d_weight = -(d * d / (inliers_threshold_)) + 1;

    //it->first is index to scene point
    //using normals to weight inliers
    Eigen::Vector3f scene_p_normal = scene_normals_->points[it->first].getNormalVector3fMap ();
    Eigen::Vector3f model_p_normal = recog_model->normals_->points[it->second->at (closest).first].getNormalVector3fMap ();
    float dotp = scene_p_normal.dot (model_p_normal) * 1.f; //[-1,1] from antiparallel trough perpendicular to parallel

    if (dotp < 0.f)
      dotp = 0.f;

    explained_indices.push_back (it->first);
    explained_indices_distances.push_back (d_weight * dotp);

  }

  recog_model->bad_information_ = static_cast<int> (recog_model->outlier_indices_.size ());
  recog_model->explained_ = explained_indices;
  recog_model->explained_distances_ = explained_indices_distances;

  return true;
}

template<typename ModelT, typename SceneT>
void pcl::GlobalHypothesesVerification<ModelT, SceneT>::computeClutterCue(boost::shared_ptr<RecognitionModel> & recog_model)
{
  if (detect_clutter_)
  {

    float rn_sqr = radius_neighborhood_GO_ * radius_neighborhood_GO_;
    std::vector<int> nn_indices;
    std::vector<float> nn_distances;

    std::vector < std::pair<int, int> > neighborhood_indices; //first is indices to scene point and second is indices to explained_ scene points
    for (int i = 0; i < static_cast<int> (recog_model->explained_.size ()); i++)
    {
      if (scene_downsampled_tree_->radiusSearch (scene_cloud_downsampled_->points[recog_model->explained_[i]], radius_neighborhood_GO_, nn_indices,
          nn_distances, std::numeric_limits<int>::max ()))
      {
        for (size_t k = 0; k < nn_distances.size (); k++)
        {
          if (nn_indices[k] != i)
            neighborhood_indices.push_back (std::make_pair (nn_indices[k], i));
        }
      }
    }

    //sort neighborhood indices by id
    std::sort (neighborhood_indices.begin (), neighborhood_indices.end (),
        boost::bind (&std::pair<int, int>::first, _1) < boost::bind (&std::pair<int, int>::first, _2));

    //erase duplicated unexplained points
    neighborhood_indices.erase (
        std::unique (neighborhood_indices.begin (), neighborhood_indices.end (),
            boost::bind (&std::pair<int, int>::first, _1) == boost::bind (&std::pair<int, int>::first, _2)), neighborhood_indices.end ());

    //sort explained points
    std::vector<int> exp_idces (recog_model->explained_);
    std::sort (exp_idces.begin (), exp_idces.end ());

    recog_model->unexplained_in_neighborhood.resize (neighborhood_indices.size ());
    recog_model->unexplained_in_neighborhood_weights.resize (neighborhood_indices.size ());

    size_t p = 0;
    size_t j = 0;
    for (size_t i = 0; i < neighborhood_indices.size (); i++)
    {
      if ((j < exp_idces.size ()) && (neighborhood_indices[i].first == exp_idces[j]))
      {
        //this index is explained by the hypothesis so ignore it, advance j
        j++;
      } else
      {
        //indices_in_nb[i] < exp_idces[j]
        //recog_model->unexplained_in_neighborhood.push_back(neighborhood_indices[i]);
        recog_model->unexplained_in_neighborhood[p] = neighborhood_indices[i].first;

        if (clusters_cloud_->points[recog_model->explained_[neighborhood_indices[i].second]].intensity != 0.f
            && (clusters_cloud_->points[recog_model->explained_[neighborhood_indices[i].second]].intensity
                == clusters_cloud_->points[neighborhood_indices[i].first].intensity))
        {

          recog_model->unexplained_in_neighborhood_weights[p] = clutter_regularizer_;

        } else
        {
          //neighborhood_indices[i].first gives the index to the scene point and second to the explained scene point by the model causing this...
          //calculate weight of this clutter point based on the distance of the scene point and the model point causing it
          float d = static_cast<float> (pow (
              (scene_cloud_downsampled_->points[recog_model->explained_[neighborhood_indices[i].second]].getVector3fMap ()
                  - scene_cloud_downsampled_->points[neighborhood_indices[i].first].getVector3fMap ()).norm (), 2));
          float d_weight = -(d / rn_sqr) + 1; //points that are close have a strong weight*/

          //using normals to weight clutter points
          Eigen::Vector3f scene_p_normal = scene_normals_->points[neighborhood_indices[i].first].getNormalVector3fMap ();
          Eigen::Vector3f model_p_normal = scene_normals_->points[recog_model->explained_[neighborhood_indices[i].second]].getNormalVector3fMap ();
          float dotp = scene_p_normal.dot (model_p_normal); //[-1,1] from antiparallel trough perpendicular to parallel

          if (dotp < 0)
            dotp = 0.f;

          recog_model->unexplained_in_neighborhood_weights[p] = d_weight * dotp;
        }
        p++;
      }
    }

    recog_model->unexplained_in_neighborhood_weights.resize (p);
    recog_model->unexplained_in_neighborhood.resize (p);
  }
}

#define PCL_INSTANTIATE_GoHV(T1,T2) template class PCL_EXPORTS pcl::GlobalHypothesesVerification<T1,T2>;

#endif /* PCL_RECOGNITION_IMPL_HV_GO_HPP_ */

