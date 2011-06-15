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
 * $Id$
 *
 */

#ifndef PCL_FEATURES_IMPL_SURFLET_HPP_
#define PCL_FEATURES_IMPL_SURFLET_HPP_

#include "pcl/features/surflet.h"
#include <pcl/features/pfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

//// need to be removed - just for debugging purposes
#include <iostream>
using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::SurfletEstimation<PointInT, PointOutT>::computeSurfletModel (const pcl::PointCloud<PointInT> &cloud,
                                                                  typename pcl::SurfletEstimation<PointInT, PointOutT>::SurfletModel &surflet_model)
{
  cerr << "Computing Surflet Model function called" << endl;

  cerr << "Subsampling ..." << endl;
  pcl::PointCloud<PointInT> cloud_subsampled;
  pcl::VoxelGrid<PointInT> subsampling_filter;
  /// @todo don't make a copy here - use input_ from feature
  PointCloudIn cloud_ptr = pcl::PointCloud<PointInT> (cloud).makeShared ();
  subsampling_filter.setInputCloud (cloud_ptr);
  subsampling_filter.setLeafSize (subsampling_leaf_size.x (), subsampling_leaf_size.y (), subsampling_leaf_size.z ());
  subsampling_filter.filter (cloud_subsampled);
  cerr << "After subsampling model, points: " << cloud_subsampled.width << " / " << cloud.width << endl;

  cerr << "Estimating normals ..." << endl;
  pcl::PointCloud<Normal> cloud_subsampled_normals;
  /// recompute normals of the subsampled surfaces
  pcl::NormalEstimation<PointInT, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_subsampled.makeShared());

  typename pcl::KdTreeFLANN<PointInT>::Ptr search_tree (new pcl::KdTreeFLANN<PointInT>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch (normal_estimation_search_radius);
  normal_estimation_filter.compute (cloud_subsampled_normals);

  cerr << "Computing feature hash map and alpha_m values ..." << endl;
  /// initialize alpha_m matrix
  surflet_model.alpha_m.clear();
  surflet_model.alpha_m = std::vector <std::vector <float> > (cloud_subsampled.points.size (), std::vector<float> (cloud_subsampled.points.size ()));

  /// compute feature vector for each pair of points in the subsampled point cloud
  float f1, f2, f3, f4;
  int d1, d2, d3, d4;
  surflet_model.feature_hash_map = FeatureHashMapTypePtr (new FeatureHashMapType);
  for (size_t i = 0; i < cloud_subsampled.points.size (); ++i)
  {
    for (size_t j = 0; j < cloud_subsampled.points.size (); ++j)
    {
      if (i == j)
        continue;

      // Use Eigen aligned maps directly to avoid data copies
      if (pcl::computePairFeatures (cloud_subsampled.points[i].getVector4fMap (), 
                                    cloud_subsampled_normals.points[i].getNormalVector4fMap (),
                                    cloud_subsampled.points[j].getVector4fMap (),
                                    cloud_subsampled_normals.points[j].getNormalVector4fMap (),
                                    f1, f2, f3, f4)) 
      {
        /// discretize feature vector
        d1 = floor (f1 / angle_discretization_step);
        d2 = floor (f2 / angle_discretization_step);
        d3 = floor (f3 / angle_discretization_step);
        d4 = floor (f4 / distance_discretization_step);

        /// add feature to hash map
        surflet_model.feature_hash_map->insert (std::pair<HashKeyStruct, pair<size_t, size_t> > (HashKeyStruct (d1, d2, d3, d4), pair<size_t, size_t> (i, j)));

        /// calculate alpha_m angle
        Eigen::Vector3f model_reference_point = cloud_subsampled.points[i].getVector3fMap (),
            model_reference_normal = cloud_subsampled_normals.points[i].getNormalVector3fMap (),
            model_point = cloud_subsampled.points[j].getVector3fMap ();
        Eigen::AngleAxisf rotation_mg (acos (model_reference_normal.dot (Eigen::Vector3f::UnitX ())),
                                       model_reference_normal.cross (Eigen::Vector3f::UnitX ()).normalized ());
        Eigen::Affine3f transform_mg = Eigen::Translation3f ( rotation_mg * ((-1) * model_reference_point)) * rotation_mg;
        surflet_model.alpha_m[i][j] = acos (Eigen::Vector3f::UnitY ().dot ((transform_mg * model_point).normalized ()));
      }
      else PCL_ERROR ("Computing pair feature vector between points %zu and %zu went wrong.\n", i, j);
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> bool
pcl::SurfletEstimation<PointInT, PointOutT>::poseWithVotesCompareFunction (const typename pcl::SurfletEstimation<PointInT, PointOutT>::PoseWithVotes &a,
                                                                           const typename pcl::SurfletEstimation<PointInT, PointOutT>::PoseWithVotes &b )
{
  return (a.votes > b.votes);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> bool
pcl::SurfletEstimation<PointInT, PointOutT>::clusterVotesCompareFunction (const std::pair<size_t, unsigned int> &a,
                                                                          const std::pair<size_t, unsigned int> &b)
{
  return (a.second > b.second);
}


//////////////////////////////////////////////////////////////////////////////////////////////
/// @TODO !!! cloud_model should be subsampled as before!!
/// @todo very ugly hack with PointOutT = normals
template <typename PointInT, typename PointOutT> void
pcl::SurfletEstimation<PointInT, PointOutT>::registerModelToScene (const pcl::PointCloud<PointInT> &cloud_model,
                                                                   const pcl::PointCloud<PointOutT> &cloud_model_normals,
                                                                   const pcl::PointCloud<PointInT> &cloud_scene,
                                                                   typename pcl::SurfletEstimation<PointInT, PointOutT>::SurfletModel &surflet_model,
                                                                   typename pcl::SurfletEstimation<PointInT, PointOutT>::PoseWithVotesList &results)
{
  PoseWithVotesList voted_poses;
  std::vector <std::vector <unsigned int> > accumulator_array;
  accumulator_array.resize (cloud_model.points.size ());
  for (size_t i = 0; i < cloud_model.points.size (); ++i)
  {
    std::vector <unsigned int> aux ((size_t)floor(2*M_PI / angle_discretization_step ), 0);
    accumulator_array[i] = aux;
  }
  cerr << "Accumulator array size: " << accumulator_array.size () << " x " << accumulator_array.back ().size () << endl;

  /// subsample scene cloud with same rate as the model cloud
  pcl::PointCloud<PointInT> cloud_scene_subsampled;
  pcl::VoxelGrid<PointInT> subsampling_filter;
  /// @todo don't make a copy here
  PointCloudIn cloud_scene_ptr = pcl::PointCloud<PointInT> (cloud_scene).makeShared ();
  subsampling_filter.setInputCloud (cloud_scene_ptr);
  subsampling_filter.setLeafSize (subsampling_leaf_size.x (), subsampling_leaf_size.y (), subsampling_leaf_size.z ());
  subsampling_filter.filter (cloud_scene_subsampled);
  cerr << "Scene cloud after subsampling: " << cloud_scene_subsampled.points.size () << " / " << cloud_scene.points.size () << endl;

  /// calculate subsampled scene cloud normals
  pcl::PointCloud<Normal> cloud_scene_subsampled_normals;
  /// recompute normals of the subsampled surfaces
  pcl::NormalEstimation<PointInT, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_scene_subsampled.makeShared());
  typename pcl::KdTreeFLANN<PointInT>::Ptr search_tree (new pcl::KdTreeFLANN<PointInT>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch (normal_estimation_search_radius);
  normal_estimation_filter.compute (cloud_scene_subsampled_normals);


  /// consider every <scene_reference_point_sampling_rate>-th point as the reference point => fix s_r
  float f1, f2, f3, f4;
  int d1, d2, d3, d4;
  for (size_t scene_reference_index = 0; scene_reference_index < cloud_scene_subsampled.width; scene_reference_index += scene_reference_point_sampling_rate)
  {
    Eigen::Vector3f scene_reference_point = cloud_scene_subsampled.points[scene_reference_index].getVector3fMap (),
        scene_reference_normal = cloud_scene_subsampled_normals.points[scene_reference_index].getNormalVector3fMap ();

    Eigen::AngleAxisf rotation_sg (acos (scene_reference_normal.dot (Eigen::Vector3f::UnitX ())),
                                   scene_reference_normal.cross (Eigen::Vector3f::UnitX ()). normalized());
    Eigen::Affine3f transform_sg = Eigen::Translation3f ( rotation_sg* ((-1)*scene_reference_point)) * rotation_sg;

    /// @todo optimization - search only in max_dist found in the model point cloud
    /// for every other point in the scene => now have pair (s_r, s_i) fixed
    for (size_t scene_point_index = 0; scene_point_index < cloud_scene_subsampled.width; ++ scene_point_index)
      if (scene_reference_index != scene_point_index)
      {
        if (pcl::computePairFeatures (
                                      cloud_scene_subsampled.points[scene_reference_index].getVector4fMap (),
                                      cloud_scene_subsampled_normals.points[scene_reference_index].getNormalVector4fMap (),
                                      cloud_scene_subsampled.points[scene_point_index].getVector4fMap (),
                                      cloud_scene_subsampled_normals.points[scene_point_index].getNormalVector4fMap (),
                                      f1, f2, f3, f4))
        {
          /// discretize feature vector
          d1 = floor(f1 / angle_discretization_step);
          d2 = floor(f2 / angle_discretization_step);
          d3 = floor(f3 / angle_discretization_step);
          d4 = floor(f4 / distance_discretization_step);

          /// compute alpha_s angle
          Eigen::Vector3f scene_point = cloud_scene_subsampled.points[scene_point_index].getVector3fMap ();
          Eigen::AngleAxisf rotation_sg (acos (scene_reference_normal.dot (Eigen::Vector3f::UnitX ())),
                                         scene_reference_normal.cross (Eigen::Vector3f::UnitX ()).normalized ());
          Eigen::Affine3f transform_sg = Eigen::Translation3f ( rotation_sg * ((-1) * scene_reference_point)) * rotation_sg;
          float alpha_s = acos (Eigen::Vector3f::UnitY ().dot ((transform_sg * scene_point).normalized ()));

          /// find point pairs in the model with the same discretized feature
          HashKeyStruct key = HashKeyStruct (d1, d2, d3, d4);
          pair <typename FeatureHashMapType::iterator, typename FeatureHashMapType::iterator> map_iterator_pair = surflet_model.feature_hash_map->equal_range (key);
          for (; map_iterator_pair.first != map_iterator_pair.second; ++ map_iterator_pair.first)
          {
            size_t model_reference_index = map_iterator_pair.first->second.first,
                model_point_index = map_iterator_pair.first->second.second;
            /// calculate angle alpha = alpha_m - alpha_s
            float alpha = surflet_model.alpha_m[model_reference_index][model_point_index] - alpha_s;
            unsigned int alpha_discretized = floor(alpha) + floor(M_PI / angle_discretization_step);
            accumulator_array[model_reference_index][alpha_discretized] ++;
          }
        }
        else PCL_ERROR ("Computing pair feature vector between points %zu and %zu went wrong.\n", scene_reference_index, scene_point_index);
      }

    size_t max_votes_i = 0, max_votes_j = 0;
    unsigned int max_votes = 0;

    for (size_t i = 0; i < accumulator_array.size (); ++i)
      for (size_t j = 0; j < accumulator_array.back ().size (); ++j)
      {
        if (accumulator_array[i][j] > max_votes)
        {
          max_votes = accumulator_array[i][j];
          max_votes_i = i;
          max_votes_j = j;
        }
        /// reset accumulator_array for the next set of iterations with a new scene reference point
        accumulator_array[i][j] = 0;
      }

    Eigen::Vector3f model_reference_point = cloud_model.points[max_votes_i].getVector3fMap (),
        model_reference_normal = cloud_model_normals.points[max_votes_i].getNormalVector3fMap ();
    Eigen::AngleAxisf rotation_mg (acos (model_reference_normal.dot (Eigen::Vector3f::UnitX ())), model_reference_normal.cross (Eigen::Vector3f::UnitX ()).normalized ());
    Eigen::Affine3f transform_mg = Eigen::Translation3f ( rotation_mg * ((-1) * model_reference_point)) * rotation_mg;
    Eigen::Affine3f max_transform = transform_sg.inverse () * Eigen::AngleAxisf ( (max_votes_j - floor(M_PI / angle_discretization_step)) * angle_discretization_step, Eigen::Vector3f::UnitX ()) * transform_mg;

    voted_poses.push_back (PoseWithVotes (max_transform, max_votes));
  }
  cerr << "Done with the Hough Transform ..." << endl;

  /// cluster poses for filtering out outliers and obtaining more precise results
  clusterPoses (voted_poses, results);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::SurfletEstimation<PointInT, PointOutT>::clusterPoses (typename pcl::SurfletEstimation<PointInT, PointOutT>::PoseWithVotesList &poses,
                                                           typename pcl::SurfletEstimation<PointInT, PointOutT>::PoseWithVotesList &result)
{
  cerr << "Clustering poses ..." << endl;
  /// start off by sorting the poses by the number of votes
  sort(poses.begin (), poses.end (), poseWithVotesCompareFunction);

  std::vector<PoseWithVotesList> clusters;
  std::vector<std::pair<size_t, unsigned int> > cluster_votes;
  for (size_t poses_i = 0; poses_i < poses.size(); ++ poses_i)
  {
    bool found_cluster = false;
    for (size_t clusters_i = 0; clusters_i < clusters.size(); ++ clusters_i)
    {
      if (posesWithinErrorBounds (poses[poses_i].pose, clusters[clusters_i].front ().pose))
      {
        found_cluster = true;
        clusters[clusters_i].push_back (poses[poses_i]);
        cluster_votes[clusters_i].second += poses[poses_i].votes;
        break;
      }
    }

    if (found_cluster == false)
    {
      /// create a new cluster with the current pose
      PoseWithVotesList new_cluster;
      new_cluster.push_back (poses[poses_i]);
      clusters.push_back (new_cluster);
      cluster_votes.push_back (std::pair<size_t, unsigned int> (clusters.size () - 1, poses[poses_i].votes));
    }
 }

  /// sort clusters by total number of votes
  std::sort (cluster_votes.begin (), cluster_votes.end (), clusterVotesCompareFunction);
  /// compute pose average and put them in result vector
  /// @todo some kind of threshold for determining whether a cluster has enough votes or not...
  /// now just taking the first three clusters
  result.clear ();
  size_t max_clusters = (clusters.size () < 3) ? clusters.size () : 3;
  for (size_t cluster_i = 0; cluster_i < max_clusters; ++ cluster_i)
  {
    cerr << "Winning cluster has #votes: " << cluster_votes[cluster_i].second << " and #poses voted: " << clusters[cluster_votes[cluster_i].first].size () << endl;
    Eigen::Vector3f translation_average (0.0, 0.0, 0.0);
    Eigen::Vector4f rotation_average (0.0, 0.0, 0.0, 0.0);
    for (typename PoseWithVotesList::iterator v_it = clusters[cluster_votes[cluster_i].first].begin (); v_it != clusters[cluster_votes[cluster_i].first].end (); ++ v_it)
    {
      translation_average += v_it->pose.translation ();
      /// averaging rotations by just averaging the quaternions in 4D space - reference "On Averaging Rotations" by CLAUS GRAMKOW
      rotation_average += Eigen::Quaternionf (v_it->pose.rotation ()).coeffs ();
    }

    translation_average /= clusters[cluster_votes[cluster_i].first].size ();
    rotation_average /= clusters[cluster_votes[cluster_i].first].size ();

    Eigen::Affine3f transform_average;
    transform_average.translation () = translation_average;
    transform_average.linear () = Eigen::Quaternionf (rotation_average).normalized().toRotationMatrix ();

    cerr << "Compare results translation: " << transform_average.translation() << "    " << translation_average << endl;
    cerr << "Compare results quaternion: " << Eigen::Quaternionf (transform_average.rotation()).coeffs() << "    " << rotation_average << endl;

    result.push_back (PoseWithVotes (transform_average, cluster_votes[cluster_i].second));
  }

  cerr << "#Poses: " << poses.size() << "   #Clusters: " << clusters.size() << endl;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> bool
pcl::SurfletEstimation<PointInT, PointOutT>::posesWithinErrorBounds (Eigen::Affine3f &pose1, Eigen::Affine3f &pose2)
{
  float position_diff = (pose1.translation () - pose2.translation ()).norm ();
  Eigen::AngleAxisf rotation_diff_mat (pose1.rotation ().inverse () * pose2.rotation ());

  float rotation_diff_angle = fabs (rotation_diff_mat.angle ());

  if (position_diff < clustering_position_diff_threshold && rotation_diff_angle < clustering_rotation_diff_threshold)
    return true;
  else return false;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::SurfletEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{

}




#define PCL_INSTANTIATE_SurfletEstimation(T,OutT) template class PCL_EXPORTS pcl::SurfletEstimation<T,OutT>;


#endif /* PCL_FEATURES_IMPL_SURFLET_HPP_ */
