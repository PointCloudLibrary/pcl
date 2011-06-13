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
template <typename PointInT, typename PointOutT> typename pcl::SurfletEstimation<PointInT, PointOutT>::FeatureHashMapTypePtr
pcl::SurfletEstimation<PointInT, PointOutT>::computeSurfletModel (
    const pcl::PointCloud<PointInT> &cloud /* output goes here */)
{
  cerr << "Computing Surflet Model function called" << endl;

  cerr << "Subsampling ..." << endl;
  pcl::PointCloud<PointInT> cloud_subsampled;
  /// subsample point cloud such that min dist between points is d_dist (+parameter)
  pcl::VoxelGrid<PointInT> subsampling_filter;
  /// @todo don't make a copy here - use input_ from feature
  PointCloudIn cloud_ptr = pcl::PointCloud<PointInT> (cloud).makeShared ();
  subsampling_filter.setInputCloud (cloud_ptr);
  subsampling_filter.setLeafSize (0.01, 0.01, 0.01); /// @TODO parameter goes here
  subsampling_filter.filter (cloud_subsampled);
  cerr << "After subsampling model, points: " << cloud_subsampled.width << " / " << cloud.width << endl;

  cerr << "Estimating normals ..." << endl;
  pcl::PointCloud<Normal> cloud_subsampled_normals;
  /// recompute normals of the subsampled surfaces
  pcl::NormalEstimation<PointInT, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_subsampled.makeShared());

  typename pcl::KdTreeFLANN<PointInT>::Ptr search_tree (new pcl::KdTreeFLANN<PointInT>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch(0.05); // @TODO another parameter
  normal_estimation_filter.compute (cloud_subsampled_normals);

  cerr << "Computing feature hash map" << endl;
  /// compute feature vector for each pair of points in the subsampled point cloud
  /// @TODO currently considering only unorganized pointclouds
  float f1, f2, f3, f4;
  int d1, d2, d3, d4;
  FeatureHashMapTypePtr feature_hash_map (new FeatureHashMapType);
  for (size_t i = 0; i < cloud_subsampled.width; ++i)
  {
    for (size_t j = 0; j < cloud_subsampled.width; ++j) 
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
        feature_hash_map->insert (std::pair<HashKeyStruct, pair<size_t, size_t> > (HashKeyStruct (d1, d2, d3, d4), pair<size_t, size_t> (i, j)));
        //     cerr << d1 << " " << d2 << " " << d3 << " " << d4 << endl;
        //     cerr << f1 << " " << f2 << " " << f3 << " " << f4 << endl << "------" << endl;
      }
      else 
      {
        PCL_ERROR ("Computing pair feature vector between points %zu and %zu went wrong.\n", i, j);
        /// @TODO do something if fail
      }
    }
  }

  return (feature_hash_map);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> bool
pcl::SurfletEstimation<PointInT, PointOutT>::resultsCompareFunction (const typename pcl::SurfletEstimation<PointInT, PointOutT>::PoseWithVotes &a,
                                                                     const typename pcl::SurfletEstimation<PointInT, PointOutT>::PoseWithVotes &b )
{
  return (a.votes > b.votes);
}


//////////////////////////////////////////////////////////////////////////////////////////////
/// @TODO !!! cloud_model should be subsampled as before!!
/// @todo very ugly hack with PointOutT = normals
template <typename PointInT, typename PointOutT> typename pcl::SurfletEstimation<PointInT, PointOutT>::PoseWithVotesList
pcl::SurfletEstimation<PointInT, PointOutT>::registerModelToScene (
    const pcl::PointCloud<PointInT> &cloud_model,
    const pcl::PointCloud<PointOutT> &cloud_model_normals,
    const pcl::PointCloud<PointInT> &cloud_scene,
    typename pcl::SurfletEstimation<PointInT, PointOutT>::FeatureHashMapTypePtr feature_hashmap_model)
{
  PoseWithVotesList voted_poses;
  std::vector <std::vector <unsigned int> > accumulator_array;
  accumulator_array.resize (cloud_model.width);
  for (size_t i = 0; i < cloud_model.width; ++i)
  {
    std::vector <unsigned int> aux ((size_t)floor(2*M_PI / angle_discretization_step ), 0);
    accumulator_array[i] = aux;
  }
  cerr << "Accumulator array size: " << accumulator_array.size() << " x " << accumulator_array.back ().size () << endl;

  //  unsigned int accumulator_array [cloud_model.width][ (size_t)floor(2*M_PI / angle_discretization_step )];

  /// subsample scene cloud with same rate as the model cloud
  pcl::PointCloud<PointInT> cloud_scene_subsampled;
  /// subsample point cloud such that min dist between points is d_dist (+parameter)
  pcl::VoxelGrid<PointInT> subsampling_filter;
  /// @todo don't make a copy here
  PointCloudIn cloud_scene_ptr = pcl::PointCloud<PointInT> (cloud_scene).makeShared ();
  subsampling_filter.setInputCloud (cloud_scene_ptr);
  subsampling_filter.setLeafSize (0.01, 0.01, 0.01); /// @TODO parameter goes here
  subsampling_filter.filter (cloud_scene_subsampled);

  /// calculate subsampled scene cloud normals
  pcl::PointCloud<Normal> cloud_scene_subsampled_normals;
  /// recompute normals of the subsampled surfaces
  pcl::NormalEstimation<PointInT, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_scene_subsampled.makeShared());
  typename pcl::KdTreeFLANN<PointInT>::Ptr search_tree (new pcl::KdTreeFLANN<PointInT>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch(0.05); // @TODO another parameter
  normal_estimation_filter.compute (cloud_scene_subsampled_normals);


  /// consider every N/5-th point in the scene as a reference point s_r @TODO parameter here
  float f1, f2, f3, f4;
  int d1, d2, d3, d4;
  for (size_t scene_reference_index = 0; scene_reference_index < cloud_scene_subsampled.width; scene_reference_index += 5)
  {
    Eigen::Vector3f scene_reference_point = cloud_scene_subsampled.points[scene_reference_index].getVector3fMap (),
        scene_reference_normal = cloud_scene_subsampled_normals.points[scene_reference_index].getNormalVector3fMap ();
    Eigen::AngleAxisf rotation_sg (acos (scene_reference_normal.dot (Eigen::Vector3f::UnitX ())), scene_reference_normal.cross (Eigen::Vector3f::UnitX ()));
    Eigen::Affine3f transform_sg = Eigen::Translation3f ( rotation_sg* ((-1)*scene_reference_point)) * rotation_sg;

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

          HashKeyStruct key = HashKeyStruct (d1, d2, d3, d4);
          pair <typename FeatureHashMapType::iterator, typename FeatureHashMapType::iterator> map_iterator_pair = feature_hashmap_model->equal_range (key);
          for (; map_iterator_pair.first != map_iterator_pair.second; ++ map_iterator_pair.first)
          {
            size_t model_reference_index = map_iterator_pair.first->second.first,
                model_point_index = map_iterator_pair.first->second.second;
            /// calculate angle alpha
            Eigen::Vector3f model_reference_point = cloud_model.points[model_reference_index].getVector3fMap (), //(cloud_model.points[model_reference_index].x, cloud_model.points[model_reference_index].y, cloud_model.points[model_reference_index].z),
                model_reference_normal = cloud_model_normals.points[model_reference_index].getNormalVector3fMap (), //(cloud_model_normals.points[model_reference_index].normal),
                scene_point = cloud_scene_subsampled.points[scene_point_index].getVector3fMap (), //(cloud_scene_subsampled.points[scene_point_index].x, cloud_scene_subsampled.points[scene_point_index].y, cloud_scene_subsampled.points[scene_point_index].z),
                model_point = cloud_model.points[model_point_index].getVector3fMap (); //(cloud_model.points[model_point_index].x, cloud_model.points[model_point_index].y, cloud_model.points[model_point_index].z);
            Eigen::AngleAxisf rotation_mg (acos (model_reference_normal.dot (Eigen::Vector3f::UnitX ())), model_reference_normal.cross (Eigen::Vector3f::UnitX ()));
            Eigen::Affine3f transform_mg = Eigen::Translation3f ( rotation_mg * ((-1) * model_reference_point)) * rotation_mg;

            //            cerr << "Test - should be origin " << transform_mg * model_reference_point << "     " << transform_sg * scene_reference_point << endl;

            float alpha = acos ((transform_sg * scene_point).dot (transform_mg * model_point));
            Eigen::Vector3f axis_test = ((transform_sg*scene_point).normalized().cross( (transform_mg*model_point).normalized())).normalized();
            //            cerr << "axis should be UnitX: " << axis_test << endl;

            unsigned int alpha_discretized = floor(alpha) + floor(M_PI / angle_discretization_step);
            accumulator_array[model_reference_index][alpha_discretized] ++;
          }

        }
        else {
          cerr << "Computing pair feature vector between points " << scene_reference_index << " and " << scene_point_index << " went wrong." << endl;
          /// @TODO do something if fail
        }
      }

    unsigned int max_votes = 0;
    Eigen::Affine3f max_transform;
    for (size_t i = 0; i < accumulator_array.size(); ++i)
    {
      for (size_t j = 0; j < accumulator_array.back().size(); ++j) 
      {
        unsigned int val = accumulator_array[i][j];
        if (val > max_votes)
        {
          max_votes = val;

          Eigen::Vector3f model_reference_point = cloud_model.points[i].getVector3fMap (),
              model_reference_normal = cloud_model_normals.points[i].getNormalVector3fMap ();
          Eigen::AngleAxisf rotation_mg (acos (model_reference_normal.dot (Eigen::Vector3f::UnitX ())), model_reference_normal.cross (Eigen::Vector3f::UnitX ()));
          Eigen::Affine3f transform_mg = Eigen::Translation3f ( rotation_mg * ((-1) * model_reference_point)) * rotation_mg;
          max_transform = transform_sg.inverse () * Eigen::AngleAxisf ( (j - floor(M_PI / angle_discretization_step)) * angle_discretization_step, Eigen::Vector3f::UnitX ()) * transform_mg;
        }

        /// reset accumulator_array for the next set of iterations with a new scene reference point
        accumulator_array[i][j] = 0;
      }
    }

//    cerr << "max_votes: " << max_votes << endl;
    voted_poses.push_back (PoseWithVotes (max_transform, max_votes));
  }


  /// cluster poses for filtering out outliers and obtaining more precise results
  PoseWithVotesList results;
  clusterPoses (voted_poses, results);

  return results;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::SurfletEstimation<PointInT, PointOutT>::clusterPoses (typename pcl::SurfletEstimation<PointInT, PointOutT>::PoseWithVotesList &poses,
                                                           typename pcl::SurfletEstimation<PointInT, PointOutT>::PoseWithVotesList &result)
{
  /// start off by sorting the poses by the number of votes
  sort(poses.begin (), poses.end (), resultsCompareFunction);

  std::vector<PoseWithVotesList> clusters;
  std::vector<unsigned int> cluster_votes;
  for (size_t poses_i = 0; poses_i < poses.size(); ++ poses_i)
  {
    bool found_cluster = false;
    for (size_t clusters_i = 0; clusters_i < clusters.size(); ++ clusters_i)
      if (posesWithinErrorBounds (poses[poses_i].pose, clusters[clusters_i].front ().pose))
      {
        found_cluster = true;
        clusters[clusters_i].push_back (poses[poses_i]);
        cluster_votes[clusters_i] += poses[poses_i].votes;
        break;
      }

    if (found_cluster == false)
    {
      /// create a new cluster with the current pose
      PoseWithVotesList new_cluster;
      new_cluster.push_back (poses[poses_i]);
      clusters.push_back (new_cluster);
      cluster_votes.push_back (poses[poses_i].votes);
    }
 }

  /// compute pose average and put them in result vector


}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> bool
pcl::SurfletEstimation<PointInT, PointOutT>::posesWithinErrorBounds (Eigen::Affine3f &pose1, Eigen::Affine3f &pose2)
{
  float position_diff = (pose1.translation () - pose2.translation ()).norm ();
  //// !!!!!!!!!!!!!!!!!!!!
  Eigen::AngleAxisf rotation_diff_mat; //= pose1.rotation().inverse() * pose2.rotation();
  float rotation_diff_angle = rotation_diff_mat.angle ();

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
