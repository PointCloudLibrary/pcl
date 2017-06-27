/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#ifndef PCL_FLAT_KEYPOINT_IMPL_H_
#define PCL_FLAT_KEYPOINT_IMPL_H_

#include <pcl/features/normal_3d.h>

#include <pcl/keypoints/flat_keypoint.h>



//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> bool
  pcl::FlatKeypoint<PointInT, PointOutT, NormalT>::initCompute ()
{
  if (!Keypoint<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] init failed!\n", name_.c_str ());
    return (false);
  }

  if (normals_->empty ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] normals not set!\n", name_.c_str ());
    return (false);   
  }

  if (normals_->size () != surface_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] normals given, but the number of normals does not match the number of input points!\n", name_.c_str ());
    return (false);
  }

  if (R_discard_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : R_discard (%f) must be strict positive!\n",
      name_.c_str (), R_discard_);
    return (false);
  }

  if (R1_search_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : R1_search (%f) must be strict positive!\n",
      name_.c_str (), R1_search_);
    return (false);
  }

  if (R2_search_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : R2_search (%f) must be strict positive!\n",
      name_.c_str (), R2_search_);
    return (false);
  }

  if (T1_search_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : T1_search (%f) must be strict positive!\n",
      name_.c_str (), T1_search_);
    return (false);
  }

  if (T2_search_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : T2_search (%f) must be strict positive!\n",
      name_.c_str (), T2_search_);
    return (false);
  }

  if (min_neighbors_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : the minimum number of neighbors (%f) must be strict positive!\n",
      name_.c_str (), min_neighbors_);
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> void
  pcl::FlatKeypoint<PointInT, PointOutT, NormalT>::applyFlatKeypointsDetector (PointCloudInConstPtr cloud, std::vector<float> &points_flatness, const double R_discard, const double R_search, const double T_search, pcl::PointIndicesPtr keypoints_indices)
{
  bool right_left = true;

  size_t n_cloud_points = cloud->size();

  std::vector<bool> selectable_point_seed(n_cloud_points, true);
  std::vector<bool> selectable_point_best(n_cloud_points, true);

  keypoints_indices->indices.resize(n_cloud_points);
  int n_features = 0;  


  std::vector<int> neigh_indices;
  std::vector<float> neigh_sqr_dists;

  randomGenerator_.setParameters(0, n_cloud_points-1);

  float coverage_percentage_best = 0.0f;
  int n_unselectable_points_best = 0;
  float coverage_percentage_seed = 0.0f;
  int n_unselectable_points_seed = 0;
  //search for flat points until the algorithm covers Tsearch percentage of the cloud
  while((coverage_percentage_best <= T_search) && (coverage_percentage_seed <= T_search))
  {
    //select a random point
    int point_index = randomGenerator_.run();

    //if the random point has already been extracted, search for the points at left or right of the random point until find a not-already-selected point
    while((!selectable_point_seed[point_index]))
    {
      if(right_left)
      {
        point_index = (++point_index) % n_cloud_points;
      }
      else
      {
        point_index--;
        if(point_index == -1)
        {
          point_index = (int)n_cloud_points - 1;
        }
      }
    }
    right_left = !right_left;


    //remove the selected point (and its neighbours) from the list of seed points (dark green points in Fig.6 of paper)
    tree_->radiusSearch (*cloud, point_index, R_discard, neigh_indices, neigh_sqr_dists);
    for(int di = 0; di < neigh_indices.size(); di++)
    {
      if(selectable_point_seed[neigh_indices[di]])
      {
        n_unselectable_points_seed++;
      }
      selectable_point_seed[neigh_indices[di]] = false;
    }

    //find neighbour points (yellow points in Fig.6 of paper)
    if(R_search != R_discard)
    {
      tree_->radiusSearch (*cloud, point_index, R_search, neigh_indices, neigh_sqr_dists);
    }

    if(neigh_indices.size() < min_neighbors_)
    {
      continue;					
    }

    //iterate on yellow points 
    float best_flatness = -1.0f;
    int best_flatness_index = -1;		
    for(size_t index = 0; index < neigh_indices.size(); index++)
    {	
      int yellow_point_idx = neigh_indices[index];

      //check if the point is selectable as the best
      if(!selectable_point_best[yellow_point_idx])
      {
        continue;
      }

      //compute flatness if it hasn't already been computed
      float flatness = 0.0f;
      if(points_flatness[yellow_point_idx] != -2.0f)
      {					
        flatness = points_flatness[yellow_point_idx];
      }
      else
      {
        std::vector<int> flatness_neigh_indices;
        std::vector<float> flatness_neigh_sqr_dists;
        tree_->radiusSearch (*cloud, yellow_point_idx, search_radius_, flatness_neigh_indices, flatness_neigh_sqr_dists);


        //check the minimum number of neighbors for flatness computation
        if( flatness_neigh_indices.size() < min_neighbors_)
        {
          points_flatness[yellow_point_idx] = -1.0f;
          continue;					
        }

        if(!pcl::isFinite ((*normals_)[yellow_point_idx]))
        {
          points_flatness[yellow_point_idx] = -1.0f;
          continue;
        }


        //compute flatness score
        Eigen::Vector3f candidate_normal = (*normals_)[yellow_point_idx].getNormalVector3fMap();

        float mean_cos = 0.0f;

        //compute cos score
        int n_valid_normals = 0;
        for(size_t ne = 0; ne < flatness_neigh_indices.size(); ++ne)
        {
          if(!pcl::isFinite ((*normals_)[flatness_neigh_indices[ne]]))
          {
            continue;
          }

          Eigen::Vector3f neigh_normal = (*normals_)[flatness_neigh_indices[ne]].getNormalVector3fMap();

          mean_cos += candidate_normal.dot(neigh_normal);
          n_valid_normals++;
        }

        if(n_valid_normals < min_neighbors_)
        {
          points_flatness[yellow_point_idx] = -1.0f;
        }

        flatness = mean_cos / (float)n_valid_normals;

        points_flatness[yellow_point_idx] = flatness;	
      }


      if(best_flatness < flatness)
      {
        best_flatness = flatness;
        best_flatness_index = yellow_point_idx;
      }
    }

    if(best_flatness_index == -1)
    {
      continue;					
    }

    //add flattest feature point index to the vector
    keypoints_indices->indices[n_features] = best_flatness_index;
    n_features++;


    //remove the flattest point (and its neighbours) from the list of best points (dark fuchsia points in Fig.6 of paper)
    tree_->radiusSearch (*cloud, best_flatness_index, R_discard, neigh_indices, neigh_sqr_dists);

    for(size_t di = 0; di < neigh_indices.size(); di++)
    {
      if(selectable_point_best[neigh_indices[di]])
      {
        n_unselectable_points_best++;
      }
      selectable_point_best[neigh_indices[di]] = false;
    }

    coverage_percentage_best = n_unselectable_points_best/(float)n_cloud_points;
    coverage_percentage_seed = n_unselectable_points_seed/(float)n_cloud_points;

  } // while((coverage_percentage_best <= Tsearch) && (coverage_percentage_seed <= Tsearch))


  keypoints_indices->indices.resize(n_features);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> void
  pcl::FlatKeypoint<PointInT, PointOutT, NormalT>::detectKeypoints (PointCloudOut &output)
{
  // Make sure the output cloud is empty
  output.points.clear ();

  size_t n_cloud_points = input_->size ();

  std::vector<float> feature_point_scores_step1 (n_cloud_points, -2.0f);

  //first scan of the 2-step process
  pcl::PointIndicesPtr keypoints_indices_step1 (new pcl::PointIndices());
  applyFlatKeypointsDetector (input_, feature_point_scores_step1, R_discard_, R1_search_, T1_search_, keypoints_indices_step1);

  //create cloud comprising the flat points extrated at step 1.
  typename pcl::PointCloud<PointInT>::Ptr cloud_2step (new pcl::PointCloud<PointInT>());
  pcl::copyPointCloud<PointInT> (*input_, keypoints_indices_step1->indices, *cloud_2step);

  //Get the scores of points of reduced cloud cloud_2step from the scores of the points of the original cloud input_
  std::vector<float> feature_point_scores_step2 (keypoints_indices_step1->indices.size(), -2.0f);
  for(size_t i = 0; i < feature_point_scores_step2.size(); i++)
  {
    feature_point_scores_step2[i] = feature_point_scores_step1[keypoints_indices_step1->indices[i]];
  }

  //second scan of the 2-step process
  pcl::PointIndicesPtr keypoints_indices_step2 (new pcl::PointIndices());
  tree_->setInputCloud (cloud_2step);
  applyFlatKeypointsDetector (cloud_2step, feature_point_scores_step2, R_discard_, R2_search_, T2_search_, keypoints_indices_step2);


  //Get the indices of the points of the original cloud that have been selected as feature points from the the reduced cloud cloud_2step
  keypoints_indices_->indices.resize (keypoints_indices_step2->indices.size());
  for(size_t i = 0; i < keypoints_indices_step2->indices.size(); i++)
  {
    keypoints_indices_->indices[i] = keypoints_indices_step1->indices[keypoints_indices_step2->indices[i]];
  }

  pcl::copyPointCloud<PointInT, PointOutT> (*input_, keypoints_indices_->indices, output);

  output.header = input_->header;
  output.width = static_cast<uint32_t> (output.points.size ());
  output.height = 1;

}

#define PCL_INSTANTIATE_FlatKeypoint(T,U,N) template class PCL_EXPORTS pcl::FlatKeypoint<T,U,N>; 

#endif /* PCL_FLAT_KEYPOINT_IMPL_H_ */
