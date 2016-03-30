/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2010-2011, Willow Garage, Inc.
*  Copyright (c) 2012-, Open Perception, Inc.
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
*   * Neither the name of the copyright holder(s) nor the names of its
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
*
*/

#ifndef PCL_FEATURES_IMPL_FLARE_H_
#define PCL_FEATURES_IMPL_FLARE_H_


#include <pcl/features/flare.h>
#include <utility>
#include <pcl/common/transforms.h>
#include <pcl/features/lrf_utils.h>


//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT, typename SignedDistanceT> bool
  pcl::FLARELocalReferenceFrameEstimation<PointInT, PointNT, PointOutT, SignedDistanceT>::initCompute ()
{
  if (!FeatureFromNormals<PointInT, PointNT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  if(!sampled_surface_ && sampled_tree_)
  {
    PCL_WARN ("[pcl::%s::initCompute] sampled_surface_ is not set even if sampled_tree_ is already set. sampled_tree_ will be rebuilt from surface_. Use sampled_surface_.\n", getClassName ().c_str ());
  }

  if(tangent_radius_ == 0.0f)
  {
    PCL_ERROR ("[pcl::%s::initCompute] tangent_radius_ not set.\n", getClassName ().c_str ());
    return (false);
  }

  // If no search sampled_surface_ has been defined, use the surface_ dataset as the search sampled_surface_ itself
  if (!sampled_surface_)
  {
    fake_sampled_surface_ = true;
    sampled_surface_ = surface_;
  }

  // Check if a space search locator was given for sampled_surface_
  if (!sampled_tree_)
  {
    if (sampled_surface_->isOrganized () && surface_->isOrganized () && input_->isOrganized ())
      sampled_tree_.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
    else
      sampled_tree_.reset (new pcl::search::KdTree<PointInT> (false));
  }

  if (sampled_tree_->getInputCloud () != sampled_surface_) // Make sure the tree searches the sampled surface
    sampled_tree_->setInputCloud (sampled_surface_); 

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT, typename SignedDistanceT> bool
  pcl::FLARELocalReferenceFrameEstimation<PointInT, PointNT, PointOutT, SignedDistanceT>::deinitCompute ()
{
  // Reset the surface
  if (fake_surface_)
  {
    surface_.reset ();
    fake_surface_ = false;
  }
  // Reset the sampled surface
  if (fake_sampled_surface_)
  {
    sampled_surface_.reset ();
    fake_sampled_surface_ = false;
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT, typename SignedDistanceT> SignedDistanceT
  pcl::FLARELocalReferenceFrameEstimation<PointInT, PointNT, PointOutT, SignedDistanceT>::computePointLRF (const int &index,
  Eigen::Matrix3f &lrf)
{
  Eigen::Vector3f x_axis, y_axis;
  Eigen::Vector3f fitted_normal; //z_axis

  //find Z axis

  //extract support points for the computation of Z axis
  std::vector<int> neighbours_indices;
  std::vector<float> neighbours_distances;
  int n_neighbours = this->searchForNeighbors (index, search_parameter_, neighbours_indices, neighbours_distances);

  if (n_neighbours < min_neighbors_for_normal_axis_)
  {
    if(!pcl::isFinite ((*normals_)[index]))
    {
      //normal is invalid
      //setting lrf to NaN
      lrf.setConstant (std::numeric_limits<float>::quiet_NaN ());
      return (std::numeric_limits<SignedDistanceT>::max ());
    }

    //set z_axis as the normal of index point
    fitted_normal = (*normals_)[index].getNormalVector3fMap();

    return (std::numeric_limits<SignedDistanceT>::max ());
  }
  else
  {
    ////find centroid for plane fitting
    //  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    //  Eigen::Vector3f mean_normal = Eigen::Vector3f::Zero();
    //for(size_t ne = 0; ne < neighbours_indices.size(); ++ne)
    //{
    //    centroid += (*surface_)[ neighbours_indices[ne] ].getVector3fMap();
    //    mean_normal += (*normals_)[ neighbours_indices[ne] ].getNormalVector3fMap();
    //}
    //  centroid /= (float)neighbours_indices.size();
    //  mean_normal.normalize(); 


    ////plane fitting
    //  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();
    //  float temp;
    //  Eigen::Vector3f no_centroid = Eigen::Vector3f::Zero();
    //for(size_t ne = 0; ne < neighbours_indices.size(); ++ne)
    //{
    //    no_centroid = (*surface_)[ neighbours_indices[ne] ].getVector3fMap() - centroid;

    //    covariance_matrix (0,0) += no_centroid.x() * no_centroid.x();
    //    covariance_matrix (1,1) += no_centroid.y() * no_centroid.y();
    //    covariance_matrix (2,2) = no_centroid.z() * no_centroid.z();

    //    temp =  no_centroid.x() * no_centroid.y();
    //    covariance_matrix (0,1) += temp;
    //    covariance_matrix (1,0) += temp;
    //    temp =  no_centroid.x() * no_centroid.z();
    //    covariance_matrix (0,2) += temp;
    //    covariance_matrix (2,0) += temp;
    //    temp =  no_centroid.y() * no_centroid.z();
    //    covariance_matrix (1,2) += temp;
    //    covariance_matrix (2,1) += temp;
    //  }

    //EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
    //EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
    //pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);

    //fitted_normal.x() = eigen_vector [0];
    //fitted_normal.y() = eigen_vector [1];
    //fitted_normal.z() = eigen_vector [2];

    float plane_curvature;
    normal_estimation_.computePointNormal(*surface_, neighbours_indices, fitted_normal(0), fitted_normal(1), fitted_normal(2), plane_curvature);

    //disambiguate Z axis with normal mean
    if (!normalDisambiguation<PointNT> (*normals_, neighbours_indices, fitted_normal) )
    {
      //all normals in the neighbourood are invalid
      //setting lrf to NaN
      lrf.setConstant (std::numeric_limits<float>::quiet_NaN ());
      return (std::numeric_limits<SignedDistanceT>::max ());
    }
  }

  //setting LRF Z axis
  lrf.row (2).matrix () = fitted_normal;


  //find X axis

  //extract support points for Rx radius
  n_neighbours = sampled_tree_->radiusSearch( (*input_)[index], tangent_radius_, neighbours_indices, neighbours_distances);

  if (n_neighbours < min_neighbors_for_tangent_axis_)
  {
    //set X axis as a random axis
    randomOrthogonalAxis (fitted_normal, x_axis);
    y_axis = fitted_normal.cross (x_axis);

    return (std::numeric_limits<SignedDistanceT>::max ());
  }

  //find point with the largest signed distance from tangent plane

  SignedDistanceT shapeScore;
  SignedDistanceT bestShapeScore = -std::numeric_limits<SignedDistanceT>::max();
  int bestShapeIndex = -1;

  Eigen::Vector3f best_margin_point;

  float radius2 = tangent_radius_ * tangent_radius_;

  float margin_distance2 = margin_thresh_ * margin_thresh_ * radius2;


  Vector3fMapConst feature_point = (*input_)[index].getVector3fMap ();


  for (int curr_neigh = 0; curr_neigh < n_neighbours; ++curr_neigh)
  {
    const int& curr_neigh_idx = neighbours_indices[curr_neigh];
    const float& neigh_distance_sqr = neighbours_distances[curr_neigh];
    if (neigh_distance_sqr <= margin_distance2)
    {
      continue;
    }

    //point curr_neigh_idx is inside the ring between marginThresh and Radius

    shapeScore =	(*sampled_surface_)[curr_neigh_idx].getVector3fMap()[0] * fitted_normal[0] +
      (*sampled_surface_)[curr_neigh_idx].getVector3fMap()[1] * fitted_normal[1] +
      (*sampled_surface_)[curr_neigh_idx].getVector3fMap()[2] * fitted_normal[2];

    if(shapeScore > bestShapeScore)
    {
      bestShapeIndex = curr_neigh_idx;
      bestShapeScore = shapeScore;
    }

  } //for each neighbor

  if (bestShapeIndex == -1)
  {

    randomOrthogonalAxis (fitted_normal, x_axis);
    y_axis = fitted_normal.cross (x_axis);

    return (std::numeric_limits<SignedDistanceT>::max ());
  }

  //find orthogonal axis directed to bestShapeIndex point projection on plane with fittedNormal as axis
  directedOrthogonalAxis (fitted_normal, feature_point, sampled_surface_->at(bestShapeIndex).getVector3fMap(), x_axis);

  y_axis = fitted_normal.cross (x_axis);

  lrf.row (0).matrix () = x_axis;
  lrf.row (1).matrix () = y_axis;
  //z axis already set

  bestShapeScore -= (fitted_normal[0]*feature_point[0] + fitted_normal[1]*feature_point[1] + fitted_normal[2]*feature_point[2]);
  return (bestShapeScore);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT, typename SignedDistanceT> void
  pcl::FLARELocalReferenceFrameEstimation<PointInT, PointNT, PointOutT, SignedDistanceT>::computeFeature (PointCloudOut &output)
{
  //check whether used with search radius or search k-neighbors
  if (this->getKSearch () != 0)
  {
    PCL_ERROR(
      "[pcl::%s::computeFeature] Error! Search method set to k-neighborhood. Call setKSearch(0) and setRadiusSearch( radius ) to use this class.\n",
      getClassName().c_str());
    return;
  }

  if(signed_distances_from_highest_points_.size () != indices_->size ())
    signed_distances_from_highest_points_.resize (indices_->size ());

  for (size_t point_idx = 0; point_idx < indices_->size (); ++point_idx)
  {
    Eigen::Matrix3f currentLrf;
    PointOutT &rf = output[point_idx];

    signed_distances_from_highest_points_[point_idx] = computePointLRF ((*indices_)[point_idx], currentLrf);
    if(  signed_distances_from_highest_points_[point_idx] == std::numeric_limits<SignedDistanceT>::max () )
    {
      output.is_dense = false;
    }

    for (int d = 0; d < 3; ++d)
    {
      rf.x_axis[d] = currentLrf (0, d);
      rf.y_axis[d] = currentLrf (1, d);
      rf.z_axis[d] = currentLrf (2, d);
    }
  }
}


#define PCL_INSTANTIATE_FLARELocalReferenceFrameEstimation(T,NT,OutT,SdT) template class PCL_EXPORTS pcl::FLARELocalReferenceFrameEstimation<T,NT,OutT,SdT>;

#endif // PCL_FEATURES_IMPL_FLARE_H_
