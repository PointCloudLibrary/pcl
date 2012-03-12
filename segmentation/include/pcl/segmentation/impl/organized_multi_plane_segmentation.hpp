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
 *
 *
 */

#ifndef PCL_SEGMENTATION_IMPL_ORGANIZED_MULTI_PLANE_SEGMENTATION_H_
#define PCL_SEGMENTATION_IMPL_ORGANIZED_MULTI_PLANE_SEGMENTATION_H_

#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <boost/make_shared.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedMultiPlaneSegmentation<PointT, PointNT, PointLT>::segment (std::vector<ModelCoefficients>& model_coefficients, 
                                                                         std::vector<PointIndices>& inlier_indices)
{
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> label_indices;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
  std::vector <Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > covariances;
  segment (model_coefficients, inlier_indices, centroids, covariances, labels, label_indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedMultiPlaneSegmentation<PointT, PointNT, PointLT>::segment (std::vector<ModelCoefficients>& model_coefficients, 
                                                                         std::vector<PointIndices>& inlier_indices,
                                                                         std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& centroids,
                                                                         std::vector <Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >& covariances,
                                                                         pcl::PointCloud<PointLT>& labels,
                                                                         std::vector<pcl::PointIndices>& label_indices)
{
  if (!initCompute ())
    return;

  // Check that we got the same number of points and normals
  if (static_cast<int> (normals_->points.size ()) != static_cast<int> (input_->points.size ()))
  {
    PCL_ERROR ("[pcl::%s::segment] Number of points in input cloud (%zu) and normal cloud (%zu) do not match!\n",
               getClassName ().c_str (), input_->points.size (),
               normals_->points.size ());
    return;
  }

  // Check that the cloud is organized
  if (!input_->isOrganized ())
  {
    PCL_ERROR ("[pcl::%s::segment] Organized point cloud is required for this plane extraction method!\n",
               getClassName ().c_str ());
    return;
  }

  // Calculate range part of planes' hessian normal form
  std::vector<float> plane_d (input_->points.size ());
  
  for (unsigned int i = 0; i < input_->size (); ++i)
    plane_d[i] = input_->points[i].getVector3fMap ().dot (normals_->points[i].getNormalVector3fMap ());
  
  // Make a comparator
  PlaneCoefficientComparator<PointT,PointNT> plane_comparator (plane_d);
  plane_comparator.setInputCloud (input_);
  plane_comparator.setInputNormals (normals_);
  plane_comparator.setAngularThreshold (static_cast<float> (angular_threshold_));
  plane_comparator.setDistanceThreshold (static_cast<float> (distance_threshold_));

  // Set up the output
  OrganizedConnectedComponentSegmentation<PointT,pcl::Label> connected_component (boost::make_shared<PlaneCoefficientComparator<PointT,PointNT> >(plane_comparator));
  connected_component.setInputCloud (input_);
  connected_component.segment (labels, label_indices);

  Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero ();
  Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
  Eigen::Matrix3f clust_cov;
  pcl::ModelCoefficients model;
  model.values.resize (4);

  // Fit Planes to each cluster
  for (size_t i = 0; i < label_indices.size (); i++)
  {
    if (static_cast<int> (label_indices[i].indices.size ()) > min_inliers_)
    {
      pcl::computeMeanAndCovarianceMatrix (*input_, label_indices[i].indices, clust_cov, clust_centroid);
      Eigen::Vector4f plane_params;
      
      EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
      EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
      pcl::eigen33 (clust_cov, eigen_value, eigen_vector);
      plane_params[0] = eigen_vector[0];
      plane_params[1] = eigen_vector[1];
      plane_params[2] = eigen_vector[2];
      plane_params[3] = 0;
      plane_params[3] = -1 * plane_params.dot (clust_centroid);

      vp -= clust_centroid;
      float cos_theta = vp.dot (plane_params);
      if (cos_theta < 0)
      {
        plane_params *= -1;
        plane_params[3] = 0;
        plane_params[3] = -1 * plane_params.dot (clust_centroid);
      }
      
      model.values[0] = plane_params[0];
      model.values[1] = plane_params[1];
      model.values[2] = plane_params[2];
      model.values[3] = plane_params[3];
      model_coefficients.push_back (model);
      inlier_indices.push_back (label_indices[i]);
      centroids.push_back (clust_centroid);
      covariances.push_back (clust_cov);
    }
  }
  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointNT, typename PointLT> void
pcl::OrganizedMultiPlaneSegmentation<PointT, PointNT, PointLT>::segment (std::vector<PlanarRegion<PointT> >& regions)
{
  std::vector<ModelCoefficients> model_coefficients;
  std::vector<PointIndices> inlier_indices;  
  PointCloudLPtr labels (new PointCloudL);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  pcl::PointCloud<PointT> boundary_cloud;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
  std::vector <Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > covariances;
  segment (model_coefficients, inlier_indices, centroids, covariances, *labels, label_indices);
  regions.resize (model_coefficients.size ());
  boundary_indices.resize (model_coefficients.size ());
  
  for (size_t i = 0; i < model_coefficients.size (); i++)
  {
    boundary_cloud.resize (0);
    pcl::OrganizedConnectedComponentSegmentation<PointT,PointLT>::findLabeledRegionBoundary (inlier_indices[i].indices[0], labels, boundary_indices[i]);
    boundary_cloud.points.resize (boundary_indices[i].indices.size ());
    for (unsigned j = 0; j < boundary_indices[i].indices.size (); j++)
      boundary_cloud.points[j] = input_->points[boundary_indices[i].indices[j]];
    
    Eigen::Vector3f centroid = Eigen::Vector3f (centroids[i][0],centroids[i][1],centroids[i][2]);
    Eigen::Vector4f model = Eigen::Vector4f (model_coefficients[i].values[0],
                                             model_coefficients[i].values[1],
                                             model_coefficients[i].values[2],
                                             model_coefficients[i].values[3]);
    regions[i] = PlanarRegion<PointT> (centroid,
                                       covariances[i], 
                                       static_cast<unsigned int> (inlier_indices[i].indices.size ()),
                                       boundary_cloud.points,
                                       model);
    

  }

}

#define PCL_INSTANTIATE_OrganizedMultiPlaneSegmentation(T,NT,LT) template class PCL_EXPORTS pcl::OrganizedMultiPlaneSegmentation<T,NT,LT>;

#endif  // PCL_SEGMENTATION_IMPL_MULTI_PLANE_SEGMENTATION_H_
