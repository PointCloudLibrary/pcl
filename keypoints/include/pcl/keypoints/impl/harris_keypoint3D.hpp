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

#ifndef PCL_HARRIS_KEYPOINT_3D_IMPL_H_
#define PCL_HARRIS_KEYPOINT_3D_IMPL_H_

#include "pcl/keypoints/harris_keypoint3D.h"
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::setMethod (ResponseMethod method)
{
  method_ = method;
}

template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::setThreshold (float threshold)
{
  threshold_= threshold;
}

template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::setRadius (float radius)
{
  radius_ = radius;
}

template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::setNonMaxSupression (bool nonmax)
{
  nonmax_ = nonmax;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::detectKeypoints (PointCloudOut &output)
{
  boost::shared_ptr<pcl::PointCloud<PointInT> > cloud (new pcl::PointCloud<PointInT> ());
  pcl::PassThrough<PointInT> pass_;
#if 0  
  if (indices_->empty () || indices_->size() == indices_->size () == (input_->width * input_->height) ||
      indices_->size () == input_->points.size ())
  {
    pass_.setInputCloud (input_);
  }
  else
  {
    boost::shared_ptr<pcl::PointCloud<PointInT> > sub_cloud (new pcl::PointCloud<PointInT> ());
    pcl::ExtractIndices<PointInT> extract;
    extract.setIndices (indices_);
    extract.setInputCloud (input_);
    extract.filter (*sub_cloud);
    pass_.setInputCloud (sub_cloud);
  }
#else
  pass_.setInputCloud (input_);
#endif
  
  pass_.filter (*cloud);
  // estimate normals
  boost::shared_ptr<pcl::PointCloud<pcl::Normal> > normals (new pcl::PointCloud<Normal> ());
  pcl::NormalEstimation<PointInT, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud(cloud);
  normal_estimation.setRadiusSearch(radius_);
  normal_estimation.compute (*normals);
  
  boost::shared_ptr<pcl::PointCloud<PointOutT> > response (new pcl::PointCloud<PointOutT> ());
  switch (method_)
  {
    case HARRIS:
      responseHarris(cloud, normals, *response);
      break;
    case NOBLE:
      responseNoble(cloud, normals, *response);
      break;
    case LOWE:
      responseLowe(cloud, normals, *response);
      break;
    case CURVATURE:
      responseCurvature(cloud, normals, *response);
      break;
    case TOMASI:
      responseTomasi(cloud, normals, *response);
      break;     
  }
  
  // just return the response
  if (!nonmax_)
    output = *response;
  else
  {
    output.points.clear ();
    output.points.reserve (response->points.size());
    std::vector<int> nn_indices;
    std::vector<float> nn_dists;
    pcl::search::KdTree<pcl::PointXYZI> response_search;
    response_search.setInputCloud(response);
    for (size_t idx = 0; idx < response->points.size(); ++idx)
    {
      response_search.radiusSearch (idx, radius_, nn_indices, nn_dists);
      bool is_maxima = true;
      for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
      {
        if (response->points[idx].intensity < response->points[*iIt].intensity)
        {
          is_maxima = false;
          break;
        }
      }
      if (is_maxima)
        output.points.push_back (response->points[idx]);
    }
    
    output.height = 1;
    output.width = output.points.size();
  }
}

template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::responseHarris (typename PointCloudIn::ConstPtr input, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &output) const
{
  output.points.clear ();
  output.points.reserve (input->points.size());
  
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  pcl::search::KdTree<PointInT> search;
  search.setInputCloud(input);
  
  PointOutT point;
  for (typename PointCloudIn::const_iterator pointIt = input->begin(); pointIt != input->end(); ++pointIt)
  //for (std::vector<int>::const_iterator idxIt = indices_->begin(); idxIt != indices_->end(); ++idxIt)
  {
    search.radiusSearch (*pointIt, radius_, nn_indices, nn_dists);

    Eigen::Matrix3f covariance_matrix;
    covariance_matrix.setZero();
    for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
    {
      const Eigen::Vector3f* vec = reinterpret_cast<const Eigen::Vector3f*> (&(normals->at(*iIt).normal_x));
      covariance_matrix += (*vec) * (vec->transpose());
    }
    point.x = pointIt->x;
    point.y = pointIt->y;
    point.z = pointIt->z;
    point.intensity = covariance_matrix.determinant () - 0.04 * covariance_matrix.trace () * covariance_matrix.trace ();    
    output.points.push_back(point);
  }
  output.height = 1;
  output.width = output.points.size ();
}

template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::responseNoble (typename PointCloudIn::ConstPtr input, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &output) const
{
  output.points.clear ();
  output.points.reserve (input->points.size());
  
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  pcl::search::KdTree<PointInT> search;
  search.setInputCloud(input);
  
  PointOutT point;
  for (typename PointCloudIn::const_iterator pointIt = input->begin(); pointIt != input->end(); ++pointIt)
  {
    search.radiusSearch (*pointIt, radius_, nn_indices, nn_dists);

    Eigen::Matrix3f covariance_matrix;
    covariance_matrix.setZero();
    for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
    {
      const Eigen::Vector3f* vec = reinterpret_cast<const Eigen::Vector3f*> (&(normals->at(*iIt).normal_x));
      covariance_matrix += (*vec) * (vec->transpose());
    }
    point.x = pointIt->x;
    point.y = pointIt->y;
    point.z = pointIt->z;
    point.intensity = covariance_matrix.determinant () / covariance_matrix.trace ();
    output.points.push_back(point);
  }
  output.height = 1;
  output.width = output.points.size ();
}

template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::responseLowe (typename PointCloudIn::ConstPtr input, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &output) const
{
  output.points.clear ();
  output.points.reserve (input->points.size());
  
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  pcl::search::KdTree<PointInT> search;
  search.setInputCloud(input);
  
  PointOutT point;
  for (typename PointCloudIn::const_iterator pointIt = input->begin(); pointIt != input->end(); ++pointIt)
  {
    search.radiusSearch (*pointIt, radius_, nn_indices, nn_dists);

    Eigen::Matrix3f covariance_matrix;
    covariance_matrix.setZero();
    for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
    {
      const Eigen::Vector3f* vec = reinterpret_cast<const Eigen::Vector3f*> (&(normals->at(*iIt).normal_x));
      covariance_matrix += (*vec) * (vec->transpose());
    }
    point.x = pointIt->x;
    point.y = pointIt->y;
    point.z = pointIt->z;
    point.intensity = covariance_matrix.determinant () / (covariance_matrix.trace () * covariance_matrix.trace ());
    output.points.push_back(point);
  }
  output.height = 1;
  output.width = output.points.size ();
}

template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::responseCurvature (typename PointCloudIn::ConstPtr input, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &output) const
{
  output.points.clear ();
  output.points.reserve (input->points.size());
  
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  pcl::search::KdTree<PointInT> search;
  search.setInputCloud(input);
  
  PointOutT point;
  for (unsigned idx = 0; idx < input->points.size(); ++idx)
  {
    search.radiusSearch (idx, radius_, nn_indices, nn_dists);

    Eigen::Matrix3f covariance_matrix;
    covariance_matrix.setZero();
    for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
    {
      const Eigen::Vector3f* vec = reinterpret_cast<const Eigen::Vector3f*> (&(normals->at(*iIt).normal_x));
      covariance_matrix += (*vec) * (vec->transpose());
    }
    point.x = input->points[idx].x;
    point.y = input->points[idx].y;
    point.z = input->points[idx].z;
    point.intensity = (*normals)[idx].curvature;
    output.points.push_back(point);
  }
  output.height = 1;
  output.width = output.points.size ();
}

template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::responseTomasi (typename PointCloudIn::ConstPtr input, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &output) const
{
  output.points.clear ();
  output.points.reserve (input->points.size());
  
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  pcl::search::KdTree<PointInT> search;
  search.setInputCloud(input);
  
  PointOutT point;
  for (typename PointCloudIn::const_iterator pointIt = input->begin(); pointIt != input->end(); ++pointIt)
  {
    search.radiusSearch (*pointIt, radius_, nn_indices, nn_dists);

    Eigen::Matrix3f covariance_matrix;
    covariance_matrix.setZero();
    for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
    {
      const Eigen::Vector3f* vec = reinterpret_cast<const Eigen::Vector3f*> (&(normals->at(*iIt).normal_x));
      covariance_matrix += (*vec) * (vec->transpose());
    }
    point.x = pointIt->x;
    point.y = pointIt->y;
    point.z = pointIt->z;
    
    EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
    EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
    pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);
    point.intensity = eigen_values[0];
    output.points.push_back(point);
  }
  output.height = 1;
  output.width = output.points.size ();
}

#define PCL_INSTANTIATE_HarrisKeypoint3D(T,U) template class PCL_EXPORTS pcl::HarrisKeypoint3D<T,U>;

#endif // #ifndef PCL_HARRIS_KEYPOINT_3D_IMPL_H_

