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
pcl::HarrisKeypoint3D<PointInT, PointOutT>::setRefine (bool do_refine)
{
  refine_ = do_refine;
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
  typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT>);
  pcl::PassThrough<PointInT> pass_;
  pass_.setInputCloud (input_);
  pass_.filter (*cloud);
//  typename pcl::PointCloud<PointInT>::ConstPtr cloud;
//  cloud = input_;
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
    response_search.setInputCloud (response);
    for (size_t idx = 0; idx < response->points.size (); ++idx)
    {
      if (response->points[idx].intensity < threshold_)
        continue;
      
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
    
    if (refine_)
      refineCorners (cloud, normals, output);
    
    output.height = 1;
    output.width = output.points.size();
  }
}

#if 0
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
  float covar[6];
  for (typename PointCloudIn::const_iterator pointIt = input->begin(); pointIt != input->end(); ++pointIt)
  {
    search.radiusSearch (*pointIt, radius_, nn_indices, nn_dists);

    covar[0] = covar[1] = covar[2] = covar[3] = covar[4] = covar[5] = 0;
    for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
    {
      covar[0] += normals->points[*iIt].normal_x * normals->points[*iIt].normal_x;
      covar[1] += normals->points[*iIt].normal_x * normals->points[*iIt].normal_y;
      covar[2] += normals->points[*iIt].normal_x * normals->points[*iIt].normal_z;
      covar[3] += normals->points[*iIt].normal_y * normals->points[*iIt].normal_y;
      covar[4] += normals->points[*iIt].normal_y * normals->points[*iIt].normal_z;
      covar[5] += normals->points[*iIt].normal_z * normals->points[*iIt].normal_z;
    }
    point.x = pointIt->x;
    point.y = pointIt->y;
    point.z = pointIt->z;
    
    float trace = covar[0] + covar[3] + covar[5];
    point.intensity = covar[0] * covar[3] * covar[5] + 2 * covar[1] * covar[2] * covar[4] - covar[2] * covar[2] * covar[3] - 
                      covar[4] * covar[4] * covar[0] - covar[1] * covar[1] * covar[5] - 0.04 * trace * trace;
    output.points.push_back(point);
  }
  output.height = input->height;
  output.width = input->width;
}
#else
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
      covariance_matrix += normals->points[*iIt].getNormalVector3fMap () * 
        normals->points[*iIt].getNormalVector3fMap ().transpose();

    point.x = pointIt->x;
    point.y = pointIt->y;
    point.z = pointIt->z;
    point.intensity = covariance_matrix.determinant () - 0.04 * covariance_matrix.trace () * covariance_matrix.trace ();    
    output.points.push_back(point);
  }
  output.height = input->height;
  output.width = input->width;
}
#endif

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
      covariance_matrix += normals->points[*iIt].getNormalVector3fMap () * 
        normals->points[*iIt].getNormalVector3fMap ().transpose ();

    point.x = pointIt->x;
    point.y = pointIt->y;
    point.z = pointIt->z;
    point.intensity = covariance_matrix.determinant () / covariance_matrix.trace ();
    output.points.push_back(point);
  }
  output.height = input->height;
  output.width = input->width;
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
      covariance_matrix += normals->points[*iIt].getNormalVector3fMap () * 
        normals->points[*iIt].getNormalVector3fMap ().transpose ();

    point.x = pointIt->x;
    point.y = pointIt->y;
    point.z = pointIt->z;
    point.intensity = covariance_matrix.determinant () / (covariance_matrix.trace () * covariance_matrix.trace ());
    output.points.push_back(point);
  }
  output.height = input->height;
  output.width = input->width;
}

template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::responseCurvature (typename PointCloudIn::ConstPtr input, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &output) const
{
  output.points.clear ();
  output.points.reserve (input->points.size());
  
  PointOutT point;
  for (unsigned idx = 0; idx < input->points.size(); ++idx)
  {
    point.x = input->points[idx].x;
    point.y = input->points[idx].y;
    point.z = input->points[idx].z;
    point.intensity = (*normals)[idx].curvature;
    output.points.push_back(point);
  }
  // does not change the order
  output.height = input->height;
  output.width = input->width;
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
      if (pcl_isnan(normals->points[*iIt].normal_x + normals->points[*iIt].normal_y + normals->points[*iIt].normal_z))
        continue;
      covariance_matrix += normals->points[*iIt].getNormalVector3fMap () * 
        normals->points[*iIt].getNormalVector3fMap ().transpose ();
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
  output.height = input->height;
  output.width = input->width;
}

#if 0
template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::refineCorners (typename PointCloudIn::ConstPtr surface, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &corners) const
{
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  pcl::search::KdTree<PointInT> search;
  search.setInputCloud(surface);

  float sumSqr[15];
  float diff;
  const unsigned max_iterations = 10;
  for (typename PointCloudOut::iterator cornerIt = corners.begin(); cornerIt != corners.end(); ++cornerIt)
  {
    unsigned iterations = 0;
    do {
      memset (sumSqr, 0, sizeof(float) * 15);
      PointInT corner;
      corner.x = cornerIt->x;
      corner.y = cornerIt->y;
      corner.z = cornerIt->z;
      search.radiusSearch (corner, radius_, nn_indices, nn_dists);
      for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
      {
        float a = normals->points[*iIt].normal_x * normals->points[*iIt].normal_x;
        float b = normals->points[*iIt].normal_x * normals->points[*iIt].normal_y;
        float c = normals->points[*iIt].normal_x * normals->points[*iIt].normal_z;
        float d = normals->points[*iIt].normal_y * normals->points[*iIt].normal_y;
        float e = normals->points[*iIt].normal_y * normals->points[*iIt].normal_z;
        float f = normals->points[*iIt].normal_z * normals->points[*iIt].normal_z;
        
        sumSqr[0] += a;
        sumSqr[1] += b;
        sumSqr[2] += c;
        sumSqr[3] += d;
        sumSqr[4] += e;
        sumSqr[5] += f;
        sumSqr[6] += a * surface->points[*iIt].x + b * surface->points[*iIt].y + c * surface->points[*iIt].z;
        sumSqr[7] += b * surface->points[*iIt].x + d * surface->points[*iIt].y + e * surface->points[*iIt].z;
        sumSqr[8] += c * surface->points[*iIt].x + e * surface->points[*iIt].y + f * surface->points[*iIt].z;
      }
      
      float det = invert3x3SymMatrix (sumSqr, sumSqr + 9);
      if (det != 0)
      {
        cornerIt->x = sumSqr[ 9] * sumSqr[6] + sumSqr[10] * sumSqr[7] + sumSqr[11] * sumSqr[8];
        cornerIt->y = sumSqr[10] * sumSqr[6] + sumSqr[12] * sumSqr[7] + sumSqr[13] * sumSqr[8];
        cornerIt->z = sumSqr[11] * sumSqr[6] + sumSqr[13] * sumSqr[7] + sumSqr[14] * sumSqr[8];
      }
      diff = (cornerIt->x - corner.x) * (cornerIt->x - corner.x) +
             (cornerIt->y - corner.y) * (cornerIt->y - corner.y) +
             (cornerIt->z - corner.z) * (cornerIt->z - corner.z);
    } while (diff > 1e-5 && ++iterations < max_iterations);
  }
}
#else
template <typename PointInT, typename PointOutT> void 
pcl::HarrisKeypoint3D<PointInT, PointOutT>::refineCorners (typename PointCloudIn::ConstPtr surface, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &corners) const
{
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  pcl::search::KdTree<PointInT> search;
  search.setInputCloud(surface);

  Eigen::Matrix3f nnT;
  Eigen::Matrix3f NNT;
  Eigen::Vector3f NNTp;
  float diff;
  const unsigned max_iterations = 10;
  for (typename PointCloudOut::iterator cornerIt = corners.begin(); cornerIt != corners.end(); ++cornerIt)
  {
    unsigned iterations = 0;
    do {
      NNT.setZero();
      NNTp.setZero();
      PointInT corner;
      corner.x = cornerIt->x;
      corner.y = cornerIt->y;
      corner.z = cornerIt->z;
      search.radiusSearch (corner, radius_, nn_indices, nn_dists);
      for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
      {
        nnT = normals->points[*iIt].getNormalVector3fMap () * normals->points[*iIt].getNormalVector3fMap ().transpose();
        NNT += nnT;
        NNTp += nnT * surface->points[*iIt].getVector3fMap ();
      }
      if (NNT.determinant() != 0)
        cornerIt->getVector3fMap () = NNT.inverse () * NNTp;
      
      diff = (cornerIt->x - corner.x) * (cornerIt->x - corner.x) +
             (cornerIt->y - corner.y) * (cornerIt->y - corner.y) +
             (cornerIt->z - corner.z) * (cornerIt->z - corner.z);
    } while (diff > 1e-5 && ++iterations < max_iterations);
  }
}
#endif
#define PCL_INSTANTIATE_HarrisKeypoint3D(T,U) template class PCL_EXPORTS pcl::HarrisKeypoint3D<T,U>;

#endif // #ifndef PCL_HARRIS_KEYPOINT_3D_IMPL_H_

