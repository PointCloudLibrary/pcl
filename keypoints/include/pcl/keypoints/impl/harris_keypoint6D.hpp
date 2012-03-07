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

#ifndef PCL_HARRIS_KEYPOINT_6D_IMPL_H_
#define PCL_HARRIS_KEYPOINT_6D_IMPL_H_

#include "pcl/keypoints/harris_keypoint6D.h"
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fast_intensity_gradient.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/integral_image_normal.h>

template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint6D<PointInT, PointOutT, NormalT>::setThreshold (float threshold)
{
  threshold_= threshold;
}

template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint6D<PointInT, PointOutT, NormalT>::setRadius (float radius)
{
  search_radius_ = radius;
}

template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint6D<PointInT, PointOutT, NormalT>::setRefine (bool do_refine)
{
  refine_ = do_refine;
}

template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint6D<PointInT, PointOutT, NormalT>::setNonMaxSupression (bool nonmax)
{
  nonmax_ = nonmax;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint6D<PointInT, PointOutT, NormalT>::calculateCombinedCovar (const std::vector<int>& neighbors, float* coefficients) const
{
  memset (coefficients, 0, sizeof (float) * 21);
  unsigned count = 0;
  for (std::vector<int>::const_iterator iIt = neighbors.begin(); iIt != neighbors.end(); ++iIt)
  {
    if (pcl_isfinite (normals_->points[*iIt].normal_x) && pcl_isfinite (intensity_gradients_->points[*iIt].gradient [0]))
    {
      coefficients[ 0] += normals_->points[*iIt].normal_x * normals_->points[*iIt].normal_x;
      coefficients[ 1] += normals_->points[*iIt].normal_x * normals_->points[*iIt].normal_y;
      coefficients[ 2] += normals_->points[*iIt].normal_x * normals_->points[*iIt].normal_z;
      coefficients[ 3] += normals_->points[*iIt].normal_x * intensity_gradients_->points[*iIt].gradient [0];
      coefficients[ 4] += normals_->points[*iIt].normal_x * intensity_gradients_->points[*iIt].gradient [1];
      coefficients[ 5] += normals_->points[*iIt].normal_x * intensity_gradients_->points[*iIt].gradient [2];

      coefficients[ 6] += normals_->points[*iIt].normal_y * normals_->points[*iIt].normal_y;
      coefficients[ 7] += normals_->points[*iIt].normal_y * normals_->points[*iIt].normal_z;
      coefficients[ 8] += normals_->points[*iIt].normal_y * intensity_gradients_->points[*iIt].gradient [0];
      coefficients[ 9] += normals_->points[*iIt].normal_y * intensity_gradients_->points[*iIt].gradient [1];
      coefficients[10] += normals_->points[*iIt].normal_y * intensity_gradients_->points[*iIt].gradient [2];

      coefficients[11] += normals_->points[*iIt].normal_z * normals_->points[*iIt].normal_z;
      coefficients[12] += normals_->points[*iIt].normal_z * intensity_gradients_->points[*iIt].gradient [0];
      coefficients[13] += normals_->points[*iIt].normal_z * intensity_gradients_->points[*iIt].gradient [1];
      coefficients[14] += normals_->points[*iIt].normal_z * intensity_gradients_->points[*iIt].gradient [2];

      coefficients[15] += intensity_gradients_->points[*iIt].gradient [0] * intensity_gradients_->points[*iIt].gradient [0];
      coefficients[16] += intensity_gradients_->points[*iIt].gradient [0] * intensity_gradients_->points[*iIt].gradient [1];
      coefficients[17] += intensity_gradients_->points[*iIt].gradient [0] * intensity_gradients_->points[*iIt].gradient [2];

      coefficients[18] += intensity_gradients_->points[*iIt].gradient [1] * intensity_gradients_->points[*iIt].gradient [1];
      coefficients[19] += intensity_gradients_->points[*iIt].gradient [1] * intensity_gradients_->points[*iIt].gradient [2];

      coefficients[20] += intensity_gradients_->points[*iIt].gradient [2] * intensity_gradients_->points[*iIt].gradient [2];

      ++count;
    }
  }
  if (count > 0)
  {
    float norm = 1.0 / float (count);
    coefficients[ 0] *= norm;
    coefficients[ 1] *= norm;
    coefficients[ 2] *= norm;
    coefficients[ 3] *= norm;
    coefficients[ 4] *= norm;
    coefficients[ 5] *= norm;
    coefficients[ 6] *= norm;
    coefficients[ 7] *= norm;
    coefficients[ 8] *= norm;
    coefficients[ 9] *= norm;
    coefficients[10] *= norm;
    coefficients[11] *= norm;
    coefficients[12] *= norm;
    coefficients[13] *= norm;
    coefficients[14] *= norm;
    coefficients[15] *= norm;
    coefficients[16] *= norm;
    coefficients[17] *= norm;
    coefficients[18] *= norm;
    coefficients[19] *= norm;
    coefficients[10] *= norm;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint6D<PointInT, PointOutT, NormalT>::detectKeypoints (PointCloudOut &output)
{
  //double start = pcl::getTime ();
  //double stop;
  if (normals_->empty ())
  {
    normals_->reserve (surface_->size ());
    if (input_->height == 1 ) // not organized
    {
      //start = pcl::getTime ();
      pcl::NormalEstimation<PointInT, NormalT> normal_estimation;
      normal_estimation.setInputCloud(surface_);
      normal_estimation.setRadiusSearch(search_radius_);
      normal_estimation.compute (*normals_);
    }
    else
    {
      IntegralImageNormalEstimation<PointInT, NormalT> normal_estimation;
      normal_estimation.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointInT, NormalT>::SIMPLE_3D_GRADIENT);

      //double start2 = pcl::getTime ();
      normal_estimation.setInputCloud(surface_);
      normal_estimation.setNormalSmoothingSize (5.0);
      normal_estimation.compute (*normals_);
      //double stop2 = pcl::getTime ();
      //std::cout << "ii normal estimation: " << (stop2 - start2) * 1000.0 << "ms\n";
    }
    //stop = pcl::getTime ();
    //std::cout << "normal estimation: " << (stop - start) * 1000.0 << "ms\n";
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  cloud->resize (surface_->size ());
  for (unsigned idx = 0; idx < surface_->size (); ++idx)
  {
    cloud->points [idx].x = surface_->points [idx].x;
    cloud->points [idx].y = surface_->points [idx].y;
    cloud->points [idx].z = surface_->points [idx].z;
    //grayscale = 0.2989 * R + 0.5870 * G + 0.1140 * B

    cloud->points [idx].intensity = 0.00390625 * (0.114 * float(surface_->points [idx].b) + 0.5870 * float(surface_->points [idx].g) + 0.2989 * float(surface_->points [idx].r));
    //if (cloud->points [idx].intensity > 1.0)
      //std::cout << "intensity :" << cloud->points [idx].intensity << std::endl;
    //cloud->points [idx].intensity = 0.0;
  }
  pcl::copyPointCloud (*surface_, *cloud);

  IntensityGradientEstimation<PointXYZI, NormalT, IntensityGradient> grad_est;
  grad_est.setInputCloud (cloud);
  grad_est.setInputNormals (normals_);
  grad_est.setRadiusSearch (search_radius_);
  grad_est.compute (*intensity_gradients_);
  /*
  float mean = 0;
  for (unsigned idx = 0; idx < intensity_gradients_->size (); ++idx)
  {
    float len = intensity_gradients_->points [idx].gradient_x * intensity_gradients_->points [idx].gradient_x +
                intensity_gradients_->points [idx].gradient_y * intensity_gradients_->points [idx].gradient_y +
                intensity_gradients_->points [idx].gradient_z * intensity_gradients_->points [idx].gradient_z ;

    if (!pcl_isfinite (len))
      continue;

    mean += sqrt (len);
  }

  mean /= intensity_gradients_->size ();
  */
//  float max_len = 0;
  for (unsigned idx = 0; idx < intensity_gradients_->size (); ++idx)
  {
    float len = intensity_gradients_->points [idx].gradient_x * intensity_gradients_->points [idx].gradient_x +
                intensity_gradients_->points [idx].gradient_y * intensity_gradients_->points [idx].gradient_y +
                intensity_gradients_->points [idx].gradient_z * intensity_gradients_->points [idx].gradient_z ;

//    if (len > max_len)
//      max_len = len;
    if (len > 200.0) //mean * 0.2)
    {
      len = 1.0 / sqrt (len);
      //len = 0.05;
      intensity_gradients_->points [idx].gradient_x *= len;
      intensity_gradients_->points [idx].gradient_y *= len;
      intensity_gradients_->points [idx].gradient_z *= len;
    }
    else
    {
      intensity_gradients_->points [idx].gradient_x = 0;
      intensity_gradients_->points [idx].gradient_y = 0;
      intensity_gradients_->points [idx].gradient_z = 0;
    }
  }
//  std::cout << "gradient length: " << sqrt(max_len) << std::endl;

//  FastIntensityGradientEstimation<pcl::PointXYZI, pcl::IntensityGradient> gradient_estimation;
//  gradient_estimation.setInputCloud (cloud);
//  gradient_estimation.setInputCloud (cloud);
//  gradient_estimation.setRadiusSearch (search_radius_);
//  gradient_estimation.compute (*intensity_gradients_);
  //start = pcl::getTime ();
  boost::shared_ptr<pcl::PointCloud<PointOutT> > response (new pcl::PointCloud<PointOutT> ());
  response->points.reserve (input_->points.size());
  responseTomasi(*response);

  // just return the response
  if (!nonmax_)
    output = *response;
  else
  {
    output.points.clear ();
    output.points.reserve (response->points.size());
    std::vector<int> nn_indices;
    std::vector<float> nn_dists;

    //#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(8)
    for (size_t idx = 0; idx < response->points.size (); ++idx)
    {
      if (!isFinite (response->points[idx]) || response->points[idx].intensity < threshold_)
        continue;
      tree_->radiusSearch (idx, search_radius_, nn_indices, nn_dists);
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
        #pragma omp critical
        output.points.push_back (response->points[idx]);
    }
    if (refine_)
      refineCorners (output);
    output.height = 1;
    output.width = output.points.size();
  }

  // we don not change the denseness
  output.is_dense = input_->is_dense;
}

template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint6D<PointInT, PointOutT, NormalT>::responseTomasi (PointCloudOut &output) const
{
  // get the 6x6 covar-mat
  PointOutT pointOut;
  PCL_ALIGN (16) float covar [21];
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  Eigen::SelfAdjointEigenSolver <Eigen::Matrix<float, 6, 6> > solver;
  Eigen::Matrix<float, 6, 6> covariance;
  #pragma omp parallel for shared (output) private (pointOut, covar, covariance, nn_indices, nn_dists, solver)
  for (unsigned pIdx = 0; pIdx < input_->size (); ++pIdx)
  {
    const PointInT& pointIn = input_->points [pIdx];
    pointOut.intensity = 0.0; //std::numeric_limits<float>::quiet_NaN ();
    if (isFinite (pointIn))
    {
      tree_->radiusSearch (pointIn, search_radius_, nn_indices, nn_dists);
      calculateCombinedCovar (nn_indices, covar);

      float trace = covar [0] + covar [6] + covar [11] + covar [15] + covar [18] + covar [20];
      if (trace != 0)
      {
        covariance.coeffRef ( 0) = covar [ 0];
        covariance.coeffRef ( 1) = covar [ 1];
        covariance.coeffRef ( 2) = covar [ 2];
        covariance.coeffRef ( 3) = covar [ 3];
        covariance.coeffRef ( 4) = covar [ 4];
        covariance.coeffRef ( 5) = covar [ 5];

        covariance.coeffRef ( 7) = covar [ 6];
        covariance.coeffRef ( 8) = covar [ 7];
        covariance.coeffRef ( 9) = covar [ 8];
        covariance.coeffRef (10) = covar [ 9];
        covariance.coeffRef (11) = covar [10];

        covariance.coeffRef (14) = covar [11];
        covariance.coeffRef (15) = covar [12];
        covariance.coeffRef (16) = covar [13];
        covariance.coeffRef (17) = covar [14];

        covariance.coeffRef (21) = covar [15];
        covariance.coeffRef (22) = covar [16];
        covariance.coeffRef (23) = covar [17];

        covariance.coeffRef (28) = covar [18];
        covariance.coeffRef (29) = covar [19];

        covariance.coeffRef (35) = covar [20];

        covariance.coeffRef ( 6) = covar [ 1];

        covariance.coeffRef (12) = covar [ 2];
        covariance.coeffRef (13) = covar [ 7];

        covariance.coeffRef (18) = covar [ 3];
        covariance.coeffRef (19) = covar [ 8];
        covariance.coeffRef (20) = covar [12];

        covariance.coeffRef (24) = covar [ 4];
        covariance.coeffRef (25) = covar [ 9];
        covariance.coeffRef (26) = covar [13];
        covariance.coeffRef (27) = covar [16];

        covariance.coeffRef (30) = covar [ 5];
        covariance.coeffRef (31) = covar [10];
        covariance.coeffRef (32) = covar [14];
        covariance.coeffRef (33) = covar [17];
        covariance.coeffRef (34) = covar [19];

        solver.compute (covariance);
        pointOut.intensity = solver.eigenvalues () [3];
//        std::cout << "eigenvalues: \n" << solver.eigenvalues () [2] << " , "
//                                       << solver.eigenvalues () [3] << " , "
//                                       << solver.eigenvalues () [4] << std::endl;
        //std::cout << "covariance matrix:\n" << covariance << std::endl;
        //std::cout << "eigenvalues: \n" << solver.eigenvalues () << std::endl;
      }
    }

    pointOut.x = pointIn.x;
    pointOut.y = pointIn.y;
    pointOut.z = pointIn.z;
    #pragma omp critical
    output.points.push_back(pointOut);
  }
  output.height = input_->height;
  output.width = input_->width;
}

template <typename PointInT, typename PointOutT, typename NormalT> void
pcl::HarrisKeypoint6D<PointInT, PointOutT, NormalT>::refineCorners (PointCloudOut &corners) const
{
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  pcl::search::KdTree<PointInT> search;
  search.setInputCloud(surface_);

  Eigen::Matrix3f nnT;
  Eigen::Matrix3f NNT;
  Eigen::Vector3f NNTp;
  const Eigen::Vector3f* normal;
  const Eigen::Vector3f* point;
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
      search.radiusSearch (corner, search_radius_, nn_indices, nn_dists);
      for (std::vector<int>::const_iterator iIt = nn_indices.begin(); iIt != nn_indices.end(); ++iIt)
      {
        normal = reinterpret_cast<const Eigen::Vector3f*> (&(normals_->points[*iIt].normal_x));
        point = reinterpret_cast<const Eigen::Vector3f*> (&(surface_->points[*iIt].x));
        nnT = (*normal) * (normal->transpose());
        NNT += nnT;
        NNTp += nnT * (*point);
      }
      if (NNT.determinant() != 0)
        *(reinterpret_cast<Eigen::Vector3f*>(&(cornerIt->x))) = NNT.inverse () * NNTp;

      diff = (cornerIt->x - corner.x) * (cornerIt->x - corner.x) +
             (cornerIt->y - corner.y) * (cornerIt->y - corner.y) +
             (cornerIt->z - corner.z) * (cornerIt->z - corner.z);

    } while (diff > 1e-6 && ++iterations < max_iterations);
  }
}

#define PCL_INSTANTIATE_HarrisKeypoint6D(T,U,N) template class PCL_EXPORTS pcl::HarrisKeypoint6D<T,U,N>;
#endif // #ifndef PCL_HARRIS_KEYPOINT_6D_IMPL_H_

