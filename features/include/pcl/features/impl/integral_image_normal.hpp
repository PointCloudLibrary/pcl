/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef PCL_FEATURES_INTEGRALIMAGE_BASED_IMPL_NORMAL_ESTIMATOR_H_
#define PCL_FEATURES_INTEGRALIMAGE_BASED_IMPL_NORMAL_ESTIMATOR_H_

#include "pcl/features/integral_image_normal.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation::compute (const pcl::PointCloud<PointInT> &cloud,
                                             pcl::PointCloud<PointOutT> &normals,
                                             const float maxDepthChangeFactor,
                                             const float normalSmoothingSize,
                                             const NormalEstimationMethod normal_estimation_method)
{
  // compute depth-change map
  unsigned char * depthChangeMap = new unsigned char[cloud.height*cloud.width];
  memset(depthChangeMap, 255, cloud.height*cloud.width);

  for (unsigned int row_index = 0; row_index < cloud.height-1; ++row_index)
  {
    for (unsigned int col_index = 0; col_index < cloud.width-1; ++col_index)
    {
      const float depth = cloud.points[row_index*cloud.width + col_index].z;
      const float depthR = cloud.points[row_index*cloud.width + col_index+1].z;
      const float depthD = cloud.points[(row_index+1)*cloud.width + col_index].z;

      const float depthDependendDepthChange = (maxDepthChangeFactor * depth)/(500.0f*0.001f);

      if (abs(depth-depthR) > depthDependendDepthChange)
      {
        depthChangeMap[row_index*cloud.width + col_index] = 0;
        depthChangeMap[row_index*cloud.width + col_index+1] = 0;
      }
      if (abs(depth-depthD) > depthDependendDepthChange)
      {
        depthChangeMap[row_index*cloud.width + col_index] = 0;
        depthChangeMap[(row_index+1)*cloud.width + col_index] = 0;
      }
    }
  }

    
  // compute distance map
  float * distanceMap = new float[cloud.width*cloud.height];
  for (unsigned int index = 0; index < (cloud.width*cloud.height); ++index)
  {
    if (depthChangeMap[index] == 0)
    {
      distanceMap[index] = 0.0f;
    }
    else
    {
      distanceMap[index] = 640.0f;
    }
  }

  // first pass
  for (unsigned int row_index = 1; row_index < cloud.height; ++row_index)
  {
    for (unsigned int col_index = 1; col_index < cloud.width; ++col_index)
    {
      const float upLeft = distanceMap[(row_index-1)*cloud.width + col_index-1] + 1.4f;
      const float up = distanceMap[(row_index-1)*cloud.width + col_index] + 1.0f;
      const float upRight = distanceMap[(row_index-1)*cloud.width + col_index+1] + 1.4f;
      const float left = distanceMap[row_index*cloud.width + col_index-1] + 1.0f;
      const float center = distanceMap[row_index*cloud.width + col_index];

      const float minValue = ::std::min(
        ::std::min(upLeft, up),
        ::std::min(left, upRight) );

      if (minValue < center)
      {
        distanceMap[row_index*cloud.width + col_index] = minValue;
      }
    }
  }

  // second pass
  for (int row_index = cloud.height-2; row_index >= 0; --row_index)
  {
    for (int col_index = cloud.width-2; col_index >= 0; --col_index)
    {
      const float lowerLeft = distanceMap[(row_index+1)*cloud.width + col_index-1] + 1.4f;
      const float lower = distanceMap[(row_index+1)*cloud.width + col_index] + 1.0f;
      const float lowerRight = distanceMap[(row_index+1)*cloud.width + col_index+1] + 1.4f;
      const float right = distanceMap[row_index*cloud.width + col_index+1] + 1.0f;
      const float center = distanceMap[row_index*cloud.width + col_index];

      const float minValue = ::std::min(
        ::std::min(lowerLeft, lower),
        ::std::min(right, lowerRight) );

      if (minValue < center)
      {
        distanceMap[row_index*cloud.width + col_index] = minValue;
      }
    }
  }

  // setup normal estimation
  IntegralImageNormalEstimation normalEstimator;
  normalEstimator.setInputData(
    reinterpret_cast<float*>(&(cloud.points[0])),
    cloud.width, cloud.height,
    3, sizeof(cloud.points[0])/sizeof(float), (sizeof(cloud.points[0])/sizeof(float))*cloud.width, 10.0f,
    normal_estimation_method );
  

  // estimate normals
  normals.width = cloud.width;
  normals.height = cloud.height;
  normals.points.resize(cloud.width*cloud.height);

  pcl::Normal zeroNormal;
  zeroNormal.normal_x = 0;
  zeroNormal.normal_y = 0;
  zeroNormal.normal_z = 0;

  for (int row_index = normalSmoothingSize; row_index < cloud.height-normalSmoothingSize; ++row_index)
  {
    for (int col_index = normalSmoothingSize; col_index < cloud.width-normalSmoothingSize; ++col_index)
    {
      const float depth = cloud.points[row_index*cloud.width + col_index].z;

      if (depth != 0)
      {
        float smoothing = normalSmoothingSize*static_cast<float>(depth)/(500.0f*0.001f);
        smoothing = ::std::min(distanceMap[row_index*cloud.width + col_index], smoothing);

        if (smoothing > 2.0f)
        {
          normalEstimator.setRectSize(smoothing, smoothing);

          ::pcl::Normal normal = normalEstimator.compute(col_index, row_index);

          normals.points[row_index*normals.width + col_index] = normal;
        }
        else
        {
          normals.points[row_index*normals.width + col_index] = zeroNormal;
        }
      }
      else
      {
        normals.points[row_index*normals.width + col_index] = zeroNormal;
      }
    }
  }

  delete[] depthChangeMap;
  delete[] distanceMap;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::IntegralImageNormalEstimation::compute (const pcl::PointCloud<PointInT> &cloud,
                                             pcl::PointCloud<PointOutT> &normals,
                                             const bool useDepthDependentSmoothing,
                                             const float maxDepthChangeFactor,
                                             const float normalSmoothingSize,
                                             const NormalEstimationMethod normal_estimation_method)
{
  // compute depth-change map
  unsigned char * depthChangeMap = new unsigned char[cloud.height*cloud.width];
  memset (depthChangeMap, 255, cloud.height*cloud.width);

  for (unsigned int row_index = 0; row_index < cloud.height-1; ++row_index)
  {
    for (unsigned int col_index = 0; col_index < cloud.width-1; ++col_index)
    {
      const float depth = cloud.points[row_index*cloud.width + col_index].z;
      const float depthR = cloud.points[row_index*cloud.width + col_index+1].z;
      const float depthD = cloud.points[(row_index+1)*cloud.width + col_index].z;

      const float depthDependendDepthChange = (maxDepthChangeFactor * depth)/(500.0f*0.001f);

      if (abs(depth-depthR) > depthDependendDepthChange)
      {
        depthChangeMap[row_index*cloud.width + col_index] = 0;
        depthChangeMap[row_index*cloud.width + col_index+1] = 0;
      }
      if (abs(depth-depthD) > depthDependendDepthChange)
      {
        depthChangeMap[row_index*cloud.width + col_index] = 0;
        depthChangeMap[(row_index+1)*cloud.width + col_index] = 0;
      }
    }
  }

    
  // compute distance map
  float * distanceMap = new float[cloud.width*cloud.height];
  for (int index = 0; index < (cloud.width*cloud.height); ++index)
  {
    if (depthChangeMap[index] == 0)
    {
      distanceMap[index] = 0.0f;
    }
    else
    {
      distanceMap[index] = 640.0f;
    }
  }

  // first pass
  for (unsigned int row_index = 1; row_index < cloud.height; ++row_index)
  {
    for (unsigned int col_index = 1; col_index < cloud.width; ++col_index)
    {
      const float upLeft = distanceMap[(row_index-1)*cloud.width + col_index-1] + 1.4f;
      const float up = distanceMap[(row_index-1)*cloud.width + col_index] + 1.0f;
      const float upRight = distanceMap[(row_index-1)*cloud.width + col_index+1] + 1.4f;
      const float left = distanceMap[row_index*cloud.width + col_index-1] + 1.0f;
      const float center = distanceMap[row_index*cloud.width + col_index];

      const float minValue = ::std::min(
        ::std::min(upLeft, up),
        ::std::min(left, upRight) );

      if (minValue < center)
      {
        distanceMap[row_index*cloud.width + col_index] = minValue;
      }
    }
  }

  // second pass
  for (int row_index = cloud.height-2; row_index >= 0; --row_index)
  {
    for (int col_index = cloud.width-2; col_index >= 0; --col_index)
    {
      const float lowerLeft = distanceMap[(row_index+1)*cloud.width + col_index-1] + 1.4f;
      const float lower = distanceMap[(row_index+1)*cloud.width + col_index] + 1.0f;
      const float lowerRight = distanceMap[(row_index+1)*cloud.width + col_index+1] + 1.4f;
      const float right = distanceMap[row_index*cloud.width + col_index+1] + 1.0f;
      const float center = distanceMap[row_index*cloud.width + col_index];

      const float minValue = ::std::min(
        ::std::min(lowerLeft, lower),
        ::std::min(right, lowerRight) );

      if (minValue < center)
      {
        distanceMap[row_index*cloud.width + col_index] = minValue;
      }
    }
  }


  // setup normal estimation
  IntegralImageNormalEstimation normalEstimator;
  normalEstimator.setInputData(
    reinterpret_cast<float*>(&(cloud.points[0])),
    cloud.width, cloud.height,
    3, sizeof(cloud.points[0])/sizeof(float), (sizeof(cloud.points[0])/sizeof(float))*cloud.width, 10.0f,
    normal_estimation_method );
  

  // estimate normals
  normals.width = cloud.width;
  normals.height = cloud.height;
  normals.points.resize(cloud.width*cloud.height);

  pcl::Normal zeroNormal;
  zeroNormal.normal_x = 0;
  zeroNormal.normal_y = 0;
  zeroNormal.normal_z = 0;

  if (useDepthDependentSmoothing)
  {
    for (int row_index = normalSmoothingSize; row_index < cloud.height-normalSmoothingSize; ++row_index)
    {
      for (int col_index = normalSmoothingSize; col_index < cloud.width-normalSmoothingSize; ++col_index)
      {
        const float depth = cloud.points[row_index*cloud.width + col_index].z;

        if (depth != 0)
        {
          float smoothing = normalSmoothingSize*static_cast<float>(depth)/(500.0f*0.001f);
          smoothing = ::std::min(distanceMap[row_index*cloud.width + col_index], smoothing);

          if (smoothing > 2.0f)
          {
            normalEstimator.setRectSize(smoothing, smoothing);

            ::pcl::Normal normal = normalEstimator.compute(col_index, row_index);

            normals.points[row_index*normals.width + col_index] = normal;
          }
          else
          {
            normals.points[row_index*normals.width + col_index] = zeroNormal;
          }
        }
        else
        {
          normals.points[row_index*normals.width + col_index] = zeroNormal;
        }
      }
    }
  }
  else
  {
    for (int row_index = normalSmoothingSize; row_index < cloud.height-normalSmoothingSize; ++row_index)
    {
      for (int col_index = normalSmoothingSize; col_index < cloud.width-normalSmoothingSize; ++col_index)
      {
        const float depth = cloud.points[row_index*cloud.width + col_index].z;

        if (depth != 0)
        {
          float smoothing = normalSmoothingSize*static_cast<float>(1.0f)/(500.0f*0.001f);
          smoothing = ::std::min(distanceMap[row_index*cloud.width + col_index], smoothing);

          if (smoothing > 2.0f)
          {
            normalEstimator.setRectSize(smoothing, smoothing);

            ::pcl::Normal normal = normalEstimator.compute(col_index, row_index);

            normals.points[row_index*normals.width + col_index] = normal;
          }
          else
          {
            normals.points[row_index*normals.width + col_index] = zeroNormal;
          }
        }
        else
        {
          normals.points[row_index*normals.width + col_index] = zeroNormal;
        }
      }
    }
  }

  delete[] depthChangeMap;
  delete[] distanceMap;
}

#endif

