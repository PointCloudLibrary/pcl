/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 */

#include <gtest/gtest.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/common/io.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_tests.h>

#include <pcl/common/centroid.h>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, compute3DCentroidFloat)
{
  pcl::PointIndices pindices;
  std::vector<int> indices;
  PointXYZ point;
  PointCloud<PointXYZ> cloud;
  Eigen::Vector4f centroid;

  // test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 0);

  // test empty cloud non_dense
  cloud.is_dense = false;
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 0);

  cloud.clear ();
  indices.clear ();
  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // eight points with (0, 0, 0) as centroid and covarmat (1, 0, 0, 0, 1, 0, 0, 0, 1)
  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;

  EXPECT_EQ (compute3DCentroid (cloud, centroid), 8);
  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 0);
  EXPECT_EQ (centroid [2], 0);
  EXPECT_EQ (centroid [3], 1);

  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;
  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);
  EXPECT_EQ (centroid [3], 1.0);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;

  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 8);

  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 0);
  EXPECT_EQ (centroid [2], 0);
  EXPECT_EQ (centroid [3], 1);

  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  indices.push_back (8); // add the NaN
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);
  EXPECT_EQ (centroid [3], 1.0);

  pindices.indices = indices;
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);
  EXPECT_EQ (centroid [3], 1.0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, compute3DCentroidDouble)
{
  pcl::PointIndices pindices;
  std::vector<int> indices;
  PointXYZ point;
  PointCloud<PointXYZ> cloud;
  Eigen::Vector4d centroid;

  // test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 0);

  // test empty cloud non_dense
  cloud.is_dense = false;
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 0);

  cloud.clear ();
  indices.clear ();
  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // eight points with (0, 0, 0) as centroid and covarmat (1, 0, 0, 0, 1, 0, 0, 0, 1)
  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;

  EXPECT_EQ (compute3DCentroid (cloud, centroid), 8);
  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 0);
  EXPECT_EQ (centroid [2], 0);
  EXPECT_EQ (centroid [3], 1);

  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;
  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;

  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;
  EXPECT_EQ (compute3DCentroid (cloud, centroid), 8);

  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 0);
  EXPECT_EQ (centroid [2], 0);
  EXPECT_EQ (centroid [3], 1);

  centroid [0] = -100;
  centroid [1] = -200;
  centroid [2] = -300;
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  indices.push_back (8); // add the NaN
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);
  EXPECT_EQ (centroid [3], 1.0);
  
  pindices.indices = indices;
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);
  EXPECT_EQ (centroid [3], 1.0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, compute3DCentroidCloudIterator)
{
  pcl::PointIndices pindices;
  std::vector<int> indices;
  PointXYZ point;
  PointCloud<PointXYZ> cloud;
  Eigen::Vector4f centroid_f;

  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;

  ConstCloudIterator<PointXYZ> it (cloud, indices);
  
  EXPECT_EQ (compute3DCentroid (it, centroid_f), 4);

  EXPECT_EQ (centroid_f[0], 0.0f);
  EXPECT_EQ (centroid_f[1], 1.0f);
  EXPECT_EQ (centroid_f[2], 0.0f);
  EXPECT_EQ (centroid_f[3], 1.0f);
  
  Eigen::Vector4d centroid_d;
  it.reset ();
  EXPECT_EQ (compute3DCentroid (it, centroid_d), 4);

  EXPECT_EQ (centroid_d[0], 0.0);
  EXPECT_EQ (centroid_d[1], 1.0);
  EXPECT_EQ (centroid_d[2], 0.0);
  EXPECT_EQ (centroid_d[3], 1.0);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, computeCovarianceMatrix)
{
  PointCloud<PointXYZ> cloud;
  PointXYZ point;
  std::vector <int> indices;
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;

  centroid [0] = 0;
  centroid [1] = 0;
  centroid [2] = 0;

  // test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (computeCovarianceMatrix (cloud, centroid, covariance_matrix), 0);

  // test empty cloud non_dense
  cloud.is_dense = false;
  EXPECT_EQ (computeCovarianceMatrix (cloud, centroid, covariance_matrix), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (computeCovarianceMatrix (cloud, centroid, covariance_matrix), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (computeCovarianceMatrix (cloud, indices, centroid, covariance_matrix), 0);

  cloud.clear ();
  indices.clear ();
  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // eight points with (0, 0, 0) as centroid and covarmat (1, 0, 0, 0, 1, 0, 0, 0, 1)

  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [0] = 0;
  centroid [1] = 0;
  centroid [2] = 0;

  EXPECT_EQ (computeCovarianceMatrix (cloud, centroid, covariance_matrix), 8);
  EXPECT_EQ (covariance_matrix (0, 0), 8);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 8);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 8);

  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [1] = 1;

  EXPECT_EQ (computeCovarianceMatrix (cloud, indices, centroid, covariance_matrix), 4);
  EXPECT_EQ (covariance_matrix (0, 0), 4);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 0);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 4);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [1] = 0;

  EXPECT_EQ (computeCovarianceMatrix (cloud, centroid, covariance_matrix), 8);
  EXPECT_EQ (covariance_matrix (0, 0), 8);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 8);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 8);

  indices.push_back (8); // add the NaN
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [1] = 1;

  EXPECT_EQ (computeCovarianceMatrix (cloud, indices, centroid, covariance_matrix), 4);
  EXPECT_EQ (covariance_matrix (0, 0), 4);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 0);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 4);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, computeCovarianceMatrixNormalized)
{
  PointCloud<PointXYZ> cloud;
  PointXYZ point;
  std::vector <int> indices;
  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;

  centroid [0] = 0;
  centroid [1] = 0;
  centroid [2] = 0;

  // test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, centroid, covariance_matrix), 0);

  // test empty cloud non_dense
  cloud.is_dense = false;
  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, centroid, covariance_matrix), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, centroid, covariance_matrix), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, indices, centroid, covariance_matrix), 0);

  cloud.clear ();
  indices.clear ();
  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // eight points with (0, 0, 0) as centroid and covarmat (1, 0, 0, 0, 1, 0, 0, 0, 1)

  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [0] = 0;
  centroid [1] = 0;
  centroid [2] = 0;

  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, centroid, covariance_matrix), 8);

  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [1] = 1;

  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, indices, centroid, covariance_matrix), 4);

  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 0);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [1] = 0;

  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, centroid, covariance_matrix), 8);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  indices.push_back (8); // add the NaN
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [1] = 1;

  EXPECT_EQ (computeCovarianceMatrixNormalized (cloud, indices, centroid, covariance_matrix), 4);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 0);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, computeDemeanedCovariance)
{
  PointCloud<PointXYZ> cloud;
  PointXYZ point;
  std::vector <int> indices;
  Eigen::Matrix3f covariance_matrix;

  // test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (computeCovarianceMatrix (cloud, covariance_matrix), 0);

  // test empty cloud non_dense
  cloud.is_dense = false;
  EXPECT_EQ (computeCovarianceMatrix (cloud, covariance_matrix), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (computeCovarianceMatrix (cloud, covariance_matrix), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (computeCovarianceMatrix (cloud, indices, covariance_matrix), 0);

  cloud.clear ();
  indices.clear ();

  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // eight points with (0, 0, 0) as centroid and covarmat (1, 0, 0, 0, 1, 0, 0, 0, 1)

  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;

  EXPECT_EQ (computeCovarianceMatrix (cloud, covariance_matrix), 8);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;

  EXPECT_EQ (computeCovarianceMatrix (cloud, indices, covariance_matrix), 4);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;

  EXPECT_EQ (computeCovarianceMatrix (cloud, covariance_matrix), 8);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  indices.push_back (8); // add the NaN
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;

  EXPECT_EQ (computeCovarianceMatrix (cloud, indices, covariance_matrix), 4);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, computeMeanAndCovariance)
{
  PointCloud<PointXYZ> cloud;
  PointXYZ point;
  std::vector <int> indices;
  Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f centroid;

  // test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, covariance_matrix, centroid), 0);

  // test empty cloud non_dense
  cloud.is_dense = false;
  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, covariance_matrix, centroid), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, covariance_matrix, centroid), 0);

  // test non-empty cloud non_dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, indices, covariance_matrix, centroid), 0);

  cloud.clear ();
  indices.clear ();

  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // eight points with (0, 0, 0) as centroid and covarmat (1, 0, 0, 0, 1, 0, 0, 0, 1)

  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [0] = -100;
  centroid [1] = -101;
  centroid [2] = -102;
  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, covariance_matrix, centroid), 8);

  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 0);
  EXPECT_EQ (centroid [2], 0);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  indices.resize (4); // only positive y values
  indices [0] = 2;
  indices [1] = 3;
  indices [2] = 6;
  indices [3] = 7;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [0] = -100;
  centroid [1] = -101;
  centroid [2] = -102;

  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, indices, covariance_matrix, centroid), 4);
  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 1);
  EXPECT_EQ (centroid [2], 0);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 0);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [0] = -100;
  centroid [1] = -101;
  centroid [2] = -102;

  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, covariance_matrix, centroid), 8);
  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 0);
  EXPECT_EQ (centroid [2], 0);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 1);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);

  indices.push_back (8); // add the NaN
  covariance_matrix << -100, -101, -102, -110, -111, -112, -120, -121, -122;
  centroid [0] = -100;
  centroid [1] = -101;
  centroid [2] = -102;

  EXPECT_EQ (computeMeanAndCovarianceMatrix (cloud, indices, covariance_matrix, centroid), 4);
  EXPECT_EQ (centroid [0], 0);
  EXPECT_EQ (centroid [1], 1);
  EXPECT_EQ (centroid [2], 0);
  EXPECT_EQ (covariance_matrix (0, 0), 1);
  EXPECT_EQ (covariance_matrix (0, 1), 0);
  EXPECT_EQ (covariance_matrix (0, 2), 0);
  EXPECT_EQ (covariance_matrix (1, 0), 0);
  EXPECT_EQ (covariance_matrix (1, 1), 0);
  EXPECT_EQ (covariance_matrix (1, 2), 0);
  EXPECT_EQ (covariance_matrix (2, 0), 0);
  EXPECT_EQ (covariance_matrix (2, 1), 0);
  EXPECT_EQ (covariance_matrix (2, 2), 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, CentroidPoint)
{
  PointXYZ p1; p1.getVector3fMap () << 1, 2, 3;
  PointXYZ p2; p2.getVector3fMap () << 3, 2, 1;
  PointXYZ p3; p3.getVector3fMap () << 5, 5, 5;

  // Zero points (get should have no effect)
  {
    CentroidPoint<PointXYZ> centroid;
    PointXYZ c (100, 100, 100);
    centroid.get (c);
    EXPECT_XYZ_EQ (PointXYZ (100, 100, 100), c);
  }
  // Single point
  {
    CentroidPoint<PointXYZ> centroid;
    centroid.add (p1);
    PointXYZ c;
    centroid.get (c);
    EXPECT_XYZ_EQ (p1, c);
  }
  // Multiple points
  {
    CentroidPoint<PointXYZ> centroid;
    centroid.add (p1);
    centroid.add (p2);
    centroid.add (p3);
    PointXYZ c;
    centroid.get (c);
    EXPECT_XYZ_EQ (PointXYZ (3, 3, 3), c);
  }

  // Retrieve centroid into a different point type
  {
    CentroidPoint<PointXYZ> centroid;
    centroid.add (p1);
    PointXYZRGB c; c.rgba = 0x00FFFFFF;
    centroid.get (c);
    EXPECT_XYZ_EQ (p1, c);
    EXPECT_EQ (0x00FFFFFF, c.rgba);
  }

  // Centroid with XYZ and RGB
  {
    PointXYZRGB cp1; cp1.getVector3fMap () << 4, 2, 4; cp1.rgba = 0xFF330000;
    PointXYZRGB cp2; cp2.getVector3fMap () << 2, 4, 2; cp2.rgba = 0xFF003300;
    PointXYZRGB cp3; cp3.getVector3fMap () << 3, 3, 3; cp3.rgba = 0xFF000033;
    CentroidPoint<PointXYZRGB> centroid;
    centroid.add (cp1);
    centroid.add (cp2);
    centroid.add (cp3);
    PointXYZRGB c;
    centroid.get (c);
    EXPECT_XYZ_EQ (PointXYZ (3, 3, 3), c);
    EXPECT_EQ (0xFF111111, c.rgba);
  }

  // Centroid with normal and curavture
  {
    Normal np1; np1.getNormalVector4fMap () << 1, 0, 0, 0; np1.curvature = 0.2;
    Normal np2; np2.getNormalVector4fMap () << -1, 0, 0, 0; np2.curvature = 0.1;
    Normal np3; np3.getNormalVector4fMap () << 0, 1, 0, 0; np3.curvature = 0.9;
    CentroidPoint<Normal> centroid;
    centroid.add (np1);
    centroid.add (np2);
    centroid.add (np3);
    Normal c;
    centroid.get (c);
    EXPECT_NORMAL_EQ (np3, c);
    EXPECT_FLOAT_EQ (0.4, c.curvature);
  }

  // Centroid with XYZ and intensity
  {
    PointXYZI ip1; ip1.getVector3fMap () << 1, 2, 3; ip1.intensity = 0.8;
    PointXYZI ip2; ip2.getVector3fMap () << 3, 2, 1; ip2.intensity = 0.2;
    PointXYZI ip3; ip3.getVector3fMap () << 5, 5, 5; ip3.intensity = 0.2;
    CentroidPoint<PointXYZI> centroid;
    centroid.add (ip1);
    centroid.add (ip2);
    centroid.add (ip3);
    PointXYZI c;
    centroid.get (c);
    EXPECT_XYZ_EQ (PointXYZ (3, 3, 3), c);
    EXPECT_FLOAT_EQ (0.4, c.intensity);
  }

  // Centroid with label
  {
    Label lp1; lp1.label = 1;
    Label lp2; lp2.label = 1;
    Label lp3; lp3.label = 2;
    CentroidPoint<Label> centroid;
    centroid.add (lp1);
    centroid.add (lp2);
    centroid.add (lp3);
    Label c;
    centroid.get (c);
    EXPECT_EQ (1, c.label);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, computeCentroid)
{
  std::vector<int> indices;
  PointXYZI point;
  PointCloud<PointXYZI> cloud;
  PointXYZINormal centroid;

  // Test empty cloud which is dense
  cloud.is_dense = true;
  EXPECT_EQ (0, computeCentroid (cloud, centroid));

  // Test empty cloud which is not dense
  cloud.is_dense = false;
  EXPECT_EQ (0, computeCentroid (cloud, centroid));

  // Test non-empty cloud which is not dense
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  EXPECT_EQ (0, computeCentroid (cloud, centroid));

  // Test non-empty cloud which is not dense (with indices)
  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  indices.push_back (1);
  EXPECT_EQ (0, computeCentroid (cloud, indices, centroid));

  cloud.clear ();
  indices.clear ();
  for (point.x = -1; point.x < 2; point.x += 2)
  {
    for (point.y = -1; point.y < 2; point.y += 2)
    {
      for (point.z = -1; point.z < 2; point.z += 2)
      {
        point.intensity = point.y;
        cloud.push_back (point);
      }
    }
  }
  cloud.is_dense = true;

  // Eight points with (0, 0, 0) as centroid
  centroid.x = -100;
  centroid.y = -200;
  centroid.z = -300;
  centroid.intensity = -400;
  centroid.curvature = -500;

  EXPECT_EQ (8, computeCentroid (cloud, centroid));
  EXPECT_FLOAT_EQ (0, centroid.x);
  EXPECT_FLOAT_EQ (0, centroid.y);
  EXPECT_FLOAT_EQ (0, centroid.z);
  EXPECT_FLOAT_EQ (0, centroid.intensity);
  EXPECT_FLOAT_EQ (-500, centroid.curvature);

  centroid.x = -100;
  centroid.y = -200;
  centroid.z = -300;
  centroid.intensity = -400;
  centroid.curvature = -500;

  // Only positive y values
  indices.resize (4);
  indices[0] = 2;
  indices[1] = 3;
  indices[2] = 6;
  indices[3] = 7;
  EXPECT_EQ (4, computeCentroid (cloud, indices, centroid));

  EXPECT_FLOAT_EQ (0, centroid.x);
  EXPECT_FLOAT_EQ (1, centroid.y);
  EXPECT_FLOAT_EQ (0, centroid.z);
  EXPECT_FLOAT_EQ (1, centroid.intensity);
  EXPECT_FLOAT_EQ (-500, centroid.curvature);

  point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
  cloud.push_back (point);
  cloud.is_dense = false;

  centroid.x = -100;
  centroid.y = -200;
  centroid.z = -300;
  centroid.intensity = -400;
  centroid.curvature = -500;

  EXPECT_EQ (8, computeCentroid (cloud, centroid));

  EXPECT_FLOAT_EQ (0, centroid.x);
  EXPECT_FLOAT_EQ (0, centroid.y);
  EXPECT_FLOAT_EQ (0, centroid.z);
  EXPECT_FLOAT_EQ (0, centroid.intensity);
  EXPECT_FLOAT_EQ (-500, centroid.curvature);

  centroid.x = -100;
  centroid.y = -200;
  centroid.z = -300;
  centroid.intensity = -400;
  centroid.curvature = -500;

  indices.push_back (8); // add the NaN
  EXPECT_EQ (4, computeCentroid (cloud, indices, centroid));

  EXPECT_FLOAT_EQ (0, centroid.x);
  EXPECT_FLOAT_EQ (1, centroid.y);
  EXPECT_FLOAT_EQ (0, centroid.z);
  EXPECT_FLOAT_EQ (1, centroid.intensity);
  EXPECT_FLOAT_EQ (-500, centroid.curvature);
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

