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

  pindices.indices = indices;
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);
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
  
  pindices.indices = indices;
  EXPECT_EQ (compute3DCentroid (cloud, indices, centroid), 4);

  EXPECT_EQ (centroid [0], 0.0);
  EXPECT_EQ (centroid [1], 1.0);
  EXPECT_EQ (centroid [2], 0.0);
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
  
  Eigen::Vector4d centroid_d;
  it.reset ();
  EXPECT_EQ (compute3DCentroid (it, centroid_d), 4);

  EXPECT_EQ (centroid_d[0], 0.0);
  EXPECT_EQ (centroid_d[1], 1.0);
  EXPECT_EQ (centroid_d[2], 0.0);
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

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

