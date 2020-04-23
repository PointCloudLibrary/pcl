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
 * $Id$
 *
 */

#include <pcl/test/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

using KdTreePtr = search::KdTree<PointXYZ>::Ptr;

PointCloud<PointXYZ> cloud;
std::vector<int> indices;
KdTreePtr tree;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, computePointNormal)
{
  Eigen::Vector4f plane_parameters;
  float curvature;

  PointCloud<PointXYZ> c;
  
  PointXYZ p11 (706952.31f, 4087.6958f, 0.00000000f),
           p21 (707002.31f, 6037.6958f, 0.00000000f),
           p31 (706952.31f, 7937.6958f, 0.00000000f);
  c.push_back (p11); c.push_back (p21); c.push_back (p31);

  computePointNormal (cloud, plane_parameters, curvature);
//  std::cerr << plane_parameters << "\n";
  
  c.clear ();
  PointXYZ p12 (-439747.72f, -43597.250f, 0.0000000f),
           p22 (-439847.72f, -41697.250f, 0.0000000f),
           p32 (-439747.72f, -39797.250f, 0.0000000f);

  c.push_back (p12); c.push_back (p22); c.push_back (p32);

  computePointNormal (cloud, plane_parameters, curvature);
//  std::cerr << plane_parameters << "\n";

  c.clear ();
  PointXYZ p13 (567011.56f, -7741.8179f, 0.00000000f),
           p23 (567361.56f, -5841.8179f, 0.00000000f),
           p33 (567011.56f, -3941.8179f, 0.00000000f);

  c.push_back (p13); c.push_back (p23); c.push_back (p33);

  computePointNormal (cloud, plane_parameters, curvature);
//  std::cerr << plane_parameters << "\n";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, NormalEstimation)
{
  Eigen::Vector4f plane_parameters;
  float curvature;

  NormalEstimation<PointXYZ, Normal> n;

  // computePointNormal (indices, Vector)
  computePointNormal (cloud, indices, plane_parameters, curvature);
  EXPECT_NEAR (std::abs (plane_parameters[0]), 0.035592, 1e-4);
  EXPECT_NEAR (std::abs (plane_parameters[1]), 0.369596, 1e-4);
  EXPECT_NEAR (std::abs (plane_parameters[2]), 0.928511, 1e-4);
  EXPECT_NEAR (std::abs (plane_parameters[3]), 0.0622552, 1e-4);
  EXPECT_NEAR (curvature, 0.0693136, 1e-4);

  float nx, ny, nz;
  // computePointNormal (indices)
  n.computePointNormal (cloud, indices, nx, ny, nz, curvature);
  EXPECT_NEAR (std::abs (nx), 0.035592, 1e-4);
  EXPECT_NEAR (std::abs (ny), 0.369596, 1e-4);
  EXPECT_NEAR (std::abs (nz), 0.928511, 1e-4);
  EXPECT_NEAR (curvature, 0.0693136, 1e-4);

  // computePointNormal (Vector)
  computePointNormal (cloud, plane_parameters, curvature);
  EXPECT_NEAR (plane_parameters[0],  0.035592,  1e-4);
  EXPECT_NEAR (plane_parameters[1],  0.369596,  1e-4);
  EXPECT_NEAR (plane_parameters[2],  0.928511,  1e-4);
  EXPECT_NEAR (plane_parameters[3], -0.0622552, 1e-4);
  EXPECT_NEAR (curvature,            0.0693136, 1e-4);

  // flipNormalTowardsViewpoint (Vector)
  flipNormalTowardsViewpoint (cloud.points[0], 0, 0, 0, plane_parameters);
  EXPECT_NEAR (plane_parameters[0], -0.035592,  1e-4);
  EXPECT_NEAR (plane_parameters[1], -0.369596,  1e-4);
  EXPECT_NEAR (plane_parameters[2], -0.928511,  1e-4);
  EXPECT_NEAR (plane_parameters[3],  0.0799743, 1e-4);

  // flipNormalTowardsViewpoint
  flipNormalTowardsViewpoint (cloud.points[0], 0, 0, 0, nx, ny, nz);
  EXPECT_NEAR (nx, -0.035592, 1e-4);
  EXPECT_NEAR (ny, -0.369596, 1e-4);
  EXPECT_NEAR (nz, -0.928511, 1e-4);

  // Object
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());

  // set parameters
  PointCloud<PointXYZ>::Ptr cloudptr = cloud.makeShared ();
  n.setInputCloud (cloudptr);
  EXPECT_EQ (n.getInputCloud (), cloudptr);
  pcl::IndicesPtr indicesptr (new pcl::Indices (indices));
  n.setIndices (indicesptr);
  EXPECT_EQ (n.getIndices (), indicesptr);
  n.setSearchMethod (tree);
  EXPECT_EQ (n.getSearchMethod (), tree);
  n.setKSearch (static_cast<int> (indices.size ()));

  // estimate
  n.compute (*normals);
  EXPECT_EQ (normals->points.size (), indices.size ());

  for (const auto &point : normals->points)
  {
    EXPECT_NEAR (point.normal[0], -0.035592, 1e-4);
    EXPECT_NEAR (point.normal[1], -0.369596, 1e-4);
    EXPECT_NEAR (point.normal[2], -0.928511, 1e-4);
    EXPECT_NEAR (point.curvature, 0.0693136, 1e-4);
  }

  PointCloud<PointXYZ>::Ptr surfaceptr = cloudptr;
  n.setSearchSurface (surfaceptr);
  EXPECT_EQ (n.getSearchSurface (), surfaceptr);

  // Additional test for searchForNeigbhors
  surfaceptr.reset (new PointCloud<PointXYZ>);
  *surfaceptr = *cloudptr;
  surfaceptr->points.resize (640 * 480);
  surfaceptr->width = 640;
  surfaceptr->height = 480;
  EXPECT_EQ (surfaceptr->points.size (), surfaceptr->width * surfaceptr->height);
  n.setSearchSurface (surfaceptr);
  tree.reset ();
  n.setSearchMethod (tree);

  // estimate
  n.compute (*normals);
  EXPECT_EQ (normals->points.size (), indices.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// There was an issue where the vectors for the indices and the
// distances of the neighbor search weren't initialized correctly
// due to a wrong usage of OpenMP.
// See PR #3614 for details.
// This tests if the vectors are initialized correctly.
template<typename PointT>
class DummySearch : public pcl::search::Search<PointT>
{
  public:
    virtual int nearestKSearch (const PointT &point, int k, std::vector<int> &k_indices,
                                std::vector<float> &k_sqr_distances ) const
    {
      (void)point;

      EXPECT_GE (k_indices.size(), k);
      EXPECT_GE (k_sqr_distances.size(), k);

	  return k;
    }

    virtual int radiusSearch (const PointT& point, double radius, std::vector<int>& k_indices,
                              std::vector<float>& k_sqr_distances, unsigned int max_nn = 0 ) const
    {
      (void)point;
      (void)radius;
      (void)k_indices;
      (void)k_sqr_distances;

      return max_nn;
    }
};


TEST (PCL, NormalEstimationOpenMPInitVectors)
{
  NormalEstimationOMP<PointXYZ, Normal> n (4); // instantiate 4 threads

  DummySearch<PointXYZ>::Ptr dummy (new DummySearch<PointXYZ>);

  // Object
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());

  // set parameters
  PointCloud<PointXYZ>::Ptr cloudptr = cloud.makeShared ();
  n.setInputCloud (cloudptr);
  EXPECT_EQ (n.getInputCloud (), cloudptr);
  pcl::IndicesPtr indicesptr (new pcl::Indices (indices));
  n.setIndices (indicesptr);
  EXPECT_EQ (n.getIndices (), indicesptr);
  n.setSearchMethod (dummy);
  EXPECT_EQ (n.getSearchMethod (), dummy);
  n.setKSearch (static_cast<int> (indices.size ()));

  // This will cause a call to DummySearch::nearestKSearch
  // which checks if the vectors for the indices and the
  // distances are correctly set up. See PR #3614.
  n.compute (*normals);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, NormalEstimationOpenMP)
{
  NormalEstimationOMP<PointXYZ, Normal> n (4); // instantiate 4 threads

  // Object
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());

  // set parameters
  PointCloud<PointXYZ>::Ptr cloudptr = cloud.makeShared ();
  n.setInputCloud (cloudptr);
  EXPECT_EQ (n.getInputCloud (), cloudptr);
  pcl::IndicesPtr indicesptr (new pcl::Indices (indices));
  n.setIndices (indicesptr);
  EXPECT_EQ (n.getIndices (), indicesptr);
  n.setSearchMethod (tree);
  EXPECT_EQ (n.getSearchMethod (), tree);
  n.setKSearch (static_cast<int> (indices.size ()));

  // estimate
  n.compute (*normals);
  EXPECT_EQ (normals->points.size (), indices.size ());

  for (const auto &point : normals->points)
  {
    EXPECT_NEAR (point.normal[0], -0.035592, 1e-4);
    EXPECT_NEAR (point.normal[1], -0.369596, 1e-4);
    EXPECT_NEAR (point.normal[2], -0.928511, 1e-4);
    EXPECT_NEAR (point.curvature, 0.0693136, 1e-4);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This tests the indexing issue from #3573
// In certain cases when you used a subset of the indices
// and set the depth dependent smoothing to false the
// IntegralImageNormalEstimation could crash or produce
// incorrect normals.
// This test reproduces the issue.
TEST (PCL, IntegralImageNormalEstimationIndexingIssue)
{
  PointCloud<PointXYZ>::Ptr cloudptr(new PointCloud<PointXYZ>());

  cloudptr->width = 100;
  cloudptr->height = 100;
  cloudptr->points.clear();
  cloudptr->points.resize(cloudptr->width * cloudptr->height);

  int centerX = cloudptr->width >> 1;
  int centerY = cloudptr->height >> 1;

  int idx = 0;
  for (int ypos = -centerY; ypos < centerY; ypos++)
  {
    for (int xpos = -centerX; xpos < centerX; xpos++)
	{
      double z = xpos < 0.0 ? 1.0 : 0.0;
      double y = ypos;
      double x = xpos;

      cloudptr->points[idx++] = PointXYZ(float(x), float(y), float(z));
    }
  }

  pcl::IndicesPtr indicesptr (new pcl::Indices ());
  indicesptr->resize(cloudptr->size() / 2);
  for (std::size_t i = 0; i < cloudptr->size() / 2; ++i)
  {
    (*indicesptr)[i] = i + cloudptr->size() / 2;
  }

  IntegralImageNormalEstimation<PointXYZ, Normal> n;

  // Object
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());

  // set parameters
  n.setInputCloud (cloudptr);
  n.setIndices (indicesptr);
  n.setDepthDependentSmoothing (false);

  // estimate
  n.compute (*normals);

  std::vector<PointXYZ> normalsVec;
  normalsVec.resize(normals->size());
  for(std::size_t i = 0; i < normals->size(); ++i )
  {
    normalsVec[i].x = normals->points[i].normal_x;
    normalsVec[i].y = normals->points[i].normal_y;
    normalsVec[i].z = normals->points[i].normal_z;
  }

  for (const auto &point : normals->points)
  {
  if (std::isnan( point.normal_x ) ||
	  std::isnan( point.normal_y ) ||
	  std::isnan( point.normal_z ))
    {
      continue;
	}

    EXPECT_NEAR (point.normal[0], 0.0, 1e-4);
    EXPECT_NEAR (point.normal[1], 0.0, 1e-4);
    EXPECT_NEAR (point.normal[2], -1.0, 1e-4);
    EXPECT_TRUE (std::isnan(point.curvature));
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  if (loadPCDFile<PointXYZ> (argv[1], cloud) < 0)
  {
    std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }

  indices.resize (cloud.points.size ());
  for (int i = 0; i < static_cast<int> (indices.size ()); ++i)
    indices[i] = i;

  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud.makeShared ());

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
