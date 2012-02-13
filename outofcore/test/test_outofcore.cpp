/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 */

/* \author
 *      Jacob Schloss (jacob.schloss@urbanrobotics.net),
 *      Justin Rosen (jmylesrosen@gmail.com)
 */

#include <gtest/gtest.h>

#include <vector>

#include <stdio.h>

using namespace std;

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;

#include "pcl/outofcore/impl/octree_base.hpp"
#include "pcl/outofcore/impl/octree_base_node.hpp"

#include "pcl/outofcore/pointCloudTools.h"
#include "pcl/outofcore/impl/octree_disk_container.hpp"
#include "pcl/outofcore/impl/octree_ram_container.hpp"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/foreach.hpp>

// For doing exhaustive checks this is set low remove those, and this can be
// set much higher
const static int numPts = 1e4;

const static boost::uint32_t rngseed = 0xAAFF33DD;

const static boost::filesystem::path filename_otreeA = "treeA/tree_test.oct_idx";
const static boost::filesystem::path filename_otreeB = "treeB/tree_test.oct_idx";

const static boost::filesystem::path filename_otreeA_LOD = "treeA_LOD/tree_test.oct_idx";
const static boost::filesystem::path filename_otreeB_LOD = "treeB_LOD/tree_test.oct_idx";

std::vector<PointCloudTools::point> points;

TEST (PCL, Octree_Build)
{

  boost::filesystem::remove_all (filename_otreeA.parent_path ());
  boost::filesystem::remove_all (filename_otreeB.parent_path ());

  double min[3] = {0, 0, 0};
  double max[3] = {1, 1, 1};

  // Build two trees using each constructor
  octree_disk treeA (min, max, .1, filename_otreeA, "ECEF");
  octree_disk treeB (4, min, max, filename_otreeB, "ECEF");

  // Equidistributed uniform pseudo-random number generator
  boost::mt19937 rng(rngseed);
  // For testing sparse
  //boost::uniform_real<double> dist(0,1);
  // For testing less sparse
  boost::normal_distribution<double> dist (0.5, .1);

  // Create a point
  PointCloudTools::point p;
  p.r = p.g = p.b = 0;
  p.nx = p.ny = p.nz = 1;
  p.cameraCount = 0;
  p.error = 0;
  p.triadID = 0;

  points.resize (numPts);

  // Radomize it's position in space
  for(int i = 0; i < numPts; i++)
  {
    p.x = dist (rng);
    p.y = dist (rng);
    p.z = dist (rng);

    points[i] = p;
  }

  // Add to tree
  treeA.addDataToLeaf(points);

  // Add to tree
  treeB.addDataToLeaf(points);

}

TEST (PCL, Octree_Build_LOD)
{

  boost::filesystem::remove_all (filename_otreeA_LOD.parent_path ());
  boost::filesystem::remove_all (filename_otreeB_LOD.parent_path ());

  double min[3] = {0, 0, 0};
  double max[3] = {1, 1, 1};

  // Build two trees using each constructor
  octree_disk treeA (min, max, .1, filename_otreeA_LOD, "ECEF");
  octree_disk treeB (4, min, max, filename_otreeB_LOD, "ECEF");

  // Equidistributed uniform pseudo-random number generator
  boost::mt19937 rng(rngseed);
  // For testing sparse
  //boost::uniform_real<double> dist(0,1);
  // For testing less sparse
  boost::normal_distribution<double> dist (0.5, .1);

  // Create a point
  PointCloudTools::point p;
  p.r = p.g = p.b = 0;
  p.nx = p.ny = p.nz = 1;
  p.cameraCount = 0;
  p.error = 0;
  p.triadID = 0;

  points.resize (numPts);

  // Radomize it's position in space
  for(int i = 0; i < numPts; i++)
  {
    p.x = dist (rng);
    p.y = dist (rng);
    p.z = dist (rng);

    points[i] = p;
  }

  // Add to tree
  treeA.addDataToLeaf_and_genLOD (points);

  // Add to tree
  treeB.addDataToLeaf_and_genLOD (points);
}

TEST(PCL, Bounding_Box)
{

  octree_disk treeA (filename_otreeA, false);
  octree_disk treeB (filename_otreeB, false);

  double min_otreeA[3];
  double max_otreeA[3];
  treeA.getBB (min_otreeA, max_otreeA);

  double min_otreeB[3];
  double max_otreeB[3];
  treeB.getBB (min_otreeB, max_otreeB);

  ASSERT_EQ (min_otreeA[0], 0);
  ASSERT_EQ (min_otreeA[1], 0);
  ASSERT_EQ (min_otreeA[2], 0);
  ASSERT_EQ (max_otreeA[0], 1);
  ASSERT_EQ (max_otreeA[1], 1);
  ASSERT_EQ (max_otreeA[2], 1);

  ASSERT_EQ (min_otreeB[0], 0);
  ASSERT_EQ (min_otreeB[1], 0);
  ASSERT_EQ (min_otreeB[2], 0);
  ASSERT_EQ (max_otreeB[0], 1);
  ASSERT_EQ (max_otreeB[1], 1);
  ASSERT_EQ (max_otreeB[2], 1);
}

void point_test(octree_disk& t)
{
  boost::mt19937 rng(rngseed);
  boost::uniform_real<double> dist(0,1);

  double qboxmin[3];
  double qboxmax[3];

  for(int i = 0; i < 10; i++)
  {
    //std::cout << "query test round " << i << std::endl;
    for(int j = 0; j < 3; j++)
    {
      qboxmin[j] = dist(rng);
      qboxmax[j] = dist(rng);

      if(qboxmax[j] < qboxmin[j])
      {
        std::swap(qboxmin[j], qboxmax[j]);
      }
    }

    //query the trees
    std::list<PointCloudTools::point> p_ot;
    t.queryBBIncludes(qboxmin, qboxmax, t.getDepth(), p_ot);

    //query the list
    std::vector<PointCloudTools::point> pointsinregion;
    BOOST_FOREACH(const PointCloudTools::point& p, points)
    {
      if((qboxmin[0] <= p.x) && (p.x <= qboxmax[0]) && (qboxmin[1] <= p.y) && (p.y <= qboxmax[1]) && (qboxmin[2] <= p.z) && (p.z <= qboxmax[2]))
      {
        pointsinregion.push_back(p);
      }
    }

    ASSERT_EQ (p_ot.size (), pointsinregion.size ());

    //very slow exhaustive comparison
    BOOST_FOREACH(const PointCloudTools::point& p, pointsinregion)
    {
      std::list<PointCloudTools::point>::iterator it;
      it = std::find(p_ot.begin(), p_ot.end(), p);

      if(it != p_ot.end())
      {
        p_ot.erase(it);
      }
      else
      {
        std::cerr << "Dropped Point from tree1!" << std::endl;
        ASSERT_TRUE(false);
      }
    }

    ASSERT_TRUE(p_ot.empty());
  }
}

TEST (PCL, Point_Query)
{
  octree_disk treeA(filename_otreeA, false);
  octree_disk treeB(filename_otreeB, false);

  point_test(treeA);
  point_test(treeB);
}

TEST (PCL, Ram_Tree)
{
  double min[3] = {0,0,0};
  double max[3] = {1,1,1};

  octree_ram t(min, max, .1, filename_otreeA, "ECEF");

  boost::mt19937 rng(rngseed);
  //boost::uniform_real<double> dist(0,1);//for testing sparse
  boost::normal_distribution<double> dist(0.5, .1);//for testing less sparse

  PointCloudTools::point p;
  p.r = p.g = p.b = 0;
  p.nx = p.ny = p.nz = 1;
  p.cameraCount = 0;
  p.error = 0;
  p.triadID = 0;

  points.resize(numPts);
  for(int i = 0; i < numPts; i++)
  {
    p.x = dist(rng);
    p.y = dist(rng);
    p.z = dist(rng);

    points[i] = p;
  }

  t.addDataToLeaf_and_genLOD(points);
  //t.addDataToLeaf(points);

  double qboxmin[3];
  double qboxmax[3];
  for(int i = 0; i < 10; i++)
  {
    //std::cout << "query test round " << i << std::endl;
    for(int j = 0; j < 3; j++)
    {
      qboxmin[j] = dist(rng);
      qboxmax[j] = dist(rng);

      if(qboxmax[j] < qboxmin[j])
      {
        std::swap(qboxmin[j], qboxmax[j]);
      }
    }

    //query the trees
    std::list<PointCloudTools::point> p_ot1;
    t.queryBBIncludes(qboxmin, qboxmax, t.getDepth(), p_ot1);

    //query the list
    std::vector<PointCloudTools::point> pointsinregion;
    BOOST_FOREACH(const PointCloudTools::point& p, points)
    {
      if((qboxmin[0] <= p.x) && (p.x <= qboxmax[0]) && (qboxmin[1] <= p.y) && (p.y <= qboxmax[1]) && (qboxmin[2] <= p.z) && (p.z <= qboxmax[2]))
      {
        pointsinregion.push_back(p);
      }
    }

    ASSERT_EQ(p_ot1.size(), pointsinregion.size());

    //very slow exhaustive comparison
    BOOST_FOREACH(const PointCloudTools::point& p, pointsinregion)
    {
      std::list<PointCloudTools::point>::iterator it;
      it = std::find(p_ot1.begin(), p_ot1.end(), p);

      if(it != p_ot1.end())
      {
        p_ot1.erase(it);
      }
      else
      {
        std::cerr << "Dropped Point from tree1!" << std::endl;
        ASSERT_TRUE(false);
      }
    }

    ASSERT_TRUE(p_ot1.empty());
  }
}

//TEST (PCL, Octree_Teardown)
//{
//  boost::filesystem::remove_all (filename_otreeA.parent_path ());
//  boost::filesystem::remove_all (filename_otreeB.parent_path ());
//}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
