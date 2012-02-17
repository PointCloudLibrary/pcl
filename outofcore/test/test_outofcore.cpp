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
 *      Justin Rosen (jmylesrosen@gmail.com),
 *      Stephen Fox (stfox88@gmail.com)
 */

#include <gtest/gtest.h>

#include <vector>
#include <cstdio>

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

/** \brief Unit tests for UR out of core octree code which test public interface of octree_base 
 *
 * These need to be rewritten. I've written a thorough test for the
 * constructors, but they don't match the specified behavior
 * exactly. In particular, they don't always throw the exceptions that
 * they should in case there's an existing tree, or its trying to read
 * a non-existing tree. Accessors and mutators also should be tested
 * in these contexts.
 *
 * \note if you would like to see what the trees look like on-disk,
 * uncomment out the TearDown method in the test class. If you'd
 * like to see the LOD (which I haven't added to the test class yet),
 * uncomment out the SetUp method as well.
 * \note Unit tests fail right now :)
 * \todo Look into the cause of failure
 *
 * \note octree_disk is typedef'd in octree_base.hpp
 */

// For doing exhaustive checks this is set low remove those, and this can be
// set much higher
const static int numPts = 1e4;

const static boost::uint32_t rngseed = 0xAAFF33DD;

const static boost::filesystem::path filename_otreeA = "treeA/tree_test.oct_idx";
const static boost::filesystem::path filename_otreeB = "treeB/tree_test.oct_idx";

const static boost::filesystem::path filename_otreeA_LOD = "treeA_LOD/tree_test.oct_idx";
const static boost::filesystem::path filename_otreeB_LOD = "treeB_LOD/tree_test.oct_idx";

typedef pcl::PointXYZ PointT;

std::vector<PointT> points;

/** \brief helper function to compare two points. is there a templated function in pcl to do this?*/
bool 
compPt ( PointT p1, PointT p2 )
{
  if( p1.x != p2.x || p1.y != p2.y || p1.z != p2.z )
    return false;
  
  return true;
}

TEST (PCL, Octree_Build)
{

  boost::filesystem::remove_all (filename_otreeA.parent_path ());
  boost::filesystem::remove_all (filename_otreeB.parent_path ());

  double min[3] = {0, 0, 0};
  double max[3] = {1, 1, 1};

  // Build two trees using each constructor
  // depth of treeA will be same as B because 1/2^3 > .1 and 1/2^4 < .1
  // depth really affects performance
  /// \todo benchmark performance based on depth
  octree_disk treeA (min, max, .1, filename_otreeA, "ECEF");
  octree_disk treeB (4, min, max, filename_otreeB, "ECEF");

  // Equidistributed uniform pseudo-random number generator
  boost::mt19937 rng(rngseed);

  // For testing sparse 
  //boost::uniform_real<double> dist(0,1);

  // For testing less sparse
  boost::normal_distribution<double> dist (0.5, .1);

  // Create a point
  PointT p;
  points.resize (numPts);

  //ignore these fields from the UR point for now
  // p.r = p.g = p.b = 0;
  // p.nx = p.ny = p.nz = 1;
  // p.cameraCount = 0;
  // p.error = 0;
  // p.triadID = 0;

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
  PointT p;

/*
  p.r = p.g = p.b = 0;
  p.nx = p.ny = p.nz = 1;
  p.cameraCount = 0;
  p.error = 0;
  p.triadID = 0;
*/
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

  double min[3] = {0, 0, 0};
  double max[3] = {1, 1, 1};

  octree_disk treeA (filename_otreeA, false);
  octree_disk treeB (filename_otreeB, false);

  double min_otreeA[3];
  double max_otreeA[3];
  treeA.getBB (min_otreeA, max_otreeA);

  double min_otreeB[3];
  double max_otreeB[3];
  treeB.getBB (min_otreeB, max_otreeB);

  for(int i=0; i<3; i++)
  {
    ASSERT_EQ (min_otreeA[i], min[i]);
    ASSERT_EQ (max_otreeA[i], max[i]);

    ASSERT_EQ (min_otreeB[i], min[i]);
    ASSERT_EQ (max_otreeB[i], max[i]);
  }
}

void point_test(octree_disk& t)
{
  boost::mt19937 rng(rngseed);
  boost::uniform_real<double> dist(0,1);

  double query_box_min[3];
  double qboxmax[3];

  for(int i = 0; i < 10; i++)
  {
    //std::cout << "query test round " << i << std::endl;
    for(int j = 0; j < 3; j++)
    {
      query_box_min[j] = dist(rng);
      qboxmax[j] = dist(rng);

      if(qboxmax[j] < query_box_min[j])
      {
        std::swap(query_box_min[j], qboxmax[j]);
      }
    }

    //query the trees
    std::list<PointT> p_ot;
    t.queryBBIncludes(query_box_min, qboxmax, t.getDepth(), p_ot);

    //query the list
    std::vector<PointT> pointsinregion;
    BOOST_FOREACH(const PointT& p, points)
    {
      if((query_box_min[0] <= p.x) && (p.x <= qboxmax[0]) && (query_box_min[1] <= p.y) && (p.y <= qboxmax[1]) && (query_box_min[2] <= p.z) && (p.z <= qboxmax[2]))
      {
        pointsinregion.push_back(p);
      }
    }

    ASSERT_EQ (p_ot.size (), pointsinregion.size ());

    //very slow exhaustive comparison
    BOOST_FOREACH(const PointT& p, pointsinregion)
    {
      std::list<PointT>::iterator it;
      it = std::find_first_of(p_ot.begin(), p_ot.end(), pointsinregion.begin (), pointsinregion.end (), compPt);

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
  PointT p;
/*
  p.r = p.g = p.b = 0;
  p.nx = p.ny = p.nz = 1;
  p.cameraCount = 0;
  p.error = 0;
  p.triadID = 0;
*/
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
    std::list<PointT> p_ot1;
    t.queryBBIncludes(qboxmin, qboxmax, t.getDepth(), p_ot1);

    //query the list
    std::vector<PointT> pointsinregion;
    BOOST_FOREACH(const PointT& p, points)
    {
      if((qboxmin[0] <= p.x) && (p.x <= qboxmax[0]) && (qboxmin[1] <= p.y) && (p.y <= qboxmax[1]) && (qboxmin[2] <= p.z) && (p.z <= qboxmax[2]))
      {
        pointsinregion.push_back(p);
      }
    }

    ASSERT_EQ(p_ot1.size(), pointsinregion.size());

    //very slow exhaustive comparison
    BOOST_FOREACH(const PointT& p, pointsinregion)
    {
      std::list<PointT>::iterator it;
      it = std::find_first_of(p_ot1.begin(), p_ot1.end(), pointsinregion.begin (), pointsinregion.end (), compPt);

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

class OutofcoreTest : public testing::Test
{
  protected:

    virtual void SetUp ()
    {
      //clear existing trees from test path
      boost::filesystem::remove_all (filename_otreeA.parent_path ());
      boost::filesystem::remove_all (filename_otreeB.parent_path ());

      boost::filesystem::remove_all (filename_otreeA_LOD.parent_path ());
      boost::filesystem::remove_all (filename_otreeB_LOD.parent_path ());
      smallest_voxel_dim = 0.1f;
    }

    virtual void TearDown ()
    {
      //remove the trees from disk because they can take a lot of space
      boost::filesystem::remove_all (filename_otreeA.parent_path ());
      boost::filesystem::remove_all (filename_otreeB.parent_path ());

      boost::filesystem::remove_all (filename_otreeA_LOD.parent_path ());
      boost::filesystem::remove_all (filename_otreeB_LOD.parent_path ());
    }
    double smallest_voxel_dim;

};


/** \brief Thorough test of the constructors, including exceptions and specified behavior
 *
 * \note Currently, not everything passes. There seems to be unpredictable behavior when 
 * an octree is created where one already exists
 */
TEST_F (OutofcoreTest, Constructors)
{
  //Case 1: create octree on-disk by resolution
  //Case 2: create octree on-disk by depth
  //Case 3: try to create an octree in existing tree and handle exception
  //Case 4: load existing octree from disk
  //Case 5: try to load non-existent octree from disk

  //Specify the lower corner of the axis-aligned bounding box
  static const double min[3] = { -1024, -1024, -1024 };
  //Specify the upper corner of the axis-aligned bounding box
  static const double max[3] = { 1024, 1024, 1024 };

  vector<PointT> some_points;
  for(int i=0; i< 1000; i++)
    some_points.push_back (PointT (rand ()%1024, rand ()%1024, rand ()%1024));

  //(Case 1)
  //Create Octree based on resolution of smallest voxel
  octree_disk octreeA (min, max, smallest_voxel_dim, filename_otreeA, "ECEF");
  octreeA.addDataToLeaf (some_points);
  ASSERT_EQ ( octreeA.getNumPoints (octreeA.getDepth ()), some_points.size () );
  

  //(Case 2)
  //create Octree by specifying depth
  int depth = 4;
  octree_disk octreeB (4, min, max, filename_otreeB, "ECEF");
  octreeB.addDataToLeaf (some_points);
  ASSERT_EQ ( octreeB.getNumPoints (octreeB.getDepth ()), some_points.size () );

}


TEST_F (OutofcoreTest, ConstructorSafety)
{
  //these could be global in the test class
  //Specify the lower corner of the axis-aligned bounding box
  const double min[3] = { -1024, -1024, -1024 };
  //Specify the upper corner of the axis-aligned bounding box
  const double max[3] = { 1024, 1024, 1024 };

  //(Case 3) Constructor Safety. These should throw OCT_CHILD_EXISTS exceptions and write an error
  //message of conflicting file path
  OctreeException *C=0, *D=0;
  try
  {
    octree_disk octreeC (min, max, smallest_voxel_dim, filename_otreeA, "ECEF");
  }
  catch(OctreeException& e)
  {
    *C = e;
  }
  catch(...)
  {
    FAIL () << "octreeC has thrown an unknown exception when there is an existing tree in the desired path\n";
  }

  //check that it threw the proper exception
  if( C != 0 )
  {
    ASSERT_EQ (OctreeException (OctreeException::OCT_CHILD_EXISTS).what (), C->what ()) << "OctreeC failing with: " << C->what () << ", a known but unexpected OctreeException." << endl;
  }
  else
  {
    FAIL () << "octreeC is probably overwriting an existing directory tree\n";
  }
  
  //(Case 3) second type of constructor
  try
  {
    octree_disk octreeD (4, min, max, filename_otreeB, "ECEF");
  }
  catch (OctreeException& e)
  {
    *D = e;
  }
  catch (...)
  {
    FAIL () << "octreeD has thrown an unknown exception when there is an existing tree in the desired path\n";
  }

  //check that it threw the proper exception
  if ( D != 0 )
  {
    ASSERT_EQ (OctreeException (OctreeException::OCT_CHILD_EXISTS).what (), D->what ()) << "OctreeD failing with: " << D->what () << ", a known but unexpected OctreeException." << endl;
  }
  else
  {
    FAIL () << "octreeD is overwriting an existing directory tree\n";
  }

  //Case 4: Load existing tree from disk
  octree_disk octree_from_disk (filename_otreeA, false);
  ASSERT_EQ ( octree_from_disk.getNumPoints (octree_from_disk.getDepth ()), 1000 );
  
  //Case 5: Try to load non-existent tree from disk
  //root node should be created at this point
  /// \todo Shouldn't these complain (throw an exception) for bad path?
  /// \note according to UR documentation, the rest will be generated on insertion or query
  /// \note we might want to change the specification here
  boost::filesystem::path bogus_path_name ("treeBogus/tree_bogus.oct_idx");
  boost::filesystem::path bad_extension_path ("treeBadExtension/tree_bogus.bad_extension");
  
  octree_disk octree_bogus_path ( bogus_path_name, true );
  octree_disk octree_bad_extension ( bad_extension_path, true );
  ASSERT_TRUE (boost::filesystem::exists (bogus_path_name));
  ASSERT_TRUE (boost::filesystem::exists (bad_extension_path));
}


int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
