 /*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2009-2012, Urban Robotics, Inc.
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

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

using namespace pcl::outofcore;

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/foreach.hpp>

/** \brief Unit tests for UR out of core octree code which test public interface of octree_base 
 */

// For doing exhaustive checks this is set low remove those, and this can be
// set much higher
const static uint64_t numPts = 1e4;

const static boost::uint32_t rngseed = 0xAAFF33DD;

const static boost::filesystem::path filename_otreeA = "treeA/tree_test.oct_idx";
const static boost::filesystem::path filename_otreeB = "treeB/tree_test.oct_idx";

const static boost::filesystem::path filename_otreeA_LOD = "treeA_LOD/tree_test.oct_idx";
const static boost::filesystem::path filename_otreeB_LOD = "treeB_LOD/tree_test.oct_idx";

const static  boost::filesystem::path outofcore_path ("point_cloud_octree/tree_test.oct_idx");


typedef pcl::PointXYZ PointT;

// UR Typedefs
typedef octree_base<octree_disk_container < PointT > , PointT > octree_disk;
typedef octree_base_node<octree_disk_container < PointT > , PointT > octree_disk_node;

typedef octree_base<octree_ram_container< PointT> , PointT> octree_ram;
typedef octree_base_node< octree_ram_container<PointT> , PointT> octree_ram_node;

std::vector<PointT, Eigen::aligned_allocator<PointT> > points;

/** \brief helper function to compare two points. is there a templated function in pcl to do this for arbitrary point types?*/
bool 
compPt ( PointT p1, PointT p2 )
{
  if( p1.x != p2.x || p1.y != p2.y || p1.z != p2.z )
    return false;
  
  return true;
}

TEST (PCL, Outofcore_Octree_Build)
{

  boost::filesystem::remove_all (filename_otreeA.parent_path ());
  boost::filesystem::remove_all (filename_otreeB.parent_path ());

  double min[3] = {0, 0, 0};
  double max[3] = {1, 1, 1};

  // Build two trees using each constructor
  // depth of treeA will be same as B because 1/2^3 > .1 and 1/2^4 < .1
  // depth really affects performance
  octree_disk treeA (min, max, .1, filename_otreeA, "ECEF");
  octree_disk treeB (4, min, max, filename_otreeB, "ECEF");

  // Equidistributed uniform pseudo-random number generator
  boost::mt19937 rng(rngseed);

  // For testing sparse 
  //boost::uniform_real<double> dist(0,1);

  // For testing less sparse
  boost::normal_distribution<float> dist ( 0.5f, .1f );

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
  for(size_t i = 0; i < numPts; i++)
  {
    p.x = dist ( rng );
    p.y = dist ( rng );
    p.z = dist ( rng );

    points[i] = p;
  }

  // Add to tree
  treeA.addDataToLeaf(points);

  // Add to tree
  treeB.addDataToLeaf(points);

}

TEST (PCL, Outofcore_Octree_Build_LOD)
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
  boost::normal_distribution<float> dist (0.5f, .1f);

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
  for(size_t i = 0; i < numPts; i++)
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

TEST(PCL, Outofcore_Bounding_Box)
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
  boost::uniform_real<float> dist(0,1);

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
    std::vector<PointT, Eigen::aligned_allocator<PointT> > pointsinregion;

    for(vector<PointT, Eigen::aligned_allocator<PointT> >::iterator pointit = points.begin (); pointit != points.end (); ++pointit)
    {
      if((query_box_min[0] <= pointit->x) && (pointit->x <= qboxmax[0]) && (query_box_min[1] <= pointit->y) && (pointit->y <= qboxmax[1]) && (query_box_min[2] <= pointit->z) && (pointit->z <= qboxmax[2]))
      {
        pointsinregion.push_back(*pointit);
      }
    }

    ASSERT_EQ (p_ot.size (), pointsinregion.size ());

    //very slow exhaustive comparison
    while( !p_ot.empty () )
    {
      std::list<PointT>::iterator it;
      it = std::find_first_of(p_ot.begin(), p_ot.end(), pointsinregion.begin (), pointsinregion.end (), compPt);

      if(it != p_ot.end())
      {
        p_ot.erase(it);
      }
      else
      {
        FAIL () <<  "Dropped Point from tree1!" << std::endl;
        break;
      }
    }

    ASSERT_TRUE(p_ot.empty());
  }
}

TEST (PCL, Outofcore_Point_Query)
{
  octree_disk treeA(filename_otreeA, false);
  octree_disk treeB(filename_otreeB, false);

  point_test(treeA);
  point_test(treeB);
}

TEST (PCL, Outofcore_Ram_Tree)
{
  double min[3] = {0,0,0};
  double max[3] = {1,1,1};

  octree_ram t(min, max, .1, filename_otreeA, "ECEF");

  boost::mt19937 rng(rngseed);
  //boost::uniform_real<double> dist(0,1);//for testing sparse
  boost::normal_distribution<float> dist(0.5f, .1f);//for testing less sparse
  PointT p;
/*
  p.r = p.g = p.b = 0;
  p.nx = p.ny = p.nz = 1;
  p.cameraCount = 0;
  p.error = 0;
  p.triadID = 0;
*/
  points.resize(numPts);
  for(size_t i = 0; i < numPts; i++)
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
    std::vector<PointT, Eigen::aligned_allocator<PointT> > pointsinregion;
    BOOST_FOREACH(const PointT& p, points)
    {
      if((qboxmin[0] <= p.x) && (p.x <= qboxmax[0]) && (qboxmin[1] <= p.y) && (p.y <= qboxmax[1]) && (qboxmin[2] <= p.z) && (p.z <= qboxmax[2]))
      {
        pointsinregion.push_back(p);
      }
    }

    ASSERT_EQ(p_ot1.size(), pointsinregion.size());

    //very slow exhaustive comparison
    while( !p_ot1.empty () )
    {
      std::list<PointT>::iterator it;
      it = std::find_first_of(p_ot1.begin(), p_ot1.end(), pointsinregion.begin (), pointsinregion.end (), compPt);

      if(it != p_ot1.end())
      {
        p_ot1.erase(it);
      }
      else
      {
        break;
        FAIL () <<  "Dropped Point from tree1!" << std::endl;
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
      smallest_voxel_dim = 0.1f;
    }

    virtual void TearDown ()
    {

    }

    void cleanUpFilesystem ()
    {
      //clear existing trees from test path
      boost::filesystem::remove_all (filename_otreeA.parent_path ());
      boost::filesystem::remove_all (filename_otreeB.parent_path ());

      boost::filesystem::remove_all (filename_otreeA_LOD.parent_path ());
      boost::filesystem::remove_all (filename_otreeB_LOD.parent_path ());

      boost::filesystem::remove_all (outofcore_path.parent_path ());
    }

    double smallest_voxel_dim;

};


/** \brief Thorough test of the constructors, including exceptions and specified behavior */
TEST_F (OutofcoreTest, Outofcore_Constructors)
{
  //Case 1: create octree on-disk by resolution
  //Case 2: create octree on-disk by depth
  //Case 3: try to create an octree in existing tree and handle exception
  //Case 4: load existing octree from disk
  //Case 5: try to load non-existent octree from disk

  cleanUpFilesystem ();

  //Specify the lower corner of the axis-aligned bounding box
  const double min[3] = { -1024, -1024, -1024 };
  //Specify the upper corner of the axis-aligned bounding box
  const double max[3] = { 1024, 1024, 1024 };

  vector<PointT, Eigen::aligned_allocator<PointT> > some_points;
  for(int i=0; i< 1000; i++)
    some_points.push_back (PointT (static_cast<float>( rand () % 1024 ), static_cast<float>( rand () % 1024 ), static_cast<float>( rand () % 1024 ) ));
  

  //(Case 1)
  //Create Octree based on resolution of smallest voxel, automatically computing depth
  octree_disk octreeA (min, max, smallest_voxel_dim, filename_otreeA, "ECEF");
  octreeA.addDataToLeaf (some_points);
  ASSERT_EQ ( octreeA.getNumPoints (octreeA.getDepth ()), some_points.size () );
  
  //(Case 2)
  //create Octree by prespecified depth in constructor
  int depth = 4;
  octree_disk octreeB (depth, min, max, filename_otreeB, "ECEF");
  octreeB.addDataToLeaf (some_points);
  ASSERT_EQ ( octreeB.getNumPoints (octreeB.getDepth ()), some_points.size () );
}

TEST_F (OutofcoreTest, Outofcore_ConstructorSafety)
{
  //Specify the lower corner of the axis-aligned bounding box
  const double min[3] = { -1024, -1024, -1024 };
  //Specify the upper corner of the axis-aligned bounding box
  const double max[3] = { 1024, 1024, 1024 };

  //(Case 3) Constructor Safety. These should throw OCT_CHILD_EXISTS exceptions and write an error
  //message of conflicting file path
  EXPECT_TRUE (boost::filesystem::exists (filename_otreeA));
  EXPECT_TRUE (boost::filesystem::exists (filename_otreeB));

  /**\todo behaviors of the two constructors don't match. the one with resolution in the constructor doesn't check if a tree is being overwritten.
   */

  //  EXPECT_ANY_THROW ({
  //      octree_disk octreeC (min, max, smallest_voxel_dim, filename_otreeA, "ECEF");
  //    });

  EXPECT_ANY_THROW ({
      octree_disk octreeD (4, min, max, filename_otreeB, "ECEF");
    });

  //(Case 4): Load existing tree from disk
  octree_disk octree_from_disk (filename_otreeB, true);
  vector<uint64_t> numPoints = octree_from_disk.getNumPoints ();
  
  EXPECT_EQ ( octree_from_disk.getNumPoints (octree_from_disk.getDepth ()), 1000 ) << octree_from_disk.getDepth ();
}

TEST_F (OutofcoreTest, Outofcore_ConstructorBadPaths)
{
  //(Case 5): Try to load non-existent tree from disk
  //root node should be created at this point
  /// \todo Shouldn't these throw an exception for bad path?
  boost::filesystem::path non_existent_path_name ("treeBogus/tree_bogus.oct_idx");
  boost::filesystem::path bad_extension_path ("treeBadExtension/tree_bogus.bad_extension");

  ASSERT_FALSE (boost::filesystem::exists (non_existent_path_name));
  EXPECT_ANY_THROW ({octree_disk octree_bogus_path ( non_existent_path_name, true );});

  ASSERT_FALSE (boost::filesystem::exists (bad_extension_path));
  EXPECT_ANY_THROW ({octree_disk octree_bad_extension ( bad_extension_path, true );});

  cleanUpFilesystem ();
}

#if 1
TEST_F (OutofcoreTest, Outofcore_PointcloudConstructor)
{
  cleanUpFilesystem ();
  
  //Specify the lower corner of the axis-aligned bounding box
  const double min[3] = { -1, -1, -1 };
  
  //Specify the upper corner of the axis-aligned bounding box
  const double max[3] = { 1024, 1024, 1024 };

  //create a point cloud
  PointCloud<PointT>::Ptr test_cloud (new PointCloud<PointT> () );
  
  test_cloud->width = numPts;
  test_cloud->height = 1;
  test_cloud->resize (numPts);
  
  //generate some random points
  for(size_t i=0; i < numPts; i++)
  {
    PointT tmp ( static_cast<float> (i % 1024), 
                 static_cast<float> (i % 1024), 
                 static_cast<float> (i % 1024));
    
    test_cloud->points.push_back (tmp);
  }

  octree_disk pcl_cloud (min, max, 32.0, outofcore_path, "ECEF");

  pcl_cloud.addPointCloud (test_cloud);
  
  ASSERT_EQ ( test_cloud->points.size (), pcl_cloud.getNumPoints (pcl_cloud.getDepth ()));
  
  cleanUpFilesystem ();
}
#endif

/* [--- */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
