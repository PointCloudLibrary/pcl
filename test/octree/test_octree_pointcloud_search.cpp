/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#include <vector>

#include <stdio.h>

using namespace std;

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_tests.h>

using namespace pcl;

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_dynamic_depth.h>

using namespace octree;

TEST (PCL, Octree_Pointcloud_Test)
{

  size_t i;
  int test_runs = 100;
  int pointcount = 300;

  int test, point;

  float resolution = 0.01f;

  OctreePointCloudSearch<PointXYZ> octreeB (resolution);

  // create shared pointcloud instances
  PointCloud<PointXYZ>::Ptr cloudB (new PointCloud<PointXYZ> ());

  // assign input point clouds to octree
  octreeB.setInputCloud (cloudB);

  for (test = 0; test < test_runs; ++test)
  {

    // clean up
    cloudB->points.clear ();
    octreeB.deleteTree ();

    for (point = 0; point < pointcount; point++)
    {
      // gereate a random point
      PointXYZ newPoint (static_cast<float> (1024.0 * rand () / RAND_MAX), 
                         static_cast<float> (1024.0 * rand () / RAND_MAX), 
                         static_cast<float> (1024.0 * rand () / RAND_MAX));

      // OctreePointCloudPointVector can store all points..
      cloudB->push_back (newPoint);
    }

    // check if all points from leaf data can be found in input pointcloud data sets
    octreeB.defineBoundingBox ();
    octreeB.addPointsFromInputCloud ();

    //  test iterator
    OctreePointCloudPointVector<PointXYZ>::Iterator b_it;
    OctreePointCloudPointVector<PointXYZ>::Iterator b_it_end = octreeB.end();

    // iterate over tree

    unsigned int node_count = 0;
    unsigned int branch_count = 0;
    unsigned int leaf_count = 0;

    double minx, miny, minz, maxx, maxy, maxz;
    octreeB.getBoundingBox (minx, miny, minz, maxx, maxy, maxz);

    // iterate over tree
    for (b_it = octreeB.begin(); b_it != b_it_end; ++b_it)
    {
      // depth should always be less than tree depth
      unsigned int depth = b_it.getCurrentOctreeDepth ();
      ASSERT_LE(depth, octreeB.getTreeDepth());

      Eigen::Vector3f voxel_min, voxel_max;
      octreeB.getVoxelBounds (b_it, voxel_min, voxel_max);

      ASSERT_GE(voxel_min.x (), minx - 1e-4);
      ASSERT_GE(voxel_min.y (), miny - 1e-4);
      ASSERT_GE(voxel_min.z (), minz - 1e-4);

      ASSERT_LE(voxel_max.x (), maxx + 1e-4);
      ASSERT_LE(voxel_max.y (), maxy + 1e-4);
      ASSERT_LE(voxel_max.z (), maxz + 1e-4);

      // store node, branch and leaf count
      const OctreeNode* node = b_it.getCurrentOctreeNode ();
      if (node->getNodeType () == BRANCH_NODE)
      {
        branch_count++;
      }
      else if (node->getNodeType () == LEAF_NODE)
      {
        leaf_count++;
      }
      node_count++;
    }

    // compare node, branch and leaf count against actual tree values
    ASSERT_EQ(node_count, branch_count + leaf_count);
    ASSERT_EQ(leaf_count, octreeB.getLeafCount ());

    for (i = 0; i < cloudB->points.size (); i++)
    {

      std::vector<int> pointIdxVec;
      octreeB.voxelSearch (cloudB->points[i], pointIdxVec);

      bool bIdxFound = false;
      std::vector<int>::const_iterator current = pointIdxVec.begin ();
      while (current != pointIdxVec.end ())
      {
        if (*current == static_cast<int> (i))
        {
          bIdxFound = true;
          break;
        }
        ++current;
      }

      ASSERT_EQ(bIdxFound, true);

    }

  }

}

// helper class for priority queue
class prioPointQueueEntry
{
public:
  prioPointQueueEntry () : point_ (), pointDistance_ (), pointIdx_ ()
  {
  }
  prioPointQueueEntry (PointXYZ& point_arg, double pointDistance_arg, int pointIdx_arg) :
    point_ (point_arg),
    pointDistance_ (pointDistance_arg),
    pointIdx_ (pointIdx_arg)
  {
  }

  bool
  operator< (const prioPointQueueEntry& rhs_arg) const
  {
    return (this->pointDistance_ < rhs_arg.pointDistance_);
  }

  PointXYZ point_;
  double pointDistance_;
  int pointIdx_;

};

TEST (PCL, Octree_Pointcloud_Nearest_K_Neighbour_Search)
{

  const unsigned int test_runs = 10;
  unsigned int test_id;

  // instantiate point cloud
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  size_t i;

  srand (static_cast<unsigned int> (time (NULL)));

  unsigned int K;

  std::priority_queue<prioPointQueueEntry, pcl::PointCloud<prioPointQueueEntry>::VectorType> pointCandidates;

  // create octree
  OctreePointCloudSearch<PointXYZ> octree (0.1);
  octree.setInputCloud (cloudIn);

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point

    PointXYZ searchPoint (static_cast<float> (10.0 * rand () / RAND_MAX), 
                          static_cast<float> (10.0 * rand () / RAND_MAX),
                          static_cast<float> (10.0 * rand () / RAND_MAX));

    K = 1 + rand () % 10;

    // generate point cloud
    cloudIn->width = 1000;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);
    for (i = 0; i < 1000; i++)
    {
      cloudIn->points[i] = PointXYZ (static_cast<float> (5.0  * rand () / RAND_MAX),
                                     static_cast<float> (10.0 * rand () / RAND_MAX),
                                     static_cast<float> (10.0 * rand () / RAND_MAX));
    }

    double pointDist;

    k_indices.clear ();
    k_sqr_distances.clear ();

    k_indices_bruteforce.clear ();
    k_sqr_distances_bruteforce.clear ();

    // push all points and their distance to the search point into a priority queue - bruteforce approach.
    for (i = 0; i < cloudIn->points.size (); i++)
    {
      pointDist = ((cloudIn->points[i].x - searchPoint.x) * (cloudIn->points[i].x - searchPoint.x)
          + (cloudIn->points[i].y - searchPoint.y) * (cloudIn->points[i].y - searchPoint.y)
          + (cloudIn->points[i].z - searchPoint.z) * (cloudIn->points[i].z - searchPoint.z));

      prioPointQueueEntry pointEntry (cloudIn->points[i], pointDist, static_cast<int> (i));

      pointCandidates.push (pointEntry);
    }

    // pop priority queue until we have the nearest K elements
    while (pointCandidates.size () > K)
      pointCandidates.pop ();

    // copy results into vectors
    unsigned idx = static_cast<unsigned> (pointCandidates.size ());
    k_indices_bruteforce.resize (idx);
    k_sqr_distances_bruteforce.resize (idx);
    while (pointCandidates.size ())
    {
      --idx;
      k_indices_bruteforce [idx] = pointCandidates.top ().pointIdx_;
      k_sqr_distances_bruteforce [idx] = static_cast<float> (pointCandidates.top ().pointDistance_);

      pointCandidates.pop ();
    }
    
    // octree nearest neighbor search
    octree.deleteTree ();
    octree.addPointsFromInputCloud ();
    octree.nearestKSearch (searchPoint, static_cast<int> (K), k_indices, k_sqr_distances);

    ASSERT_EQ( k_indices.size(), k_indices_bruteforce.size());

    // compare nearest neighbor results of octree with bruteforce search
    i = 0;
    while (k_indices_bruteforce.size ())
    {
      ASSERT_EQ( k_indices.back(), k_indices_bruteforce.back());
      EXPECT_NEAR(k_sqr_distances.back(), k_sqr_distances_bruteforce.back(), 1e-4);

      k_indices_bruteforce.pop_back ();
      k_indices.pop_back ();

      k_sqr_distances_bruteforce.pop_back ();
      k_sqr_distances.pop_back ();

    }

  }

}

TEST (PCL, Octree_Pointcloud_Box_Search)
{

  const unsigned int test_runs = 30;
  unsigned int test_id;

  // instantiate point cloud
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  size_t i;

  srand (static_cast<unsigned int> (time (NULL)));

  // create octree
  OctreePointCloudSearch<PointXYZ> octree (1);
  octree.setInputCloud (cloudIn);

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    std::vector<int> k_indices;

    // generate point cloud
    cloudIn->width = 300;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);
    for (i = 0; i < cloudIn->points.size(); i++)
    {
      cloudIn->points[i] = PointXYZ (static_cast<float> (10.0 * rand () / RAND_MAX),
                                     static_cast<float> (10.0 * rand () / RAND_MAX),
                                     static_cast<float> (10.0 * rand () / RAND_MAX));
    }


    // octree points to octree
    octree.deleteTree ();
    octree.addPointsFromInputCloud ();

    // define a random search area

    Eigen::Vector3f lowerBoxCorner (static_cast<float> (4.0 * rand () / RAND_MAX),
                                    static_cast<float> (4.0 * rand () / RAND_MAX),
                                    static_cast<float> (4.0 * rand () / RAND_MAX));
    Eigen::Vector3f upperBoxCorner (static_cast<float> (5.0 + 4.0 * rand () / RAND_MAX),
                                    static_cast<float> (5.0 + 4.0 * rand () / RAND_MAX),
                                    static_cast<float> (5.0 + 4.0 * rand () / RAND_MAX));

    octree.boxSearch (lowerBoxCorner, upperBoxCorner, k_indices);

    // test every point in point cloud
    for (i = 0; i < 300; i++)
    {
      std::size_t j;
      bool inBox;
      bool idxInResults;
      const PointXYZ& pt = cloudIn->points[i];

      inBox = (pt.x > lowerBoxCorner (0)) && (pt.x < upperBoxCorner (0)) &&
              (pt.y > lowerBoxCorner (1)) && (pt.y < upperBoxCorner (1)) &&
              (pt.z > lowerBoxCorner (2)) && (pt.z < upperBoxCorner (2));

      idxInResults = false;
      for (j = 0; (j < k_indices.size ()) && (!idxInResults); ++j)
      {
        if (i == static_cast<unsigned int> (k_indices[j]))
          idxInResults = true;
      }

      ASSERT_EQ(idxInResults, inBox);

    }

  }
}

TEST(PCL, Octree_Pointcloud_Approx_Nearest_Neighbour_Search)
{
  const unsigned int test_runs = 100;
  unsigned int test_id;

  unsigned int bestMatchCount = 0;

  // instantiate point cloud
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  size_t i;
  srand (static_cast<unsigned int> (time (NULL)));

  double voxelResolution = 0.1;

  // create octree
  OctreePointCloudSearch<PointXYZ> octree (voxelResolution);
  octree.setInputCloud (cloudIn);

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point

    PointXYZ searchPoint (static_cast<float> (10.0 * rand () / RAND_MAX), 
                          static_cast<float> (10.0 * rand () / RAND_MAX),
                          static_cast<float> (10.0 * rand () / RAND_MAX));

    // generate point cloud
    cloudIn->width = 1000;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);
    for (i = 0; i < 1000; i++)
    {
      cloudIn->points[i] = PointXYZ (static_cast<float> (5.0  * rand () / RAND_MAX),
                                     static_cast<float> (10.0 * rand () / RAND_MAX),
                                     static_cast<float> (10.0 * rand () / RAND_MAX));
    }

    // brute force search
    double pointDist;
    double BFdistance = numeric_limits<double>::max ();
    int BFindex = 0;

    for (i = 0; i < cloudIn->points.size (); i++)
    {
      pointDist = ((cloudIn->points[i].x - searchPoint.x) * (cloudIn->points[i].x - searchPoint.x)
          + (cloudIn->points[i].y - searchPoint.y) * (cloudIn->points[i].y - searchPoint.y)
          + (cloudIn->points[i].z - searchPoint.z) * (cloudIn->points[i].z - searchPoint.z));

      if (pointDist < BFdistance)
      {
        BFindex = static_cast<int> (i);
        BFdistance = pointDist;
      }

    }

    int ANNindex;
    float ANNdistance;

    // octree approx. nearest neighbor search
    octree.deleteTree ();
    octree.addPointsFromInputCloud ();
    octree.approxNearestSearch (searchPoint, ANNindex, ANNdistance);

    if (BFindex == ANNindex)
    {
      EXPECT_NEAR(ANNdistance, BFdistance, 1e-4);
      bestMatchCount++;
    }

  }

  // we should have found the absolute nearest neighbor at least once
  ASSERT_EQ( (bestMatchCount > 0), true);

}

TEST (PCL, Octree_Pointcloud_Neighbours_Within_Radius_Search)
{

  const unsigned int test_runs = 100;
  unsigned int test_id;

  // instantiate point clouds
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());
  PointCloud<PointXYZ>::Ptr cloudOut (new PointCloud<PointXYZ> ());

  size_t i;

  srand (static_cast<unsigned int> (time (NULL)));

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point
    PointXYZ searchPoint (static_cast<float> (10.0 * rand () / RAND_MAX), 
                          static_cast<float> (10.0 * rand () / RAND_MAX),
                          static_cast<float> (10.0 * rand () / RAND_MAX));

    cloudIn->width = 1000;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);

    // generate point cloud data
    for (i = 0; i < 1000; i++)
    {
      cloudIn->points[i] = PointXYZ (static_cast<float> (10.0 * rand () / RAND_MAX),
                                     static_cast<float> (10.0 * rand () / RAND_MAX),
                                     static_cast<float> (5.0  * rand () / RAND_MAX));
    }

    OctreePointCloudSearch<PointXYZ> octree (0.001);

    // build octree
    octree.setInputCloud (cloudIn);
    octree.addPointsFromInputCloud ();

    double pointDist;
    double searchRadius = 5.0 * static_cast<float> (rand () / RAND_MAX);

    // bruteforce radius search
    vector<int> cloudSearchBruteforce;
    for (i = 0; i < cloudIn->points.size (); i++)
    {
      pointDist = sqrt (
          (cloudIn->points[i].x - searchPoint.x) * (cloudIn->points[i].x - searchPoint.x)
              + (cloudIn->points[i].y - searchPoint.y) * (cloudIn->points[i].y - searchPoint.y)
              + (cloudIn->points[i].z - searchPoint.z) * (cloudIn->points[i].z - searchPoint.z));

      if (pointDist <= searchRadius)
      {
        // add point candidates to vector list
        cloudSearchBruteforce.push_back (static_cast<int> (i));
      }
    }

    vector<int> cloudNWRSearch;
    vector<float> cloudNWRRadius;

    // execute octree radius search
    octree.radiusSearch (searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius);

    ASSERT_EQ( cloudNWRRadius.size(), cloudSearchBruteforce.size());

    // check if result from octree radius search can be also found in bruteforce search
    std::vector<int>::const_iterator current = cloudNWRSearch.begin ();
    while (current != cloudNWRSearch.end ())
    {
      pointDist = sqrt (
          (cloudIn->points[*current].x - searchPoint.x) * (cloudIn->points[*current].x - searchPoint.x)
              + (cloudIn->points[*current].y - searchPoint.y) * (cloudIn->points[*current].y - searchPoint.y)
              + (cloudIn->points[*current].z - searchPoint.z) * (cloudIn->points[*current].z - searchPoint.z));

      ASSERT_EQ( (pointDist<=searchRadius), true);

      ++current;
    }

    // check if result limitation works
    octree.radiusSearch (searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius, 5);

    ASSERT_EQ( cloudNWRRadius.size() <= 5, true);

  }

}

TEST (PCL, Octree_Pointcloud_Ray_Traversal)
{

  const unsigned int test_runs = 100;
  unsigned int test_id;

  // instantiate point clouds
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  octree::OctreePointCloudSearch<PointXYZ> octree_search (0.02f);

  // Voxels in ray
  pcl::PointCloud<pcl::PointXYZ>::VectorType voxelsInRay, voxelsInRay2;

  // Indices in ray
  std::vector<int> indicesInRay, indicesInRay2;

  srand (static_cast<unsigned int> (time (NULL)));

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // delete octree
    octree_search.deleteTree ();
    // define octree bounding box 10x10x10
    octree_search.defineBoundingBox (0.0, 0.0, 0.0, 10.0, 10.0, 10.0);

    cloudIn->width = 4;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);

    Eigen::Vector3f p (static_cast<float> (10.0 * rand () / RAND_MAX), 
                       static_cast<float> (10.0 * rand () / RAND_MAX),
                       static_cast<float> (10.0 * rand () / RAND_MAX));

    // origin
    Eigen::Vector3f o (static_cast<float> (12.0 * rand () / RAND_MAX), 
                       static_cast<float> (12.0 * rand () / RAND_MAX),
                       static_cast<float> (12.0 * rand () / RAND_MAX));

    cloudIn->points[0] = pcl::PointXYZ (p[0], p[1], p[2]);

    // direction vector
    Eigen::Vector3f dir (p - o);

    float tmin = 1.0;
    for (unsigned int j = 1; j < 4; j++)
    {
      tmin = tmin - 0.25f;
      Eigen::Vector3f n_p = o + (tmin * dir);
      cloudIn->points[j] = pcl::PointXYZ (n_p[0], n_p[1], n_p[2]);
    }

    // insert cloud point into octree
    octree_search.setInputCloud (cloudIn);
    octree_search.addPointsFromInputCloud ();

    octree_search.getIntersectedVoxelCenters (o, dir, voxelsInRay);
    octree_search.getIntersectedVoxelIndices (o, dir, indicesInRay);

    // check if all voxels in the cloud are penetraded by the ray
    ASSERT_EQ( voxelsInRay.size (), cloudIn->points.size ());
    // check if all indices of penetrated voxels are in cloud
    ASSERT_EQ( indicesInRay.size (), cloudIn->points.size ());

    octree_search.getIntersectedVoxelCenters (o, dir, voxelsInRay2, 1);
    octree_search.getIntersectedVoxelIndices (o, dir, indicesInRay2, 1);

    // check if only the point from a single voxel has been returned
    ASSERT_EQ( voxelsInRay2.size (), 1u );
    ASSERT_EQ( indicesInRay2.size (), 1u );

    // check if this point is the closest point to the origin
    pcl::PointXYZ pt = cloudIn->points[ indicesInRay2[0] ];
    Eigen::Vector3f d = Eigen::Vector3f (pt.x, pt.y, pt.z) - o;
    float min_dist = d.norm ();

    for (unsigned int i = 0; i < cloudIn->width * cloudIn->height; i++) {
    	pt = cloudIn->points[i];
    	d = Eigen::Vector3f (pt.x, pt.y, pt.z) - o;
    	ASSERT_GE( d.norm (), min_dist );
    }

  }

}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
