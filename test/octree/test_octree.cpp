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

TEST (PCL, Octree_Test)
{

  unsigned int i, j;
  int data[256];

  // create octree instance
  OctreeBase<int> octreeA;
  OctreeBase<int> octreeB;

  // set octree depth
  octreeA.setTreeDepth (8);
  octreeB.setTreeDepth (8);

  struct MyVoxel
  {
    unsigned int x;
    unsigned int y;
    unsigned int z;
  };
  MyVoxel voxels[256];

  srand (static_cast<unsigned int> (time (NULL)));

  // generate some voxel indices
  for (i = 0; i < 256; i++)
  {
    data[i] = i;

    voxels[i].x = i;
    voxels[i].y = 255 - i;
    voxels[i].z = i;

    // add data to leaf node voxel
    int* voxel_container = octreeA.createLeaf(voxels[i].x, voxels[i].y, voxels[i].z);
    *voxel_container = data[i];
  }

  for (i = 0; i < 128; i++)
  {
    // retrieve data from leaf node voxel
    int* voxel_container = octreeA.createLeaf(voxels[i].x, voxels[i].y, voxels[i].z);
    // check if retrieved data is identical to data[]
    ASSERT_EQ(*voxel_container, data[i]);
  }

  for (i = 128; i < 256; i++)
  {
    // check if leaf node exists in tree
    ASSERT_EQ( octreeA.existLeaf(voxels[i].x,voxels[i].y,voxels[i].z), true);

    // remove leaf node
    octreeA.removeLeaf (voxels[i].x, voxels[i].y, voxels[i].z);

    //  leaf node shouldn't exist in tree anymore
    ASSERT_EQ( octreeA.existLeaf(voxels[i].x,voxels[i].y,voxels[i].z), false);
  }

  // test serialization

  std::vector<char> treeBinaryA;
  std::vector<char> treeBinaryB;

  std::vector<int*> leafVectorA;
  std::vector<int*> leafVectorB;

  // serialize tree - generate binary octree description
  octreeA.serializeTree (treeBinaryA);

  // deserialize tree - rebuild octree based on binary octree description
  octreeB.deserializeTree (treeBinaryA);

  for (i = 0; i < 128; i++)
  {
    // check if leafs exist in both octrees
    ASSERT_EQ( octreeA.existLeaf(voxels[i].x,voxels[i].y,voxels[i].z), true);
    ASSERT_EQ( octreeB.existLeaf(voxels[i].x,voxels[i].y,voxels[i].z), true);
  }

  for (i = 128; i < 256; i++)
  {
    // these leafs were not copies and should not exist
    ASSERT_EQ( octreeB.existLeaf(voxels[i].x,voxels[i].y,voxels[i].z), false);
  }

  // testing deleteTree();
  octreeB.deleteTree ();

  // octreeB.getLeafCount() should be zero now;
  ASSERT_EQ (0u, octreeB.getLeafCount());

  // .. and previous leafs deleted..
  for (i = 0; i < 128; i++)
  {
    ASSERT_EQ(octreeB.existLeaf(voxels[i].x,voxels[i].y,voxels[i].z), false);
  }

  // test tree serialization
  octreeA.serializeTree (treeBinaryA, leafVectorA);

  // make sure, we retrieved all data objects
  ASSERT_EQ(leafVectorA.size(), octreeA.getLeafCount());

  // check if leaf data is found in octree input data
  bool bFound;
  for (i = 0; i < 128; i++)
  {
    int leafInt = *leafVectorA.back ();
    leafVectorA.pop_back ();

    bFound = false;
    for (j = 0; j < 256; j++)
      if (data[j] == leafInt)
      {
        bFound = true;
        break;
      }

    ASSERT_EQ(bFound, true);
  }

  // test tree serialization
  octreeA.serializeLeafs (leafVectorA);

  for (i = 0; i < 128; i++)
  {
    int leafInt = *leafVectorA.back ();
    leafVectorA.pop_back ();

    bFound = false;
    for (j = 0; j < 256; j++)
      if (data[j] == leafInt)
      {
        bFound = true;
        break;
      }

    ASSERT_EQ(bFound, true);
  }

  // test tree serialization with leaf data vectors
  octreeA.serializeTree (treeBinaryA, leafVectorA);
  octreeB.deserializeTree (treeBinaryA, leafVectorA);

  // test size and leaf count of reconstructed octree
  ASSERT_EQ(octreeA.getLeafCount(), octreeB.getLeafCount());
  ASSERT_EQ(128u, octreeB.getLeafCount());

  octreeB.serializeTree (treeBinaryB, leafVectorB);

  // compare octree data content of octree A and octree B
  ASSERT_EQ(leafVectorB.size(), octreeB.getLeafCount());
  ASSERT_EQ(leafVectorA.size(), leafVectorB.size());

  for (i = 0; i < leafVectorB.size (); i++)
  {
    ASSERT_EQ( (*leafVectorA[i] == *leafVectorB[i]), true);
  }

  //  test iterator

  OctreeBase<int>::Iterator a_it;
  OctreeBase<int>::Iterator a_it_end = octreeA.end();

  unsigned int node_count = 0;
  unsigned int branch_count = 0;
  unsigned int leaf_count = 0;

  // iterate over tree
  for (a_it=octreeA.begin(); a_it!=a_it_end; ++a_it)
  {
    // depth should always be less than tree depth
    unsigned int depth = a_it.getCurrentOctreeDepth ();
    ASSERT_LE(depth, octreeA.getTreeDepth());

    // store node, branch and leaf count
    const OctreeNode* node = a_it.getCurrentOctreeNode ();
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
  ASSERT_EQ(leaf_count, octreeA.getLeafCount ());

}

TEST (PCL, Octree_Dynamic_Depth_Test)
{

  size_t i;
  int test_runs = 100;
  int pointcount = 300;

  int test, point;

  float resolution = 0.01f;

  const static size_t leafAggSize = 5;

  OctreePointCloudDynamicDepth<PointXYZ> octree (resolution, leafAggSize);

  // create shared pointcloud instances
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());

  // assign input point clouds to octree
  octree.setInputCloud (cloud);

  for (test = 0; test < test_runs; ++test)
  {
    // clean up
    cloud->points.clear ();
    octree.deleteTree ();
    
    PointXYZ newPoint (1.5, 2.5, 3.5);
    cloud->push_back (newPoint);

    for (point = 0; point < 15; point++)
    {
      // gereate a random point
      PointXYZ newPoint (1.0, 2.0, 3.0);

      // OctreePointCloudPointVector can store all points..
      cloud->push_back (newPoint);
    }

    // check if all points from leaf data can be found in input pointcloud data sets
    octree.addPointsFromInputCloud ();

    unsigned int leaf_node_counter = 0;
    // iterate over tree
    OctreePointCloudDynamicDepth<PointXYZ>::LeafNodeIterator it2;
    const OctreePointCloudDynamicDepth<PointXYZ>::LeafNodeIterator it2_end = octree.leaf_end();
    for (it2 = octree.leaf_begin(); it2 != it2_end; ++it2)
    {
      ++leaf_node_counter;
      unsigned int depth = it2.getCurrentOctreeDepth ();
      ASSERT_EQ((depth==1)||(depth==6), true);
    }

    // clean up
    cloud->points.clear ();
    octree.deleteTree ();

    for (point = 0; point < pointcount; point++)
    {
      // gereate a random point
      PointXYZ newPoint (static_cast<float> (1024.0 * rand () / RAND_MAX),
          static_cast<float> (1024.0 * rand () / RAND_MAX),
          static_cast<float> (1024.0 * rand () / RAND_MAX));

      // OctreePointCloudPointVector can store all points..
      cloud->push_back (newPoint);
    }


    // check if all points from leaf data can be found in input pointcloud data sets
    octree.defineBoundingBox ();
    octree.addPointsFromInputCloud ();

    //  test iterator
    OctreePointCloudDynamicDepth<PointXYZ>::LeafNodeIterator it;
    const OctreePointCloudDynamicDepth<PointXYZ>::LeafNodeIterator it_end = octree.leaf_end();
    unsigned int leaf_count = 0;

    std::vector<int> indexVector;

    // iterate over tree
    for (it = octree.leaf_begin(); it != it_end; ++it)
    {
      OctreeNode* node = it.getCurrentOctreeNode ();


      ASSERT_EQ(node->getNodeType(), LEAF_NODE);

      OctreeContainerPointIndices& container = it.getLeafContainer();
      if (it.getCurrentOctreeDepth () < octree.getTreeDepth ())
        ASSERT_LE(container.getSize(), leafAggSize);

      // add points from leaf node to indexVector
      container.getPointIndices (indexVector);

      // test points against bounding box of leaf node
      std::vector<int> tmpVector;
      container.getPointIndices (tmpVector);

      Eigen::Vector3f min_pt, max_pt;
      octree.getVoxelBounds (it, min_pt, max_pt);

      for (i=0; i<tmpVector.size(); ++i)
      {
        ASSERT_GE(cloud->points[tmpVector[i]].x, min_pt(0));
        ASSERT_GE(cloud->points[tmpVector[i]].y, min_pt(1));
        ASSERT_GE(cloud->points[tmpVector[i]].z, min_pt(2));
        ASSERT_LE(cloud->points[tmpVector[i]].x, max_pt(0));
        ASSERT_LE(cloud->points[tmpVector[i]].y, max_pt(1));
        ASSERT_LE(cloud->points[tmpVector[i]].z, max_pt(2));
      }

      leaf_count++;

    }
    ASSERT_EQ(pointcount, indexVector.size());

    // make sure all indices are within indexVector
    for (i = 0; i < indexVector.size (); ++i)
    {
#if !defined(__APPLE__)
      bool indexFound = (std::find(indexVector.begin(), indexVector.end(), i) != indexVector.end());
      ASSERT_EQ(indexFound, true);
#endif
    }

    // compare node, branch and leaf count against actual tree values
    ASSERT_EQ(leaf_count, octree.getLeafCount ());

  }
}

TEST (PCL, Octree_Pointcloud_Test)
{

  size_t i;
  int test_runs = 100;
  int pointcount = 300;

  int test, point;

  float resolution = 0.01f;

  // instantiate OctreePointCloudSinglePoint and OctreePointCloudPointVector classes
  OctreePointCloudSinglePoint<PointXYZ> octreeA (resolution);
  OctreePointCloudPointVector<PointXYZ> octreeC (resolution);

  // create shared pointcloud instances
  PointCloud<PointXYZ>::Ptr cloudA (new PointCloud<PointXYZ> ());
  PointCloud<PointXYZ>::Ptr cloudB (new PointCloud<PointXYZ> ());

  // assign input point clouds to octree
  octreeA.setInputCloud (cloudA);
  octreeC.setInputCloud (cloudB);

  for (test = 0; test < test_runs; ++test)
  {

    // clean up
    cloudA->points.clear ();
    octreeA.deleteTree ();

    octreeC.deleteTree ();

    for (point = 0; point < pointcount; point++)
    {
      // gereate a random point
      PointXYZ newPoint (static_cast<float> (1024.0 * rand () / RAND_MAX), 
                         static_cast<float> (1024.0 * rand () / RAND_MAX), 
                         static_cast<float> (1024.0 * rand () / RAND_MAX));

      if (!octreeA.isVoxelOccupiedAtPoint (newPoint))
      {
        // add point only to OctreePointCloudSinglePoint if voxel at point location doesn't exist
        octreeA.addPointToCloud (newPoint, cloudA);
      }

      // OctreePointCloudPointVector can store all points..
      cloudB->push_back (newPoint);
    }

    ASSERT_EQ(octreeA.getLeafCount(), cloudA->points.size());

    // checks for getVoxelDataAtPoint() and isVoxelOccupiedAtPoint() functionality
    for (i = 0; i < cloudA->points.size (); i++)
    {
      ASSERT_EQ(octreeA.isVoxelOccupiedAtPoint(cloudA->points[i]), true);
      octreeA.deleteVoxelAtPoint (cloudA->points[i]);
      ASSERT_EQ(octreeA.isVoxelOccupiedAtPoint(cloudA->points[i]), false);
    }

    ASSERT_EQ(octreeA.getLeafCount(), static_cast<unsigned int> (0));
  }

}

TEST (PCL, Octree_Pointcloud_Density_Test)
{

  // instantiate point cloud and fill it with point data

  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  for (float z = 0.05f; z < 7.0f; z += 0.1f)
    for (float y = 0.05f; y < 7.0f; y += 0.1f)
      for (float x = 0.05f; x < 7.0f; x += 0.1f)
        cloudIn->push_back (PointXYZ (x, y, z));

  cloudIn->width = static_cast<uint32_t> (cloudIn->points.size ());
  cloudIn->height = 1;

  OctreePointCloudDensity<PointXYZ> octreeA (1.0f); // low resolution
  OctreePointCloudDensity<PointXYZ> octreeB (0.00001f); // high resolution

  octreeA.defineBoundingBox (7.0, 7.0, 7.0);
  octreeB.defineBoundingBox (7.0, 7.0, 7.0);

  // add point data to octree
  octreeA.setInputCloud (cloudIn);
  octreeB.setInputCloud (cloudIn);

  octreeA.addPointsFromInputCloud ();
  octreeB.addPointsFromInputCloud ();

  // check density information
  for (float z = 1.5f; z < 3.5f; z += 1.0f)
    for (float y = 1.5f; y < 3.5f; y += 1.0f)
      for (float x = 1.5f; x < 3.5f; x += 1.0f)
        ASSERT_EQ(octreeA.getVoxelDensityAtPoint (PointXYZ(x, y, z)), 1000u);

  for (float z = 0.05f; z < 5.0f; z += 0.1f)
    for (float y = 0.05f; y < 5.0f; y += 0.1f)
      for (float x = 0.05f; x < 5.0f; x += 0.1f)
        ASSERT_EQ(octreeB.getVoxelDensityAtPoint (PointXYZ(x, y, z)), 1u);
}

TEST (PCL, Octree_Pointcloud_Iterator_Test)
{
  // instantiate point cloud and fill it with point data

  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  for (float z = 0.05f; z < 7.0f; z += 0.1f)
    for (float y = 0.05f; y < 7.0f; y += 0.1f)
      for (float x = 0.05f; x < 7.0f; x += 0.1f)
        cloudIn->push_back (PointXYZ (x, y, z));

  cloudIn->width = static_cast<uint32_t> (cloudIn->points.size ());
  cloudIn->height = 1;

  OctreePointCloud<PointXYZ> octreeA (1.0f); // low resolution

  // add point data to octree
  octreeA.setInputCloud (cloudIn);
  octreeA.addPointsFromInputCloud ();

  // instantiate iterator for octreeA
  OctreePointCloud<PointXYZ>::LeafNodeIterator it1;
  OctreePointCloud<PointXYZ>::LeafNodeIterator it1_end = octreeA.leaf_end();

  std::vector<int> indexVector;
  unsigned int leafNodeCounter = 0;

  for (it1 = octreeA.leaf_begin(); it1 != it1_end; ++it1)
  {
    it1.getLeafContainer().getPointIndices(indexVector);
    leafNodeCounter++;
  }

  ASSERT_EQ(indexVector.size(), cloudIn->points.size ());
  ASSERT_EQ(leafNodeCounter, octreeA.getLeafCount());

  OctreePointCloud<PointXYZ>::Iterator it2;
  OctreePointCloud<PointXYZ>::Iterator it2_end = octreeA.end();

  unsigned int traversCounter = 0;
  for (it2 = octreeA.begin(); it2 != it2_end; ++it2)
  {
    traversCounter++;
  }

  ASSERT_EQ(octreeA.getLeafCount() + octreeA.getBranchCount(), traversCounter);

  // breadth-first iterator test

  unsigned int lastDepth = 0;
  unsigned int branchNodeCount = 0;
  unsigned int leafNodeCount = 0;

  bool leafNodeVisited = false;

  OctreePointCloud<PointXYZ>::BreadthFirstIterator bfIt;
  const OctreePointCloud<PointXYZ>::BreadthFirstIterator bfIt_end = octreeA.breadth_end();

  for (bfIt = octreeA.breadth_begin(); bfIt != bfIt_end; ++bfIt)
  {
    // tree depth of visited nodes must grow
    ASSERT_EQ( bfIt.getCurrentOctreeDepth()>=lastDepth, true);
    lastDepth = bfIt.getCurrentOctreeDepth ();

    if (bfIt.isBranchNode ())
    {
      branchNodeCount++;
      // leaf nodes are traversed in the end
      ASSERT_EQ( leafNodeVisited, false);
    }

    if (bfIt.isLeafNode ())
    {
      leafNodeCount++;
      leafNodeVisited = true;
    }
  }

  // check if every branch node and every leaf node has been visited
  ASSERT_EQ( leafNodeCount, octreeA.getLeafCount());
  ASSERT_EQ( branchNodeCount, octreeA.getBranchCount());

}

TEST(PCL, Octree_Pointcloud_Occupancy_Test)
{
  const unsigned int test_runs = 100;
  unsigned int test_id;

  // instantiate point cloud

  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  OctreePointCloudOccupancy<PointXYZ> octree (0.00001f);

  size_t i;

  srand (static_cast<unsigned int> (time (NULL)));

  for (test_id = 0; test_id < test_runs; test_id++)
  {

    cloudIn->width = 1000;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);

    // generate point data for point cloud
    for (i = 0; i < 1000; i++)
    {
      cloudIn->points[i] = PointXYZ (static_cast<float> (5.0  * rand () / RAND_MAX),
                                     static_cast<float> (10.0 * rand () / RAND_MAX),
                                     static_cast<float> (10.0 * rand () / RAND_MAX));
    }

    // create octree
    octree.setInputCloud (cloudIn);
    octree.addPointsFromInputCloud ();

    // check occupancy of voxels
    for (i = 0; i < 1000; i++)
    {
      ASSERT_EQ(octree.isVoxelOccupiedAtPoint(cloudIn->points[i]), true);
      octree.deleteVoxelAtPoint (cloudIn->points[i]);
      ASSERT_EQ(octree.isVoxelOccupiedAtPoint(cloudIn->points[i]), false);
    }

  }

}

TEST (PCL, Octree_Pointcloud_Change_Detector_Test)
{
  // instantiate point cloud

  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  OctreePointCloudChangeDetector<PointXYZ> octree (0.01f);

  size_t i;

  srand (static_cast<unsigned int> (time (NULL)));

  cloudIn->width = 1000;
  cloudIn->height = 1;
  cloudIn->points.resize (cloudIn->width * cloudIn->height);

  // generate point data for point cloud
  for (i = 0; i < 1000; i++)
  {
    cloudIn->points[i] = PointXYZ (static_cast<float> (5.0  * rand () / RAND_MAX),
                                   static_cast<float> (10.0 * rand () / RAND_MAX),
                                   static_cast<float> (10.0 * rand () / RAND_MAX));
  }

  // assign point cloud to octree
  octree.setInputCloud (cloudIn);

  // add points from cloud to octree
  octree.addPointsFromInputCloud ();

  // switch buffers - reset tree
  octree.switchBuffers ();

  // add points from cloud to new octree buffer
  octree.addPointsFromInputCloud ();

  // add 1000 additional points
  for (i = 0; i < 1000; i++)
  {
    octree.addPointToCloud (
        PointXYZ (static_cast<float> (100.0 + 5.0  * rand () / RAND_MAX), 
                  static_cast<float> (100.0 + 10.0 * rand () / RAND_MAX),
                  static_cast<float> (100.0 + 10.0 * rand () / RAND_MAX)),
        cloudIn);
  }

  vector<int> newPointIdxVector;

  // get a vector of new points, which did not exist in previous buffer
  octree.getPointIndicesFromNewVoxels (newPointIdxVector);

  // should be 1000
  ASSERT_EQ( newPointIdxVector.size(), static_cast<std::size_t> (1000));

  // all point indices found should have an index of >= 1000
  for (i = 0; i < 1000; i++)
  {
    ASSERT_EQ( ( newPointIdxVector [i] >= 1000 ), true);
  }

}

TEST (PCL, Octree_Pointcloud_Voxel_Centroid_Test)
{

  // instantiate point cloud

  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  OctreePointCloudVoxelCentroid<PointXYZ> octree (1.0f);
  octree.defineBoundingBox (10.0, 10.0, 10.0);

  size_t i;

  srand (static_cast<unsigned int> (time (NULL)));

  cloudIn->width = 10 * 3;
  cloudIn->height = 1;
  cloudIn->points.resize (cloudIn->width * cloudIn->height);

  // generate point data for point cloud
  for (i = 0; i < 10; i++)
  {
    // these three points should always be assigned to the same voxel in the octree
    cloudIn->points[i * 3 + 0] = PointXYZ (static_cast<float> (i) + 0.2f, static_cast<float> (i) + 0.2f, static_cast<float> (i) + 0.2f);
    cloudIn->points[i * 3 + 1] = PointXYZ (static_cast<float> (i) + 0.4f, static_cast<float> (i) + 0.4f, static_cast<float> (i) + 0.4f);
    cloudIn->points[i * 3 + 2] = PointXYZ (static_cast<float> (i) + 0.6f, static_cast<float> (i) + 0.6f, static_cast<float> (i) + 0.6f);
  }

  // assign point cloud to octree
  octree.setInputCloud (cloudIn);

  // add points from cloud to octree
  octree.addPointsFromInputCloud ();

  pcl::PointCloud<PointXYZ>::VectorType voxelCentroids;
  octree.getVoxelCentroids (voxelCentroids);

  // we expect 10 voxel centroids
  ASSERT_EQ (voxelCentroids.size(), static_cast<std::size_t> (10));

  // check centroid calculation
  for (i = 0; i < 10; i++)
  {
    EXPECT_NEAR(voxelCentroids[i].x, static_cast<float> (i)+0.4, 1e-4);
    EXPECT_NEAR(voxelCentroids[i].y, static_cast<float> (i)+0.4, 1e-4);
    EXPECT_NEAR(voxelCentroids[i].z, static_cast<float> (i)+0.4, 1e-4);
  }

  // ADDING AN ADDITIONAL POINT CLOUD TO OctreePointCloudVoxelCentroid

  // generate new point data
  for (i = 0; i < 10; i++)
  {
    // these three points should always be assigned to the same voxel in the octree
    cloudIn->points[i * 3 + 0] = PointXYZ (static_cast<float> (i) + 0.1f, static_cast<float> (i) + 0.1f, static_cast<float> (i) + 0.1f);
    cloudIn->points[i * 3 + 1] = PointXYZ (static_cast<float> (i) + 0.4f, static_cast<float> (i) + 0.4f, static_cast<float> (i) + 0.4f);
    cloudIn->points[i * 3 + 2] = PointXYZ (static_cast<float> (i) + 0.7f, static_cast<float> (i) + 0.7f, static_cast<float> (i) + 0.7f);
  }

  // add points from new cloud to octree
  octree.setInputCloud (cloudIn);
  octree.addPointsFromInputCloud ();

  voxelCentroids.clear();
  octree.getVoxelCentroids (voxelCentroids);

  // check centroid calculation
  for (i = 0; i < 10; i++)
  {
    EXPECT_NEAR(voxelCentroids[i].x, static_cast<float> (i)+0.4, 1e-4);
    EXPECT_NEAR(voxelCentroids[i].y, static_cast<float> (i)+0.4, 1e-4);
    EXPECT_NEAR(voxelCentroids[i].z, static_cast<float> (i)+0.4, 1e-4);
  }

}

TEST (PCL, Octree_Pointcloud_Adjacency)
{

  const unsigned int test_runs = 100;
  unsigned int test_id;

  srand (static_cast<unsigned int> (time (NULL)));

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // instantiate point cloud
    PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());
    
    float resolution = static_cast<float> (0.01 * rand () / RAND_MAX) + 0.00001f;
    //Define a random point
    PointXYZ point (static_cast<float> (0.5 * rand () / RAND_MAX), 
                    static_cast<float> (0.5 * rand () / RAND_MAX),
                    static_cast<float> (0.5 * rand () / RAND_MAX));
    //This is done to define the grid
    PointXYZ p1 (10,10,10);
    PointXYZ p2 (-10,-10,-10);
    cloudIn->push_back(p1);
    cloudIn->push_back(p2);
    cloudIn->push_back (point);

    // generate neighbors
    for (int dx = -1; dx <= 1; ++dx)
    {
      for (int dy = -1; dy <= 1; ++dy)
      {
        for (int dz = -1; dz <= 1; ++dz)
        {
          float factor = 1.0f;//*std::sqrt (dx*dx + dy*dy + dz*dz);          
          PointXYZ neighbor (point.x+dx*resolution*factor, point.y+dy*resolution*factor, point.z+dz*resolution*factor); 
          cloudIn->push_back (neighbor);
        }
      }
    }
    
    //Add another point that isn't touching previous or neighbors
    PointXYZ point2 (static_cast<float> (point.x + 10*resolution), 
                     static_cast<float> (point.y + 10*resolution),
                     static_cast<float> (point.z + 10*resolution));
    cloudIn->push_back (point2);
    // Add points which are not neighbors
    for (int i = 0; i < 100; ++i)
    {
      PointXYZ not_neighbor_point (static_cast<float> (10.0 * rand () / RAND_MAX), 
                     static_cast<float> (10.0 * rand () / RAND_MAX),
                     static_cast<float> (10.0 * rand () / RAND_MAX));
      if ( (point2.getVector3fMap () - not_neighbor_point.getVector3fMap ()).norm () > 3*resolution )
      {
        cloudIn->push_back(not_neighbor_point);
      }
    }
    
      

    octree::OctreePointCloudAdjacency<PointXYZ> octree (resolution);
    octree.setInputCloud (cloudIn);
    octree.addPointsFromInputCloud (); 
      
    //Point should have 26 neighbors, plus itself
    ASSERT_EQ( octree.getLeafContainerAtPoint (point)->getNumNeighbors () == 27, true);
  
    //Other point should only have itself for neighbor
    ASSERT_EQ( octree.getLeafContainerAtPoint (point2)->getNumNeighbors () == 1, true);

  }

}

TEST (PCL, Octree_CentroidPoint)
{
  PointXYZ p1; p1.getVector3fMap () << 1, 2, 3;
  PointXYZ p2; p2.getVector3fMap () << 3, 2, 1;
  PointXYZ p3; p3.getVector3fMap () << 5, 5, 5;
  PointXYZRGB cp1; cp1.getVector3fMap () << 4, 2, 4; cp1.rgba = 0x330000;
  PointXYZRGB cp2; cp2.getVector3fMap () << 2, 4, 2; cp2.rgba = 0x003300;
  PointXYZRGB cp3; cp3.getVector3fMap () << 3, 3, 3; cp3.rgba = 0x000033;
  Normal np1; np1.getNormalVector4fMap () << 1, 0, 0, 0;
  Normal np2; np2.getNormalVector4fMap () << -1, 0, 0, 0;
  Normal np3; np3.getNormalVector4fMap () << 0, 1, 0, 0;

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
    PointXYZRGB c; c.rgba = 0xFFFFFF;
    centroid.get (c);
    EXPECT_XYZ_EQ (p1, c);
    EXPECT_EQ (0xFFFFFF, c.rgba);
  }

  // Centroid with XYZ and RGB
  {
    CentroidPoint<PointXYZRGB> centroid;
    centroid.add (cp1);
    centroid.add (cp2);
    centroid.add (cp3);
    PointXYZRGB c;
    centroid.get (c);
    EXPECT_XYZ_EQ (PointXYZ (3, 3, 3), c);
    EXPECT_EQ (0x111111, c.rgba);
  }

  // Centroid with normal
  {
    CentroidPoint<Normal> centroid;
    centroid.add (np1);
    centroid.add (np2);
    centroid.add (np3);
    Normal c;
    centroid.get (c);
    EXPECT_NORMAL_EQ (np3, c);
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
