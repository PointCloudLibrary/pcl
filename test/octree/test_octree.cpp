/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2017-, Open Perception, Inc.
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
 *
 */
#include <gtest/gtest.h>

#include <vector>

#include <stdio.h>

using namespace std;

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>

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
    ASSERT_EQ (data[i], *voxel_container);
  }

  for (i = 128; i < 256; i++)
  {
    // check if leaf node exists in tree
    ASSERT_TRUE (octreeA.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));

    // remove leaf node
    octreeA.removeLeaf (voxels[i].x, voxels[i].y, voxels[i].z);

    //  leaf node shouldn't exist in tree anymore
    ASSERT_FALSE (octreeA.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));
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
    ASSERT_TRUE (octreeA.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));
    ASSERT_TRUE (octreeB.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));
  }

  for (i = 128; i < 256; i++)
  {
    // these leafs were not copies and should not exist
    ASSERT_FALSE (octreeB.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));
  }

  // testing deleteTree();
  octreeB.deleteTree ();

  // octreeB.getLeafCount() should be zero now;
  ASSERT_EQ (0u, octreeB.getLeafCount ());

  // .. and previous leafs deleted..
  for (i = 0; i < 128; i++)
  {
    ASSERT_FALSE (octreeB.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));
  }

  // test tree serialization
  octreeA.serializeTree (treeBinaryA, leafVectorA);

  // make sure, we retrieved all data objects
  ASSERT_EQ (octreeA.getLeafCount (), leafVectorA.size ());

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

    ASSERT_TRUE (bFound);
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

    ASSERT_TRUE (bFound);
  }

  // test tree serialization with leaf data vectors
  octreeA.serializeTree (treeBinaryA, leafVectorA);
  octreeB.deserializeTree (treeBinaryA, leafVectorA);

  // test size and leaf count of reconstructed octree
  ASSERT_EQ (octreeA.getLeafCount (), octreeB.getLeafCount ());
  ASSERT_EQ (128u, octreeB.getLeafCount ());

  octreeB.serializeTree (treeBinaryB, leafVectorB);

  // compare octree data content of octree A and octree B
  ASSERT_EQ (leafVectorB.size (), octreeB.getLeafCount ());
  ASSERT_EQ (leafVectorA.size (), leafVectorB.size ());

  for (i = 0; i < leafVectorB.size (); i++)
  {
    ASSERT_EQ (*leafVectorA[i], *leafVectorB[i]);
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
    ASSERT_LE (depth, octreeA.getTreeDepth ());

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
  ASSERT_EQ (branch_count + leaf_count, node_count);
  ASSERT_EQ (octreeA.getLeafCount (), leaf_count);
}

TEST (PCL, Octree_Dynamic_Depth_Test)
{

  size_t i;
  int test_runs = 100;
  int pointcount = 300;

  int test, point;

  float resolution = 0.01f;

  const static size_t leafAggSize = 5;

  OctreePointCloudPointVector<PointXYZ> octree (resolution);

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
    octree.defineBoundingBox ();
    octree.enableDynamicDepth (leafAggSize);
    octree.addPointsFromInputCloud ();

    unsigned int leaf_node_counter = 0;
    // iterate over tree
    OctreePointCloudPointVector<PointXYZ>::LeafNodeIterator it2;
    const OctreePointCloudPointVector<PointXYZ>::LeafNodeIterator it2_end = octree.leaf_end();
    for (it2 = octree.leaf_begin(); it2 != it2_end; ++it2)
    {
      ++leaf_node_counter;
      unsigned int depth = it2.getCurrentOctreeDepth ();
      ASSERT_TRUE ((depth == 1) || (depth == 6));
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
    octree.enableDynamicDepth (leafAggSize);
    octree.addPointsFromInputCloud ();

    //  test iterator
    OctreePointCloudPointVector<PointXYZ>::LeafNodeIterator it;
    const OctreePointCloudPointVector<PointXYZ>::LeafNodeIterator it_end = octree.leaf_end();
    unsigned int leaf_count = 0;

    std::vector<int> indexVector;

    // iterate over tree
    for (it = octree.leaf_begin(); it != it_end; ++it)
    {
      OctreeNode* node = it.getCurrentOctreeNode ();


      ASSERT_EQ (LEAF_NODE, node->getNodeType());

      OctreeContainerPointIndices& container = it.getLeafContainer();
      if (it.getCurrentOctreeDepth () < octree.getTreeDepth ())
        ASSERT_LE (container.getSize (), leafAggSize);

      // add points from leaf node to indexVector
      container.getPointIndices (indexVector);

      // test points against bounding box of leaf node
      std::vector<int> tmpVector;
      container.getPointIndices (tmpVector);

      Eigen::Vector3f min_pt, max_pt;
      octree.getVoxelBounds (it, min_pt, max_pt);

      for (i=0; i<tmpVector.size(); ++i)
      {
        ASSERT_GE (cloud->points[tmpVector[i]].x, min_pt(0));
        ASSERT_GE (cloud->points[tmpVector[i]].y, min_pt(1));
        ASSERT_GE (cloud->points[tmpVector[i]].z, min_pt(2));
        ASSERT_LE (cloud->points[tmpVector[i]].x, max_pt(0));
        ASSERT_LE (cloud->points[tmpVector[i]].y, max_pt(1));
        ASSERT_LE (cloud->points[tmpVector[i]].z, max_pt(2));
      }

      leaf_count++;

    }

    ASSERT_EQ (indexVector.size(), pointcount);

    // make sure all indices are within indexVector
    for (i = 0; i < indexVector.size (); ++i)
    {
#if !defined(__APPLE__)
      bool indexFound = (std::find(indexVector.begin(), indexVector.end(), i) != indexVector.end());
      ASSERT_TRUE (indexFound);
#endif
    }

    // compare node, branch and leaf count against actual tree values
    ASSERT_EQ (octree.getLeafCount (), leaf_count);
  }
}

TEST (PCL, Octree2Buf_Test)
{

  // create octree instances
  Octree2BufBase<int> octreeA;
  Octree2BufBase<int> octreeB;

  // set octree depth
  octreeA.setTreeDepth (8);
  octreeB.setTreeDepth (8);

  struct MyVoxel
  {
    unsigned int x;
    unsigned int y;
    unsigned int z;
  };

  unsigned int i, j;
  int data[256];
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
    data[i] = *voxel_container;

  }

  ASSERT_EQ (static_cast<unsigned int> (256), octreeA.getLeafCount ());

  for (i = 0; i < 128; i++)
  {
    // retrieve and check data from leaf voxel
    int* voxel_container = octreeA.findLeaf(voxels[i].x, voxels[i].y, voxels[i].z);
    ASSERT_EQ (data[i], *voxel_container);
  }

  for (i = 128; i < 256; i++)
  {
    // check if leaf node exists in tree
    ASSERT_TRUE (octreeA.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));

    // remove leaf node
    octreeA.removeLeaf (voxels[i].x, voxels[i].y, voxels[i].z);

    //  leaf node shouldn't exist in tree anymore
    ASSERT_FALSE (octreeA.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));
  }

  ////////////

  // test serialization

  std::vector<char> treeBinaryA;
  std::vector<char> treeBinaryB;

  std::vector<int*> leafVectorA;
  std::vector<int*> leafVectorB;

  // serialize tree - generate binary octree description
  octreeA.serializeTree (treeBinaryA);

  // deserialize tree - rebuild octree based on binary octree description
  octreeB.deserializeTree (treeBinaryA);

  // check if leafs exist in octrees
  for (i = 0; i < 128; i++)
  {
    ASSERT_TRUE (octreeB.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));
  }

  // these leafs should not exist..
  for (i = 128; i < 256; i++)
  {
    ASSERT_FALSE (octreeB.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));
  }

  // checking deleteTree();
  octreeB.deleteTree ();
  octreeB.setTreeDepth (8);

  // octreeB.getLeafCount() should be zero now;
  ASSERT_EQ (static_cast<unsigned int> (0), octreeB.getLeafCount ());

  for (i = 0; i < 128; i++)
  {
    ASSERT_FALSE (octreeB.existLeaf (voxels[i].x, voxels[i].y, voxels[i].z));
  }

  // test tree serialization
  octreeA.serializeTree (treeBinaryA, leafVectorA);

  // make sure, we retrieved all data objects
  ASSERT_EQ (octreeA.getLeafCount (), leafVectorA.size ());

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

    ASSERT_TRUE (bFound);
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

    ASSERT_TRUE (bFound);
  }

  // test tree serialization with leaf data vectors
  octreeA.serializeTree (treeBinaryA, leafVectorA);
  octreeB.deserializeTree (treeBinaryA, leafVectorA);

  ASSERT_EQ (octreeA.getLeafCount (), octreeB.getLeafCount ());
  ASSERT_EQ (static_cast<unsigned int> (128), octreeB.getLeafCount ());

  octreeB.serializeTree (treeBinaryB, leafVectorB);

  // test size and leaf count of reconstructed octree
  ASSERT_EQ (leafVectorB.size (), octreeB.getLeafCount ());
  ASSERT_EQ (leafVectorA.size (), leafVectorB.size ());

  for (i = 0; i < leafVectorB.size (); i++)
  {
    ASSERT_EQ (*leafVectorA[i], *leafVectorB[i]);
  }
}

TEST (PCL, Octree2Buf_Base_Double_Buffering_Test)
{

#define TESTPOINTS 3000

  // create octree instances
  Octree2BufBase<int> octreeA;
  Octree2BufBase<int> octreeB;

  std::vector<char> treeBinaryA;
  std::vector<char> treeBinaryB;

  std::vector<int*> leafVectorA;
  std::vector<int*> leafVectorB;

  octreeA.setTreeDepth (5);
  octreeB.setTreeDepth (5);

  struct MyVoxel
  {
    unsigned int x;
    unsigned int y;
    unsigned int z;
  };

  unsigned int i, j, k, runs;
  int data[TESTPOINTS];
  MyVoxel voxels[TESTPOINTS];

  srand (static_cast<unsigned int> (time (NULL)));

  const unsigned int test_runs = 20;

  for (j = 0; j < test_runs; j++)
  {
    octreeA.deleteTree ();
    octreeB.deleteTree ();
    octreeA.setTreeDepth (5);
    octreeB.setTreeDepth (5);

    runs = rand () % 20 + 1;
    for (k = 0; k < runs; k++)
    {
      // switch buffers
      octreeA.switchBuffers ();
      octreeB.switchBuffers ();

      for (i = 0; i < TESTPOINTS; i++)
      {
        data[i] = rand ();

        voxels[i].x = rand () % 4096;
        voxels[i].y = rand () % 4096;
        voxels[i].z = rand () % 4096;

        // add data to octree

        int* container = octreeA.createLeaf(voxels[i].x, voxels[i].y, voxels[i].z);
        *container = data[i];

      }

      // test serialization
      octreeA.serializeTree (treeBinaryA, leafVectorA, true);
      octreeB.deserializeTree (treeBinaryA, leafVectorA, true);
    }

    octreeB.serializeTree (treeBinaryB, leafVectorB, true);

    // check leaf count of rebuilt octree
    ASSERT_EQ (octreeA.getLeafCount (), octreeB.getLeafCount ());
    ASSERT_EQ (leafVectorB.size (), octreeB.getLeafCount ());
    ASSERT_EQ (leafVectorA.size (), leafVectorB.size ());

    // check if octree octree structure is consistent.
    for (i = 0; i < leafVectorB.size (); i++)
    {
      ASSERT_EQ (*leafVectorA[i], *leafVectorB[i]);
    }

  }

}

TEST (PCL, Octree2Buf_Base_Double_Buffering_XOR_Test)
{

#define TESTPOINTS 3000

  // create octree instances
  Octree2BufBase<int> octreeA;
  Octree2BufBase<int> octreeB;

  std::vector<char> treeBinaryA;
  std::vector<char> treeBinaryB;

  std::vector<int*> leafVectorA;
  std::vector<int*> leafVectorB;

  octreeA.setTreeDepth (5);
  octreeB.setTreeDepth (5);

  struct MyVoxel
  {
    unsigned int x;
    unsigned int y;
    unsigned int z;
  };

  unsigned int i, j;
  int data[TESTPOINTS];
  MyVoxel voxels[TESTPOINTS];

  srand (static_cast<unsigned int> (time (NULL)));

  const unsigned int test_runs = 15;

  for (j = 0; j < test_runs; j++)
  {
    for (i = 0; i < TESTPOINTS; i++)
    {
      data[i] = rand ();

      voxels[i].x = rand () % 4096;
      voxels[i].y = rand () % 4096;
      voxels[i].z = rand () % 4096;

      // add data to octree

      int* container = octreeA.createLeaf(voxels[i].x, voxels[i].y, voxels[i].z);
      *container = data[i];
    }

    // test serialization - XOR tree binary data
    octreeA.serializeTree (treeBinaryA, leafVectorA, true);
    octreeB.deserializeTree (treeBinaryA, leafVectorA, true);
    octreeB.serializeTree (treeBinaryB, leafVectorB, true);

    // check leaf count of rebuilt octree
    ASSERT_EQ (octreeA.getLeafCount (), octreeB.getLeafCount ());
    ASSERT_EQ (leafVectorB.size (), octreeB.getLeafCount ());
    ASSERT_EQ (leafVectorA.size (), leafVectorB.size ());
    ASSERT_EQ (treeBinaryA.size (), octreeB.getBranchCount ());
    ASSERT_EQ (treeBinaryA.size (), treeBinaryB.size ());

    // check if octree octree structure is consistent.
    for (i = 0; i < leafVectorB.size (); i++)
    {
      ASSERT_EQ (*leafVectorA[i], *leafVectorB[i]);
    }

    // switch buffers
    octreeA.switchBuffers ();
    octreeB.switchBuffers ();
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
  OctreePointCloudSearch<PointXYZ> octreeB (resolution);
  OctreePointCloudPointVector<PointXYZ> octreeC (resolution);

  // create shared pointcloud instances
  PointCloud<PointXYZ>::Ptr cloudA (new PointCloud<PointXYZ> ());
  PointCloud<PointXYZ>::Ptr cloudB (new PointCloud<PointXYZ> ());

  // assign input point clouds to octree
  octreeA.setInputCloud (cloudA);
  octreeB.setInputCloud (cloudB);
  octreeC.setInputCloud (cloudB);

  for (test = 0; test < test_runs; ++test)
  {

    // clean up
    cloudA->points.clear ();
    octreeA.deleteTree ();

    cloudB->points.clear ();
    octreeB.deleteTree ();

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

    ASSERT_EQ (cloudA->points.size (), octreeA.getLeafCount ());

    // checks for getVoxelDataAtPoint() and isVoxelOccupiedAtPoint() functionality
    for (i = 0; i < cloudA->points.size (); i++)
    {
      ASSERT_TRUE (octreeA.isVoxelOccupiedAtPoint (cloudA->points[i]));
      octreeA.deleteVoxelAtPoint (cloudA->points[i]);
      ASSERT_FALSE (octreeA.isVoxelOccupiedAtPoint (cloudA->points[i]));
    }

    ASSERT_EQ (static_cast<unsigned int> (0), octreeA.getLeafCount ());

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
      ASSERT_LE (depth, octreeB.getTreeDepth ());

      Eigen::Vector3f voxel_min, voxel_max;
      octreeB.getVoxelBounds (b_it, voxel_min, voxel_max);

      ASSERT_GE (voxel_min.x (), minx - 1e-4);
      ASSERT_GE (voxel_min.y (), miny - 1e-4);
      ASSERT_GE (voxel_min.z (), minz - 1e-4);

      ASSERT_LE (voxel_max.x (), maxx + 1e-4);
      ASSERT_LE (voxel_max.y (), maxy + 1e-4);
      ASSERT_LE (voxel_max.z (), maxz + 1e-4);

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
    ASSERT_EQ (branch_count + leaf_count, node_count);
    ASSERT_EQ (octreeB.getLeafCount (), leaf_count);

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

      ASSERT_TRUE (bIdxFound);
    }
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
        ASSERT_EQ (1000u, octreeA.getVoxelDensityAtPoint (PointXYZ(x, y, z)));

  for (float z = 0.05f; z < 5.0f; z += 0.1f)
    for (float y = 0.05f; y < 5.0f; y += 0.1f)
      for (float x = 0.05f; x < 5.0f; x += 0.1f)
        ASSERT_EQ (1u, octreeB.getVoxelDensityAtPoint (PointXYZ (x, y, z)));
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

  ASSERT_EQ (cloudIn->points.size (), indexVector.size());
  ASSERT_EQ (octreeA.getLeafCount (), leafNodeCounter);

  OctreePointCloud<PointXYZ>::Iterator it2;
  OctreePointCloud<PointXYZ>::Iterator it2_end = octreeA.end();

  unsigned int traversCounter = 0;
  for (it2 = octreeA.begin(); it2 != it2_end; ++it2)
  {
    traversCounter++;
  }

  ASSERT_EQ (octreeA.getLeafCount () + octreeA.getBranchCount (), traversCounter);

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
    ASSERT_TRUE (bfIt.getCurrentOctreeDepth () >= lastDepth);
    lastDepth = bfIt.getCurrentOctreeDepth ();

    if (bfIt.isBranchNode ())
    {
      branchNodeCount++;
      // leaf nodes are traversed in the end
      ASSERT_FALSE (leafNodeVisited);
    }

    if (bfIt.isLeafNode ())
    {
      leafNodeCount++;
      leafNodeVisited = true;
    }
  }

  // check if every branch node and every leaf node has been visited
  ASSERT_EQ (octreeA.getLeafCount (), leafNodeCount);
  ASSERT_EQ (octreeA.getBranchCount (), branchNodeCount);
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
      ASSERT_TRUE (octree.isVoxelOccupiedAtPoint (cloudIn->points[i]));
      octree.deleteVoxelAtPoint (cloudIn->points[i]);
      ASSERT_FALSE (octree.isVoxelOccupiedAtPoint (cloudIn->points[i]));
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
  ASSERT_EQ (static_cast<std::size_t> (1000), newPointIdxVector.size ());

  // all point indices found should have an index of >= 1000
  for (i = 0; i < 1000; i++)
  {
    ASSERT_TRUE (newPointIdxVector[i] >= 1000);
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
  ASSERT_EQ (static_cast<std::size_t> (10), voxelCentroids.size());

  // check centroid calculation
  for (i = 0; i < 10; i++)
  {
    EXPECT_NEAR (voxelCentroids[i].x, static_cast<float> (i) + 0.4, 1e-4);
    EXPECT_NEAR (voxelCentroids[i].y, static_cast<float> (i) + 0.4, 1e-4);
    EXPECT_NEAR (voxelCentroids[i].z, static_cast<float> (i) + 0.4, 1e-4);
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
    EXPECT_NEAR (voxelCentroids[i].x, static_cast<float> (i) + 0.4, 1e-4);
    EXPECT_NEAR (voxelCentroids[i].y, static_cast<float> (i) + 0.4, 1e-4);
    EXPECT_NEAR (voxelCentroids[i].z, static_cast<float> (i) + 0.4, 1e-4);
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

    ASSERT_EQ (k_indices_bruteforce.size (), k_indices.size());

    // compare nearest neighbor results of octree with bruteforce search
    i = 0;
    while (k_indices_bruteforce.size ())
    {
      ASSERT_EQ (k_indices_bruteforce.back(), k_indices.back ());
      EXPECT_NEAR (k_sqr_distances_bruteforce.back (), k_sqr_distances.back (), 1e-4);

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

      ASSERT_EQ (idxInResults, inBox);
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
      EXPECT_NEAR (ANNdistance, BFdistance, 1e-4);
      bestMatchCount++;
    }

  }

  // we should have found the absolute nearest neighbor at least once
  ASSERT_TRUE (bestMatchCount > 0);
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

    ASSERT_EQ (cloudSearchBruteforce.size (), cloudNWRRadius.size ());

    // check if result from octree radius search can be also found in bruteforce search
    std::vector<int>::const_iterator current = cloudNWRSearch.begin ();
    while (current != cloudNWRSearch.end ())
    {
      pointDist = sqrt (
          (cloudIn->points[*current].x - searchPoint.x) * (cloudIn->points[*current].x - searchPoint.x)
              + (cloudIn->points[*current].y - searchPoint.y) * (cloudIn->points[*current].y - searchPoint.y)
              + (cloudIn->points[*current].z - searchPoint.z) * (cloudIn->points[*current].z - searchPoint.z));

      ASSERT_TRUE (pointDist <= searchRadius);

      ++current;
    }

    // check if result limitation works
    octree.radiusSearch (searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius, 5);

    ASSERT_TRUE (cloudNWRRadius.size () <= 5);

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
    ASSERT_EQ (cloudIn->points.size (), voxelsInRay.size ());
    // check if all indices of penetrated voxels are in cloud
    ASSERT_EQ (cloudIn->points.size (), indicesInRay.size ());

    octree_search.getIntersectedVoxelCenters (o, dir, voxelsInRay2, 1);
    octree_search.getIntersectedVoxelIndices (o, dir, indicesInRay2, 1);

    // check if only the point from a single voxel has been returned
    ASSERT_EQ (1u, voxelsInRay2.size ());
    ASSERT_EQ (1u, indicesInRay2.size ());

    // check if this point is the closest point to the origin
    pcl::PointXYZ pt = cloudIn->points[ indicesInRay2[0] ];
    Eigen::Vector3f d = Eigen::Vector3f (pt.x, pt.y, pt.z) - o;
    float min_dist = d.norm ();

    for (unsigned int i = 0; i < cloudIn->width * cloudIn->height; i++)
    {
      pt = cloudIn->points[i];
      d = Eigen::Vector3f (pt.x, pt.y, pt.z) - o;
      ASSERT_GE (d.norm (), min_dist);
    }
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



    OctreePointCloudAdjacency<PointXYZ> octree (resolution);
    octree.setInputCloud (cloudIn);
    octree.addPointsFromInputCloud ();

    OctreePointCloudAdjacencyContainer<PointXYZ> *leaf_container;

    leaf_container = octree.getLeafContainerAtPoint (point);
    //Point should have 26 neighbors, plus itself
    ASSERT_EQ (27, leaf_container->size ());

    leaf_container = octree.getLeafContainerAtPoint (point2);

    //Other point should only have itself for neighbor
    ASSERT_EQ (1, leaf_container->size ());
  }
}

TEST (PCL, Octree_Pointcloud_Bounds)
{
    const double SOME_RESOLUTION (10 + 1/3.0);
    const int SOME_DEPTH (4);
    const double DESIRED_MAX = ((1<<SOME_DEPTH) + 0.5)*SOME_RESOLUTION;
    const double DESIRED_MIN = 0;

    OctreePointCloud<PointXYZ> tree (SOME_RESOLUTION);
    tree.defineBoundingBox (DESIRED_MIN, DESIRED_MIN, DESIRED_MIN, DESIRED_MAX, DESIRED_MAX, DESIRED_MAX);

    double min_x, min_y, min_z, max_x, max_y, max_z;
    tree.getBoundingBox (min_x, min_y, min_z, max_x, max_y, max_z);

    ASSERT_GE (max_x, DESIRED_MAX);
    ASSERT_GE (DESIRED_MIN, min_x);

    const double LARGE_MIN = 1e7-45*SOME_RESOLUTION;
    const double LARGE_MAX = 1e7-5*SOME_RESOLUTION;
    tree.defineBoundingBox (LARGE_MIN, LARGE_MIN, LARGE_MIN, LARGE_MAX, LARGE_MAX, LARGE_MAX);
    tree.getBoundingBox (min_x, min_y, min_z, max_x, max_y, max_z);
    const unsigned int depth = tree.getTreeDepth ();
    tree.defineBoundingBox (min_x, min_y, min_z, max_x, max_y, max_z);

    ASSERT_EQ (depth, tree.getTreeDepth ());

    double min_x2, min_y2, min_z2, max_x2, max_y2, max_z2;
    tree.getBoundingBox (min_x2, min_y2, min_z2, max_x2, max_y2, max_z2);

    ASSERT_DOUBLE_EQ (min_x2, min_x);
    ASSERT_DOUBLE_EQ (max_x2, max_x);
}
/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
