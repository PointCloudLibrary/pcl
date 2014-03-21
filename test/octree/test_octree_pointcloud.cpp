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

#include <pcl/pcl_tests.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/random.h>
#include <pcl/common/generate.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/octree/octree_pointcloud_dynamic_depth.h>

using namespace pcl;
using namespace octree;
using namespace pcl::common;

TEST (OctreePointCloud, Iterator)
{
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());

  for (float z = 0.05f; z < 7.0f; z += 0.1f)
    for (float y = 0.05f; y < 7.0f; y += 0.1f)
      for (float x = 0.05f; x < 7.0f; x += 0.1f)
        cloud->push_back (PointXYZ (x, y, z));

  cloud->width = static_cast<uint32_t> (cloud->points.size ());
  cloud->height = 1;

  OctreePointCloud<PointXYZ> octree (1.0f);

  // Add point data to octree
  octree.setInputCloud (cloud);
  octree.addPointsFromInputCloud ();

  OctreePointCloud<PointXYZ>::LeafNodeIterator it1;
  OctreePointCloud<PointXYZ>::LeafNodeIterator it1_end = octree.leaf_end ();

  std::vector<int> index_vector;
  unsigned int leaf_node_counter = 0;

  for (it1 = octree.leaf_begin (); it1 != it1_end; ++it1)
  {
    it1.getLeafContainer ().getPointIndices (index_vector);
    leaf_node_counter++;
  }

  ASSERT_EQ (cloud->points.size (), index_vector.size ());
  ASSERT_EQ (leaf_node_counter, octree.getLeafCount ());

  OctreePointCloud<PointXYZ>::Iterator it2;
  OctreePointCloud<PointXYZ>::Iterator it2_end = octree.end();

  unsigned int travers_counter = 0;
  for (it2 = octree.begin (); it2 != it2_end; ++it2)
    travers_counter++;

  ASSERT_EQ (octree.getLeafCount () + octree.getBranchCount (), travers_counter);

}

TEST (OctreePointCloud, SinglePoint)
{
  const size_t TEST_RUNS = 100;
  const size_t POINT_COUNT = 300;
  srand (static_cast<unsigned int> (time (NULL)));

  OctreePointCloudSinglePoint<PointXYZ> octree (0.01f);

  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
  octree.setInputCloud (cloud);

  for (size_t test_id = 0; test_id < TEST_RUNS; test_id++)
  {
    cloud->points.clear ();
    octree.deleteTree ();

    for (size_t point = 0; point < POINT_COUNT; point++)
    {
      // Gereate a random point
      PointXYZ new_point (static_cast<float> (1024.0 * rand () / RAND_MAX),
                          static_cast<float> (1024.0 * rand () / RAND_MAX),
                          static_cast<float> (1024.0 * rand () / RAND_MAX));
      // Add point only if voxel at point location doesn't exist
      if (!octree.isVoxelOccupiedAtPoint (new_point))
        octree.addPointToCloud (new_point, cloud);
    }

    ASSERT_EQ (octree.getLeafCount (), cloud->points.size ());

    // Checks for getVoxelDataAtPoint() and isVoxelOccupiedAtPoint() functionality
    for (size_t i = 0; i < cloud->points.size (); i++)
    {
      ASSERT_TRUE (octree.isVoxelOccupiedAtPoint (cloud->points[i]));
      octree.deleteVoxelAtPoint (cloud->points[i]);
      ASSERT_FALSE (octree.isVoxelOccupiedAtPoint (cloud->points[i]));
    }

    ASSERT_EQ (0, octree.getLeafCount ());
  }
}

TEST (OctreePointCloud, Density)
{
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());

  for (float z = 0.05f; z < 7.0f; z += 0.1f)
    for (float y = 0.05f; y < 7.0f; y += 0.1f)
      for (float x = 0.05f; x < 7.0f; x += 0.1f)
        cloud->push_back (PointXYZ (x, y, z));

  cloud->width = static_cast<uint32_t> (cloud->points.size ());
  cloud->height = 1;

  OctreePointCloudDensity<PointXYZ> octree_a (1.00000f); // low resolution
  OctreePointCloudDensity<PointXYZ> octree_b (0.00001f); // high resolution

  octree_a.defineBoundingBox (7.0, 7.0, 7.0);
  octree_b.defineBoundingBox (7.0, 7.0, 7.0);

  // Add point data to octree
  octree_a.setInputCloud (cloud);
  octree_b.setInputCloud (cloud);
  octree_a.addPointsFromInputCloud ();
  octree_b.addPointsFromInputCloud ();

  // Check density information
  for (float z = 1.5f; z < 3.5f; z += 1.0f)
    for (float y = 1.5f; y < 3.5f; y += 1.0f)
      for (float x = 1.5f; x < 3.5f; x += 1.0f)
        ASSERT_EQ (1000, octree_a.getVoxelDensityAtPoint (PointXYZ (x, y, z)));

  for (float z = 0.05f; z < 5.0f; z += 0.1f)
    for (float y = 0.05f; y < 5.0f; y += 0.1f)
      for (float x = 0.05f; x < 5.0f; x += 0.1f)
        ASSERT_EQ (1, octree_b.getVoxelDensityAtPoint (PointXYZ (x, y, z)));
}

TEST (OctreePointCloud, Occupancy)
{
  const size_t TEST_RUNS = 50;
  srand (static_cast<unsigned int> (time (NULL)));

  OctreePointCloudOccupancy<PointXYZ> octree (0.00001f);

  for (size_t test_id = 0; test_id < TEST_RUNS; test_id++)
  {
    // Generate random point cloud
    UniformGenerator<float>::Parameters x (0.0f,  5.0f, time (0) + 0);
    UniformGenerator<float>::Parameters y (0.0f, 10.0f, time (0) + 1);
    UniformGenerator<float>::Parameters z (0.0f, 10.0f, time (0) + 2);
    CloudGenerator<pcl::PointXYZ, UniformGenerator<float> > generator (x, y, z);
    PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());
    generator.fill (1000, 1, *cloud_in);

    // Create octree
    octree.setInputCloud (cloud_in);
    octree.addPointsFromInputCloud ();

    // Check occupancy of voxels
    for (size_t i = 0; i < 1000; i++)
    {
      ASSERT_TRUE (octree.isVoxelOccupiedAtPoint (cloud_in->points[i]));
      octree.deleteVoxelAtPoint (cloud_in->points[i]);
      ASSERT_FALSE (octree.isVoxelOccupiedAtPoint (cloud_in->points[i]));
    }
  }
}

TEST (OctreePointCloud, VoxelCentroid)
{
  srand (static_cast<unsigned int> (time (NULL)));

  OctreePointCloudVoxelCentroid<PointXYZ> octree (1.0f);
  octree.defineBoundingBox (10.0, 10.0, 10.0);

  // Generate point data for point cloud
  PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> (10 * 3, 1));
  for (size_t i = 0; i < 10; i++)
  {
    // These three points should always be assigned to the same voxel in the octree
    cloud_in->points[i * 3 + 0] = PointXYZ (0.2f + i, 0.2f + i, 0.2f + i);
    cloud_in->points[i * 3 + 1] = PointXYZ (0.4f + i, 0.4f + i, 0.4f + i);
    cloud_in->points[i * 3 + 2] = PointXYZ (0.6f + i, 0.6f + i, 0.6f + i);
  }

  // Add points from cloud to octree
  octree.setInputCloud (cloud_in);
  octree.addPointsFromInputCloud ();

  pcl::PointCloud<PointXYZ>::VectorType voxel_centroids;
  octree.getVoxelCentroids (voxel_centroids);

  // We expect 10 voxel centroids
  ASSERT_EQ (10, voxel_centroids.size ());

  // Check centroid calculation
  for (size_t i = 0; i < 10; i++)
    EXPECT_XYZ_NEAR (PointXYZ (0.4f + i, 0.4f + i, 0.4f + i), voxel_centroids[i], 1e-4);

  // Generate new point data
  for (size_t i = 0; i < 10; i++)
  {
    // These three points should always be assigned to the same voxel in the octree
    cloud_in->points[i * 3 + 0] = PointXYZ (0.1f + i, 0.1f + i, 0.1f + i);
    cloud_in->points[i * 3 + 1] = PointXYZ (0.4f + i, 0.4f + i, 0.4f + i);
    cloud_in->points[i * 3 + 2] = PointXYZ (0.7f + i, 0.7f + i, 0.7f + i);
  }

  // Add points from new cloud to octree
  octree.setInputCloud (cloud_in);
  octree.addPointsFromInputCloud ();

  voxel_centroids.clear();
  octree.getVoxelCentroids (voxel_centroids);

  // Check centroid calculation
  for (size_t i = 0; i < 10; i++)
    EXPECT_XYZ_NEAR (PointXYZ (0.4f + i, 0.4f + i, 0.4f + i), voxel_centroids[i], 1e-4);
}

TEST (OctreePointCloud, ChangeDetector)
{
  srand (static_cast<unsigned int> (time (NULL)));

  // Generate random point cloud
  UniformGenerator<float>::Parameters x (0.0f,  5.0f, time (0) + 0);
  UniformGenerator<float>::Parameters y (0.0f, 10.0f, time (0) + 1);
  UniformGenerator<float>::Parameters z (0.0f, 10.0f, time (0) + 2);
  CloudGenerator<pcl::PointXYZ, UniformGenerator<float> > generator (x, y, z);
  PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());
  generator.fill (1000, 1, *cloud_in);

  // Add points from cloud to octree
  OctreePointCloudChangeDetector<PointXYZ> octree (0.01f);
  octree.setInputCloud (cloud_in);
  octree.addPointsFromInputCloud ();

  // Switch buffers - reset tree
  octree.switchBuffers ();

  // Add points from cloud to new octree buffer
  octree.addPointsFromInputCloud ();

  // Add 1000 additional points
  for (size_t i = 0; i < 1000; i++)
  {
    PointXYZ new_point (static_cast<float> (100.0 +  5.0 * rand () / RAND_MAX),
                        static_cast<float> (100.0 + 10.0 * rand () / RAND_MAX),
                        static_cast<float> (100.0 + 10.0 * rand () / RAND_MAX));
    octree.addPointToCloud (new_point, cloud_in);
  }

  // Get a vector of new points, which did not exist in previous buffer
  std::vector<int> new_point_idx_vector;
  octree.getPointIndicesFromNewVoxels (new_point_idx_vector);

  // Should be 1000
  ASSERT_EQ (1000, new_point_idx_vector.size ());

  // All point indices found should have an index of >= 1000
  for (size_t i = 0; i < 1000; i++)
    ASSERT_LE (1000, new_point_idx_vector[i]);
}

TEST (OctreePointCloud, Adjacency)
{
  const size_t TEST_RUNS = 100;
  srand (static_cast<unsigned int> (time (NULL)));

  for (size_t test_id = 0; test_id < TEST_RUNS; test_id++)
  {
    // Instantiate point cloud
    PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());

    // Select random resolution
    float resolution = static_cast<float> (0.01 * rand () / RAND_MAX) + 0.00001f;

    // This is done to define the grid
    cloud_in->push_back (PointXYZ ( 10, 10, 10));
    cloud_in->push_back (PointXYZ (-10,-10,-10));

    // Define a random point
    PointXYZ point (static_cast<float> (0.5 * rand () / RAND_MAX),
                    static_cast<float> (0.5 * rand () / RAND_MAX),
                    static_cast<float> (0.5 * rand () / RAND_MAX));
    cloud_in->push_back (point);

    // Generate neighbors
    for (int dx = -1; dx <= 1; ++dx)
    {
      for (int dy = -1; dy <= 1; ++dy)
      {
        for (int dz = -1; dz <= 1; ++dz)
        {
          float factor = 1.0f * resolution;
          PointXYZ neighbor (point.x + factor * dx, point.y + factor * dy, point.z + factor * dz);
          cloud_in->push_back (neighbor);
        }
      }
    }

    // Add another point that isn't touching previous or neighbors
    PointXYZ point2 (static_cast<float> (point.x + 10 * resolution),
                     static_cast<float> (point.y + 10 * resolution),
                     static_cast<float> (point.z + 10 * resolution));
    cloud_in->push_back (point2);

    // Add points which are not neighbors
    for (int i = 0; i < 100; ++i)
    {
      PointXYZ not_neighbor_point (static_cast<float> (10.0 * rand () / RAND_MAX),
                                   static_cast<float> (10.0 * rand () / RAND_MAX),
                                   static_cast<float> (10.0 * rand () / RAND_MAX));
      if ((point2.getVector3fMap () - not_neighbor_point.getVector3fMap ()).norm () > 3 * resolution)
        cloud_in->push_back (not_neighbor_point);
    }

    octree::OctreePointCloudAdjacency<PointXYZ> octree (resolution);
    octree.setInputCloud (cloud_in);
    octree.addPointsFromInputCloud ();

    // Point should have 26 neighbors, plus itself
    ASSERT_EQ (27, octree.getLeafContainerAtPoint (point)->getNumNeighbors ());

    // Other point should only have itself for neighbor
    ASSERT_EQ (1, octree.getLeafContainerAtPoint (point2)->getNumNeighbors ());
  }
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

// TODO: move elsewhere together with CentroidPoint
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
