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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/random.h>
#include <pcl/common/generate.h>

#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_search.h>

using namespace pcl;
using namespace octree;
using namespace pcl::common;

// Helper struct for priority queue
struct PrioPointQueueEntry
{

  PrioPointQueueEntry ()
  : point_ ()
  , point_distance_ ()
  , point_idx_ ()
  {
  }

  PrioPointQueueEntry (PointXYZ& point_arg, double point_distance_arg, int point_idx_arg)
  : point_ (point_arg)
  , point_distance_ (point_distance_arg)
  , point_idx_ (point_idx_arg)
  {
  }

  bool
  operator< (const PrioPointQueueEntry& rhs_arg) const
  {
    return (this->point_distance_ < rhs_arg.point_distance_);
  }

  PointXYZ point_;
  double point_distance_;
  int point_idx_;

};

TEST (OctreePointCloudSearch, Basic)
{
  const size_t TEST_RUNS = 10;
  srand (static_cast<unsigned int> (time (NULL)));

  float resolution = 0.01f;
  OctreePointCloudSearch<PointXYZ> octree (resolution);

  for (size_t test_id = 0; test_id < TEST_RUNS; test_id++)
  {
    // Generate random point cloud
    UniformGenerator<float>::Parameters x (0.0f, 1024.0f, time (0) + 0);
    UniformGenerator<float>::Parameters y (0.0f, 1024.0f, time (0) + 1);
    UniformGenerator<float>::Parameters z (0.0f, 1024.0f, time (0) + 2);
    CloudGenerator<pcl::PointXYZ, UniformGenerator<float> > generator (x, y, z);
    PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());
    generator.fill (300, 1, *cloud_in);

    // Populate octree
    octree.deleteTree ();
    octree.setInputCloud (cloud_in);
    octree.defineBoundingBox ();
    octree.addPointsFromInputCloud ();

    double minx, miny, minz, maxx, maxy, maxz;
    octree.getBoundingBox (minx, miny, minz, maxx, maxy, maxz);

    unsigned int node_count = 0;
    unsigned int branch_count = 0;
    unsigned int leaf_count = 0;

    // Iterate over the tree checking depth and bounds, and counting nodes,
    // leaves, and branches
    OctreePointCloudPointVector<PointXYZ>::Iterator b_it;
    OctreePointCloudPointVector<PointXYZ>::Iterator b_it_end = octree.end ();
    for (b_it = octree.begin (); b_it != b_it_end; ++b_it)
    {
      // Depth should always be less than tree depth
      ASSERT_GE (octree.getTreeDepth (), b_it.getCurrentOctreeDepth ());

      // Check voxel bounds
      Eigen::Vector3f voxel_min, voxel_max;
      octree.getVoxelBounds (b_it, voxel_min, voxel_max);
      ASSERT_LE (minx - 1e-4, voxel_min.x ());
      ASSERT_LE (miny - 1e-4, voxel_min.y ());
      ASSERT_LE (minz - 1e-4, voxel_min.z ());

      ASSERT_GE (maxx + 1e-4, voxel_max.x ());
      ASSERT_GE (maxy + 1e-4, voxel_max.y ());
      ASSERT_GE (maxz + 1e-4, voxel_max.z ());

      // Update node, branch and leaf count
      const OctreeNode* node = b_it.getCurrentOctreeNode ();
      if (node->getNodeType () == BRANCH_NODE)
        branch_count++;
      else if (node->getNodeType () == LEAF_NODE)
        leaf_count++;
      node_count++;
    }

    // Compare node, branch and leaf count against actual tree values
    ASSERT_EQ (octree.getBranchCount () + octree.getLeafCount (), node_count);
    ASSERT_EQ (octree.getBranchCount (), branch_count);
    ASSERT_EQ (octree.getLeafCount (), leaf_count);

    // Test voxel search
    for (size_t i = 0; i < cloud_in->points.size (); i++)
    {
      std::vector<int> point_idx_vec;
      octree.voxelSearch (cloud_in->points[i], point_idx_vec);
      bool b_idx_found = false;
      std::vector<int>::const_iterator current = point_idx_vec.begin ();
      while (current != point_idx_vec.end ())
      {
        if (*current == static_cast<int> (i))
        {
          b_idx_found = true;
          break;
        }
        ++current;
      }
      EXPECT_TRUE (b_idx_found);
    }
  }
}

TEST (OctreePointCloudSearch, NearestKNeighbourSearch)
{
  const size_t TEST_RUNS = 10;
  srand (static_cast<unsigned int> (time (NULL)));

  OctreePointCloudSearch<PointXYZ> octree (0.1);

  for (size_t test_id = 0; test_id < TEST_RUNS; test_id++)
  {
    // Define a random search point
    PointXYZ search_point (static_cast<float> (10.0 * rand () / RAND_MAX),
                           static_cast<float> (10.0 * rand () / RAND_MAX),
                           static_cast<float> (10.0 * rand () / RAND_MAX));

    // Select random K
    unsigned int K = 1 + rand () % 10;

    // Generate random point cloud
    UniformGenerator<float>::Parameters x (0.0f,  5.0f, time (0) + 0);
    UniformGenerator<float>::Parameters y (0.0f, 10.0f, time (0) + 1);
    UniformGenerator<float>::Parameters z (0.0f, 10.0f, time (0) + 2);
    CloudGenerator<pcl::PointXYZ, UniformGenerator<float> > generator (x, y, z);
    PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());
    generator.fill (1000, 1, *cloud_in);

    // Bruteforce approach: push all points and their distance to the search
    // point into a priority queue
    std::priority_queue<PrioPointQueueEntry,
                        pcl::PointCloud<PrioPointQueueEntry>::VectorType> point_candidates;
    for (size_t i = 0; i < cloud_in->size (); i++)
    {
      double point_dist = (cloud_in->points[i].getVector3fMap () -
                           search_point.getVector3fMap ()).squaredNorm ();
      PrioPointQueueEntry point_entry (cloud_in->points[i], point_dist, static_cast<int> (i));
      point_candidates.push (point_entry);
    }

    // Pop priority queue until we have the nearest K elements
    while (point_candidates.size () > K)
      point_candidates.pop ();

    // Copy results into vectors
    unsigned idx = static_cast<unsigned> (point_candidates.size ());
    std::vector<int> k_indices_bruteforce (idx);
    std::vector<float> k_sqr_distances_bruteforce (idx);
    while (point_candidates.size ())
    {
      --idx;
      k_indices_bruteforce[idx] = point_candidates.top ().point_idx_;
      k_sqr_distances_bruteforce[idx] = static_cast<float> (point_candidates.top ().point_distance_);
      point_candidates.pop ();
    }

    // Octree nearest neighbor search
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    octree.deleteTree ();
    octree.setInputCloud (cloud_in);
    octree.addPointsFromInputCloud ();
    octree.nearestKSearch (search_point, static_cast<int> (K), k_indices, k_sqr_distances);

    ASSERT_EQ (k_indices_bruteforce.size (), k_indices.size ());

    // Compare nearest neighbor results of octree with bruteforce search
    while (k_indices_bruteforce.size ())
    {
      ASSERT_EQ (k_indices_bruteforce.back (), k_indices.back ());
      EXPECT_NEAR (k_sqr_distances_bruteforce.back (), k_sqr_distances.back (), 1e-4);
      k_indices_bruteforce.pop_back ();
      k_indices.pop_back ();
      k_sqr_distances_bruteforce.pop_back ();
      k_sqr_distances.pop_back ();
    }
  }
}

TEST (OctreePointCloudSearch, BoxSearch)
{
  const size_t TEST_RUNS = 30;
  srand (static_cast<unsigned int> (time (NULL)));

  OctreePointCloudSearch<PointXYZ> octree (1);

  for (size_t test_id = 0; test_id < TEST_RUNS; test_id++)
  {
    // Generate random point cloud
    UniformGenerator<float>::Parameters x (0.0f, 10.0f, time (0) + 0);
    UniformGenerator<float>::Parameters y (0.0f, 10.0f, time (0) + 1);
    UniformGenerator<float>::Parameters z (0.0f, 10.0f, time (0) + 2);
    CloudGenerator<pcl::PointXYZ, UniformGenerator<float> > generator (x, y, z);
    PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());
    generator.fill (300, 1, *cloud_in);

    // Define a random search area
    Eigen::Vector3f lower_box_corner (static_cast<float> (4.0 * rand () / RAND_MAX),
                                      static_cast<float> (4.0 * rand () / RAND_MAX),
                                      static_cast<float> (4.0 * rand () / RAND_MAX));
    Eigen::Vector3f upper_box_corner (static_cast<float> (5.0 + 4.0 * rand () / RAND_MAX),
                                      static_cast<float> (5.0 + 4.0 * rand () / RAND_MAX),
                                      static_cast<float> (5.0 + 4.0 * rand () / RAND_MAX));


    // Octree box search
    std::vector<int> k_indices;
    octree.deleteTree ();
    octree.setInputCloud (cloud_in);
    octree.addPointsFromInputCloud ();
    octree.boxSearch (lower_box_corner, upper_box_corner, k_indices);

    // Test every point in point cloud
    for (size_t i = 0; i < cloud_in->size (); i++)
    {
      const PointXYZ& pt = cloud_in->points[i];
      bool in_box = (pt.x > lower_box_corner (0)) && (pt.x < upper_box_corner (0)) &&
                    (pt.y > lower_box_corner (1)) && (pt.y < upper_box_corner (1)) &&
                    (pt.z > lower_box_corner (2)) && (pt.z < upper_box_corner (2));
      bool idx_in_results = false;
      for (size_t j = 0; (j < k_indices.size ()) && (!idx_in_results); ++j)
        if (i == static_cast<unsigned int> (k_indices[j]))
          idx_in_results = true;
      ASSERT_EQ (in_box, idx_in_results);
    }
  }
}

TEST (OctreePointCloudSearch, ApproxNearestNeighbourSearch)
{
  const size_t TEST_RUNS = 100;
  srand (static_cast<unsigned int> (time (NULL)));

  double voxel_resolution = 0.1;
  OctreePointCloudSearch<PointXYZ> octree (voxel_resolution);

  unsigned int best_match_count = 0;

  for (size_t test_id = 0; test_id < TEST_RUNS; test_id++)
  {
    // Define a random search point
    PointXYZ search_point (static_cast<float> (10.0 * rand () / RAND_MAX),
                           static_cast<float> (10.0 * rand () / RAND_MAX),
                           static_cast<float> (10.0 * rand () / RAND_MAX));

    // Generate random point cloud
    UniformGenerator<float>::Parameters x (0.0f,  5.0f, time (0) + 0);
    UniformGenerator<float>::Parameters y (0.0f, 10.0f, time (0) + 1);
    UniformGenerator<float>::Parameters z (0.0f, 10.0f, time (0) + 2);
    CloudGenerator<pcl::PointXYZ, UniformGenerator<float> > generator (x, y, z);
    PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());
    generator.fill (1000, 1, *cloud_in);

    // Bruteforce search
    double bf_distance = std::numeric_limits<double>::max ();
    int bf_index = 0;
    for (size_t i = 0; i < cloud_in->size (); i++)
    {
      double point_dist = (cloud_in->points[i].getVector3fMap () -
                           search_point.getVector3fMap ()).squaredNorm ();
      if (point_dist < bf_distance)
      {
        bf_index = static_cast<int> (i);
        bf_distance = point_dist;
      }
    }

    // Octree approximate nearest neighbor search
    int ann_index;
    float ann_distance;
    octree.deleteTree ();
    octree.setInputCloud (cloud_in);
    octree.addPointsFromInputCloud ();
    octree.approxNearestSearch (search_point, ann_index, ann_distance);
    if (bf_index == ann_index)
    {
      EXPECT_NEAR (ann_distance, bf_distance, 1e-4);
      best_match_count++;
    }
  }

  // We should have found the absolute nearest neighbor at least once
  ASSERT_LT (0, best_match_count);
}

TEST (OctreePointCloudSearch, NeighboursWithinRadiusSearch)
{
  const size_t TEST_RUNS = 100;
  srand (static_cast<unsigned int> (time (NULL)));

  for (size_t test_id = 0; test_id < TEST_RUNS; test_id++)
  {
    // Define a random search point
    PointXYZ search_point (static_cast<float> (10.0 * rand () / RAND_MAX),
                           static_cast<float> (10.0 * rand () / RAND_MAX),
                           static_cast<float> (10.0 * rand () / RAND_MAX));

    // Generate random point cloud
    UniformGenerator<float>::Parameters x (0.0f, 10.0f, time (0) + 0);
    UniformGenerator<float>::Parameters y (0.0f, 10.0f, time (0) + 1);
    UniformGenerator<float>::Parameters z (0.0f,  5.0f, time (0) + 2);
    CloudGenerator<pcl::PointXYZ, UniformGenerator<float> > generator (x, y, z);
    PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());
    generator.fill (1000, 1, *cloud_in);

    // Select random search radius
    double search_radius = static_cast<float> (5.0 * rand () / RAND_MAX);
    double search_radius_sqr = search_radius * search_radius;

    // Build octree
    OctreePointCloudSearch<PointXYZ> octree (0.001);
    octree.setInputCloud (cloud_in);
    octree.addPointsFromInputCloud ();

    // Bruteforce radius search
    std::set<int> cloud_search_bruteforce;
    for (size_t i = 0; i < cloud_in->size (); i++)
    {
      double point_dist = (cloud_in->points[i].getVector3fMap () -
                           search_point.getVector3fMap ()).squaredNorm ();
      if (point_dist <= search_radius_sqr)
        cloud_search_bruteforce.insert (static_cast<int> (i));
    }

    // Octree radius search
    std::vector<int> nwr_index;
    std::vector<float> nwr_radius;
    octree.radiusSearch (search_point, search_radius, nwr_index, nwr_radius);

    ASSERT_EQ (cloud_search_bruteforce.size (), nwr_radius.size ());

    // Check that output distances are correct and that they are less then
    // the search radius
    for (size_t i = 0; i < nwr_index.size (); ++i)
    {
      double point_dist = (cloud_in->points[nwr_index[i]].getVector3fMap () -
                           search_point.getVector3fMap ()).squaredNorm ();
      ASSERT_GE (search_radius_sqr, point_dist);
      ASSERT_GE (search_radius_sqr, nwr_radius[i]);
      ASSERT_FLOAT_EQ (point_dist, nwr_radius[i]);
    }

    // Check if result from octree radius search can be also found in bruteforce search
    for (size_t i = 0; i < nwr_index.size (); ++i)
      EXPECT_TRUE (cloud_search_bruteforce.count (nwr_index[i]) != 0);

    // Check if result limitation works
    octree.radiusSearch (search_point, search_radius, nwr_index, nwr_radius, 5);
    ASSERT_GE (5, nwr_radius.size ());
  }
}

TEST (OctreePointCloudSearch, RayTraversal)
{
  const size_t TEST_RUNS = 10;
  srand (static_cast<unsigned int> (time (NULL)));

  OctreePointCloudSearch<PointXYZ> octree (0.02f);

  for (size_t test_id = 0; test_id < TEST_RUNS; test_id++)
  {
    // Delete octree
    octree.deleteTree ();
    // Define octree bounding box 10x10x10
    octree.defineBoundingBox (0.0, 0.0, 0.0, 10.0, 10.0, 10.0);

    PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> (4, 1));

    Eigen::Vector3f p (static_cast<float> (10.0 * rand () / RAND_MAX),
                       static_cast<float> (10.0 * rand () / RAND_MAX),
                       static_cast<float> (10.0 * rand () / RAND_MAX));
    cloud_in->points[0].getVector3fMap () = p;

    // Origin
    Eigen::Vector3f o (static_cast<float> (12.0 * rand () / RAND_MAX),
                       static_cast<float> (12.0 * rand () / RAND_MAX),
                       static_cast<float> (12.0 * rand () / RAND_MAX));

    // Direction vector
    Eigen::Vector3f dir = p - o;

    float tmin = 1.0;
    for (unsigned int j = 1; j < 4; j++)
    {
      tmin = tmin - 0.25f;
      cloud_in->points[j].getVector3fMap () = o + (tmin * dir);
    }

    // Insert cloud point into octree
    octree.setInputCloud (cloud_in);
    octree.addPointsFromInputCloud ();

    pcl::PointCloud<pcl::PointXYZ>::VectorType voxels_in_ray;
    std::vector<int> indices_in_ray;
    octree.getIntersectedVoxelCenters (o, dir, voxels_in_ray);
    octree.getIntersectedVoxelIndices (o, dir, indices_in_ray);

    // Check if all voxels in the cloud are penetraded by the ray
    ASSERT_EQ (cloud_in->size (), voxels_in_ray.size ());
    // Check if all indices of penetrated voxels are in cloud
    ASSERT_EQ (cloud_in->size (), indices_in_ray.size ());

    pcl::PointCloud<pcl::PointXYZ>::VectorType voxels_in_ray2;
    std::vector<int> indices_in_ray2;
    octree.getIntersectedVoxelCenters (o, dir, voxels_in_ray2, 1);
    octree.getIntersectedVoxelIndices (o, dir, indices_in_ray2, 1);

    // Check if only the point from a single voxel has been returned
    ASSERT_EQ (1, voxels_in_ray2.size ());
    ASSERT_EQ (1, indices_in_ray2.size ());

    // Check if this point is the closest point to the origin
    float min_dist = (cloud_in->points[indices_in_ray2[0]].getVector3fMap () - o).norm ();
    for (size_t i = 0; i < cloud_in->size (); i++)
      EXPECT_LE (min_dist, (cloud_in->points[i].getVector3fMap () - o).norm ());
  }
}

int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}

