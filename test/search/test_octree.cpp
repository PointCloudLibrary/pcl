/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
#include <pcl/test/gtest.h>
#include <vector>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/octree.h> // for pcl::search::Octree

using namespace pcl;
using namespace octree;

// helper class for priority queue
class prioPointQueueEntry
{
public:
  prioPointQueueEntry ()
  = default;
  prioPointQueueEntry (PointXYZ& point_arg, double pointDistance_arg, int pointIdx_arg)
  {
    point_ = point_arg;
    pointDistance_ = pointDistance_arg;
    pointIdx_ = pointIdx_arg;
  }

  bool
  operator< (const prioPointQueueEntry& rhs_arg) const
  {
    return (this->pointDistance_ < rhs_arg.pointDistance_);
  }

  PointXYZ point_;
  double pointDistance_;int pointIdx_;
};

TEST (PCL, Octree_Pointcloud_Nearest_K_Neighbour_Search)
{
  constexpr unsigned int test_runs = 1;

  // instantiate point cloud
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  const unsigned int seed = time (nullptr);
  srand (seed);
  SCOPED_TRACE("seed=" + std::to_string(seed));

  std::priority_queue<prioPointQueueEntry, pcl::PointCloud<prioPointQueueEntry>::VectorType> pointCandidates;

  // create octree
  pcl::search::Octree<PointXYZ> octree(0.1);

  pcl::Indices k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  for (unsigned int test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point
    PointXYZ searchPoint (static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))), 
                          static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))),
                          static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))));

    const unsigned int K = 1 + rand () % 10;

    // generate point cloud
    cloudIn->width = 1000;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);
    for (std::size_t i = 0; i < 1000; i++)
    {
      (*cloudIn)[i] = PointXYZ (static_cast<float> (5.0  * (rand () / static_cast<double> (RAND_MAX))),
                                     static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))),
                                     static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))));
    }

    k_indices.clear ();
    k_sqr_distances.clear ();

    k_indices_bruteforce.clear ();
    k_sqr_distances_bruteforce.clear ();

    // push all points and their distance to the search point into a priority queue - bruteforce approach.
    for (std::size_t i = 0; i < cloudIn->size (); i++)
    {
      double pointDist = (((*cloudIn)[i].x - searchPoint.x) * ((*cloudIn)[i].x - searchPoint.x) +
                          ((*cloudIn)[i].y - searchPoint.y) * ((*cloudIn)[i].y - searchPoint.y) +
                          ((*cloudIn)[i].z - searchPoint.z) * ((*cloudIn)[i].z - searchPoint.z));

      prioPointQueueEntry pointEntry ((*cloudIn)[i], pointDist, static_cast<int> (i));

      pointCandidates.push (pointEntry);
    }

    // pop priority queue until we have the nearest K elements
    while (pointCandidates.size () > K)
      pointCandidates.pop ();

    // copy results into vectors
    auto idx = static_cast<unsigned> (pointCandidates.size ());
    k_indices_bruteforce.resize (idx);
    k_sqr_distances_bruteforce.resize (idx);
    while (!pointCandidates.empty ())
    {
      --idx;
      k_indices_bruteforce [idx] = pointCandidates.top ().pointIdx_;
      k_sqr_distances_bruteforce [idx] = static_cast<float> (pointCandidates.top ().pointDistance_);

      pointCandidates.pop ();
    }
    // octree nearest neighbor search
    octree.setInputCloud (cloudIn);
    octree.nearestKSearch (searchPoint, static_cast<int> (K), k_indices, k_sqr_distances);

    ASSERT_EQ ( k_indices.size() , k_indices_bruteforce.size() );

    // compare nearest neighbor results of octree with bruteforce search
    while (!k_indices_bruteforce.empty ())
    {
      ASSERT_EQ ( k_indices.back() , k_indices_bruteforce.back() );
      EXPECT_NEAR (k_sqr_distances.back(), k_sqr_distances_bruteforce.back(), 1e-4);

      k_indices_bruteforce.pop_back();
      k_indices.pop_back();

      k_sqr_distances_bruteforce.pop_back();
      k_sqr_distances.pop_back();
    }
  }
}

#if 0
TEST (PCL, Octree_Pointcloud_Approx_Nearest_Neighbour_Search)
{
  constexpr unsigned int test_runs = 100;
  unsigned int bestMatchCount = 0;

  // instantiate point cloud
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  const unsigned int seed = time (nullptr);
  srand (seed);
  SCOPED_TRACE("seed=" + std::to_string(seed));

  double voxelResolution = 0.1;

  // create octree
  pcl::search::Search<PointXYZ>* octree = new pcl::search::Octree<PointXYZ> (voxelResolution);

  for (unsigned int test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point
    PointXYZ searchPoint (10.0 * ((double)rand () / (double)RAND_MAX), 10.0 * ((double)rand () / (double)RAND_MAX),
                          10.0 * ((double)rand () / (double)RAND_MAX));

    // generate point cloud
    cloudIn->width = 1000;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);
    for (std::size_t i = 0; i < 1000; i++)
      (*cloudIn)[i] = PointXYZ (5.0 * ((double)rand () / (double)RAND_MAX),
                                     10.0 * ((double)rand () / (double)RAND_MAX),
                                     10.0 * ((double)rand () / (double)RAND_MAX));
    // brute force search
    double pointDist;
    double BFdistance = std::numeric_limits<double>::max ();
    int BFindex = 0;

    for (std::size_t i = 0; i < cloudIn->size (); i++)
    {
      pointDist = (((*cloudIn)[i].x - searchPoint.x) * ((*cloudIn)[i].x - searchPoint.x)
          + ((*cloudIn)[i].y - searchPoint.y) * ((*cloudIn)[i].y - searchPoint.y) + ((*cloudIn)[i].z
          - searchPoint.z) * ((*cloudIn)[i].z - searchPoint.z));

      if (pointDist < BFdistance)
      {
        BFindex = i;
        BFdistance = pointDist;
      }
    }

    int ANNindex;
    float ANNdistance;

    octree->setInputCloud (cloudIn);
    octree->approxNearestSearch (searchPoint, ANNindex, ANNdistance);

    if (BFindex == ANNindex)
    {
      EXPECT_NEAR (ANNdistance, BFdistance, 1e-4);
      bestMatchCount++;
    }
  }
  // we should have found the absolute nearest neighbor at least once
  //ASSERT_GT (bestMatchCount, 0);
}
#endif
#if 0
TEST (PCL, Octree_RadiusSearch_GPU)
{
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());
  // generate point cloud data
  cloudIn->width = 1000;
  cloudIn->height = 1;
  cloudIn->points.resize (cloudIn->width * cloudIn->height);

  for (int i = 0; i < 1000; i++)
  {
    (*cloudIn)[i] = PointXYZ (10.0 * ((double)rand () / (double)RAND_MAX),
        10.0 * ((double)rand () / (double)RAND_MAX),
        5.0 * ((double)rand () / (double)RAND_MAX));
  }

  Search<PointXYZ>* octree = new pcl::octree::OctreeWrapper<PointXYZ>(0.1f);
  octree->setInputCloud(cloudIn);

  std::vector <PointXYZ > point;
  const PointXYZ searchPoint (10.0 * ((double)rand () / (double)RAND_MAX), 10.0 * ((double)rand () / (double)RAND_MAX),
      10.0 * ((double)rand () / (double)RAND_MAX));
  point.push_back(searchPoint);
  point.push_back(searchPoint);
  point.push_back(searchPoint);
  double searchRadius = 5.0 * ((double)rand () / (double)RAND_MAX);
  double radius =5;
  std::vector < double > radiuses;
  radiuses.push_back(radius);
  radiuses.push_back(radius);
  radiuses.push_back(radius);
  std::vector<pcl::Indices > k_indices;
  std::vector<std::vector<float> > k_distances;
  int max_nn = -1;

  octree->radiusSearch (point, radiuses, k_indices,k_distances,max_nn );
}

#endif
TEST (PCL, Octree_Pointcloud_Neighbours_Within_Radius_Search)
{
  constexpr unsigned int test_runs = 100;

  // instantiate point clouds
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());
  PointCloud<PointXYZ>::Ptr cloudOut (new PointCloud<PointXYZ> ());

  const unsigned int seed = time (nullptr);
  srand (seed);
  SCOPED_TRACE("seed=" + std::to_string(seed));

  for (unsigned int test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point
    PointXYZ searchPoint (static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))), 
                          static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))),
                          static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))));

    cloudIn->width = 1000;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);

    // generate point cloud data
    for (std::size_t i = 0; i < 1000; i++)
    {
      (*cloudIn)[i] = PointXYZ (static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))),
                                     static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))),
                                     static_cast<float> (5.0 *  (rand () / static_cast<double> (RAND_MAX))));
    }

    pcl::search::Search<PointXYZ>* octree = new pcl::search::Octree<PointXYZ> (0.001);

    // build octree
    double pointDist;
    double searchRadius = 5.0 * rand () / static_cast<double> (RAND_MAX);
    
    // bruteforce radius search
    std::vector<int> cloudSearchBruteforce;
    for (std::size_t i = 0; i < cloudIn->size (); i++)
    {
      pointDist = std::sqrt (
                        ((*cloudIn)[i].x - searchPoint.x) * ((*cloudIn)[i].x - searchPoint.x)
                            + ((*cloudIn)[i].y - searchPoint.y) * ((*cloudIn)[i].y - searchPoint.y)
                            + ((*cloudIn)[i].z - searchPoint.z) * ((*cloudIn)[i].z - searchPoint.z));

      if (pointDist <= searchRadius)
      {
        // add point candidates to vector list
        cloudSearchBruteforce.push_back (static_cast<int> (i));
      }
    }

    pcl::Indices cloudNWRSearch;
    std::vector<float> cloudNWRRadius;

    // execute octree radius search
    octree->setInputCloud (cloudIn);
    octree->radiusSearch (searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius);

    ASSERT_EQ ( cloudNWRRadius.size() , cloudSearchBruteforce.size());

    // check if result from octree radius search can be also found in bruteforce search
    for (const auto& current : cloudNWRSearch)
    {
      pointDist = std::sqrt (
          ((*cloudIn)[current].x-searchPoint.x) * ((*cloudIn)[current].x-searchPoint.x) +
          ((*cloudIn)[current].y-searchPoint.y) * ((*cloudIn)[current].y-searchPoint.y) +
          ((*cloudIn)[current].z-searchPoint.z) * ((*cloudIn)[current].z-searchPoint.z)
      );

      ASSERT_LE (pointDist, searchRadius);
    }

    // check if result limitation works
    octree->radiusSearch(searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius, 5);
    ASSERT_LE (cloudNWRRadius.size(), 5);
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
