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
#include <gtest/gtest.h>
#include <vector>
#include <stdio.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/pcl_search.h>

using namespace std;
using namespace pcl;
using namespace octree;

// helper class for priority queue
class prioPointQueueEntry
{
public:
  prioPointQueueEntry ()
  {
  }
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
  const unsigned int test_runs = 1;
  unsigned int test_id;

  // instantiate point cloud
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  size_t i;
  srand (static_cast<unsigned int> (time (NULL)));
  unsigned int K;

  std::priority_queue<prioPointQueueEntry, pcl::PointCloud<prioPointQueueEntry>::VectorType> pointCandidates;

  // create octree
  pcl::search::Search<PointXYZ>* octree = new pcl::search::Octree<PointXYZ> (0.1);

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point
    PointXYZ searchPoint (static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))), 
                          static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))),
                          static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))));

    K = 1 + rand () % 10;

    // generate point cloud
    cloudIn->width = 1000;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);
    for (i = 0; i < 1000; i++)
    {
      cloudIn->points[i] = PointXYZ (static_cast<float> (5.0  * (rand () / static_cast<double> (RAND_MAX))),
                                     static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))),
                                     static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))));
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
          + (cloudIn->points[i].y - searchPoint.y) * (cloudIn->points[i].y - searchPoint.y) + (cloudIn->points[i].z
          - searchPoint.z) * (cloudIn->points[i].z - searchPoint.z));

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
    octree->setInputCloud (cloudIn);
    octree->nearestKSearch (searchPoint, static_cast<int> (K), k_indices, k_sqr_distances);

    ASSERT_EQ ( k_indices.size() , k_indices_bruteforce.size() );

    // compare nearest neighbor results of octree with bruteforce search
    i = 0;
    while (k_indices_bruteforce.size ())
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
  const unsigned int test_runs = 100;
  unsigned int test_id;
  unsigned int bestMatchCount = 0;

  // instantiate point cloud
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  size_t i;
  srand (time (NULL));

  double voxelResolution = 0.1;

  // create octree
  pcl::search::Search<PointXYZ>* octree = new pcl::search::Octree<PointXYZ> (voxelResolution);

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point
    PointXYZ searchPoint (10.0 * ((double)rand () / (double)RAND_MAX), 10.0 * ((double)rand () / (double)RAND_MAX),
                          10.0 * ((double)rand () / (double)RAND_MAX));

    // generate point cloud
    cloudIn->width = 1000;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);
    for (i = 0; i < 1000; i++)
      cloudIn->points[i] = PointXYZ (5.0 * ((double)rand () / (double)RAND_MAX),
                                     10.0 * ((double)rand () / (double)RAND_MAX),
                                     10.0 * ((double)rand () / (double)RAND_MAX));
    // brute force search
    double pointDist;
    double BFdistance = numeric_limits<double>::max ();
    int BFindex = 0;

    for (i = 0; i < cloudIn->points.size (); i++)
    {
      pointDist = ((cloudIn->points[i].x - searchPoint.x) * (cloudIn->points[i].x - searchPoint.x)
          + (cloudIn->points[i].y - searchPoint.y) * (cloudIn->points[i].y - searchPoint.y) + (cloudIn->points[i].z
          - searchPoint.z) * (cloudIn->points[i].z - searchPoint.z));

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
  //ASSERT_EQ ( (bestMatchCount > 0) , true);
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

  int i=0;
  for (i = 0; i < 1000; i++)
  {
    cloudIn->points[i] = PointXYZ (10.0 * ((double)rand () / (double)RAND_MAX),
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
  vector < double > radiuses;
  radiuses.push_back(radius);
  radiuses.push_back(radius);
  radiuses.push_back(radius);
  std::vector<std::vector<int> > k_indices;
  std::vector<std::vector<float> > k_distances;
  int max_nn = -1;

  octree->radiusSearch (point, radiuses, k_indices,k_distances,max_nn );
}

#endif
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
    PointXYZ searchPoint (static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))), 
                          static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))),
                          static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))));

    cloudIn->width = 1000;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);

    // generate point cloud data
    for (i = 0; i < 1000; i++)
    {
      cloudIn->points[i] = PointXYZ (static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))),
                                     static_cast<float> (10.0 * (rand () / static_cast<double> (RAND_MAX))),
                                     static_cast<float> (5.0 *  (rand () / static_cast<double> (RAND_MAX))));
    }

    pcl::search::Search<PointXYZ>* octree = new pcl::search::Octree<PointXYZ> (0.001);

    // build octree
    double pointDist;
    double searchRadius = 5.0 * rand () / static_cast<double> (RAND_MAX);
    
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
    octree->setInputCloud (cloudIn);
    octree->radiusSearch (searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius);

    ASSERT_EQ ( cloudNWRRadius.size() , cloudSearchBruteforce.size());

    // check if result from octree radius search can be also found in bruteforce search
    std::vector<int>::const_iterator current = cloudNWRSearch.begin();
    while (current != cloudNWRSearch.end())
    {
      pointDist = sqrt (
          (cloudIn->points[*current].x-searchPoint.x) * (cloudIn->points[*current].x-searchPoint.x) +
          (cloudIn->points[*current].y-searchPoint.y) * (cloudIn->points[*current].y-searchPoint.y) +
          (cloudIn->points[*current].z-searchPoint.z) * (cloudIn->points[*current].z-searchPoint.z)
      );

      ASSERT_EQ ( (pointDist<=searchRadius) , true);

      ++current;
    }

    // check if result limitation works
    octree->radiusSearch(searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius, 5);
    ASSERT_EQ ( cloudNWRRadius.size() <= 5, true);
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
