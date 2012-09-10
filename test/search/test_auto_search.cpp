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
 * $Id: test_feature.cpp 2177 2011-08-23 13:58:35Z shapovalov $
 *
 */
#include <iostream>
#include <gtest/gtest.h>
#include <pcl/common/time.h>
#include <pcl/search/pcl_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;

PointCloud<PointXYZ> cloud, cloud_big;

void
init ()
{
  float resolution = 0.1;
  for (float z = -0.5f; z <= 0.5f; z += resolution)
    for (float y = -0.5f; y <= 0.5f; y += resolution)
      for (float x = -0.5f; x <= 0.5f; x += resolution)
        cloud.points.push_back (PointXYZ (x, y, z));
  cloud.width  = cloud.points.size ();
  cloud.height = 1;

  cloud_big.width  = 640;
  cloud_big.height = 480;
  srand (time (NULL));
  // Randomly create a new point cloud
  for (size_t i = 0; i < cloud_big.width * cloud_big.height; ++i)
    cloud_big.points.push_back (PointXYZ (1024 * rand () / (RAND_MAX + 1.0),
                                         1024 * rand () / (RAND_MAX + 1.0),
                                         1024 * rand () / (RAND_MAX + 1.0)));
}

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
  srand (time (NULL));
  unsigned int K;

  std::priority_queue<prioPointQueueEntry, std::vector<prioPointQueueEntry, Eigen::aligned_allocator<prioPointQueueEntry> > > pointCandidates;

  // create octree
  pcl::search::Search<PointXYZ>* octree = new  pcl::search::AutotunedSearch<PointXYZ>(pcl::search::OCTREE);

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point

    PointXYZ searchPoint (10.0 * ((double)rand () / (double)RAND_MAX), 10.0 * ((double)rand () / (double)RAND_MAX),
                          10.0 * ((double)rand () / (double)RAND_MAX));

    K = rand () % 10;

    // generate point cloud
    cloudIn->width = 1000;
    cloudIn->height = 1;
    cloudIn->points.resize (cloudIn->width * cloudIn->height);
    for (i = 0; i < 1000; i++)
    {
      cloudIn->points[i] = PointXYZ (5.0 * ((double)rand () / (double)RAND_MAX),
                                     10.0 * ((double)rand () / (double)RAND_MAX),
                                     10.0 * ((double)rand () / (double)RAND_MAX));
    }


    double pointDist;

    k_indices.clear();
    k_sqr_distances.clear();

    k_indices_bruteforce.clear();
    k_sqr_distances_bruteforce.clear();

    // push all points and their distance to the search point into a priority queue - bruteforce approach.
    for (i = 0; i < cloudIn->points.size (); i++)
    {
      pointDist = ((cloudIn->points[i].x - searchPoint.x) * (cloudIn->points[i].x - searchPoint.x)
          + (cloudIn->points[i].y - searchPoint.y) * (cloudIn->points[i].y - searchPoint.y) + (cloudIn->points[i].z
          - searchPoint.z) * (cloudIn->points[i].z - searchPoint.z));

      prioPointQueueEntry pointEntry (cloudIn->points[i], pointDist, i);

      pointCandidates.push (pointEntry);
    }

    // pop priority queue until we have the nearest K elements
    while (pointCandidates.size () > K)
      pointCandidates.pop ();

    // copy results into vectors
    while (pointCandidates.size ())
    {
      k_indices_bruteforce.push_back (pointCandidates.top ().pointIdx_);
      k_sqr_distances_bruteforce.push_back (pointCandidates.top ().pointDistance_);

      pointCandidates.pop ();
    }

    // octree nearest neighbor search
  octree->setInputCloud (cloudIn);
    octree->nearestKSearch (searchPoint, (int)K, k_indices, k_sqr_distances);

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

TEST (PCL, Octree_Pointcloud_Approx_Nearest_Neighbour_Search)
{
  const unsigned int test_runs = 100;
  unsigned int test_id;

  unsigned int bestMatchCount = 0;

  // instantiate point cloud
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  size_t i;
  srand (time (NULL));

  // create octree
  pcl::search::Search<PointXYZ>* octree = new pcl::search::AutotunedSearch<PointXYZ>(pcl::search::OCTREE);


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
    {
      cloudIn->points[i] = PointXYZ (5.0 * ((double)rand () / (double)RAND_MAX),
                                     10.0 * ((double)rand () / (double)RAND_MAX),
                                     10.0 * ((double)rand () / (double)RAND_MAX));
    }


    // brute force search
    double pointDist;
    double BFdistance = numeric_limits<double>::max ();
    int BFindex = 0;

    for (i = 0; i < cloudIn->points.size (); i++)
    {
      pointDist = ((cloudIn->points[i].x - searchPoint.x) * (cloudIn->points[i].x - searchPoint.x)
          + (cloudIn->points[i].y - searchPoint.y) * (cloudIn->points[i].y - searchPoint.y) + (cloudIn->points[i].z
          - searchPoint.z) * (cloudIn->points[i].z - searchPoint.z));

      if (pointDist<BFdistance)
      {
        BFindex = i;
        BFdistance = pointDist;
      }

    }

    int ANNindex;
    float ANNdistance;

    // octree approx. nearest neighbor search
  octree->setInputCloud (cloudIn);
    octree->approxNearestSearch (searchPoint,  ANNindex, ANNdistance);

    if (BFindex==ANNindex)
    {
      EXPECT_NEAR (ANNdistance, BFdistance, 1e-4);
      bestMatchCount++;
    }

  }

  // we should have found the absolute nearest neighbor at least once
  ASSERT_EQ ( (bestMatchCount > 0) , true);
}


TEST (PCL, KdTreeWrapper_nearestKSearch)
{

  pcl::search::Search<PointXYZ>* kdtree = new pcl::search::AutotunedSearch<PointXYZ>(pcl::search::KDTREE);
  kdtree->setInputCloud (cloud.makeShared ());
  PointXYZ test_point (0.01f, 0.01f, 0.01f);
  unsigned int no_of_neighbors = 20;
  multimap<float, int> sorted_brute_force_result;
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    float distance = euclideanDistance (cloud.points[i], test_point);
    sorted_brute_force_result.insert (make_pair (distance, (int) i));
  }
  float max_dist = 0.0f;
  unsigned int counter = 0;
  for (multimap<float, int>::iterator it = sorted_brute_force_result.begin (); it != sorted_brute_force_result.end () && counter < no_of_neighbors; ++it)
  {
    max_dist = max (max_dist, it->first);
    ++counter;
  }

  vector<int> k_indices;
  k_indices.resize (no_of_neighbors);
  vector<float> k_distances;
  k_distances.resize (no_of_neighbors);

  kdtree->nearestKSearch (test_point, no_of_neighbors, k_indices, k_distances);

  EXPECT_EQ (k_indices.size (), no_of_neighbors);

  // Check if all found neighbors have distance smaller than max_dist
  for (size_t i = 0; i < k_indices.size (); ++i)
  {
    const PointXYZ& point = cloud.points[k_indices[i]];
    bool ok = euclideanDistance (test_point, point) <= max_dist;
    if (!ok)
      ok = (fabs (euclideanDistance (test_point, point)) - max_dist) <= 1e-6;
    //if (!ok)  cerr << k_indices[i] << " is not correct...\n";
    //else      cerr << k_indices[i] << " is correct...\n";
    EXPECT_EQ (ok, true);
  }

  ScopeTime scopeTime ("FLANN nearestKSearch");
  {
    pcl::search::Search<PointXYZ>* kdtree = new pcl::search::AutotunedSearch<PointXYZ>(pcl::search::KDTREE);
    //kdtree->initSearchDS ();
    kdtree->setInputCloud (cloud_big.makeShared ());
    for (size_t i = 0; i < cloud_big.points.size (); ++i)
      kdtree->nearestKSearch (cloud_big.points[i], no_of_neighbors, k_indices, k_distances);
  }

}	


/* Function to auto evaluate the best search structure for the given dataset */
TEST (PCL, AutoTunedSearch_Evaluate)
{
  pcl::search::Search<PointXYZ>* search = new pcl::search::AutotunedSearch<PointXYZ>(pcl::search::AUTO_TUNED);

  pcl::PCDReader pcd;
  pcl::PointCloud<PointXYZ>::Ptr cloudIn (new pcl::PointCloud<PointXYZ>);

  if (pcd.read ("office1.pcd", *cloudIn) == -1)
  {
    std::cout <<"Couldn't read input cloud" << std::endl;
    return;
  }

 search->evaluateSearchMethods (cloudIn, pcl::search::NEAREST_K_SEARCH);
 search->evaluateSearchMethods (cloudIn, pcl::search::NEAREST_RADIUS_SEARCH);
}



int 
main(int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  init ();

  // Testing using explicit instantiation of inherited class
  pcl::search::Search<PointXYZ>* kdtree = new pcl::search::AutotunedSearch<PointXYZ>(pcl::search::KDTREE);
  kdtree->setInputCloud (cloud.makeShared ());

  return (RUN_ALL_TESTS ());
};
/* ]--- */
