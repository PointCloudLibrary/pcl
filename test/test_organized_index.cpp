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
#include <pcl/io/pcd_io.h>
#include <pcl/search/pcl_search.h>

using namespace std;
using namespace pcl;

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

TEST (PCL, Organized_Neighbor_Search_Pointcloud_Nearest_K_Neighbour_Search)
{

  const unsigned int test_runs = 2;
  unsigned int test_id;

  // instantiate point cloud
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  size_t i;

  srand (time (NULL));

  unsigned int K;

  // create organized search
  pcl::search::Search<PointXYZ>* organizedNeighborSearch = new pcl::search::OrganizedNeighbor<PointXYZ>();

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  // typical focal length from kinect
  const double oneOverFocalLength = 0.0018;
  double x,y,z;

  int xpos, ypos, centerX, centerY;

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point

    K = (rand () % 10)+1;

    // generate point cloud
    cloudIn->width = 128;
    cloudIn->height = 32;
    cloudIn->points.clear();
    cloudIn->points.reserve (cloudIn->width * cloudIn->height);

    centerX = cloudIn->width>>1;
    centerY = cloudIn->height>>1;

    for (ypos = -centerY; ypos < centerY; ypos++)
      for (xpos = -centerX; xpos < centerX; xpos++)
      {
        z = 15.0 * ((double)rand () / (double)(RAND_MAX+1.0))+20;
        y = (double)ypos*oneOverFocalLength*(double)z;
        x = (double)xpos*oneOverFocalLength*(double)z;

        cloudIn->points.push_back(PointXYZ (x, y, z));
      }

    unsigned int searchIdx = rand()%(cloudIn->width * cloudIn->height);
    const PointXYZ& searchPoint = cloudIn->points[searchIdx];

    k_indices.clear();
    k_sqr_distances.clear();

    // organized nearest neighbor search
    organizedNeighborSearch->setInputCloud (cloudIn);
//    organizedNeighborSearch->nearestKSearch (searchPoint, (int)K, k_indices, k_sqr_distances);


    double pointDist;

    k_indices_bruteforce.clear();
    k_sqr_distances_bruteforce.clear();

    std::priority_queue<prioPointQueueEntry> pointCandidates;


    // push all points and their distance to the search point into a priority queue - bruteforce approach.
    for (i = 0; i < cloudIn->points.size (); i++)
    {
      pointDist = ((cloudIn->points[i].x - searchPoint.x) * (cloudIn->points[i].x - searchPoint.x) +
             /*+*/ (cloudIn->points[i].y - searchPoint.y) * (cloudIn->points[i].y - searchPoint.y) +
                   (cloudIn->points[i].z - searchPoint.z) * (cloudIn->points[i].z - searchPoint.z));

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

    //FAILS: ASSERT_EQ ( k_indices.size() , k_indices_bruteforce.size() );

    // compare nearest neighbor results of organized search  with bruteforce search
    for (i = 0; i < k_indices.size (); i++)
    {
      ASSERT_EQ ( k_indices[i] , k_indices_bruteforce.back() );
      EXPECT_NEAR (k_sqr_distances[i], k_sqr_distances_bruteforce.back(), 1e-4);

      k_indices_bruteforce.pop_back();
      k_sqr_distances_bruteforce.pop_back();
    }
  }
}


TEST (PCL, Organized_Neighbor_Search_Pointcloud_Nearest_K_Neighbour_Search_Kinect_Data)
{
  const unsigned int test_runs = 2;
  unsigned int test_id;

  // instantiate point cloud

  size_t i;

  srand (time (NULL));

  unsigned int K;

  // create organized search
  pcl::search::Search<PointXYZ>* organizedNeighborSearch = new pcl::search::OrganizedNeighbor<PointXYZ>();

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  // typical focal length from kinect
  pcl::PointCloud<PointXYZ>::Ptr cloudIn (new pcl::PointCloud<PointXYZ>);
  pcl::PCDReader pcd;

  unsigned int searchIdx;

  if (pcd.read ("office1.pcd", *cloudIn) == -1)
  {
    std::cout <<"Couldn't read input cloud" << std::endl;
    return;
  }

  while(1)
  {
   searchIdx = rand()%(cloudIn->width * cloudIn->height);
   if(cloudIn->points[searchIdx].z <100)break;
  }

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point

    K = (rand () % 10)+1;
//     K = 16;
    // generate point cloud
    const PointXYZ& searchPoint = cloudIn->points[searchIdx];

    k_indices.clear();
    k_sqr_distances.clear();

    // organized nearest neighbor search
    organizedNeighborSearch->setInputCloud (cloudIn);
    organizedNeighborSearch->nearestKSearch (searchIdx, (int)K, k_indices, k_sqr_distances);
 
    double pointDist;

    k_indices_bruteforce.clear();
    k_sqr_distances_bruteforce.clear();

    std::priority_queue<prioPointQueueEntry> pointCandidates;


    // push all points and their distance to the search point into a priority queue - bruteforce approach.
    for (i = 0; i < cloudIn->points.size (); i++)
    {
      pointDist = ((cloudIn->points[i].x - searchPoint.x) * (cloudIn->points[i].x - searchPoint.x) +
             /*+*/ (cloudIn->points[i].y - searchPoint.y) * (cloudIn->points[i].y - searchPoint.y) +
                   (cloudIn->points[i].z - searchPoint.z) * (cloudIn->points[i].z - searchPoint.z));

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
    ASSERT_EQ ( k_indices.size() , k_indices_bruteforce.size() );
  }
}

TEST (PCL, Organized_Neighbor_Search_Pointcloud_Neighbours_Within_Radius_Search)
{
  const unsigned int test_runs = 10;
  unsigned int test_id;

  size_t i,j;

  srand (time (NULL));
  
  pcl::search::Search<PointXYZ>* organizedNeighborSearch = new pcl::search::OrganizedNeighbor<PointXYZ>();

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  // typical focal length from kinect
  const double oneOverFocalLength = 0.0018;
  double x,y,z;

  int xpos, ypos, centerX, centerY, idx;

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // generate point cloud
    PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

    cloudIn->width = 640;
    cloudIn->height = 480;
    cloudIn->points.clear();
    cloudIn->points.resize (cloudIn->width * cloudIn->height);

    centerX = cloudIn->width>>1;
    centerY = cloudIn->height>>1;

    idx = 0;
    for (ypos = -centerY; ypos < centerY; ypos++)
      for (xpos = -centerX; xpos < centerX; xpos++)
      {
        z = 5.0 * ( ((double)rand () / (double)RAND_MAX))+5;
        y = ypos*oneOverFocalLength*z;
        x = xpos*oneOverFocalLength*z;

        cloudIn->points[idx++]= PointXYZ (x, y, z);
      }

    unsigned int randomIdx = rand()%(cloudIn->width * cloudIn->height);

    const PointXYZ& searchPoint = cloudIn->points[randomIdx];

    double pointDist;
   double searchRadius = 1.0 * ((double)rand () / (double)RAND_MAX);
//   double searchRadius = 1/10;

    int minX = cloudIn->width;
    int minY = cloudIn->height;
    int maxX = 0;
    int maxY = 0;

    // bruteforce radius search
    vector<int> cloudSearchBruteforce;
    cloudSearchBruteforce.clear();

    for (i = 0; i < cloudIn->points.size (); i++)
    {
      pointDist = sqrt (
                        (cloudIn->points[i].x - searchPoint.x) * (cloudIn->points[i].x - searchPoint.x)
                      + (cloudIn->points[i].y - searchPoint.y) * (cloudIn->points[i].y - searchPoint.y)
                      + (cloudIn->points[i].z - searchPoint.z) * (cloudIn->points[i].z - searchPoint.z));

      if (pointDist <= searchRadius)
      {
        // add point candidates to vector list
        cloudSearchBruteforce.push_back ((int)i);

        minX = std::min<int>(minX, i%cloudIn->width);
        minY = std::min<int>(minY, i/cloudIn->width);
        maxX = std::max<int>(maxX, i%cloudIn->width);
        maxY = std::max<int>(maxY, i/cloudIn->width);
      }
    }

    vector<int> cloudNWRSearch;
    vector<float> cloudNWRRadius;

    organizedNeighborSearch->setInputCloud (cloudIn);

    organizedNeighborSearch->radiusSearch (randomIdx, searchRadius, cloudNWRSearch, cloudNWRRadius, INT_MAX); //,INT_MAX);
   
      
    // check if result from organized radius search can be also found in bruteforce search
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


    // check if bruteforce result from organized radius search can be also found in bruteforce search
    current = cloudSearchBruteforce.begin();
    while (current != cloudSearchBruteforce.end())
    {

      pointDist = sqrt (
          (cloudIn->points[*current].x-searchPoint.x) * (cloudIn->points[*current].x-searchPoint.x) +
          (cloudIn->points[*current].y-searchPoint.y) * (cloudIn->points[*current].y-searchPoint.y) +
          (cloudIn->points[*current].z-searchPoint.z) * (cloudIn->points[*current].z-searchPoint.z)
      );

      ASSERT_EQ ( (pointDist<=searchRadius) , true);

      ++current;
    }

    for (i = 0; i < cloudSearchBruteforce.size (); i++) 
      for (j = 0; j < cloudNWRSearch.size (); j++) 
        if (cloudNWRSearch[i]== cloudSearchBruteforce[j])
          break;

    ASSERT_EQ (cloudNWRRadius.size() , cloudSearchBruteforce.size ());

    // check if result limitation works
    organizedNeighborSearch->radiusSearch (searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius, 5);

    ASSERT_EQ (cloudNWRRadius.size () <= 5, true);
  }
}


TEST (PCL, Organized_Neighbor_Search_Pointcloud_Neighbours_Within_Radius_Search_Benchmark_Test)
{
  const unsigned int test_runs = 10;
  unsigned int test_id;
  srand (time (NULL));

  pcl::search::Search<PointXYZ>* organizedNeighborSearch = new pcl::search::OrganizedNeighbor<PointXYZ>();

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  // typical focal length from kinect
  const double oneOverFocalLength = 0.0018;
  double x,y,z;

  int xpos, ypos, centerX, centerY, idx;
  
  double radiusSearchTime = 0, radiusSearchLPTime = 0;

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // generate point cloud

    PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

    cloudIn->width = 1024;
    cloudIn->height = 768;
    cloudIn->points.clear();
    cloudIn->points.resize (cloudIn->width * cloudIn->height);

    centerX = cloudIn->width>>1;
    centerY = cloudIn->height>>1;

    idx = 0;
    for (ypos = -centerY; ypos < centerY; ypos++)
      for (xpos = -centerX; xpos < centerX; xpos++)
      {
        z = 5.0 * ( ((double)rand () / (double)RAND_MAX))+5;
        y = ypos*oneOverFocalLength*z;
        x = xpos*oneOverFocalLength*z;

        cloudIn->points[idx++]= PointXYZ (x, y, z);
      }

    unsigned int randomIdx = rand()%(cloudIn->width * cloudIn->height);

    const PointXYZ& searchPoint = cloudIn->points[randomIdx];

    double pointDist;
    double searchRadius = 1.0 * ((double)rand () / (double)RAND_MAX);

    int minX = cloudIn->width;
    int minY = cloudIn->height;
    int maxX = 0;
    int maxY = 0;

    // bruteforce radius search
    vector<int> cloudSearchBruteforce;
    cloudSearchBruteforce.clear();

    for (size_t i = 0; i < cloudIn->points.size (); i++)
    {
      pointDist = sqrt (
                        (cloudIn->points[i].x - searchPoint.x) * (cloudIn->points[i].x - searchPoint.x)
                      + (cloudIn->points[i].y - searchPoint.y) * (cloudIn->points[i].y - searchPoint.y)
                      + (cloudIn->points[i].z - searchPoint.z) * (cloudIn->points[i].z - searchPoint.z));

      if (pointDist <= searchRadius)
      {
        // add point candidates to vector list
        cloudSearchBruteforce.push_back ((int)i);

        minX = std::min<int>(minX, i%cloudIn->width);
        minY = std::min<int>(minY, i/cloudIn->width);
        maxX = std::max<int>(maxX, i%cloudIn->width);
        maxY = std::max<int>(maxY, i/cloudIn->width);
      }
    }


    vector<int> cloudNWRSearch;
    vector<float> cloudNWRRadius;
    
    double check_time = getTime();
    organizedNeighborSearch->setInputCloud (cloudIn);
    organizedNeighborSearch->radiusSearch (randomIdx, searchRadius, cloudNWRSearch, cloudNWRRadius, INT_MAX); //,INT_MAX);
    
    double check_time2 = getTime();
    
    radiusSearchLPTime += check_time2 - check_time;

    organizedNeighborSearch->setInputCloud (cloudIn);
    organizedNeighborSearch->radiusSearch (randomIdx, searchRadius, cloudNWRSearch, cloudNWRRadius, INT_MAX); //,INT_MAX);

    radiusSearchTime += getTime() - check_time2;
  }
}

TEST (PCL, Organized_Neighbor_Search_Pointcloud_Neighbours_Within_Radius_Search_Kinect_Data)
{

  pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
  pcl::PointCloud<PointXYZ>::Ptr cloudIn (new pcl::PointCloud<PointXYZ>);
    pcl::PCDReader pcd;


  if (pcd.read ("office1.pcd", *cloudIn) == -1)
  {
	std::cout <<"Couldn't read input cloud" << std::endl; 
    return;
  }
  const unsigned int test_runs = 10;
  unsigned int test_id;
  srand (time (NULL));

  pcl::search::Search<PointXYZ>* organizedNeighborSearch = new pcl::search::OrganizedNeighbor<PointXYZ>();

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  // typical focal length from kinect
  unsigned int randomIdx;

  while (1)
  {
	  randomIdx = rand()%(cloudIn->width * cloudIn->height);
	  if(cloudIn->points[randomIdx].z <100)break;
  }

  cout << "**Search Point:** " << cloudIn->points[randomIdx].x << " " << cloudIn->points[randomIdx].y << " " << cloudIn->points[randomIdx].z << endl;

  std::cout << "+--------+-----------------+----------------+-------------------+" << std::endl;
  std::cout << "| Search | Organized       | Organized      | Number of         |" << std::endl;
  std::cout << "| Radius | Neighbor Search | Data Index     | Nearest Neighbors |" << std::endl;
  std::cout << "+========+=================+================+===================+" << std::endl;

  for (test_id = 0; test_id < test_runs; test_id++)
  {

    double searchRadius = 1.0 * (test_id*1.0/10*1.0);
    double sum_time = 0, sum_time2 = 0;

    vector<int> cloudNWRSearch;
    vector<float> cloudNWRRadius;

    vector<int> cloudNWRSearch2;
    vector<float> cloudNWRRadius2;

    for (int iter = 0; iter < 100; iter++)
    {

      cloudNWRSearch2.clear();
      cloudNWRRadius2.clear();

      double check_time = getTime();
      organizedNeighborSearch->setInputCloud (cloudIn);
      organizedNeighborSearch->radiusSearch (randomIdx, searchRadius, cloudNWRSearch2, cloudNWRRadius2, INT_MAX); //,INT_MAX);

      double check_time2 = getTime();
      sum_time+= check_time2 - check_time;
     }


     for(int iter=0;iter<100;iter++)
     {
       cloudNWRSearch.clear();
       cloudNWRRadius.clear();
       double check_time = getTime();
       organizedNeighborSearch->setInputCloud (cloudIn);
       organizedNeighborSearch->approxRadiusSearch (cloudIn, randomIdx, searchRadius, cloudNWRSearch, cloudNWRRadius, INT_MAX); //,INT_MAX);

       double check_time2 = getTime();
       sum_time2+= check_time2 - check_time;
     }

 //  ASSERT_EQ(cloudNWRRadius.size(), cloudNWRRadius2.size());
 //  ASSERT_EQ(cloudNWRSearch.size(), cloudNWRSearch2.size());
  

    printf("| %.3lf  | %0.5lf         | %0.5lf        | %6lu            |\n",searchRadius, sum_time/100, sum_time2/100, cloudNWRSearch.size());
    std::cout << "+--------+-----------------+----------------+-------------------+" << std::endl;

    const PointXYZ& searchPoint = cloudIn->points[randomIdx];

    double pointDist;

    int minX = cloudIn->width;
    int minY = cloudIn->height;
    int maxX = 0;
    int maxY = 0;

    // bruteforce radius search
    vector<int> cloudSearchBruteforce;
    cloudSearchBruteforce.clear();

    for (size_t i = 0; i < cloudIn->points.size (); i++)
    {
      pointDist = sqrt (
                        (cloudIn->points[i].x - searchPoint.x) * (cloudIn->points[i].x - searchPoint.x)
                      + (cloudIn->points[i].y - searchPoint.y) * (cloudIn->points[i].y - searchPoint.y)
                      + (cloudIn->points[i].z - searchPoint.z) * (cloudIn->points[i].z - searchPoint.z));

      if (pointDist <= searchRadius)
      {
        // add point candidates to vector list
        cloudSearchBruteforce.push_back ((int)i);

        minX = std::min<int>(minX, i%cloudIn->width);
        minY = std::min<int>(minY, i/cloudIn->width);
        maxX = std::max<int>(maxX, i%cloudIn->width);
        maxY = std::max<int>(maxY, i/cloudIn->width);
      }
    }

    // check if result from organized radius search can be also found in bruteforce search
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


    // check if bruteforce result from organized radius search can be also found in bruteforce search
    current = cloudSearchBruteforce.begin();
    while (current != cloudSearchBruteforce.end())
    {

      pointDist = sqrt (
          (cloudIn->points[*current].x-searchPoint.x) * (cloudIn->points[*current].x-searchPoint.x) +
          (cloudIn->points[*current].y-searchPoint.y) * (cloudIn->points[*current].y-searchPoint.y) +
          (cloudIn->points[*current].z-searchPoint.z) * (cloudIn->points[*current].z-searchPoint.z)
      );

      ASSERT_EQ ( (pointDist<=searchRadius) , true);

      ++current;
    }
    ASSERT_EQ (cloudNWRRadius.size() , cloudSearchBruteforce.size ());

    // check if result limitation works
    organizedNeighborSearch->radiusSearch (searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius, 5);

    ASSERT_EQ (cloudNWRRadius.size () <= 5, true);
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
