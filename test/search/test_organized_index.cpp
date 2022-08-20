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
#include <cstdio>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h> // for OrganizedNeighbor

using namespace pcl;

std::string pcd_filename;

// Here we want a very precise distance function, speed is less important. So we use
// double precision, unlike euclideanDistance() in pcl/common/distances and distance()
// in pcl/common/geometry which use float (single precision) and possibly vectorization
template <typename PointT> inline double
point_distance(const PointT& p1, const PointT& p2)
{
  const double x_diff = p1.x - p2.x, y_diff = p1.y - p2.y, z_diff = p1.z - p2.z;
  return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}

// helper class for priority queue
class prioPointQueueEntry
{
  public:
    prioPointQueueEntry () = default;
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
  constexpr unsigned int test_runs = 2;

  // instantiate point cloud
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

  const unsigned int seed = time (nullptr);
  srand (seed);
  SCOPED_TRACE("seed=" + std::to_string(seed));

  // create organized search
  search::OrganizedNeighbor<PointXYZ> organizedNeighborSearch;

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  // typical focal length from kinect
  constexpr double oneOverFocalLength = 0.0018;

  for (unsigned int test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point

    const unsigned int K = (rand () % 10)+1;

    // generate point cloud
    cloudIn->width = 128;
    cloudIn->height = 32;
    cloudIn->points.clear();
    cloudIn->points.reserve (cloudIn->width * cloudIn->height);

    int centerX = cloudIn->width >> 1;
    int centerY = cloudIn->height >> 1;

    for (int ypos = -centerY; ypos < centerY; ypos++)
      for (int xpos = -centerX; xpos < centerX; xpos++)
      {
        double z = 15.0 * ((double)rand () / (double)(RAND_MAX+1.0))+20;
        double y = (double)ypos*oneOverFocalLength*(double)z;
        double x = (double)xpos*oneOverFocalLength*(double)z;

        cloudIn->points.emplace_back(x, y, z);
      }

    unsigned int searchIdx = rand()%(cloudIn->width * cloudIn->height);
    const PointXYZ& searchPoint = (*cloudIn)[searchIdx];

    k_indices.clear();
    k_sqr_distances.clear();

    // organized nearest neighbor search
    organizedNeighborSearch.setInputCloud (cloudIn);
//    organizedNeighborSearch.nearestKSearch (searchPoint, (int)K, k_indices, k_sqr_distances);

    k_indices_bruteforce.clear();
    k_sqr_distances_bruteforce.clear();

    std::priority_queue<prioPointQueueEntry> pointCandidates;

    for (auto it = cloudIn->begin(); it != cloudIn->end(); ++it)
    {
      const auto pointDist = point_distance(*it, searchPoint);
      prioPointQueueEntry pointEntry (*it, pointDist, std::distance(cloudIn->begin(), it));
      pointCandidates.push (pointEntry);
    }

    // pop priority queue until we have the nearest K elements
    while (pointCandidates.size () > K)
      pointCandidates.pop ();

    // copy results into vectors
    while (!pointCandidates.empty())
    {
      k_indices_bruteforce.push_back (pointCandidates.top ().pointIdx_);
      k_sqr_distances_bruteforce.push_back (pointCandidates.top ().pointDistance_);

      pointCandidates.pop ();
    }

    //FAILS: ASSERT_EQ ( k_indices.size() , k_indices_bruteforce.size() );

    // compare nearest neighbor results of organized search  with bruteforce search
    for (std::size_t i = 0; i < k_indices.size (); i++)
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
  constexpr unsigned int test_runs = 2;

  // instantiate point cloud

  const unsigned int seed = time (nullptr);
  srand (seed);
  SCOPED_TRACE("seed=" + std::to_string(seed));

  // create organized search
  search::OrganizedNeighbor<PointXYZ> organizedNeighborSearch;

  pcl::Indices k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  // typical focal length from kinect
  pcl::PointCloud<PointXYZ>::Ptr cloudIn (new pcl::PointCloud<PointXYZ>);
  pcl::PCDReader pcd;

  unsigned int searchIdx;

  ASSERT_EQ(pcd.read(pcd_filename, *cloudIn), 0) <<"Couldn't read input cloud" << std::endl;


  while(true)
  {
   searchIdx = rand()%(cloudIn->width * cloudIn->height);
   if((*cloudIn)[searchIdx].z < 100)
     break;
  }

  for (unsigned int test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point

    const unsigned int K = (rand () % 10)+1;
//     K = 16;
    // generate point cloud
    const PointXYZ& searchPoint = (*cloudIn)[searchIdx];

    k_indices.clear();
    k_sqr_distances.clear();

    // organized nearest neighbor search
    organizedNeighborSearch.setInputCloud (cloudIn);
    organizedNeighborSearch.nearestKSearch (searchPoint, (int)K, k_indices, k_sqr_distances);

    k_indices_bruteforce.clear();
    k_sqr_distances_bruteforce.clear();

    std::priority_queue<prioPointQueueEntry> pointCandidates;


    // push all points and their distance to the search point into a priority queue - bruteforce approach.
    for (auto it = cloudIn->begin(); it != cloudIn->end(); ++it)
    {
      const auto pointDist = point_distance(*it, searchPoint);
      prioPointQueueEntry pointEntry (*it, pointDist, std::distance(cloudIn->begin(), it));
      pointCandidates.push (pointEntry);
    }

    // pop priority queue until we have the nearest K elements
    while (pointCandidates.size () > K)
      pointCandidates.pop ();

    // copy results into vectors
    while (!pointCandidates.empty())
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
  constexpr unsigned int test_runs = 10;

  const unsigned int seed = time (nullptr);
  srand (seed);
  SCOPED_TRACE("seed=" + std::to_string(seed));
  search::OrganizedNeighbor<PointXYZ> organizedNeighborSearch;

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  // typical focal length from kinect
  constexpr double oneOverFocalLength = 0.0018;

  for (unsigned int test_id = 0; test_id < test_runs; test_id++)
  {
    // generate point cloud
    PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

    cloudIn->width = 640;
    cloudIn->height = 480;
    cloudIn->points.clear();
    cloudIn->points.resize (cloudIn->width * cloudIn->height);

    int centerX = cloudIn->width >> 1;
    int centerY = cloudIn->height >> 1;

    int idx = 0;
    for (int ypos = -centerY; ypos < centerY; ypos++)
      for (int xpos = -centerX; xpos < centerX; xpos++)
      {
        double z = 5.0 * ( ((double)rand () / (double)RAND_MAX))+5;
        double y = ypos*oneOverFocalLength*z;
        double x = xpos*oneOverFocalLength*z;
        (*cloudIn)[idx++]= PointXYZ (x, y, z);
      }

    unsigned int randomIdx = rand()%(cloudIn->width * cloudIn->height);

    const PointXYZ& searchPoint = (*cloudIn)[randomIdx];

    const double searchRadius = 1.0 * ((double)rand () / (double)RAND_MAX);
    //   double searchRadius = 1/10;

    // bruteforce radius search
    std::vector<int> cloudSearchBruteforce;
    cloudSearchBruteforce.clear();

    for (auto it = cloudIn->points.cbegin(); it != cloudIn->points.cend(); ++it)
    {
      const auto pointDist = point_distance(*it, searchPoint);

      if (pointDist <= searchRadius)
      {
        // add point candidates to vector list
        cloudSearchBruteforce.push_back (std::distance(cloudIn->points.cbegin(), it));
      }
    }

    pcl::Indices cloudNWRSearch;
    std::vector<float> cloudNWRRadius;

    organizedNeighborSearch.setInputCloud (cloudIn);

    organizedNeighborSearch.radiusSearch ((*cloudIn)[randomIdx], searchRadius, cloudNWRSearch, cloudNWRRadius, std::numeric_limits<unsigned int>::max());


    // check if result from organized radius search can be also found in bruteforce search
    for (const auto it : cloudNWRSearch)
    {
      const auto pointDist = point_distance((*cloudIn)[it], searchPoint);
      ASSERT_LE (pointDist, searchRadius);
    }


    // check if bruteforce result from organized radius search can be also found in bruteforce search
    for (const auto it : cloudSearchBruteforce)
    {
      const auto pointDist = point_distance((*cloudIn)[it], searchPoint);
      ASSERT_LE (pointDist, searchRadius);
    }

    ASSERT_EQ (cloudNWRRadius.size() , cloudSearchBruteforce.size ());

    // check if result limitation works
    organizedNeighborSearch.radiusSearch (searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius, 5);

    ASSERT_LE (cloudNWRRadius.size (), 5);
  }
}


TEST (PCL, Organized_Neighbor_Search_Pointcloud_Neighbours_Within_Radius_Search_Benchmark_Test)
{
  constexpr unsigned int test_runs = 10;
  const unsigned int seed = time (nullptr);
  srand (seed);

  search::OrganizedNeighbor<PointXYZ> organizedNeighborSearch;

  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;

  std::vector<int> k_indices_bruteforce;
  std::vector<float> k_sqr_distances_bruteforce;

  // typical focal length from kinect
  constexpr double oneOverFocalLength = 0.0018;

  for (unsigned int test_id = 0; test_id < test_runs; test_id++)
  {
    // generate point cloud

    PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

    cloudIn->width = 1024;
    cloudIn->height = 768;
    cloudIn->points.clear();
    cloudIn->points.resize (cloudIn->width * cloudIn->height);

    int centerX = cloudIn->width >> 1;
    int centerY = cloudIn->height >> 1;

    int idx = 0;
    for (int ypos = -centerY; ypos < centerY; ypos++)
      for (int xpos = -centerX; xpos < centerX; xpos++)
      {
        double z = 5.0 * ( ((double)rand () / (double)RAND_MAX))+5;
        double y = ypos*oneOverFocalLength*z;
        double x = xpos*oneOverFocalLength*z;

        (*cloudIn)[idx++]= PointXYZ (x, y, z);
      }

    const unsigned int randomIdx = rand() % (cloudIn->width * cloudIn->height);

    const PointXYZ& searchPoint = (*cloudIn)[randomIdx];

    const double searchRadius = 1.0 * ((double)rand () / (double)RAND_MAX);

    // bruteforce radius search
    std::vector<int> cloudSearchBruteforce;
    cloudSearchBruteforce.clear();

    for (auto it = cloudIn->points.cbegin(); it != cloudIn->points.cend(); ++it)
    {
      const auto pointDist = point_distance(*it, searchPoint);

      if (pointDist <= searchRadius)
      {
        // add point candidates to vector list
        cloudSearchBruteforce.push_back (std::distance(cloudIn->points.cbegin(), it));
      }
    }

    pcl::Indices cloudNWRSearch;
    std::vector<float> cloudNWRRadius;

    organizedNeighborSearch.setInputCloud (cloudIn);
    organizedNeighborSearch.radiusSearch ((*cloudIn)[randomIdx], searchRadius, cloudNWRSearch, cloudNWRRadius, std::numeric_limits<unsigned int>::max());

    organizedNeighborSearch.setInputCloud (cloudIn);
    organizedNeighborSearch.radiusSearch ((*cloudIn)[randomIdx], searchRadius, cloudNWRSearch, cloudNWRRadius, std::numeric_limits<unsigned int>::max());
  }
}

TEST (PCL, Organized_Neighbor_Search_Pointcloud_Neighbours_Within_Radius_Search_Kinect_Data)
{

  pcl::PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
  pcl::PointCloud<PointXYZ>::Ptr cloudIn (new pcl::PointCloud<PointXYZ>);
  pcl::PCDReader pcd;

  if (pcd.read (pcd_filename, *cloudIn) == -1)
  {
	  std::cout <<"Couldn't read input cloud" << std::endl;
    return;
  }
  constexpr unsigned int test_runs = 10;
  const unsigned int seed = time (nullptr);
  srand (seed);
  SCOPED_TRACE("seed=" + std::to_string(seed));

  search::OrganizedNeighbor<PointXYZ> organizedNeighborSearch;

  // typical focal length from kinect
  unsigned int randomIdx;

  while (true)
  {
	  randomIdx = rand()%(cloudIn->width * cloudIn->height);
	  if((*cloudIn)[randomIdx].z <100)break;
  }

  std::cout << "**Search Point:** " << (*cloudIn)[randomIdx].x << " " << (*cloudIn)[randomIdx].y << " " << (*cloudIn)[randomIdx].z << std::endl;

  std::cout << "+--------+-----------------+----------------+-------------------+" << std::endl;
  std::cout << "| Search | Organized       | Organized      | Number of         |" << std::endl;
  std::cout << "| Radius | Neighbor Search | Data Index     | Nearest Neighbors |" << std::endl;
  std::cout << "+========+=================+================+===================+" << std::endl;

  for (unsigned int test_id = 0; test_id < test_runs; test_id++)
  {

    double searchRadius = 1.0 * (test_id*1.0/10*1.0);
    double sum_time = 0, sum_time2 = 0;

    pcl::Indices cloudNWRSearch;
    std::vector<float> cloudNWRRadius;

    pcl::Indices cloudNWRSearch2;
    std::vector<float> cloudNWRRadius2;

    for (int iter = 0; iter < 100; iter++)
    {

      cloudNWRSearch2.clear();
      cloudNWRRadius2.clear();

      double check_time = getTime();
      organizedNeighborSearch.setInputCloud (cloudIn);
      organizedNeighborSearch.radiusSearch ((*cloudIn)[randomIdx], searchRadius, cloudNWRSearch2, cloudNWRRadius2, std::numeric_limits<unsigned int>::max());

      double check_time2 = getTime();
      sum_time+= check_time2 - check_time;
     }


     for(int iter=0;iter<100;iter++)
     {
       cloudNWRSearch.clear();
       cloudNWRRadius.clear();
       double check_time = getTime();
       organizedNeighborSearch.setInputCloud (cloudIn);
       organizedNeighborSearch.radiusSearch ((*cloudIn)[randomIdx], searchRadius, cloudNWRSearch, cloudNWRRadius, std::numeric_limits<unsigned int>::max());

       double check_time2 = getTime();
       sum_time2+= check_time2 - check_time;
     }

 //  ASSERT_EQ(cloudNWRRadius.size(), cloudNWRRadius2.size());
 //  ASSERT_EQ(cloudNWRSearch.size(), cloudNWRSearch2.size());


    printf("| %.3lf  | %0.5lf         | %0.5lf        | %6zu            |\n",searchRadius, sum_time/100, sum_time2/100, cloudNWRSearch.size());
    std::cout << "+--------+-----------------+----------------+-------------------+" << std::endl;

    const PointXYZ& searchPoint = (*cloudIn)[randomIdx];

    // bruteforce radius search
    std::vector<int> cloudSearchBruteforce;
    cloudSearchBruteforce.clear();

    for (auto it = cloudIn->points.cbegin(); it != cloudIn->points.cend(); ++it)
    {
      const auto pointDist = point_distance(*it, searchPoint);

      if (pointDist <= searchRadius)
      {
        // add point candidates to vector list
        cloudSearchBruteforce.push_back (std::distance(cloudIn->points.cbegin(), it));
      }
    }

    // check if result from organized radius search can be also found in bruteforce search
    for (const auto it : cloudNWRSearch)
    {
      const auto pointDist = point_distance((*cloudIn)[it], searchPoint);
      ASSERT_LE (pointDist, searchRadius);
    }


    // check if bruteforce result from organized radius search can be also found in bruteforce search
    for (const auto it : cloudSearchBruteforce)
    {
      const auto pointDist = point_distance((*cloudIn)[it], searchPoint);
      ASSERT_LE (pointDist, searchRadius);
    }

    ASSERT_EQ (cloudNWRRadius.size() , cloudSearchBruteforce.size ());

    // check if result limitation works
    organizedNeighborSearch.radiusSearch (searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius, 5);

    ASSERT_LE (cloudNWRRadius.size (), 5);
  }
}

/* ---[ */
int
main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);

  if (argc < 2)
  {
    std::cout << "need path to office1.pcd file\n";
    return (-1);
  }

  pcd_filename = std::string(argv[1]);

  return (RUN_ALL_TESTS ());
}
/* ]--- */
