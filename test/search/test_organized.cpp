/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
/** \author Julius Kammerl (julius@kammerl.de)*/

#include <pcl/test/gtest.h>

#include <vector>


#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h> // for OrganizedNeighbor

using namespace pcl;

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

TEST (PCL, Organized_Neighbor_Pointcloud_Nearest_K_Neighbour_Search)
{
  constexpr unsigned int test_runs = 2;

  // instantiate point cloud
  PointCloud<PointXYZ>::Ptr cloudIn (new PointCloud<PointXYZ> ());

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
        double z = 15.0 * (double (rand ()) / double (RAND_MAX+1.0))+20;
        double y = ypos * oneOverFocalLength * z;
        double x = xpos * oneOverFocalLength * z;

        cloudIn->points.emplace_back(float (x), float (y), float (z));
      }

    unsigned int searchIdx = rand()%(cloudIn->width * cloudIn->height);
    const PointXYZ& searchPoint = (*cloudIn)[searchIdx];

    k_indices.clear();
    k_sqr_distances.clear();

    // organized nearest neighbor search
    organizedNeighborSearch.setInputCloud (cloudIn);
    organizedNeighborSearch.nearestKSearch (searchPoint, int (K), k_indices, k_sqr_distances);

    k_indices_bruteforce.clear();
    k_sqr_distances_bruteforce.clear();

    std::priority_queue<prioPointQueueEntry, std::vector<prioPointQueueEntry, Eigen::aligned_allocator<prioPointQueueEntry> > > pointCandidates;


    // push all points and their distance to the search point into a priority queue - bruteforce approach.
    for (std::size_t i = 0; i < cloudIn->size (); i++)
    {
      double pointDist = (((*cloudIn)[i].x - searchPoint.x) * ((*cloudIn)[i].x - searchPoint.x) +
                          ((*cloudIn)[i].y - searchPoint.y) * ((*cloudIn)[i].y - searchPoint.y) +
                          ((*cloudIn)[i].z - searchPoint.z) * ((*cloudIn)[i].z - searchPoint.z));

      prioPointQueueEntry pointEntry ((*cloudIn)[i], pointDist, int (i));

      pointCandidates.push (pointEntry);
    }

    // pop priority queue until we have the nearest K elements
    while (pointCandidates.size () > K)
      pointCandidates.pop ();

    // copy results into vectors
    while (!pointCandidates.empty ())
    {
      k_indices_bruteforce.push_back (pointCandidates.top ().pointIdx_);
      k_sqr_distances_bruteforce.push_back (float (pointCandidates.top ().pointDistance_));

      pointCandidates.pop ();
    }


    ASSERT_EQ ( k_indices.size() , k_indices_bruteforce.size() );

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

TEST (PCL, Organized_Neighbor_Pointcloud_Neighbours_Within_Radius_Search)
{
  constexpr unsigned int test_runs = 10;

  const unsigned int seed = time (nullptr);
  srand (seed);
  SCOPED_TRACE("seed=" + std::to_string(seed));

  search::OrganizedNeighbor<PointXYZ> organizedNeighborSearch;

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
        double z = 5.0 * ( (double (rand ()) / double (RAND_MAX)))+5;
        double y = ypos*oneOverFocalLength*z;
        double x = xpos*oneOverFocalLength*z;

        (*cloudIn)[idx++]= PointXYZ (float (x), float (y), float (z));
      }

    unsigned int randomIdx = rand()%(cloudIn->width * cloudIn->height);

    const PointXYZ& searchPoint = (*cloudIn)[randomIdx];

    double pointDist;
    double searchRadius = 1.0 * (double (rand ()) / double (RAND_MAX));

    // bruteforce radius search
    std::vector<int> cloudSearchBruteforce;
    cloudSearchBruteforce.clear();

    for (std::size_t i = 0; i < cloudIn->size (); i++)
    {
      pointDist = std::sqrt (
                        ((*cloudIn)[i].x - searchPoint.x) * ((*cloudIn)[i].x - searchPoint.x)
                      + ((*cloudIn)[i].y - searchPoint.y) * ((*cloudIn)[i].y - searchPoint.y)
                      + ((*cloudIn)[i].z - searchPoint.z) * ((*cloudIn)[i].z - searchPoint.z));

      if (pointDist <= searchRadius)
      {
        // add point candidates to vector list
        cloudSearchBruteforce.push_back (int (i));
      }
    }

    pcl::Indices cloudNWRSearch;
    std::vector<float> cloudNWRRadius;

    // execute organized search
    organizedNeighborSearch.setInputCloud (cloudIn);
    organizedNeighborSearch.radiusSearch (searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius);

    // check if result from organized radius search can be also found in bruteforce search
    for (const auto& current : cloudNWRSearch)
    {
      pointDist = std::sqrt (
          ((*cloudIn)[current].x-searchPoint.x) * ((*cloudIn)[current].x-searchPoint.x) +
          ((*cloudIn)[current].y-searchPoint.y) * ((*cloudIn)[current].y-searchPoint.y) +
          ((*cloudIn)[current].z-searchPoint.z) * ((*cloudIn)[current].z-searchPoint.z)
      );

      ASSERT_LE (pointDist, searchRadius);
    }


    // check if bruteforce result from organized radius search can be also found in bruteforce search
    for (const auto& current : cloudSearchBruteforce)
    {
      pointDist = std::sqrt (
          ((*cloudIn)[current].x-searchPoint.x) * ((*cloudIn)[current].x-searchPoint.x) +
          ((*cloudIn)[current].y-searchPoint.y) * ((*cloudIn)[current].y-searchPoint.y) +
          ((*cloudIn)[current].z-searchPoint.z) * ((*cloudIn)[current].z-searchPoint.z)
      );

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
  return (RUN_ALL_TESTS ());
}
/* ]--- */
