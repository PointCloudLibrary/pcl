/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
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
#include <iostream>  // For debug
#include <map>
#include <pcl/common/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;


PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());

struct MyPoint : public PointXYZ 
{
    MyPoint (float x, float y, float z) {this->x=x; this->y=y; this->z=z;}
};

PointCloud<MyPoint> cloud, cloud_big;

// Includ the implementation so that KdTree<MyPoint> works
#include <pcl/kdtree/impl/kdtree_flann.hpp>

void 
init ()
{
  float resolution = 0.1f;
  for (float z = -0.5f; z <= 0.5f; z += resolution)
    for (float y = -0.5f; y <= 0.5f; y += resolution)
      for (float x = -0.5f; x <= 0.5f; x += resolution)
        cloud.points.push_back (MyPoint (x, y, z));
  cloud.width  = static_cast<uint32_t> (cloud.points.size ());
  cloud.height = 1;

  cloud_big.width  = 640;
  cloud_big.height = 480;
  srand (static_cast<unsigned int> (time (NULL)));
  // Randomly create a new point cloud
  for (size_t i = 0; i < cloud_big.width * cloud_big.height; ++i)
    cloud_big.points.push_back (MyPoint (static_cast<float> (1024 * rand () / (RAND_MAX + 1.0)),
                                         static_cast<float> (1024 * rand () / (RAND_MAX + 1.0)),
                                         static_cast<float> (1024 * rand () / (RAND_MAX + 1.0))));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, KdTreeFLANN_radiusSearch)
{
  KdTreeFLANN<MyPoint> kdtree;
  kdtree.setInputCloud (cloud.makeShared ());
  MyPoint test_point(0.0f, 0.0f, 0.0f);
  double max_dist = 0.15;
  set<int> brute_force_result;
  for (unsigned int i=0; i<cloud.points.size(); ++i)
    if (euclideanDistance(cloud.points[i], test_point) < max_dist)
      brute_force_result.insert(i);
  vector<int> k_indices;
  vector<float> k_distances;
  kdtree.radiusSearch (test_point, max_dist, k_indices, k_distances, 100);
  
  //cout << k_indices.size()<<"=="<<brute_force_result.size()<<"?\n";
  
  for (size_t i = 0; i < k_indices.size (); ++i)
  {
    set<int>::iterator brute_force_result_it = brute_force_result.find (k_indices[i]);
    bool ok = brute_force_result_it != brute_force_result.end ();
    //if (!ok)  cerr << k_indices[i] << " is not correct...\n";
    //else      cerr << k_indices[i] << " is correct...\n";
    EXPECT_EQ (ok, true);
    if (ok)
      brute_force_result.erase (brute_force_result_it);
  }
  //for (set<int>::const_iterator it=brute_force_result.begin(); it!=brute_force_result.end(); ++it)
  //cerr << "FLANN missed "<<*it<<"\n";
  
  bool error = brute_force_result.size () > 0;
  //if (error)  cerr << "Missed too many neighbors!\n";
  EXPECT_EQ (error, false);

  {
    KdTreeFLANN<MyPoint> kdtree;
    kdtree.setInputCloud (cloud_big.makeShared ());

    ScopeTime scopeTime ("FLANN radiusSearch");
    {
      for (size_t i = 0; i < cloud_big.points.size (); ++i)
        kdtree.radiusSearch (cloud_big.points[i], 0.1, k_indices, k_distances);
    }
  }
  
  {
    KdTreeFLANN<MyPoint> kdtree;
    kdtree.setInputCloud (cloud_big.makeShared ());

    ScopeTime scopeTime ("FLANN radiusSearch (max neighbors in radius)");
    {
      for (size_t i = 0; i < cloud_big.points.size (); ++i)
        kdtree.radiusSearch (cloud_big.points[i], 0.1, k_indices, k_distances, 10);
    }
  }
  
  
  {
    KdTreeFLANN<MyPoint> kdtree (false);
    kdtree.setInputCloud (cloud_big.makeShared ());

    ScopeTime scopeTime ("FLANN radiusSearch (unsorted results)");
    {
      for (size_t i = 0; i < cloud_big.points.size (); ++i)
        kdtree.radiusSearch (cloud_big.points[i], 0.1, k_indices, k_distances);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, KdTreeFLANN_nearestKSearch)
{
  KdTreeFLANN<MyPoint> kdtree;
  kdtree.setInputCloud (cloud.makeShared ());
  MyPoint test_point (0.01f, 0.01f, 0.01f);
  unsigned int no_of_neighbors = 20;
  multimap<float, int> sorted_brute_force_result;
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    float distance = euclideanDistance (cloud.points[i], test_point);
    sorted_brute_force_result.insert (make_pair (distance, static_cast<int> (i)));
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
  kdtree.nearestKSearch (test_point, no_of_neighbors, k_indices, k_distances);
  //if (k_indices.size() != no_of_neighbors)  cerr << "Found "<<k_indices.size()<<" instead of "<<no_of_neighbors<<" neighbors.\n";
  EXPECT_EQ (k_indices.size (), no_of_neighbors);

  // Check if all found neighbors have distance smaller than max_dist
  for (size_t i = 0; i < k_indices.size (); ++i)
  {
    const MyPoint& point = cloud.points[k_indices[i]];
    bool ok = euclideanDistance (test_point, point) <= max_dist;
    if (!ok)
      ok = (fabs (euclideanDistance (test_point, point)) - max_dist) <= 1e-6;
    //if (!ok)  cerr << k_indices[i] << " is not correct...\n";
    //else      cerr << k_indices[i] << " is correct...\n";
    EXPECT_EQ (ok, true);
  }

  ScopeTime scopeTime ("FLANN nearestKSearch");
  {
    KdTreeFLANN<MyPoint> kdtree;
    kdtree.setInputCloud (cloud_big.makeShared ());
    for (size_t i = 0; i < cloud_big.points.size (); ++i)
      kdtree.nearestKSearch (cloud_big.points[i], no_of_neighbors, k_indices, k_distances);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyPointRepresentationXY : public PointRepresentation<MyPoint>
{
  public:
    MyPointRepresentationXY ()
    {
      this->nr_dimensions_ = 2;
    }

    void copyToFloatArray (const MyPoint &p, float *out) const
    {
      out[0] = p.x;
      out[1] = p.y;
    }
};

TEST (PCL, KdTreeFLANN_setPointRepresentation)
{
  PointCloud<MyPoint>::Ptr random_cloud (new PointCloud<MyPoint> ());
  random_cloud->points.push_back (MyPoint (86.6f, 42.1f, 92.4f));
  random_cloud->points.push_back (MyPoint (63.1f, 18.4f, 22.3f));
  random_cloud->points.push_back (MyPoint (35.5f, 72.5f, 37.3f));
  random_cloud->points.push_back (MyPoint (99.7f, 37.0f,  8.7f));
  random_cloud->points.push_back (MyPoint (22.4f, 84.1f, 64.0f));
  random_cloud->points.push_back (MyPoint (65.2f, 73.4f, 18.0f));
  random_cloud->points.push_back (MyPoint (60.4f, 57.1f,  4.5f));
  random_cloud->points.push_back (MyPoint (38.7f, 17.6f, 72.3f));
  random_cloud->points.push_back (MyPoint (14.2f, 95.7f, 34.7f));
  random_cloud->points.push_back (MyPoint ( 2.5f, 26.5f, 66.0f));

  KdTreeFLANN<MyPoint> kdtree;
  kdtree.setInputCloud (random_cloud);
  MyPoint p (50.0f, 50.0f, 50.0f);
  
  // Find k nearest neighbors
  const int k = 10;
  vector<int> k_indices (k);
  vector<float> k_distances (k);
  kdtree.nearestKSearch (p, k, k_indices, k_distances);
  for (int i = 0; i < k; ++i)
  {
    // Compare to ground truth values, computed independently
    static const int gt_indices[10] = {2, 7, 5, 1, 4, 6, 9, 0, 8, 3};
    static const float gt_distances[10] =
    {877.8f, 1674.7f, 1802.6f, 1937.5f, 2120.6f, 2228.8f, 3064.5f, 3199.7f, 3604.2f, 4344.8f};
    EXPECT_EQ (k_indices[i], gt_indices[i]);
    EXPECT_NEAR (k_distances[i], gt_distances[i], 0.1);
  }
  
  // Find k nearest neighbors with a different point representation
  boost::shared_ptr<MyPointRepresentationXY> ptrep (new MyPointRepresentationXY);
  kdtree.setPointRepresentation (ptrep);
  kdtree.nearestKSearch (p, k, k_indices, k_distances);
  for (int i = 0; i < k; ++i)
  {
    // Compare to ground truth values, computed independently
    static const int gt_indices[10] = {6, 2, 5, 1, 7, 0, 4, 3, 9, 8};
    static const float gt_distances[10] =
    {158.6f, 716.5f, 778.6f, 1170.2f, 1177.5f, 1402.0f, 1924.6f, 2639.1f, 2808.5f, 3370.1f};
    EXPECT_EQ (k_indices[i], gt_indices[i]);
    EXPECT_NEAR (k_distances[i], gt_distances[i], 0.1);
  }

  // Go back to the default, this time with the values rescaled
  DefaultPointRepresentation<MyPoint> point_rep;
  float alpha[3] = {1.0f, 2.0f, 3.0f};
  point_rep.setRescaleValues(alpha);
  kdtree.setPointRepresentation (point_rep.makeShared ());
  kdtree.nearestKSearch (p, k, k_indices, k_distances);
  for (int i = 0; i < k; ++i)
  {
    // Compare to ground truth values, computed independently
    static const int gt_indices[10] =  {2, 9, 4, 7, 1, 5, 8, 0, 3, 6};
    static const float gt_distances[10] =
    {3686.9f, 6769.2f, 7177.0f, 8802.3f, 11071.5f, 11637.3f, 11742.4f, 17769.0f, 18497.3f, 18942.0f};
    EXPECT_EQ (k_indices[i], gt_indices[i]);
    EXPECT_NEAR (k_distances[i], gt_distances[i], 0.1);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, KdTreeFLANN_32_vs_64_bit)
{
  KdTreeFLANN<PointXYZ> tree;
  tree.setInputCloud (cloud_in);

  std::vector<std::vector<int> > nn_indices_vector;
  for (size_t i = 0; i < cloud_in->size (); ++i)
    if (isFinite ((*cloud_in)[i]))
    {
      std::vector<int> nn_indices;
      std::vector<float> nn_dists;
      tree.radiusSearch ((*cloud_in)[i], 0.02, nn_indices, nn_dists);

      nn_indices_vector.push_back (nn_indices);
    }
  EXPECT_EQ (nn_indices_vector[0][0], 0);
  EXPECT_EQ (nn_indices_vector[0][1], 2057);
  EXPECT_EQ (nn_indices_vector[0][2], 2663);
  EXPECT_EQ (nn_indices_vector[0][3], 2268);
  EXPECT_EQ (nn_indices_vector[0][4], 14);
  EXPECT_EQ (nn_indices_vector[20][0], 20);
  EXPECT_EQ (nn_indices_vector[20][1], 4);
  EXPECT_EQ (nn_indices_vector[20][2], 2272);
  EXPECT_EQ (nn_indices_vector[20][3], 58);
  EXPECT_EQ (nn_indices_vector[20][4], 18);
  EXPECT_EQ (nn_indices_vector[40][0], 40);
  EXPECT_EQ (nn_indices_vector[40][1], 37);
  EXPECT_EQ (nn_indices_vector[40][2], 5);
  EXPECT_EQ (nn_indices_vector[40][3], 39);
  EXPECT_EQ (nn_indices_vector[40][4], 42);
  EXPECT_EQ (nn_indices_vector[60][0], 60);
  EXPECT_EQ (nn_indices_vector[60][1], 39);
  EXPECT_EQ (nn_indices_vector[60][2], 55);
  EXPECT_EQ (nn_indices_vector[60][3], 61);
  EXPECT_EQ (nn_indices_vector[60][4], 7);
  EXPECT_EQ (nn_indices_vector[80][0], 80);
  EXPECT_EQ (nn_indices_vector[80][1], 84);
  EXPECT_EQ (nn_indices_vector[80][2], 141);
  EXPECT_EQ (nn_indices_vector[80][3], 142);
  EXPECT_EQ (nn_indices_vector[80][4], 79);
  EXPECT_EQ (nn_indices_vector[80][5], 91);
  EXPECT_EQ (nn_indices_vector[80][6], 78);
  EXPECT_EQ (nn_indices_vector[100][0], 100);
  EXPECT_EQ (nn_indices_vector[100][1], 104);
  EXPECT_EQ (nn_indices_vector[100][2], 105);
  EXPECT_EQ (nn_indices_vector[100][3], 490);
  EXPECT_EQ (nn_indices_vector[100][4], 101);
  EXPECT_EQ (nn_indices_vector[100][5], 486);
  EXPECT_EQ (nn_indices_vector[100][6], 487);
  EXPECT_EQ (nn_indices_vector[120][0], 120);
  EXPECT_EQ (nn_indices_vector[120][1], 118);
  EXPECT_EQ (nn_indices_vector[120][2], 108);
  EXPECT_EQ (nn_indices_vector[120][3], 106);
  EXPECT_EQ (nn_indices_vector[120][4], 121);
  EXPECT_EQ (nn_indices_vector[120][5], 99);
  EXPECT_EQ (nn_indices_vector[120][6], 122);
  EXPECT_EQ (nn_indices_vector[140][0], 140);
  EXPECT_EQ (nn_indices_vector[140][1], 141);
  EXPECT_EQ (nn_indices_vector[140][2], 125);
  EXPECT_EQ (nn_indices_vector[140][3], 78);
  EXPECT_EQ (nn_indices_vector[160][0], 160);
  EXPECT_EQ (nn_indices_vector[160][1], 508);
  EXPECT_EQ (nn_indices_vector[160][2], 514);
  EXPECT_EQ (nn_indices_vector[160][3], 502);
  EXPECT_EQ (nn_indices_vector[160][4], 166);
  EXPECT_EQ (nn_indices_vector[180][0], 180);
  EXPECT_EQ (nn_indices_vector[180][1], 524);
  EXPECT_EQ (nn_indices_vector[180][2], 535);
  EXPECT_EQ (nn_indices_vector[180][3], 179);
  EXPECT_EQ (nn_indices_vector[180][4], 184);
  EXPECT_EQ (nn_indices_vector[180][5], 213);
  EXPECT_EQ (nn_indices_vector[200][0], 200);
  EXPECT_EQ (nn_indices_vector[200][1], 152);
  EXPECT_EQ (nn_indices_vector[200][2], 202);
  EXPECT_EQ (nn_indices_vector[200][3], 153);
  EXPECT_EQ (nn_indices_vector[200][4], 201);
  EXPECT_EQ (nn_indices_vector[220][0], 220);
  EXPECT_EQ (nn_indices_vector[220][1], 198);
  EXPECT_EQ (nn_indices_vector[220][2], 182);
  EXPECT_EQ (nn_indices_vector[220][3], 326);
  EXPECT_EQ (nn_indices_vector[220][4], 221);
  EXPECT_EQ (nn_indices_vector[240][0], 240);
  EXPECT_EQ (nn_indices_vector[240][1], 226);
  EXPECT_EQ (nn_indices_vector[240][2], 239);
  EXPECT_EQ (nn_indices_vector[240][3], 356);
  EXPECT_EQ (nn_indices_vector[240][4], 238);
  EXPECT_EQ (nn_indices_vector[240][5], 237);
  EXPECT_EQ (nn_indices_vector[260][0], 260);
  EXPECT_EQ (nn_indices_vector[260][1], 262);
  EXPECT_EQ (nn_indices_vector[260][2], 264);
  EXPECT_EQ (nn_indices_vector[260][3], 266);
  EXPECT_EQ (nn_indices_vector[260][4], 278);
  EXPECT_EQ (nn_indices_vector[280][0], 280);
  EXPECT_EQ (nn_indices_vector[280][1], 293);
  EXPECT_EQ (nn_indices_vector[280][2], 327);
  EXPECT_EQ (nn_indices_vector[280][3], 305);
  EXPECT_EQ (nn_indices_vector[280][4], 283);
  EXPECT_EQ (nn_indices_vector[280][5], 312);
  EXPECT_EQ (nn_indices_vector[300][0], 300);
  EXPECT_EQ (nn_indices_vector[300][1], 670);
  EXPECT_EQ (nn_indices_vector[300][2], 223);
  EXPECT_EQ (nn_indices_vector[300][3], 301);
  EXPECT_EQ (nn_indices_vector[300][4], 288);
  EXPECT_EQ (nn_indices_vector[300][5], 302);
  EXPECT_EQ (nn_indices_vector[300][6], 303);
  EXPECT_EQ (nn_indices_vector[320][0], 320);
  EXPECT_EQ (nn_indices_vector[320][1], 250);
  EXPECT_EQ (nn_indices_vector[320][2], 323);
  EXPECT_EQ (nn_indices_vector[320][3], 318);
  EXPECT_EQ (nn_indices_vector[320][4], 321);
  EXPECT_EQ (nn_indices_vector[320][5], 281);
  EXPECT_EQ (nn_indices_vector[320][6], 248);
  EXPECT_EQ (nn_indices_vector[340][0], 340);
  EXPECT_EQ (nn_indices_vector[340][1], 339);
  EXPECT_EQ (nn_indices_vector[340][2], 259);
  EXPECT_EQ (nn_indices_vector[340][3], 344);
  EXPECT_EQ (nn_indices_vector[340][4], 1566);
  EXPECT_EQ (nn_indices_vector[340][5], 338);
  EXPECT_EQ (nn_indices_vector[360][0], 360);
  EXPECT_EQ (nn_indices_vector[360][1], 412);
  EXPECT_EQ (nn_indices_vector[360][2], 410);
  EXPECT_EQ (nn_indices_vector[360][3], 546);
  EXPECT_EQ (nn_indices_vector[360][4], 362);
  EXPECT_EQ (nn_indices_vector[360][5], 495);
  EXPECT_EQ (nn_indices_vector[380][0], 380);
  EXPECT_EQ (nn_indices_vector[380][1], 364);
  EXPECT_EQ (nn_indices_vector[380][2], 582);
  EXPECT_EQ (nn_indices_vector[380][3], 379);
  EXPECT_EQ (nn_indices_vector[380][4], 378);
  EXPECT_EQ (nn_indices_vector[400][0], 400);
  EXPECT_EQ (nn_indices_vector[400][1], 421);
  EXPECT_EQ (nn_indices_vector[400][2], 74);
  EXPECT_EQ (nn_indices_vector[400][3], 401);
  EXPECT_EQ (nn_indices_vector[400][4], 395);
  EXPECT_EQ (nn_indices_vector[400][5], 403);
  EXPECT_EQ (nn_indices_vector[400][6], 402);
  EXPECT_EQ (nn_indices_vector[420][0], 420);
  EXPECT_EQ (nn_indices_vector[420][1], 416);
  EXPECT_EQ (nn_indices_vector[420][2], 414);
  EXPECT_EQ (nn_indices_vector[420][3], 386);
  EXPECT_EQ (nn_indices_vector[420][4], 418);
  EXPECT_EQ (nn_indices_vector[440][0], 440);
  EXPECT_EQ (nn_indices_vector[440][1], 389);
  EXPECT_EQ (nn_indices_vector[440][2], 444);
  EXPECT_EQ (nn_indices_vector[440][3], 441);
  EXPECT_EQ (nn_indices_vector[460][0], 460);
  EXPECT_EQ (nn_indices_vector[460][1], 456);
  EXPECT_EQ (nn_indices_vector[460][2], 462);
  EXPECT_EQ (nn_indices_vector[460][3], 459);
  EXPECT_EQ (nn_indices_vector[460][4], 405);
  EXPECT_EQ (nn_indices_vector[480][0], 480);
  EXPECT_EQ (nn_indices_vector[480][1], 476);
  EXPECT_EQ (nn_indices_vector[480][2], 526);
  EXPECT_EQ (nn_indices_vector[480][3], 479);
  EXPECT_EQ (nn_indices_vector[500][0], 500);
  EXPECT_EQ (nn_indices_vector[500][1], 504);
  EXPECT_EQ (nn_indices_vector[500][2], 107);
  EXPECT_EQ (nn_indices_vector[500][3], 507);
  EXPECT_EQ (nn_indices_vector[500][4], 506);
  EXPECT_EQ (nn_indices_vector[500][5], 502);
  EXPECT_EQ (nn_indices_vector[520][0], 520);
  EXPECT_EQ (nn_indices_vector[520][1], 527);
  EXPECT_EQ (nn_indices_vector[520][2], 516);
  EXPECT_EQ (nn_indices_vector[520][3], 532);
  EXPECT_EQ (nn_indices_vector[520][4], 519);
  EXPECT_EQ (nn_indices_vector[540][0], 540);
  EXPECT_EQ (nn_indices_vector[540][1], 370);
  EXPECT_EQ (nn_indices_vector[540][2], 366);
  EXPECT_EQ (nn_indices_vector[540][3], 537);
  EXPECT_EQ (nn_indices_vector[540][4], 497);
  EXPECT_EQ (nn_indices_vector[540][5], 493);
  EXPECT_EQ (nn_indices_vector[560][0], 560);
  EXPECT_EQ (nn_indices_vector[560][1], 581);
  EXPECT_EQ (nn_indices_vector[560][2], 582);
  EXPECT_EQ (nn_indices_vector[560][3], 583);
  EXPECT_EQ (nn_indices_vector[560][4], 562);
  EXPECT_EQ (nn_indices_vector[560][5], 561);
  EXPECT_EQ (nn_indices_vector[580][0], 580);
  EXPECT_EQ (nn_indices_vector[580][1], 576);
  EXPECT_EQ (nn_indices_vector[580][2], 588);
  EXPECT_EQ (nn_indices_vector[580][3], 578);
  EXPECT_EQ (nn_indices_vector[580][4], 548);
  EXPECT_EQ (nn_indices_vector[580][5], 547);
  EXPECT_EQ (nn_indices_vector[600][0], 600);
  EXPECT_EQ (nn_indices_vector[600][1], 663);
  EXPECT_EQ (nn_indices_vector[600][2], 601);
  EXPECT_EQ (nn_indices_vector[600][3], 662);
  EXPECT_EQ (nn_indices_vector[600][4], 664);
  EXPECT_EQ (nn_indices_vector[600][5], 602);
  EXPECT_EQ (nn_indices_vector[620][0], 620);
  EXPECT_EQ (nn_indices_vector[620][1], 621);
  EXPECT_EQ (nn_indices_vector[620][2], 625);
  EXPECT_EQ (nn_indices_vector[620][3], 624);
  EXPECT_EQ (nn_indices_vector[620][4], 633);
  EXPECT_EQ (nn_indices_vector[620][5], 629);
  EXPECT_EQ (nn_indices_vector[640][0], 640);
  EXPECT_EQ (nn_indices_vector[640][1], 644);
  EXPECT_EQ (nn_indices_vector[640][2], 431);
  EXPECT_EQ (nn_indices_vector[640][3], 641);
  EXPECT_EQ (nn_indices_vector[640][4], 645);
  EXPECT_EQ (nn_indices_vector[660][0], 660);
  EXPECT_EQ (nn_indices_vector[660][1], 650);
  EXPECT_EQ (nn_indices_vector[660][2], 658);
  EXPECT_EQ (nn_indices_vector[660][3], 638);
  EXPECT_EQ (nn_indices_vector[660][4], 23);
  EXPECT_EQ (nn_indices_vector[660][5], 6);
  EXPECT_EQ (nn_indices_vector[680][0], 680);
  EXPECT_EQ (nn_indices_vector[680][1], 681);
  EXPECT_EQ (nn_indices_vector[680][2], 143);
  EXPECT_EQ (nn_indices_vector[680][3], 144);
  EXPECT_EQ (nn_indices_vector[700][0], 700);
  EXPECT_EQ (nn_indices_vector[700][1], 696);
  EXPECT_EQ (nn_indices_vector[700][2], 165);
  EXPECT_EQ (nn_indices_vector[700][3], 148);
  EXPECT_EQ (nn_indices_vector[700][4], 146);
  EXPECT_EQ (nn_indices_vector[700][5], 701);
  EXPECT_EQ (nn_indices_vector[720][0], 720);
  EXPECT_EQ (nn_indices_vector[720][1], 676);
  EXPECT_EQ (nn_indices_vector[720][2], 674);
  EXPECT_EQ (nn_indices_vector[720][3], 675);
  EXPECT_EQ (nn_indices_vector[720][4], 722);
  EXPECT_EQ (nn_indices_vector[720][5], 678);
  EXPECT_EQ (nn_indices_vector[740][0], 740);
  EXPECT_EQ (nn_indices_vector[740][1], 741);
  EXPECT_EQ (nn_indices_vector[740][2], 735);
  EXPECT_EQ (nn_indices_vector[740][3], 736);
  EXPECT_EQ (nn_indices_vector[740][4], 734);
  EXPECT_EQ (nn_indices_vector[740][5], 738);
  EXPECT_EQ (nn_indices_vector[740][6], 739);
  EXPECT_EQ (nn_indices_vector[740][7], 737);
  EXPECT_EQ (nn_indices_vector[760][0], 760);
  EXPECT_EQ (nn_indices_vector[760][1], 1127);
  EXPECT_EQ (nn_indices_vector[760][2], 1092);
  EXPECT_EQ (nn_indices_vector[760][3], 1137);
  EXPECT_EQ (nn_indices_vector[760][4], 782);
  EXPECT_EQ (nn_indices_vector[760][5], 774);
  EXPECT_EQ (nn_indices_vector[780][0], 780);
  EXPECT_EQ (nn_indices_vector[780][1], 776);
  EXPECT_EQ (nn_indices_vector[780][2], 773);
  EXPECT_EQ (nn_indices_vector[780][3], 781);
  EXPECT_EQ (nn_indices_vector[800][0], 800);
  EXPECT_EQ (nn_indices_vector[800][1], 804);
  EXPECT_EQ (nn_indices_vector[800][2], 811);
  EXPECT_EQ (nn_indices_vector[800][3], 807);
  EXPECT_EQ (nn_indices_vector[800][4], 1279);
  EXPECT_EQ (nn_indices_vector[800][5], 1285);
  EXPECT_EQ (nn_indices_vector[800][6], 1283);
  EXPECT_EQ (nn_indices_vector[820][0], 820);
  EXPECT_EQ (nn_indices_vector[820][1], 824);
  EXPECT_EQ (nn_indices_vector[820][2], 801);
  EXPECT_EQ (nn_indices_vector[820][3], 786);
  EXPECT_EQ (nn_indices_vector[820][4], 821);
  EXPECT_EQ (nn_indices_vector[820][5], 823);
  EXPECT_EQ (nn_indices_vector[820][6], 813);
  EXPECT_EQ (nn_indices_vector[820][7], 822);
  EXPECT_EQ (nn_indices_vector[840][0], 840);
  EXPECT_EQ (nn_indices_vector[840][1], 2764);
  EXPECT_EQ (nn_indices_vector[840][2], 2766);
  EXPECT_EQ (nn_indices_vector[840][3], 847);
  EXPECT_EQ (nn_indices_vector[840][4], 842);
  EXPECT_EQ (nn_indices_vector[860][0], 860);
  EXPECT_EQ (nn_indices_vector[860][1], 952);
  EXPECT_EQ (nn_indices_vector[860][2], 858);
  EXPECT_EQ (nn_indices_vector[860][3], 861);
  EXPECT_EQ (nn_indices_vector[860][4], 978);
  EXPECT_EQ (nn_indices_vector[860][5], 898);
  EXPECT_EQ (nn_indices_vector[880][0], 880);
  EXPECT_EQ (nn_indices_vector[880][1], 884);
  EXPECT_EQ (nn_indices_vector[880][2], 879);
  EXPECT_EQ (nn_indices_vector[880][3], 878);
  EXPECT_EQ (nn_indices_vector[880][4], 890);
  EXPECT_EQ (nn_indices_vector[900][0], 900);
  EXPECT_EQ (nn_indices_vector[900][1], 896);
  EXPECT_EQ (nn_indices_vector[900][2], 870);
  EXPECT_EQ (nn_indices_vector[900][3], 898);
  EXPECT_EQ (nn_indices_vector[920][0], 920);
  EXPECT_EQ (nn_indices_vector[920][1], 916);
  EXPECT_EQ (nn_indices_vector[920][2], 915);
  EXPECT_EQ (nn_indices_vector[920][3], 919);
  EXPECT_EQ (nn_indices_vector[920][4], 914);
  EXPECT_EQ (nn_indices_vector[920][5], 918);
  EXPECT_EQ (nn_indices_vector[920][6], 917);
  EXPECT_EQ (nn_indices_vector[940][0], 940);
  EXPECT_EQ (nn_indices_vector[940][1], 944);
  EXPECT_EQ (nn_indices_vector[940][2], 947);
  EXPECT_EQ (nn_indices_vector[940][3], 946);
  EXPECT_EQ (nn_indices_vector[940][4], 949);
  EXPECT_EQ (nn_indices_vector[940][5], 43);
  EXPECT_EQ (nn_indices_vector[940][6], 942);
  EXPECT_EQ (nn_indices_vector[960][0], 960);
  EXPECT_EQ (nn_indices_vector[960][1], 868);
  EXPECT_EQ (nn_indices_vector[960][2], 959);
  EXPECT_EQ (nn_indices_vector[960][3], 962);
  EXPECT_EQ (nn_indices_vector[960][4], 978);
  EXPECT_EQ (nn_indices_vector[980][0], 980);
  EXPECT_EQ (nn_indices_vector[980][1], 1030);
  EXPECT_EQ (nn_indices_vector[980][2], 1113);
  EXPECT_EQ (nn_indices_vector[980][3], 1081);
  EXPECT_EQ (nn_indices_vector[980][4], 990);
  EXPECT_EQ (nn_indices_vector[980][5], 979);
  EXPECT_EQ (nn_indices_vector[1000][0], 1000);
  EXPECT_EQ (nn_indices_vector[1000][1], 1001);
  EXPECT_EQ (nn_indices_vector[1000][2], 983);
  EXPECT_EQ (nn_indices_vector[1000][3], 994);
  EXPECT_EQ (nn_indices_vector[1000][4], 998);
  EXPECT_EQ (nn_indices_vector[1020][0], 1020);
  EXPECT_EQ (nn_indices_vector[1020][1], 1021);
  EXPECT_EQ (nn_indices_vector[1020][2], 1031);
  EXPECT_EQ (nn_indices_vector[1020][3], 1015);
  EXPECT_EQ (nn_indices_vector[1020][4], 1018);
  EXPECT_EQ (nn_indices_vector[1040][0], 1040);
  EXPECT_EQ (nn_indices_vector[1040][1], 1008);
  EXPECT_EQ (nn_indices_vector[1040][2], 1023);
  EXPECT_EQ (nn_indices_vector[1040][3], 1042);
  EXPECT_EQ (nn_indices_vector[1040][4], 1041);
  EXPECT_EQ (nn_indices_vector[1040][5], 1034);
  EXPECT_EQ (nn_indices_vector[1060][0], 1060);
  EXPECT_EQ (nn_indices_vector[1060][1], 1059);
  EXPECT_EQ (nn_indices_vector[1060][2], 1064);
  EXPECT_EQ (nn_indices_vector[1060][3], 1044);
  EXPECT_EQ (nn_indices_vector[1060][4], 1062);
  EXPECT_EQ (nn_indices_vector[1060][5], 1009);
  EXPECT_EQ (nn_indices_vector[1080][0], 1080);
  EXPECT_EQ (nn_indices_vector[1080][1], 989);
  EXPECT_EQ (nn_indices_vector[1080][2], 1081);
  EXPECT_EQ (nn_indices_vector[1080][3], 1082);
  EXPECT_EQ (nn_indices_vector[1100][0], 1100);
  EXPECT_EQ (nn_indices_vector[1100][1], 1097);
  EXPECT_EQ (nn_indices_vector[1100][2], 1112);
  EXPECT_EQ (nn_indices_vector[1100][3], 1107);
  EXPECT_EQ (nn_indices_vector[1100][4], 1099);
  EXPECT_EQ (nn_indices_vector[1120][0], 1120);
  EXPECT_EQ (nn_indices_vector[1120][1], 1129);
  EXPECT_EQ (nn_indices_vector[1120][2], 1130);
  EXPECT_EQ (nn_indices_vector[1120][3], 887);
  EXPECT_EQ (nn_indices_vector[1120][4], 1116);
  EXPECT_EQ (nn_indices_vector[1120][5], 1122);
  EXPECT_EQ (nn_indices_vector[1120][6], 1121);
  EXPECT_EQ (nn_indices_vector[1140][0], 1140);
  EXPECT_EQ (nn_indices_vector[1140][1], 1136);
  EXPECT_EQ (nn_indices_vector[1140][2], 1091);
  EXPECT_EQ (nn_indices_vector[1140][3], 1089);
  EXPECT_EQ (nn_indices_vector[1140][4], 1095);
  EXPECT_EQ (nn_indices_vector[1140][5], 1133);
  EXPECT_EQ (nn_indices_vector[1160][0], 1160);
  EXPECT_EQ (nn_indices_vector[1160][1], 1161);
  EXPECT_EQ (nn_indices_vector[1160][2], 1173);
  EXPECT_EQ (nn_indices_vector[1160][3], 1178);
  EXPECT_EQ (nn_indices_vector[1160][4], 1172);
  EXPECT_EQ (nn_indices_vector[1160][5], 1179);
  EXPECT_EQ (nn_indices_vector[1160][6], 1197);
  EXPECT_EQ (nn_indices_vector[1180][0], 1180);
  EXPECT_EQ (nn_indices_vector[1180][1], 1181);
  EXPECT_EQ (nn_indices_vector[1180][2], 1198);
  EXPECT_EQ (nn_indices_vector[1180][3], 1191);
  EXPECT_EQ (nn_indices_vector[1180][4], 1190);
  EXPECT_EQ (nn_indices_vector[1180][5], 1178);
  EXPECT_EQ (nn_indices_vector[1200][0], 1200);
  EXPECT_EQ (nn_indices_vector[1200][1], 1196);
  EXPECT_EQ (nn_indices_vector[1200][2], 3173);
  EXPECT_EQ (nn_indices_vector[1200][3], 1199);
  EXPECT_EQ (nn_indices_vector[1200][4], 1195);
  EXPECT_EQ (nn_indices_vector[1220][0], 1220);
  EXPECT_EQ (nn_indices_vector[1220][1], 1210);
  EXPECT_EQ (nn_indices_vector[1220][2], 1212);
  EXPECT_EQ (nn_indices_vector[1220][3], 1221);
  EXPECT_EQ (nn_indices_vector[1220][4], 1214);
  EXPECT_EQ (nn_indices_vector[1240][0], 1240);
  EXPECT_EQ (nn_indices_vector[1240][1], 1243);
  EXPECT_EQ (nn_indices_vector[1240][2], 1241);
  EXPECT_EQ (nn_indices_vector[1240][3], 1245);
  EXPECT_EQ (nn_indices_vector[1240][4], 1736);
  EXPECT_EQ (nn_indices_vector[1240][5], 1732);
  EXPECT_EQ (nn_indices_vector[1260][0], 1260);
  EXPECT_EQ (nn_indices_vector[1260][1], 1264);
  EXPECT_EQ (nn_indices_vector[1260][2], 1271);
  EXPECT_EQ (nn_indices_vector[1280][0], 1280);
  EXPECT_EQ (nn_indices_vector[1280][1], 1281);
  EXPECT_EQ (nn_indices_vector[1280][2], 1284);
  EXPECT_EQ (nn_indices_vector[1280][3], 1291);
  EXPECT_EQ (nn_indices_vector[1280][4], 1287);
  EXPECT_EQ (nn_indices_vector[1300][0], 1300);
  EXPECT_EQ (nn_indices_vector[1300][1], 1386);
  EXPECT_EQ (nn_indices_vector[1300][2], 1388);
  EXPECT_EQ (nn_indices_vector[1300][3], 1302);
  EXPECT_EQ (nn_indices_vector[1300][4], 1335);
  EXPECT_EQ (nn_indices_vector[1300][5], 1333);
  EXPECT_EQ (nn_indices_vector[1320][0], 1320);
  EXPECT_EQ (nn_indices_vector[1320][1], 1338);
  EXPECT_EQ (nn_indices_vector[1320][2], 1319);
  EXPECT_EQ (nn_indices_vector[1320][3], 1327);
  EXPECT_EQ (nn_indices_vector[1320][4], 1310);
  EXPECT_EQ (nn_indices_vector[1320][5], 1322);
  EXPECT_EQ (nn_indices_vector[1340][0], 1340);
  EXPECT_EQ (nn_indices_vector[1340][1], 1379);
  EXPECT_EQ (nn_indices_vector[1340][2], 1356);
  EXPECT_EQ (nn_indices_vector[1340][3], 1371);
  EXPECT_EQ (nn_indices_vector[1340][4], 1342);
  EXPECT_EQ (nn_indices_vector[1340][5], 1358);
  EXPECT_EQ (nn_indices_vector[1360][0], 1360);
  EXPECT_EQ (nn_indices_vector[1360][1], 1343);
  EXPECT_EQ (nn_indices_vector[1360][2], 1367);
  EXPECT_EQ (nn_indices_vector[1360][3], 1351);
  EXPECT_EQ (nn_indices_vector[1360][4], 1364);
  EXPECT_EQ (nn_indices_vector[1360][5], 1362);
  EXPECT_EQ (nn_indices_vector[1380][0], 1380);
  EXPECT_EQ (nn_indices_vector[1380][1], 1381);
  EXPECT_EQ (nn_indices_vector[1380][2], 1385);
  EXPECT_EQ (nn_indices_vector[1380][3], 1389);
  EXPECT_EQ (nn_indices_vector[1380][4], 1384);
  EXPECT_EQ (nn_indices_vector[1380][5], 1387);
  EXPECT_EQ (nn_indices_vector[1380][6], 1382);
  EXPECT_EQ (nn_indices_vector[1400][0], 1400);
  EXPECT_EQ (nn_indices_vector[1400][1], 1382);
  EXPECT_EQ (nn_indices_vector[1400][2], 1399);
  EXPECT_EQ (nn_indices_vector[1400][3], 1068);
  EXPECT_EQ (nn_indices_vector[1420][0], 1420);
  EXPECT_EQ (nn_indices_vector[1420][1], 1421);
  EXPECT_EQ (nn_indices_vector[1420][2], 1562);
  EXPECT_EQ (nn_indices_vector[1420][3], 1423);
  EXPECT_EQ (nn_indices_vector[1420][4], 1558);
  EXPECT_EQ (nn_indices_vector[1420][5], 1425);
  EXPECT_EQ (nn_indices_vector[1440][0], 1440);
  EXPECT_EQ (nn_indices_vector[1440][1], 1474);
  EXPECT_EQ (nn_indices_vector[1440][2], 1443);
  EXPECT_EQ (nn_indices_vector[1440][3], 1478);
  EXPECT_EQ (nn_indices_vector[1440][4], 1441);
  EXPECT_EQ (nn_indices_vector[1440][5], 1445);
  EXPECT_EQ (nn_indices_vector[1460][0], 1460);
  EXPECT_EQ (nn_indices_vector[1460][1], 1464);
  EXPECT_EQ (nn_indices_vector[1460][2], 1454);
  EXPECT_EQ (nn_indices_vector[1460][3], 1416);
  EXPECT_EQ (nn_indices_vector[1460][4], 1461);
  EXPECT_EQ (nn_indices_vector[1480][0], 1480);
  EXPECT_EQ (nn_indices_vector[1480][1], 1484);
  EXPECT_EQ (nn_indices_vector[1480][2], 1500);
  EXPECT_EQ (nn_indices_vector[1480][3], 1659);
  EXPECT_EQ (nn_indices_vector[1480][4], 1507);
  EXPECT_EQ (nn_indices_vector[1480][5], 1680);
  EXPECT_EQ (nn_indices_vector[1500][0], 1500);
  EXPECT_EQ (nn_indices_vector[1500][1], 1507);
  EXPECT_EQ (nn_indices_vector[1500][2], 1480);
  EXPECT_EQ (nn_indices_vector[1500][3], 1504);
  EXPECT_EQ (nn_indices_vector[1500][4], 1499);
  EXPECT_EQ (nn_indices_vector[1520][0], 1520);
  EXPECT_EQ (nn_indices_vector[1520][1], 1521);
  EXPECT_EQ (nn_indices_vector[1520][2], 1517);
  EXPECT_EQ (nn_indices_vector[1520][3], 1483);
  EXPECT_EQ (nn_indices_vector[1520][4], 1509);
  EXPECT_EQ (nn_indices_vector[1540][0], 1540);
  EXPECT_EQ (nn_indices_vector[1540][1], 1544);
  EXPECT_EQ (nn_indices_vector[1540][2], 1558);
  EXPECT_EQ (nn_indices_vector[1540][3], 1541);
  EXPECT_EQ (nn_indices_vector[1540][4], 1562);
  EXPECT_EQ (nn_indices_vector[1540][5], 1545);
  EXPECT_EQ (nn_indices_vector[1540][6], 1559);
  EXPECT_EQ (nn_indices_vector[1560][0], 1560);
  EXPECT_EQ (nn_indices_vector[1560][1], 1551);
  EXPECT_EQ (nn_indices_vector[1560][2], 1561);
  EXPECT_EQ (nn_indices_vector[1560][3], 1421);
  EXPECT_EQ (nn_indices_vector[1560][4], 1558);
  EXPECT_EQ (nn_indices_vector[1580][0], 1580);
  EXPECT_EQ (nn_indices_vector[1580][1], 1576);
  EXPECT_EQ (nn_indices_vector[1580][2], 1632);
  EXPECT_EQ (nn_indices_vector[1580][3], 1575);
  EXPECT_EQ (nn_indices_vector[1580][4], 1579);
  EXPECT_EQ (nn_indices_vector[1580][5], 1578);
  EXPECT_EQ (nn_indices_vector[1600][0], 1600);
  EXPECT_EQ (nn_indices_vector[1600][1], 1607);
  EXPECT_EQ (nn_indices_vector[1600][2], 1581);
  EXPECT_EQ (nn_indices_vector[1600][3], 1599);
  EXPECT_EQ (nn_indices_vector[1600][4], 1598);
  EXPECT_EQ (nn_indices_vector[1600][5], 1610);
  EXPECT_EQ (nn_indices_vector[1620][0], 1620);
  EXPECT_EQ (nn_indices_vector[1620][1], 1643);
  EXPECT_EQ (nn_indices_vector[1620][2], 1615);
  EXPECT_EQ (nn_indices_vector[1620][3], 1621);
  EXPECT_EQ (nn_indices_vector[1620][4], 1618);
  EXPECT_EQ (nn_indices_vector[1620][5], 1730);
  EXPECT_EQ (nn_indices_vector[1640][0], 1640);
  EXPECT_EQ (nn_indices_vector[1640][1], 1638);
  EXPECT_EQ (nn_indices_vector[1640][2], 1635);
  EXPECT_EQ (nn_indices_vector[1640][3], 1639);
  EXPECT_EQ (nn_indices_vector[1640][4], 1636);
  EXPECT_EQ (nn_indices_vector[1640][5], 1634);
  EXPECT_EQ (nn_indices_vector[1640][6], 1637);
  EXPECT_EQ (nn_indices_vector[1660][0], 1660);
  EXPECT_EQ (nn_indices_vector[1660][1], 1681);
  EXPECT_EQ (nn_indices_vector[1660][2], 1682);
  EXPECT_EQ (nn_indices_vector[1660][3], 1685);
  EXPECT_EQ (nn_indices_vector[1660][4], 1661);
  EXPECT_EQ (nn_indices_vector[1660][5], 1672);
  EXPECT_EQ (nn_indices_vector[1680][0], 1680);
  EXPECT_EQ (nn_indices_vector[1680][1], 1659);
  EXPECT_EQ (nn_indices_vector[1680][2], 1682);
  EXPECT_EQ (nn_indices_vector[1680][3], 1679);
  EXPECT_EQ (nn_indices_vector[1680][4], 1480);
  EXPECT_EQ (nn_indices_vector[1700][0], 1700);
  EXPECT_EQ (nn_indices_vector[1700][1], 1704);
  EXPECT_EQ (nn_indices_vector[1700][2], 1705);
  EXPECT_EQ (nn_indices_vector[1700][3], 1701);
  EXPECT_EQ (nn_indices_vector[1700][4], 1702);
  EXPECT_EQ (nn_indices_vector[1720][0], 1720);
  EXPECT_EQ (nn_indices_vector[1720][1], 1468);
  EXPECT_EQ (nn_indices_vector[1720][2], 235);
  EXPECT_EQ (nn_indices_vector[1720][3], 631);
  EXPECT_EQ (nn_indices_vector[1720][4], 1437);
  EXPECT_EQ (nn_indices_vector[1740][0], 1740);
  EXPECT_EQ (nn_indices_vector[1740][1], 1807);
  EXPECT_EQ (nn_indices_vector[1740][2], 1805);
  EXPECT_EQ (nn_indices_vector[1740][3], 1804);
  EXPECT_EQ (nn_indices_vector[1740][4], 1741);
  EXPECT_EQ (nn_indices_vector[1740][5], 1806);
  EXPECT_EQ (nn_indices_vector[1760][0], 1760);
  EXPECT_EQ (nn_indices_vector[1760][1], 1745);
  EXPECT_EQ (nn_indices_vector[1760][2], 1759);
  EXPECT_EQ (nn_indices_vector[1760][3], 1770);
  EXPECT_EQ (nn_indices_vector[1760][4], 1758);
  EXPECT_EQ (nn_indices_vector[1780][0], 1780);
  EXPECT_EQ (nn_indices_vector[1780][1], 1935);
  EXPECT_EQ (nn_indices_vector[1780][2], 1779);
  EXPECT_EQ (nn_indices_vector[1780][3], 2089);
  EXPECT_EQ (nn_indices_vector[1800][0], 1800);
  EXPECT_EQ (nn_indices_vector[1800][1], 1812);
  EXPECT_EQ (nn_indices_vector[1800][2], 1789);
  EXPECT_EQ (nn_indices_vector[1800][3], 1801);
  EXPECT_EQ (nn_indices_vector[1820][0], 1820);
  EXPECT_EQ (nn_indices_vector[1820][1], 1821);
  EXPECT_EQ (nn_indices_vector[1820][2], 1822);
  EXPECT_EQ (nn_indices_vector[1820][3], 1819);
  EXPECT_EQ (nn_indices_vector[1820][4], 1818);
  EXPECT_EQ (nn_indices_vector[1840][0], 1840);
  EXPECT_EQ (nn_indices_vector[1840][1], 1838);
  EXPECT_EQ (nn_indices_vector[1840][2], 1839);
  EXPECT_EQ (nn_indices_vector[1840][3], 1837);
  EXPECT_EQ (nn_indices_vector[1860][0], 1860);
  EXPECT_EQ (nn_indices_vector[1860][1], 1854);
  EXPECT_EQ (nn_indices_vector[1860][2], 1858);
  EXPECT_EQ (nn_indices_vector[1860][3], 1879);
  EXPECT_EQ (nn_indices_vector[1860][4], 1853);
  EXPECT_EQ (nn_indices_vector[1860][5], 1877);
  EXPECT_EQ (nn_indices_vector[1880][0], 1880);
  EXPECT_EQ (nn_indices_vector[1880][1], 1882);
  EXPECT_EQ (nn_indices_vector[1880][2], 1881);
  EXPECT_EQ (nn_indices_vector[1880][3], 1879);
  EXPECT_EQ (nn_indices_vector[1880][4], 1877);
  EXPECT_EQ (nn_indices_vector[1880][5], 1878);
  EXPECT_EQ (nn_indices_vector[1900][0], 1900);
  EXPECT_EQ (nn_indices_vector[1900][1], 1903);
  EXPECT_EQ (nn_indices_vector[1900][2], 1824);
  EXPECT_EQ (nn_indices_vector[1920][0], 1920);
  EXPECT_EQ (nn_indices_vector[1920][1], 1768);
  EXPECT_EQ (nn_indices_vector[1920][2], 1895);
  EXPECT_EQ (nn_indices_vector[1920][3], 1744);
  EXPECT_EQ (nn_indices_vector[1920][4], 1919);
  EXPECT_EQ (nn_indices_vector[1940][0], 1940);
  EXPECT_EQ (nn_indices_vector[1940][1], 1850);
  EXPECT_EQ (nn_indices_vector[1940][2], 1848);
  EXPECT_EQ (nn_indices_vector[1940][3], 1938);
  EXPECT_EQ (nn_indices_vector[1960][0], 1960);
  EXPECT_EQ (nn_indices_vector[1960][1], 1961);
  EXPECT_EQ (nn_indices_vector[1960][2], 1957);
  EXPECT_EQ (nn_indices_vector[1960][3], 1962);
  EXPECT_EQ (nn_indices_vector[1980][0], 1980);
  EXPECT_EQ (nn_indices_vector[1980][1], 1989);
  EXPECT_EQ (nn_indices_vector[1980][2], 2102);
  EXPECT_EQ (nn_indices_vector[1980][3], 1993);
  EXPECT_EQ (nn_indices_vector[1980][4], 1983);
  EXPECT_EQ (nn_indices_vector[1980][5], 2098);
  EXPECT_EQ (nn_indices_vector[2000][0], 2000);
  EXPECT_EQ (nn_indices_vector[2000][1], 2001);
  EXPECT_EQ (nn_indices_vector[2000][2], 1990);
  EXPECT_EQ (nn_indices_vector[2000][3], 2004);
  EXPECT_EQ (nn_indices_vector[2000][4], 2002);
  EXPECT_EQ (nn_indices_vector[2000][5], 2005);
  EXPECT_EQ (nn_indices_vector[2020][0], 2020);
  EXPECT_EQ (nn_indices_vector[2020][1], 2024);
  EXPECT_EQ (nn_indices_vector[2020][2], 2019);
  EXPECT_EQ (nn_indices_vector[2020][3], 2018);
  EXPECT_EQ (nn_indices_vector[2020][4], 2062);
  EXPECT_EQ (nn_indices_vector[2040][0], 2040);
  EXPECT_EQ (nn_indices_vector[2040][1], 2036);
  EXPECT_EQ (nn_indices_vector[2040][2], 2034);
  EXPECT_EQ (nn_indices_vector[2040][3], 2041);
  EXPECT_EQ (nn_indices_vector[2040][4], 2033);
  EXPECT_EQ (nn_indices_vector[2040][5], 2252);
  EXPECT_EQ (nn_indices_vector[2040][6], 2038);
  EXPECT_EQ (nn_indices_vector[2060][0], 2060);
  EXPECT_EQ (nn_indices_vector[2060][1], 2049);
  EXPECT_EQ (nn_indices_vector[2060][2], 2058);
  EXPECT_EQ (nn_indices_vector[2060][3], 2054);
  EXPECT_EQ (nn_indices_vector[2060][4], 2056);
  EXPECT_EQ (nn_indices_vector[2060][5], 2053);
  EXPECT_EQ (nn_indices_vector[2060][6], 2051);
  EXPECT_EQ (nn_indices_vector[2080][0], 2080);
  EXPECT_EQ (nn_indices_vector[2080][1], 2079);
  EXPECT_EQ (nn_indices_vector[2080][2], 2084);
  EXPECT_EQ (nn_indices_vector[2080][3], 2072);
  EXPECT_EQ (nn_indices_vector[2080][4], 2110);
  EXPECT_EQ (nn_indices_vector[2100][0], 2100);
  EXPECT_EQ (nn_indices_vector[2100][1], 2096);
  EXPECT_EQ (nn_indices_vector[2100][2], 2101);
  EXPECT_EQ (nn_indices_vector[2100][3], 1981);
  EXPECT_EQ (nn_indices_vector[2100][4], 2102);
  EXPECT_EQ (nn_indices_vector[2100][5], 2090);
  EXPECT_EQ (nn_indices_vector[2120][0], 2120);
  EXPECT_EQ (nn_indices_vector[2120][1], 2030);
  EXPECT_EQ (nn_indices_vector[2120][2], 2122);
  EXPECT_EQ (nn_indices_vector[2120][3], 2006);
  EXPECT_EQ (nn_indices_vector[2120][4], 2119);
  EXPECT_EQ (nn_indices_vector[2140][0], 2140);
  EXPECT_EQ (nn_indices_vector[2140][1], 2216);
  EXPECT_EQ (nn_indices_vector[2140][2], 2139);
  EXPECT_EQ (nn_indices_vector[2140][3], 2146);
  EXPECT_EQ (nn_indices_vector[2140][4], 2149);
  EXPECT_EQ (nn_indices_vector[2160][0], 2160);
  EXPECT_EQ (nn_indices_vector[2160][1], 2192);
  EXPECT_EQ (nn_indices_vector[2160][2], 2158);
  EXPECT_EQ (nn_indices_vector[2160][3], 2161);
  EXPECT_EQ (nn_indices_vector[2160][4], 2193);
  EXPECT_EQ (nn_indices_vector[2160][5], 2159);
  EXPECT_EQ (nn_indices_vector[2180][0], 2180);
  EXPECT_EQ (nn_indices_vector[2180][1], 1996);
  EXPECT_EQ (nn_indices_vector[2180][2], 2187);
  EXPECT_EQ (nn_indices_vector[2180][3], 2188);
  EXPECT_EQ (nn_indices_vector[2180][4], 2184);
  EXPECT_EQ (nn_indices_vector[2200][0], 2200);
  EXPECT_EQ (nn_indices_vector[2200][1], 2201);
  EXPECT_EQ (nn_indices_vector[2200][2], 2202);
  EXPECT_EQ (nn_indices_vector[2200][3], 2203);
  EXPECT_EQ (nn_indices_vector[2200][4], 2209);
  EXPECT_EQ (nn_indices_vector[2200][5], 2208);
  EXPECT_EQ (nn_indices_vector[2220][0], 2220);
  EXPECT_EQ (nn_indices_vector[2220][1], 2144);
  EXPECT_EQ (nn_indices_vector[2220][2], 2139);
  EXPECT_EQ (nn_indices_vector[2220][3], 2214);
  EXPECT_EQ (nn_indices_vector[2220][4], 2134);
  EXPECT_EQ (nn_indices_vector[2220][5], 2177);
  EXPECT_EQ (nn_indices_vector[2240][0], 2240);
  EXPECT_EQ (nn_indices_vector[2240][1], 2236);
  EXPECT_EQ (nn_indices_vector[2240][2], 2241);
  EXPECT_EQ (nn_indices_vector[2240][3], 2285);
  EXPECT_EQ (nn_indices_vector[2240][4], 2234);
  EXPECT_EQ (nn_indices_vector[2240][5], 2238);
  EXPECT_EQ (nn_indices_vector[2240][6], 2233);
  EXPECT_EQ (nn_indices_vector[2260][0], 2260);
  EXPECT_EQ (nn_indices_vector[2260][1], 2265);
  EXPECT_EQ (nn_indices_vector[2260][2], 2262);
  EXPECT_EQ (nn_indices_vector[2260][3], 2264);
  EXPECT_EQ (nn_indices_vector[2260][4], 2261);
  EXPECT_EQ (nn_indices_vector[2260][5], 2266);
  EXPECT_EQ (nn_indices_vector[2280][0], 2280);
  EXPECT_EQ (nn_indices_vector[2280][1], 2276);
  EXPECT_EQ (nn_indices_vector[2280][2], 2279);
  EXPECT_EQ (nn_indices_vector[2280][3], 2277);
  EXPECT_EQ (nn_indices_vector[2280][4], 2290);
  EXPECT_EQ (nn_indices_vector[2280][5], 2275);
  EXPECT_EQ (nn_indices_vector[2280][6], 2278);
  EXPECT_EQ (nn_indices_vector[2300][0], 2300);
  EXPECT_EQ (nn_indices_vector[2300][1], 2299);
  EXPECT_EQ (nn_indices_vector[2300][2], 2304);
  EXPECT_EQ (nn_indices_vector[2300][3], 2302);
  EXPECT_EQ (nn_indices_vector[2320][0], 2320);
  EXPECT_EQ (nn_indices_vector[2320][1], 2316);
  EXPECT_EQ (nn_indices_vector[2320][2], 2326);
  EXPECT_EQ (nn_indices_vector[2320][3], 2318);
  EXPECT_EQ (nn_indices_vector[2340][0], 2340);
  EXPECT_EQ (nn_indices_vector[2340][1], 2341);
  EXPECT_EQ (nn_indices_vector[2340][2], 2342);
  EXPECT_EQ (nn_indices_vector[2340][3], 2343);
  EXPECT_EQ (nn_indices_vector[2340][4], 2337);
  EXPECT_EQ (nn_indices_vector[2360][0], 2360);
  EXPECT_EQ (nn_indices_vector[2360][1], 2361);
  EXPECT_EQ (nn_indices_vector[2360][2], 2380);
  EXPECT_EQ (nn_indices_vector[2360][3], 2357);
  EXPECT_EQ (nn_indices_vector[2360][4], 2444);
  EXPECT_EQ (nn_indices_vector[2360][5], 2489);
  EXPECT_EQ (nn_indices_vector[2380][0], 2380);
  EXPECT_EQ (nn_indices_vector[2380][1], 2489);
  EXPECT_EQ (nn_indices_vector[2380][2], 2361);
  EXPECT_EQ (nn_indices_vector[2380][3], 2381);
  EXPECT_EQ (nn_indices_vector[2380][4], 2360);
  EXPECT_EQ (nn_indices_vector[2380][5], 2490);
  EXPECT_EQ (nn_indices_vector[2400][0], 2400);
  EXPECT_EQ (nn_indices_vector[2400][1], 2399);
  EXPECT_EQ (nn_indices_vector[2400][2], 2404);
  EXPECT_EQ (nn_indices_vector[2400][3], 2403);
  EXPECT_EQ (nn_indices_vector[2400][4], 2393);
  EXPECT_EQ (nn_indices_vector[2420][0], 2420);
  EXPECT_EQ (nn_indices_vector[2420][1], 2421);
  EXPECT_EQ (nn_indices_vector[2420][2], 2424);
  EXPECT_EQ (nn_indices_vector[2420][3], 2411);
  EXPECT_EQ (nn_indices_vector[2420][4], 2425);
  EXPECT_EQ (nn_indices_vector[2440][0], 2440);
  EXPECT_EQ (nn_indices_vector[2440][1], 2442);
  EXPECT_EQ (nn_indices_vector[2440][2], 2441);
  EXPECT_EQ (nn_indices_vector[2460][0], 2460);
  EXPECT_EQ (nn_indices_vector[2460][1], 2459);
  EXPECT_EQ (nn_indices_vector[2460][2], 2463);
  EXPECT_EQ (nn_indices_vector[2460][3], 2464);
  EXPECT_EQ (nn_indices_vector[2480][0], 2480);
  EXPECT_EQ (nn_indices_vector[2480][1], 2504);
  EXPECT_EQ (nn_indices_vector[2480][2], 2486);
  EXPECT_EQ (nn_indices_vector[2480][3], 2505);
  EXPECT_EQ (nn_indices_vector[2480][4], 2485);
  EXPECT_EQ (nn_indices_vector[2480][5], 2508);
  EXPECT_EQ (nn_indices_vector[2480][6], 2479);
  EXPECT_EQ (nn_indices_vector[2500][0], 2500);
  EXPECT_EQ (nn_indices_vector[2500][1], 2501);
  EXPECT_EQ (nn_indices_vector[2500][2], 2499);
  EXPECT_EQ (nn_indices_vector[2500][3], 2498);
  EXPECT_EQ (nn_indices_vector[2520][0], 2520);
  EXPECT_EQ (nn_indices_vector[2520][1], 2521);
  EXPECT_EQ (nn_indices_vector[2520][2], 2517);
  EXPECT_EQ (nn_indices_vector[2520][3], 2523);
  EXPECT_EQ (nn_indices_vector[2520][4], 2421);
  EXPECT_EQ (nn_indices_vector[2520][5], 2522);
  EXPECT_EQ (nn_indices_vector[2540][0], 2540);
  EXPECT_EQ (nn_indices_vector[2540][1], 2541);
  EXPECT_EQ (nn_indices_vector[2540][2], 2539);
  EXPECT_EQ (nn_indices_vector[2540][3], 2538);
  EXPECT_EQ (nn_indices_vector[2540][4], 2536);
  EXPECT_EQ (nn_indices_vector[2540][5], 2552);
  EXPECT_EQ (nn_indices_vector[2540][6], 2511);
  EXPECT_EQ (nn_indices_vector[2540][7], 2542);
  EXPECT_EQ (nn_indices_vector[2560][0], 2560);
  EXPECT_EQ (nn_indices_vector[2560][1], 2404);
  EXPECT_EQ (nn_indices_vector[2560][2], 2403);
  EXPECT_EQ (nn_indices_vector[2580][0], 2580);
  EXPECT_EQ (nn_indices_vector[2580][1], 2583);
  EXPECT_EQ (nn_indices_vector[2580][2], 2584);
  EXPECT_EQ (nn_indices_vector[2580][3], 2624);
  EXPECT_EQ (nn_indices_vector[2580][4], 2619);
  EXPECT_EQ (nn_indices_vector[2580][5], 2579);
  EXPECT_EQ (nn_indices_vector[2600][0], 2600);
  EXPECT_EQ (nn_indices_vector[2600][1], 2595);
  EXPECT_EQ (nn_indices_vector[2600][2], 2593);
  EXPECT_EQ (nn_indices_vector[2600][3], 2599);
  EXPECT_EQ (nn_indices_vector[2600][4], 2596);
  EXPECT_EQ (nn_indices_vector[2600][5], 2594);
  EXPECT_EQ (nn_indices_vector[2620][0], 2620);
  EXPECT_EQ (nn_indices_vector[2620][1], 2623);
  EXPECT_EQ (nn_indices_vector[2620][2], 2621);
  EXPECT_EQ (nn_indices_vector[2620][3], 2625);
  EXPECT_EQ (nn_indices_vector[2620][4], 2626);
  EXPECT_EQ (nn_indices_vector[2640][0], 2640);
  EXPECT_EQ (nn_indices_vector[2640][1], 2646);
  EXPECT_EQ (nn_indices_vector[2640][2], 2637);
  EXPECT_EQ (nn_indices_vector[2640][3], 2592);
  EXPECT_EQ (nn_indices_vector[2640][4], 2656);
  EXPECT_EQ (nn_indices_vector[2640][5], 2639);
  EXPECT_EQ (nn_indices_vector[2660][0], 2660);
  EXPECT_EQ (nn_indices_vector[2660][1], 2661);
  EXPECT_EQ (nn_indices_vector[2660][2], 438);
  EXPECT_EQ (nn_indices_vector[2660][3], 2044);
  EXPECT_EQ (nn_indices_vector[2680][0], 2680);
  EXPECT_EQ (nn_indices_vector[2680][1], 2679);
  EXPECT_EQ (nn_indices_vector[2680][2], 2688);
  EXPECT_EQ (nn_indices_vector[2680][3], 2677);
  EXPECT_EQ (nn_indices_vector[2680][4], 3152);
  EXPECT_EQ (nn_indices_vector[2700][0], 2700);
  EXPECT_EQ (nn_indices_vector[2700][1], 2704);
  EXPECT_EQ (nn_indices_vector[2700][2], 2830);
  EXPECT_EQ (nn_indices_vector[2700][3], 2706);
  EXPECT_EQ (nn_indices_vector[2700][4], 2826);
  EXPECT_EQ (nn_indices_vector[2700][5], 2702);
  EXPECT_EQ (nn_indices_vector[2720][0], 2720);
  EXPECT_EQ (nn_indices_vector[2720][1], 2721);
  EXPECT_EQ (nn_indices_vector[2720][2], 2690);
  EXPECT_EQ (nn_indices_vector[2720][3], 2150);
  EXPECT_EQ (nn_indices_vector[2720][4], 2722);
  EXPECT_EQ (nn_indices_vector[2740][0], 2740);
  EXPECT_EQ (nn_indices_vector[2740][1], 2738);
  EXPECT_EQ (nn_indices_vector[2740][2], 2747);
  EXPECT_EQ (nn_indices_vector[2740][3], 2739);
  EXPECT_EQ (nn_indices_vector[2740][4], 2798);
  EXPECT_EQ (nn_indices_vector[2760][0], 2760);
  EXPECT_EQ (nn_indices_vector[2760][1], 2756);
  EXPECT_EQ (nn_indices_vector[2760][2], 2761);
  EXPECT_EQ (nn_indices_vector[2760][3], 2767);
  EXPECT_EQ (nn_indices_vector[2760][4], 2730);
  EXPECT_EQ (nn_indices_vector[2760][5], 2729);
  EXPECT_EQ (nn_indices_vector[2760][6], 2758);
  EXPECT_EQ (nn_indices_vector[2780][0], 2780);
  EXPECT_EQ (nn_indices_vector[2780][1], 2789);
  EXPECT_EQ (nn_indices_vector[2780][2], 2802);
  EXPECT_EQ (nn_indices_vector[2780][3], 2779);
  EXPECT_EQ (nn_indices_vector[2800][0], 2800);
  EXPECT_EQ (nn_indices_vector[2800][1], 2804);
  EXPECT_EQ (nn_indices_vector[2800][2], 2798);
  EXPECT_EQ (nn_indices_vector[2800][3], 2799);
  EXPECT_EQ (nn_indices_vector[2800][4], 2807);
  EXPECT_EQ (nn_indices_vector[2820][0], 2820);
  EXPECT_EQ (nn_indices_vector[2820][1], 2821);
  EXPECT_EQ (nn_indices_vector[2820][2], 2810);
  EXPECT_EQ (nn_indices_vector[2820][3], 2890);
  EXPECT_EQ (nn_indices_vector[2820][4], 2818);
  EXPECT_EQ (nn_indices_vector[2840][0], 2840);
  EXPECT_EQ (nn_indices_vector[2840][1], 2837);
  EXPECT_EQ (nn_indices_vector[2840][2], 2835);
  EXPECT_EQ (nn_indices_vector[2840][3], 2831);
  EXPECT_EQ (nn_indices_vector[2840][4], 2839);
  EXPECT_EQ (nn_indices_vector[2840][5], 2834);
  EXPECT_EQ (nn_indices_vector[2840][6], 2832);
  EXPECT_EQ (nn_indices_vector[2860][0], 2860);
  EXPECT_EQ (nn_indices_vector[2860][1], 2859);
  EXPECT_EQ (nn_indices_vector[2860][2], 2884);
  EXPECT_EQ (nn_indices_vector[2860][3], 2862);
  EXPECT_EQ (nn_indices_vector[2880][0], 2880);
  EXPECT_EQ (nn_indices_vector[2880][1], 2882);
  EXPECT_EQ (nn_indices_vector[2880][2], 2878);
  EXPECT_EQ (nn_indices_vector[2880][3], 2808);
  EXPECT_EQ (nn_indices_vector[2900][0], 2900);
  EXPECT_EQ (nn_indices_vector[2900][1], 2897);
  EXPECT_EQ (nn_indices_vector[2900][2], 3077);
  EXPECT_EQ (nn_indices_vector[2900][3], 2902);
  EXPECT_EQ (nn_indices_vector[2920][0], 2920);
  EXPECT_EQ (nn_indices_vector[2920][1], 2909);
  EXPECT_EQ (nn_indices_vector[2920][2], 2921);
  EXPECT_EQ (nn_indices_vector[2940][0], 2940);
  EXPECT_EQ (nn_indices_vector[2940][1], 2469);
  EXPECT_EQ (nn_indices_vector[2940][2], 2939);
  EXPECT_EQ (nn_indices_vector[2940][3], 2943);
  EXPECT_EQ (nn_indices_vector[2940][4], 2944);
  EXPECT_EQ (nn_indices_vector[2940][5], 2468);
  EXPECT_EQ (nn_indices_vector[2940][6], 2471);
  EXPECT_EQ (nn_indices_vector[2960][0], 2960);
  EXPECT_EQ (nn_indices_vector[2960][1], 3008);
  EXPECT_EQ (nn_indices_vector[2960][2], 2959);
  EXPECT_EQ (nn_indices_vector[2960][3], 2966);
  EXPECT_EQ (nn_indices_vector[2960][4], 3009);
  EXPECT_EQ (nn_indices_vector[2960][5], 2965);
  EXPECT_EQ (nn_indices_vector[2960][6], 3012);
  EXPECT_EQ (nn_indices_vector[2980][0], 2980);
  EXPECT_EQ (nn_indices_vector[2980][1], 2981);
  EXPECT_EQ (nn_indices_vector[2980][2], 2984);
  EXPECT_EQ (nn_indices_vector[2980][3], 2972);
  EXPECT_EQ (nn_indices_vector[3000][0], 3000);
  EXPECT_EQ (nn_indices_vector[3000][1], 2986);
  EXPECT_EQ (nn_indices_vector[3000][2], 2997);
  EXPECT_EQ (nn_indices_vector[3000][3], 3001);
  EXPECT_EQ (nn_indices_vector[3020][0], 3020);
  EXPECT_EQ (nn_indices_vector[3020][1], 3004);
  EXPECT_EQ (nn_indices_vector[3020][2], 3021);
  EXPECT_EQ (nn_indices_vector[3020][3], 3005);
  EXPECT_EQ (nn_indices_vector[3020][4], 3136);
  EXPECT_EQ (nn_indices_vector[3040][0], 3040);
  EXPECT_EQ (nn_indices_vector[3040][1], 3044);
  EXPECT_EQ (nn_indices_vector[3040][2], 3047);
  EXPECT_EQ (nn_indices_vector[3040][3], 3050);
  EXPECT_EQ (nn_indices_vector[3040][4], 3042);
  EXPECT_EQ (nn_indices_vector[3060][0], 3060);
  EXPECT_EQ (nn_indices_vector[3060][1], 3069);
  EXPECT_EQ (nn_indices_vector[3060][2], 3074);
  EXPECT_EQ (nn_indices_vector[3060][3], 3059);
  EXPECT_EQ (nn_indices_vector[3060][4], 3057);
  EXPECT_EQ (nn_indices_vector[3080][0], 3080);
  EXPECT_EQ (nn_indices_vector[3080][1], 3076);
  EXPECT_EQ (nn_indices_vector[3080][2], 3081);
  EXPECT_EQ (nn_indices_vector[3100][0], 3100);
  EXPECT_EQ (nn_indices_vector[3100][1], 3097);
  EXPECT_EQ (nn_indices_vector[3100][2], 3101);
  EXPECT_EQ (nn_indices_vector[3100][3], 3098);
  EXPECT_EQ (nn_indices_vector[3100][4], 3105);
  EXPECT_EQ (nn_indices_vector[3100][5], 3099);
  EXPECT_EQ (nn_indices_vector[3120][0], 3120);
  EXPECT_EQ (nn_indices_vector[3120][1], 3132);
  EXPECT_EQ (nn_indices_vector[3120][2], 3135);
  EXPECT_EQ (nn_indices_vector[3120][3], 3119);
  EXPECT_EQ (nn_indices_vector[3120][4], 3127);
  EXPECT_EQ (nn_indices_vector[3140][0], 3140);
  EXPECT_EQ (nn_indices_vector[3140][1], 3147);
  EXPECT_EQ (nn_indices_vector[3140][2], 3141);
  EXPECT_EQ (nn_indices_vector[3140][3], 3109);
  EXPECT_EQ (nn_indices_vector[3140][4], 3144);
  EXPECT_EQ (nn_indices_vector[3140][5], 3101);
  EXPECT_EQ (nn_indices_vector[3140][6], 3137);
  EXPECT_EQ (nn_indices_vector[3160][0], 3160);
  EXPECT_EQ (nn_indices_vector[3160][1], 3168);
  EXPECT_EQ (nn_indices_vector[3160][2], 3159);
  EXPECT_EQ (nn_indices_vector[3160][3], 3169);
  EXPECT_EQ (nn_indices_vector[3160][4], 3158);
  EXPECT_EQ (nn_indices_vector[3180][0], 3180);
  EXPECT_EQ (nn_indices_vector[3180][1], 3178);
  EXPECT_EQ (nn_indices_vector[3180][2], 3184);
  EXPECT_EQ (nn_indices_vector[3180][3], 3181);
  EXPECT_EQ (nn_indices_vector[3200][0], 3200);
  EXPECT_EQ (nn_indices_vector[3200][1], 3199);
  EXPECT_EQ (nn_indices_vector[3200][2], 3201);
  EXPECT_EQ (nn_indices_vector[3200][3], 3164);
  EXPECT_EQ (nn_indices_vector[3220][0], 3220);
  EXPECT_EQ (nn_indices_vector[3220][1], 3216);
  EXPECT_EQ (nn_indices_vector[3220][2], 3221);
  EXPECT_EQ (nn_indices_vector[3220][3], 3192);
  EXPECT_EQ (nn_indices_vector[3220][4], 3218);
  EXPECT_EQ (nn_indices_vector[3240][0], 3240);
  EXPECT_EQ (nn_indices_vector[3240][1], 3237);
  EXPECT_EQ (nn_indices_vector[3240][2], 3241);
  EXPECT_EQ (nn_indices_vector[3240][3], 3235);
  EXPECT_EQ (nn_indices_vector[3240][4], 3239);
  EXPECT_EQ (nn_indices_vector[3240][5], 3238);
  EXPECT_EQ (nn_indices_vector[3240][6], 3261);
  EXPECT_EQ (nn_indices_vector[3260][0], 3260);
  EXPECT_EQ (nn_indices_vector[3260][1], 3217);
  EXPECT_EQ (nn_indices_vector[3260][2], 3259);
  EXPECT_EQ (nn_indices_vector[3280][0], 3280);
  EXPECT_EQ (nn_indices_vector[3280][1], 3254);
  EXPECT_EQ (nn_indices_vector[3280][2], 3279);
  EXPECT_EQ (nn_indices_vector[3280][3], 3282);
  EXPECT_EQ (nn_indices_vector[3280][4], 2936);
  EXPECT_EQ (nn_indices_vector[3280][5], 3124);
}

/* ---[ */
int
main (int argc, char** argv)
{
  // Load the standard PCD file from disk
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `sac_plane_test.pcd` and pass it path to the test." << std::endl;
    return (-1);
  }

  // Load in the point clouds
  io::loadPCDFile (argv[1], *cloud_in);

  testing::InitGoogleTest (&argc, argv);


  init ();
  return (RUN_ALL_TESTS ());
}
/* ]--- */
