/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 */


#include <vector>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/brute_force.h>

using namespace std;

int
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    pcl::console::print_info ("Syntax is: %s [-pcd <pcd-file>] (-radius <radius> [-knn <k>] | -knn <k> )\n", argv[0]);
    return (1);
  }

  string pcd_path;
  bool use_pcd_file = pcl::console::find_switch (argc, argv, "-pcd");
  if (use_pcd_file)
    pcl::console::parse (argc, argv, "-pcd", pcd_path);

  float radius = -1;
  if (pcl::console::find_switch (argc, argv, "-radius"))
    pcl::console::parse (argc, argv, "-radius", radius);

  int k = -1;
  if (pcl::console::find_switch (argc, argv, "-knn"))
    pcl::console::parse (argc, argv, "-knn", k);

  if (radius < 0 && k < 0)
  {
    cout << "please specify at least one of the options -radius and -knn" << endl;
    return (1);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (use_pcd_file)
    pcl::io::loadPCDFile (pcd_path, *cloud);
  else
  {
    cloud->resize (1000000);
    for (size_t idx = 0; idx < cloud->size (); ++idx)
    {
      (*cloud)[idx].x = static_cast<float> (rand () / RAND_MAX);
      (*cloud)[idx].y = static_cast<float> (rand () / RAND_MAX);
      (*cloud)[idx].z = static_cast<float> (rand () / RAND_MAX);
    }
  }

  pcl::search::KdTree<pcl::PointXYZRGB> tree;
  pcl::PointXYZRGB query;
  query.x = 0.5;
  query.y = 0.5;
  query.z = 0.5;

  vector<int> kd_indices;
  vector<float> kd_distances;
  vector<int> bf_indices;
  vector<float> bf_distances;

  double start, stop;
  double kd_setup;
  double kd_search;
  double bf_setup;
  double bf_search;

  if (k > 0)
  {
    start = pcl::getTime ();
    tree.setInputCloud (cloud);
    stop = pcl::getTime ();
    cout << "setting up kd tree: " << (kd_setup = stop - start) << endl;

    start = pcl::getTime ();
    tree.nearestKSearchT (query, k, kd_indices, kd_distances);
    stop = pcl::getTime ();
    cout << "single search with kd tree; " << (kd_search = stop - start) << " :: " << kd_indices[0] << " , " << kd_distances [0] << endl;

    pcl::search::BruteForce<pcl::PointXYZRGB> brute_force;
    start = pcl::getTime ();
    brute_force.setInputCloud (cloud);
    stop = pcl::getTime ();
    cout << "setting up brute force search: " << (bf_setup = stop - start) << endl;

    start = pcl::getTime ();
    brute_force.nearestKSearchT (query, k, bf_indices, bf_distances);
    stop = pcl::getTime ();
    cout << "single search with brute force; " << (bf_search = stop - start) << " :: " << bf_indices[0] << " , " << bf_distances [0] << endl;
    cout << "amortization after searches: " << (kd_setup - bf_setup) / (bf_search - kd_search) << endl;
  }
  else
  {
    start = pcl::getTime ();
    tree.setInputCloud (cloud);
    stop = pcl::getTime ();
    cout << "setting up kd tree: " << (kd_setup = stop - start) << endl;

    start = pcl::getTime ();
    tree.radiusSearch (query, radius, kd_indices, kd_distances, k);
    stop = pcl::getTime ();
    cout << "single search with kd tree; " << (kd_search = stop - start) << " :: " << kd_indices[0] << " , " << kd_distances [0] << endl;

    pcl::search::BruteForce<pcl::PointXYZRGB> brute_force;
    start = pcl::getTime ();
    brute_force.setInputCloud (cloud);
    stop = pcl::getTime ();
    cout << "setting up brute force search: " << (bf_setup = stop - start) << endl;

    start = pcl::getTime ();
    brute_force.radiusSearch (query, radius, bf_indices, bf_distances, k);
    stop = pcl::getTime ();
    cout << "single search with brute force; " << (bf_search = stop - start) << " :: " << bf_indices[0] << " , " << bf_distances [0] << endl;
    cout << "amortization after searches: " << (kd_setup - bf_setup) / (bf_search - kd_search) << endl;
  }

  if (kd_indices.size () != bf_indices.size ())
  {
    cerr << "size of results do not match " <<kd_indices.size () << " vs. " << bf_indices.size () << endl;
  }
  else
  {
    cerr << "size of result: " <<kd_indices.size () << endl;
    for (size_t idx = 0; idx < kd_indices.size (); ++idx)
    {
      if (kd_indices[idx] != bf_indices[idx] && kd_distances[idx] != bf_distances[idx])
      {
        cerr << "results do not match: " << idx << " nearest neighbor: "
                << kd_indices[idx] << " with distance: " << kd_distances[idx] << " vs. "
                << bf_indices[idx] << " with distance: " << bf_distances[idx] << endl;
      }
    }
  }
  return (0);
}
