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
    pcl::console::print_info ("Syntax is: %s -pcd <pcd-file> (-radius <radius> | -knn <k> )\n", argv[0]);
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

  if ((radius * k) > 0)
  {
    cout << "please specify only one of the options -radius and -knn" << endl;
    return (1);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (use_pcd_file)
    pcl::io::loadPCDFile (pcd_path, *cloud);
  else
  {
    cloud->resize (1000000);
    for (unsigned idx = 0; idx < cloud->size (); ++idx)
    {
      (*cloud)[idx].x = (double)rand () / (double)RAND_MAX;
      (*cloud)[idx].y = (double)rand () / (double)RAND_MAX;
      (*cloud)[idx].z = (double)rand () / (double)RAND_MAX;
    }
  }

  pcl::search::KdTree<pcl::PointXYZRGB> tree;
  pcl::PointXYZRGB query;
  query.x = 0.5;
  query.y = 0.5;
  query.z = 0.5;

  vector<int> kd_indices;
  vector<float> kd_distances;

  double start, stop;
  double kd_setup;
  double kd_search;
  double bf_setup;
  double bf_search;

  start = pcl::getTime ();
  tree.setInputCloud (cloud);
  stop = pcl::getTime ();
  cout << "setting up kd tree: " << (kd_setup = stop - start) << endl;
  start = pcl::getTime ();
  tree.nearestKSearchT (query, k, kd_indices, kd_distances);
  stop = pcl::getTime ();
  cout << "single search with kd tree; " << (kd_search = stop - start) << " :: " << kd_indices[0] << " , " << kd_distances [0] << endl;

  vector<int> bf_indices;
  vector<float> bf_distances;
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

  if (kd_indices.size () != bf_indices.size ())
  {
    cerr << "number does not match" << endl;
  }
  else
  {
    for (unsigned idx = 0; idx < kd_indices.size (); ++idx)
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
