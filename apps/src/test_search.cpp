#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/brute_force.h>
#include <pcl/search/kdtree.h>

#include <string>
#include <vector>

int
main(int argc, char** argv)
{
  if (argc < 2) {
    pcl::console::print_info(
        "Syntax is: %s [-pcd <pcd-file>] (-radius <radius> [-knn <k>] | -knn <k> )\n",
        argv[0]);
    return 1;
  }

  std::string pcd_path;
  bool use_pcd_file = pcl::console::find_switch(argc, argv, "-pcd");
  if (use_pcd_file)
    pcl::console::parse(argc, argv, "-pcd", pcd_path);

  float radius = -1;
  if (pcl::console::find_switch(argc, argv, "-radius"))
    pcl::console::parse(argc, argv, "-radius", radius);

  int k = -1;
  if (pcl::console::find_switch(argc, argv, "-knn"))
    pcl::console::parse(argc, argv, "-knn", k);

  if (radius < 0 && k < 0) {
    std::cout << "please specify at least one of the options -radius and -knn"
              << std::endl;
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  if (use_pcd_file)
    pcl::io::loadPCDFile(pcd_path, *cloud);
  else {
    cloud->resize(1000000);
    for (std::size_t idx = 0; idx < cloud->size(); ++idx) {
      (*cloud)[idx].x = static_cast<float>(rand() / RAND_MAX);
      (*cloud)[idx].y = static_cast<float>(rand() / RAND_MAX);
      (*cloud)[idx].z = static_cast<float>(rand() / RAND_MAX);
    }
  }

  pcl::search::KdTree<pcl::PointXYZRGB> tree;
  pcl::PointXYZRGB query;
  query.x = 0.5;
  query.y = 0.5;
  query.z = 0.5;

  pcl::Indices kd_indices;
  std::vector<float> kd_distances;
  pcl::Indices bf_indices;
  std::vector<float> bf_distances;

  double start, stop;
  double kd_setup;
  double kd_search;
  double bf_setup;
  double bf_search;

  if (k > 0) {
    start = pcl::getTime();
    tree.setInputCloud(cloud);
    stop = pcl::getTime();
    std::cout << "setting up kd tree: " << (kd_setup = stop - start) << std::endl;

    start = pcl::getTime();
    tree.nearestKSearchT(query, k, kd_indices, kd_distances);
    stop = pcl::getTime();
    std::cout << "single search with kd tree; " << (kd_search = stop - start)
              << " :: " << kd_indices[0] << " , " << kd_distances[0] << std::endl;

    pcl::search::BruteForce<pcl::PointXYZRGB> brute_force;
    start = pcl::getTime();
    brute_force.setInputCloud(cloud);
    stop = pcl::getTime();
    std::cout << "setting up brute force search: " << (bf_setup = stop - start)
              << std::endl;

    start = pcl::getTime();
    brute_force.nearestKSearchT(query, k, bf_indices, bf_distances);
    stop = pcl::getTime();
    std::cout << "single search with brute force; " << (bf_search = stop - start)
              << " :: " << bf_indices[0] << " , " << bf_distances[0] << std::endl;
    std::cout << "amortization after searches: "
              << (kd_setup - bf_setup) / (bf_search - kd_search) << std::endl;
  }
  else {
    start = pcl::getTime();
    tree.setInputCloud(cloud);
    stop = pcl::getTime();
    std::cout << "setting up kd tree: " << (kd_setup = stop - start) << std::endl;

    start = pcl::getTime();
    tree.radiusSearch(query, radius, kd_indices, kd_distances, k);
    stop = pcl::getTime();
    std::cout << "single search with kd tree; " << (kd_search = stop - start)
              << " :: " << kd_indices[0] << " , " << kd_distances[0] << std::endl;

    pcl::search::BruteForce<pcl::PointXYZRGB> brute_force;
    start = pcl::getTime();
    brute_force.setInputCloud(cloud);
    stop = pcl::getTime();
    std::cout << "setting up brute force search: " << (bf_setup = stop - start)
              << std::endl;

    start = pcl::getTime();
    brute_force.radiusSearch(query, radius, bf_indices, bf_distances, k);
    stop = pcl::getTime();
    std::cout << "single search with brute force; " << (bf_search = stop - start)
              << " :: " << bf_indices[0] << " , " << bf_distances[0] << std::endl;
    std::cout << "amortization after searches: "
              << (kd_setup - bf_setup) / (bf_search - kd_search) << std::endl;
  }

  if (kd_indices.size() != bf_indices.size()) {
    std::cerr << "size of results do not match " << kd_indices.size() << " vs. "
              << bf_indices.size() << std::endl;
  }
  else {
    std::cerr << "size of result: " << kd_indices.size() << std::endl;
    for (std::size_t idx = 0; idx < kd_indices.size(); ++idx) {
      if (kd_indices[idx] != bf_indices[idx] &&
          kd_distances[idx] != bf_distances[idx]) {
        std::cerr << "results do not match: " << idx
                  << " nearest neighbor: " << kd_indices[idx]
                  << " with distance: " << kd_distances[idx] << " vs. "
                  << bf_indices[idx] << " with distance: " << bf_distances[idx]
                  << std::endl;
      }
    }
  }
  return 0;
}
