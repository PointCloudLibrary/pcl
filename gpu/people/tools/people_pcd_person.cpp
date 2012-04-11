/**
 * @brief This file is the execution node of the Human Tracking 
 * @copyright Copyright (2011) Willow Garage
 * @authors Koen Buys, Anatoly Baksheev
 **/
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_labeled_clusters.h>
#include <pcl/segmentation/seeded_hue_segmentation.h>

//#include <opencv2/opencv.hpp>

#include <pcl/gpu/people/conversions.h>
#include <pcl/gpu/people/optimized_elec.h>
#include <pcl/gpu/people/optimized_shs.h>
#include <pcl/gpu/people/label_conversion.h>
#include <pcl/gpu/people/label_segment.h>
#include <pcl/gpu/people/label_tree.h>
#include <pcl/gpu/people/tree_live.h>


#include <pcl/gpu/people/person.h>
#include <pcl/gpu/people/person.hpp>

//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <iostream>
#include <sstream>


int print_help()
{
  std::cout << "\nPeople tracking app options:" << std::endl;
  std::cout << "\t -numTrees \t<int> \tnumber of trees to load" << std::endl;
  std::cout << "\t -tree0 \t<path_to_tree_file>" << std::endl;
  std::cout << "\t -tree1 \t<path_to_tree_file>" << std::endl;
  std::cout << "\t -tree2 \t<path_to_tree_file>" << std::endl;
  std::cout << "\t -tree3 \t<path_to_tree_file>" << std::endl;
  std::cout << "\t -pcd   \t<path_to_pcd_file>" << std::endl;
  return 0;
}

int main(int argc, char** argv)
{
  if(pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
    return print_help();

  std::string treeFilenames[4] = 
  {
    "d:/TreeData/results/forest1/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt"
  };
  int         numTrees = 4;
  std::string pcdname = "d:/3/0015.pcd";
  pcl::console::parse_argument (argc, argv, "-numTrees", numTrees);
  pcl::console::parse_argument (argc, argv, "-tree0", treeFilenames[0]);
  pcl::console::parse_argument (argc, argv, "-tree1", treeFilenames[1]);
  pcl::console::parse_argument (argc, argv, "-tree2", treeFilenames[2]);
  pcl::console::parse_argument (argc, argv, "-tree3", treeFilenames[3]);
  pcl::console::parse_argument (argc, argv, "-pcd", pcdname);
  //Don't know if this assert is still needed with pcl::console?
  //AB: pcl::console does nothing if arg is not found
  assert(numTrees > 0 );
  assert(numTrees <= 4 );

  /// Create the app
  pcl::gpu::people::Person* person = new pcl::gpu::people::Person(treeFilenames[0]);
  person->addTree(treeFilenames[1]);
  person->addTree(treeFilenames[2]);
  person->addTree(treeFilenames[3]);
  return 0;
}
