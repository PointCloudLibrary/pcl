/**
 * @brief This file is the execution node of the Human Tracking 
 * @copyright Copyright (2011) Willow Garage
 * @authors Koen Buys, Anatoly Baksheev
 **/
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/gpu/people/person.h>
#include <pcl/visualization/image_viewer.h>

#include <iostream>

using namespace pcl::visualization;
using namespace pcl::console;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class PeoplePCDApp
{
public: 
  PeoplePCDApp () : final_view_("Final labeling")//, image_view_("Input image") 
  {
    final_view_.setSize (640, 480);
    //image_view_.setSize (640, 480);

    final_view_.setPosition (0, 0);    
    //image_view_.setPosition (650, 0);    
  }

 void cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
 {
   person_->process(cloud);
    
   int c;
   pcl::PointCloud<pcl::RGB> cmap;
   cmap.width  = person_->cmap_device_.cols();
   cmap.height = person_->cmap_device_.rows();
   person_->cmap_device_.download(cmap.points, c);
      
   final_view_.showRGBImage<pcl::RGB>(cmap);
   final_view_.spinOnce(1, true);
   
   //image_view_.showRGBImage<pcl::PointXYZRGB>(cloud);
   //image_view_.spinOnce(1, true);
 }     

  pcl::gpu::people::Person::Ptr person_;
  ImageViewer final_view_;
  //ImageViewer image_view_;
};

int print_help()
{
  cout << "\nPeople tracking app options:" << endl;
  cout << "\t -numTrees \t<int> \tnumber of trees to load" << endl;
  cout << "\t -tree0 \t<path_to_tree_file>" << endl;
  cout << "\t -tree1 \t<path_to_tree_file>" << endl;
  cout << "\t -tree2 \t<path_to_tree_file>" << endl;
  cout << "\t -tree3 \t<path_to_tree_file>" << endl;
  cout << "\t -pcd   \t<path_to_pcd_file>" << endl;
  return 0;
}

int main(int argc, char** argv)
{
  if(find_switch (argc, argv, "--help") || find_switch (argc, argv, "-h"))
    return print_help();

  std::string treeFilenames[4] = 
  {
    "d:/TreeData/results/forest1/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt"
  };
  int         numTrees = 4;
  string pcdname = "d:/git/pcl/gpu/people/tools/test.pcd";
  parse_argument (argc, argv, "-numTrees", numTrees);
  parse_argument (argc, argv, "-tree0", treeFilenames[0]);
  parse_argument (argc, argv, "-tree1", treeFilenames[1]);
  parse_argument (argc, argv, "-tree2", treeFilenames[2]);
  parse_argument (argc, argv, "-tree3", treeFilenames[3]);
  parse_argument (argc, argv, "-pcd", pcdname);
  //Don't know if this assert is still needed with pcl::console?
  //AB: pcl::console does nothing if arg is not found
  assert(numTrees > 0 );
  assert(numTrees <= 4 ); 

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  int res = pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdname, *cloud);
  if (res == -1) //* load the file
    return PCL_ERROR ("Couldn't read file\n"), -1;
  
  cout << "Loaded " << cloud->width * cloud->height << " data points from " << pcdname << endl;

  
  using pcl::gpu::people::RDFBodyPartsDetector;
  vector<string> names_vector(treeFilenames, treeFilenames + numTrees);
  RDFBodyPartsDetector::Ptr rdf(new RDFBodyPartsDetector(names_vector));

  /// Load the first tree
  pcl::gpu::people::Person::Ptr person(new pcl::gpu::people::Person());  
  person->rdf_detector_ = rdf;
   
  /// Create the app
  PeoplePCDApp app;
  app.person_ = person;

  /// Run the app
  {
    pcl::ScopeTime frame_time("frame_time");
    app.cloud_cb(cloud);
  }

  app.final_view_.spin();
  return 0;
}
