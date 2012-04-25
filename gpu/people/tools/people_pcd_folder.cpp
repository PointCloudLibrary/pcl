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
#include <pcl/gpu/people/people_detector.h>
#include <pcl/visualization/image_viewer.h>
#include <boost/lexical_cast.hpp>

#include <pcl/io/png_io.h>

#include <iostream>
#include <sys/types.h>
#include <vector>
#include <string>

#include <boost/filesystem.hpp>

using namespace pcl::visualization;
using namespace pcl::console;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

string 
make_name(int counter, const char* suffix)
{
  char buf[4096];
  sprintf (buf, "./people%04d_%s.png", counter, suffix);
  return buf;
}

template<typename T> void 
savePNGFile(const std::string& filename, const pcl::gpu::DeviceArray2D<T>& arr)
{
  int c;
  pcl::PointCloud<T> cloud(arr.cols(), arr.rows());
  arr.download(cloud.points, c);
  pcl::io::savePNGFile(filename, cloud);
}

template <typename T> void
savePNGFile (const std::string& filename, const pcl::PointCloud<T>& cloud)
{
  pcl::io::savePNGFile(filename, cloud);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class PeoplePCDApp
{
public: 
  typedef pcl::gpu::people::PeopleDetector PeopleDetector;

  enum { COLS = 640, ROWS = 480 };

  PeoplePCDApp () : counter_(0), cmap_device_(ROWS, COLS), final_view_("Final labeling")//, image_view_("Input image")
  {
    final_view_.setSize (COLS, ROWS);
    //image_view_.setSize (COLS, ROWS);

    final_view_.setPosition (0, 0);
    //image_view_.setPosition (650, 0);
  }

 void cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
 {
   people_detector_.process(cloud);
   ++counter_;
   //visualizeAndWrite(cloud);
 }

 void
 visualizeAndWrite(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
 {
   const pcl::gpu::people::RDFBodyPartsDetector::Labels& labels = people_detector_.rdf_detector_->getLabels();
   people_detector_.rdf_detector_->colorizeLabels(labels, cmap_device_);

   int c;
   pcl::PointCloud<pcl::RGB> cmap(cmap_device_.cols(), cmap_device_.rows());
   cmap_device_.download(cmap.points, c);

   final_view_.showRGBImage<pcl::RGB>(cmap);
   final_view_.spinOnce(1, true);

   //image_view_.showRGBImage<pcl::PointXYZRGB>(cloud);
   //image_view_.spinOnce(1, true);

   savePNGFile(make_name(counter_, "ii"), *cloud);
   savePNGFile(make_name(counter_, "c2"), cmap);
   savePNGFile(make_name(counter_, "s2"), labels);
   savePNGFile(make_name(counter_, "d1"), people_detector_.depth_device1_);
   savePNGFile(make_name(counter_, "d2"), people_detector_.depth_device2_);
 }

  int counter_;
  PeopleDetector people_detector_;
  PeopleDetector::Image cmap_device_;

  ImageViewer final_view_;
  //ImageViewer image_view_;
};

void print_help()
{
  cout << "\nPeople tracking app options:" << endl;
  cout << "\t -numTrees \t<int> \tnumber of trees to load" << endl;
  cout << "\t -tree0 \t<path_to_tree_file>" << endl;
  cout << "\t -tree1 \t<path_to_tree_file>" << endl;
  cout << "\t -tree2 \t<path_to_tree_file>" << endl;
  cout << "\t -tree3 \t<path_to_tree_file>" << endl;
  cout << "\t -pcd   \t<path_to_pcd_folder>" << endl;
}

/*function... might want it in some class?*/
bool getPcdFilesInDir (const string& directory, vector<string>& files)
{
  namespace fs = boost::filesystem;
  fs::path dir(directory);
        
  if (!fs::exists(dir) || !fs::is_directory(dir))
    return false;
    
  vector<string> result;
  fs::directory_iterator pos(dir);
  fs::directory_iterator end;           

  for(; pos != end ; ++pos)
    if (fs::is_regular_file(pos->status()) )
      if (fs::extension(*pos) == ".pcd")
        result.push_back(pos->path().string());
    
  files.swap(result);
  return true;
}

int main(int argc, char** argv)
{
  if(find_switch (argc, argv, "--help") || find_switch (argc, argv, "-h"))
    return print_help(), 0;
 
  // Hardcoded for Anatoly
  std::string treeFilenames[4] = 
  {
    "d:/TreeData/results/forest1/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt",
    "d:/TreeData/results/forest3/tree_20.txt"
  };
  int numTrees = 4;
  parse_argument (argc, argv, "-numTrees", numTrees);
  parse_argument (argc, argv, "-tree0", treeFilenames[0]);
  parse_argument (argc, argv, "-tree1", treeFilenames[1]);
  parse_argument (argc, argv, "-tree2", treeFilenames[2]);
  parse_argument (argc, argv, "-tree3", treeFilenames[3]);

  if (numTrees == 0 || numTrees > 4)
      return cout << "Invalid number of trees" << endl, -1;

  // Anatoly: you can enter your hardcoded path here again
  string pcdfolder = "d:/3/";
  parse_argument (argc, argv, "-pcd", pcdfolder);

  // loading trees
  using pcl::gpu::people::RDFBodyPartsDetector;
  vector<string> names_vector(treeFilenames, treeFilenames + numTrees);
  RDFBodyPartsDetector::Ptr rdf(new RDFBodyPartsDetector(names_vector));

  // Create the app
  PeoplePCDApp app;
  app.people_detector_.rdf_detector_ = rdf;

  // Create list of pcd files  
  vector<string> files;
  getPcdFilesInDir(pcdfolder, files);

  cout << files.size() << endl;
  
  for (size_t i = 0; i < files.size(); i++) 
  {        
    // loading cloud file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    int res = pcl::io::loadPCDFile<pcl::PointXYZRGB> (files[i], *cloud);
    if (res == -1) //* load the file
      return cout << "Couldn't read tree file" << endl, -1;

    cout << "Loaded " << cloud->width * cloud->height << " data points from " << files[i] << endl;

    /// Run the app
    {
      pcl::ScopeTime frame_time("frame_time");
      app.cloud_cb(cloud);
    }
    app.visualizeAndWrite(cloud);
    app.final_view_.spinOnce(1, true);    
  }
  return 0;
}
