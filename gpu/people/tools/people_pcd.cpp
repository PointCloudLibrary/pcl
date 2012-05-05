/**
 *  Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: $
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
#include <pcl/search/pcl_search.h>
#include <Eigen/Core>

#include <pcl/io/png_io.h>

#include <iostream>

using namespace pcl::visualization;
using namespace pcl::console;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct ProjMatrix : public pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>
{  
  using pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::projection_matrix_;
};

float estimateFocalLength(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
  ProjMatrix proj_matrix;
  proj_matrix.setInputCloud(cloud);  
  Eigen::Matrix3f KR = proj_matrix.projection_matrix_.topLeftCorner <3, 3> ();    
  return (KR(0,0) + KR(1,1))/KR(2,2)/2;
}

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
      std::cout << "Cloud Callback" << std::endl;
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
  cout << "\nPeople tracking app options (help):" << endl;
  cout << "\t -numTrees \t<int> \tnumber of trees to load" << endl;
  cout << "\t -tree0 \t<path_to_tree_file>" << endl;
  cout << "\t -tree1 \t<path_to_tree_file>" << endl;
  cout << "\t -tree2 \t<path_to_tree_file>" << endl;
  cout << "\t -tree3 \t<path_to_tree_file>" << endl;
  cout << "\t -pcd   \t<path_to_pcd_file>" << endl;  
}

int main(int argc, char** argv)
{
  cout << "People tracking on PCD files version 0.1" << std::endl;
  if(find_switch (argc, argv, "--help") || find_switch (argc, argv, "-h"))
    return print_help(), 0;
 
  std::string treeFilenames[4] = 
  {
    "d:/TreeData/results/forest1/tree_20.txt",
    "d:/TreeData/results/forest2/tree_20.txt",
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

  //string pcdname = "d:/git/pcl/gpu/people/tools/test.pcd";
  string pcdname = "d:/3/0008.pcd";
  parse_argument (argc, argv, "-pcd", pcdname);

  // loading cloud file
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  int res = pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdname, *cloud);
  if (res == -1) //* load the file
    return cout << "Couldn't read cloud file" << endl, -1;

  cout << "Loaded " << cloud->width * cloud->height << " data points from " << pcdname << endl;

  // loading trees
  using pcl::gpu::people::RDFBodyPartsDetector;
  vector<string> names_vector(treeFilenames, treeFilenames + numTrees);
  std::cout << "Trees collected " << std::endl;
  RDFBodyPartsDetector::Ptr rdf(new RDFBodyPartsDetector(names_vector));
  std::cout << "Loaded files into rdf" << std::endl;

  // Create the app
  PeoplePCDApp app;
  std::cout << "App created" << std::endl;
  app.people_detector_.rdf_detector_ = rdf;

  /// Run the app
  {
    pcl::ScopeTime frame_time("frame_time");
    app.cloud_cb(cloud);
  }
  std::cout << "calling visualisation" << std::endl;
  app.visualizeAndWrite(cloud);
  app.final_view_.spin();

  return 0;
}
