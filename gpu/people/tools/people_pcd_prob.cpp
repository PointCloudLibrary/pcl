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
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/gpu/people/people_detector.h>
#include <pcl/gpu/people/colormap.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/search/pcl_search.h>
#include <Eigen/Core>

#include <pcl/io/png_io.h>

#include <iostream>
#include <fstream>

using namespace pcl::visualization;
using namespace pcl::console;
using namespace pcl::gpu;
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

string
make_ext_name(int counter1, int counter2, const char* suffix)
{
  char buf[4096];
  sprintf (buf, "./people_%04d_%04d_%s.png", counter1, counter2, suffix);
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

      people::uploadColorMap(color_map_);
      
    }

    void cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
    {
      PCL_INFO("(I) : Cloud Callback\n");
      processReturn = people_detector_.processProb(cloud);
      ++counter_;
      //visualizeAndWrite(cloud);
    }

    void
    writeXMLFile(std::string& filename)
    {
      filebuf fb;
      fb.open (filename.c_str(), ios::out);
      ostream os(&fb);
      people_detector_.person_attribs_->writePersonXMLConfig(os);
      fb.close();
    }

    void
    readXMLFile(std::string& filename)
    {
      filebuf fb;
      fb.open (filename.c_str(), ios::in);
      istream is(&fb);
      people_detector_.person_attribs_->readPersonXMLConfig(is);
      fb.close();
    }

    void
    visualizeAndWrite(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
    {
      const PeopleDetector::Labels& labels = people_detector_.rdf_detector_->getLabels();           
      people::colorizeLabels(color_map_, labels, cmap_device_);

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

    void
    convertProbToRGB (pcl::PointCloud<pcl::device::prob_histogram>& histograms, int label, pcl::PointCloud<pcl::RGB>& rgb)
    {
      for(size_t t; t < histograms.points.size(); t++)
      {
        float value = histograms.points[t].probs[label];
        float value8 = value * 255;
        char val = static_cast<char> (value8);
        pcl::RGB p;
        p.r = val; p.b = val; p.g = val;
        rgb.points.push_back(p);
      }
      rgb.width = histograms.width;
      rgb.height = histograms.height;
    }

    void
    writeProb(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
    {
      PCL_INFO("(I) : visualizeAndWriteProb() Callback\n");
      //const pcl::device::LabelProbability& prob = people_detector_.rdf_detector_->getProbability1();
      int c;
      // first write the first iteration

        pcl::PointCloud<pcl::device::prob_histogram> prob_host(people_detector_.rdf_detector_->P_l_1_.cols(), people_detector_.rdf_detector_->P_l_1_.rows());
        people_detector_.rdf_detector_->P_l_1_.download(prob_host.points, c);
        prob_host.width = people_detector_.rdf_detector_->P_l_1_.cols();
        prob_host.height = people_detector_.rdf_detector_->P_l_1_.rows();

        PCL_DEBUG("(I) : visualizeAndWriteProb() savePNGFile");
        for(int i = 0; i < pcl::gpu::people::NUM_LABELS; i++)
        {
          pcl::PointCloud<pcl::RGB> rgb;
          convertProbToRGB(prob_host, i, rgb);
          savePNGFile(make_ext_name(counter_,i, "hist1"), rgb);
        }

        PCL_DEBUG("(I) : visualizeAndWriteProb() : cols1: %d", people_detector_.rdf_detector_->P_l_1_.cols ());
        PCL_DEBUG("(I) : visualizeAndWriteProb() : rows1: %d", people_detector_.rdf_detector_->P_l_1_.rows ());

      // and now again for the second iteration

        pcl::PointCloud<pcl::device::prob_histogram> prob_host2(people_detector_.rdf_detector_->P_l_2_.cols(), people_detector_.rdf_detector_->P_l_2_.rows());
        people_detector_.rdf_detector_->P_l_2_.download(prob_host2.points, c);
        prob_host.width = people_detector_.rdf_detector_->P_l_2_.cols();
        prob_host.height = people_detector_.rdf_detector_->P_l_2_.rows();

        PCL_DEBUG("(I) : visualizeAndWriteProb() savePNGFile");
        for(int i = 0; i < pcl::gpu::people::NUM_LABELS; i++)
        {
          pcl::PointCloud<pcl::RGB> rgb;
          convertProbToRGB(prob_host2, i, rgb);
          savePNGFile(make_ext_name(counter_, i, "hist2"), rgb);
        }
        PCL_DEBUG("(I) : visualizeAndWriteProb() : cols2: %d", people_detector_.rdf_detector_->P_l_2_.cols());
        PCL_DEBUG("(I) : visualizeAndWriteProb() : rows2: %d", people_detector_.rdf_detector_->P_l_2_.rows());

      // and now again for the Gaus test

        pcl::PointCloud<pcl::device::prob_histogram> prob_host3(people_detector_.rdf_detector_->P_l_Gaus_.cols(), people_detector_.rdf_detector_->P_l_Gaus_.rows());
        people_detector_.rdf_detector_->P_l_Gaus_.download(prob_host3.points, c);
        prob_host3.width = people_detector_.rdf_detector_->P_l_Gaus_.cols();
        prob_host3.height = people_detector_.rdf_detector_->P_l_Gaus_.rows();
        for(int i = 0; i < pcl::gpu::people::NUM_LABELS; i++)
        {
          pcl::PointCloud<pcl::RGB> rgb;
          convertProbToRGB(prob_host3, i, rgb);
          savePNGFile(make_ext_name(counter_, i, "gaus"), rgb);
        }

    }

    int counter_;
    int processReturn;
    PeopleDetector people_detector_;
    PeopleDetector::Image cmap_device_;

    ImageViewer final_view_;
    //ImageViewer image_view_;

    pcl::gpu::DeviceArray<pcl::RGB> color_map_;
};

void print_help()
{
  PCL_INFO("\nPeople tracking app options (help):\n");
  PCL_INFO("\t -numTrees \t<int> \tnumber of trees to load\n");
  PCL_INFO("\t -tree0 \t<path_to_tree_file>\n");
  PCL_INFO("\t -tree1 \t<path_to_tree_file>\n");
  PCL_INFO("\t -tree2 \t<path_to_tree_file>\n");
  PCL_INFO("\t -tree3 \t<path_to_tree_file>\n");
  PCL_INFO("\t -pcd   \t<path_to_pcd_file>\n");
  PCL_INFO("\t -XML   \t<path_to_XML_file> \tcontains person specifics, defaults to generic.xml\n");
}

int main(int argc, char** argv)
{
  PCL_INFO("(I) : Main : People tracking on PCD files version 0.1\n");
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

  std::string XMLfilename("generic.xml");
  parse_argument (argc, argv, "-XML", XMLfilename);

  if (numTrees == 0 || numTrees > 4)
  {
      PCL_ERROR("(E) : Main : Invalid number of trees\n");
      return -1;
  }
  PCL_INFO("(I) : Main : Read %d Trees\n", numTrees);
/*
  //string pcdname = "d:/git/pcl/gpu/people/tools/test.pcd";
  string pcdname = "/home/u0062536/Data/pcd/koen.pcd";
  //parse_argument (argc, argv, "-pcd", pcdname);

  PCL_INFO("(I) : Main : Will read %s\n", pcdname);
*/
  // loading cloud file
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  int res = pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/u0062536/Data/pcd/koen.pcd", *cloud);
  if (res == -1) //* load the file
  {
    PCL_ERROR("(E) : Main : Couldn't read cloud file");
    return res;
  }

  //PCL_INFO("(I) : Main : Loaded %d data points from %s",cloud->width * cloud->height, pcdname);

  // loading trees
  using pcl::gpu::people::RDFBodyPartsDetector;

  vector<string> names_vector(treeFilenames, treeFilenames + numTrees);
  PCL_DEBUG("(I) : Main : Trees collected");
  RDFBodyPartsDetector::Ptr rdf(new RDFBodyPartsDetector(names_vector));
  PCL_DEBUG("(I) : Main : Loaded files into rdf");

  // Create the app
  PeoplePCDApp app;
  PCL_DEBUG("(I) : Main : App created");
  app.people_detector_.rdf_detector_ = rdf;

  // Read in person specific configuration
  app.readXMLFile(XMLfilename);
  std::cout << "(I) : filename " << XMLfilename << std::endl;

  /// Run the app
  {
    pcl::ScopeTime frame_time("(I) : frame_time");
    app.cloud_cb(cloud);
  }
  if(app.processReturn == 2)
  {
    PCL_DEBUG("(I) : Main : calling visualisation\n");
    app.visualizeAndWrite(cloud);
    app.writeProb(cloud);
  }
  else
  {
    PCL_DEBUG("(I) : Main : no good person found\n");
  }
  app.final_view_.spin();

  return 0;
}
