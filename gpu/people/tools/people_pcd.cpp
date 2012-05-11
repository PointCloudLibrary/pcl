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
#include <pcl/io/png_io.h>
#include <pcl/gpu/people/people_detector.h>
#include <pcl/visualization/image_viewer.h>
//#include <pcl/search/pcl_search.>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>

//#include <Eigen/Core>
#include <iostream>

namespace pc = pcl::console;
using namespace pcl::visualization;
using namespace pcl;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//struct ProjMatrix : public pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>
//{  
//  using pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::projection_matrix_;
//};
//
//float estimateFocalLength(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
//{
//  ProjMatrix proj_matrix;
//  proj_matrix.setInputCloud(cloud);  
//  Eigen::Matrix3f KR = proj_matrix.projection_matrix_.topLeftCorner <3, 3> ();    
//  return (KR(0,0) + KR(1,1))/KR(2,2)/2;
//}

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

    PeoplePCDApp (pcl::Grabber& capture) : capture_(capture), exit_(false), cloud_cb_(true), counter_(0), final_view_("Final labeling")//, image_view_("Input image")
    {
      final_view_.setSize (COLS, ROWS);
      //image_view_.setSize (COLS, ROWS);

      final_view_.setPosition (0, 0);
      //image_view_.setPosition (650, 0);

      cmap_device_.create(ROWS, COLS);
      cmap_host_.points.resize(COLS * ROWS);
      depth_device_.create(ROWS, COLS);
      image_device_.create(ROWS, COLS);

      rgba_host_.points.resize(COLS * ROWS);
      rgb_host_.resize(COLS * ROWS * 3);
    }

    void
    visualizeAndWrite(bool write = false)
    {
      const pcl::device::Labels& labels = people_detector_.rdf_detector_->getLabels();
      people_detector_.rdf_detector_->colorizeLabels(labels, cmap_device_);
      
      int c;
      cmap_host_.width = cmap_device_.cols();
      cmap_host_.height = cmap_device_.rows();
      cmap_host_.points.resize(cmap_host_.width * cmap_host_.height);
      cmap_device_.download(cmap_host_.points, c);

      final_view_.showRGBImage<pcl::RGB>(cmap_host_);
      final_view_.spinOnce(1, true);

      //if (cloud_cb_)      
      //  image_view_.showRGBImage<pcl::PointXYZRGB>(cloud_host_);            
      //else      
      //  image_view_.showRGBImage(&rgb_host_[0], cmap_host_.width, cmap_host_.height);      
      //image_view_.spinOnce(1, true);

      if (write)
      {
        if (cloud_cb_)
          savePNGFile(make_name(counter_, "ii"), cloud_host_);
        else
          savePNGFile(make_name(counter_, "ii"), rgba_host_);
        savePNGFile(make_name(counter_, "c2"), cmap_host_);
        savePNGFile(make_name(counter_, "s2"), labels);
        savePNGFile(make_name(counter_, "d1"), people_detector_.depth_device1_);
        savePNGFile(make_name(counter_, "d2"), people_detector_.depth_device2_);
      }
    }
        
    void source_cb1(const boost::shared_ptr<const PointCloud<PointXYZRGBA> >& cloud)
    {
      {
        boost::mutex::scoped_lock lock(data_ready_mutex_);
        if (exit_)
          return;

        pcl::copyPointCloud(*cloud, cloud_host_);        
      }
      data_ready_cond_.notify_one();
    }

    void source_cb2(const boost::shared_ptr<openni_wrapper::Image>& image_wrapper, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper, float)
    {
      {
        boost::mutex::scoped_lock lock(data_ready_mutex_);
        if (exit_)
          return;
                 
        //getting depth
        int w = depth_wrapper->getWidth();
        int h = depth_wrapper->getHeight();
        int s = w * PeopleDetector::Depth::elem_size;
        depth_device_.upload(depth_wrapper->getDepthMetaData().Data(), s, h, w);
                      
        //getting image
        w = image_wrapper->getWidth();
        h = image_wrapper->getHeight();
        s = w * PeopleDetector::Image::elem_size;
        
        //fill rgb array
        rgb_host_.resize(w * h * 3);        
        image_wrapper->fillRGB(w, h, (unsigned char*)&rgb_host_[0]);

        // convert to rgba, TODO image_wrapper should be updated to support rgba directly
        rgba_host_.points.resize(w * h);
        rgba_host_.width = w;
        rgba_host_.height = h;
        for(int i = 0; i < rgba_host_.size(); ++i)
        {
          const unsigned char *pixel = &rgb_host_[i * 3];
          RGB& rgba = rgba_host_.points[i];         
          rgba.r = pixel[0];
          rgba.g = pixel[1];
          rgba.b = pixel[2];
        }
        image_device_.upload(&rgba_host_.points[0], s, h, w);       
      }
      data_ready_cond_.notify_one();
    }

    void
    startMainLoop (const boost::shared_ptr<const PointCloud<PointXYZRGBA> >& cloud)
    {     
      cloud_cb_ = true;
      if (cloud)
      {
        cloud_cb_= true;
        source_cb1(cloud);
      }

      typedef boost::shared_ptr<openni_wrapper::DepthImage> DepthImagePtr;
      typedef boost::shared_ptr<openni_wrapper::Image> ImagePtr;
      
      boost::function<void (const boost::shared_ptr<const PointCloud<PointXYZRGBA> >&)> func1 = boost::bind (&PeoplePCDApp::source_cb1, this, _1);
      boost::function<void (const ImagePtr&, const DepthImagePtr&, float constant)> func2 = boost::bind (&PeoplePCDApp::source_cb2, this, _1, _2, _3);                  
      boost::signals2::connection c = cloud_cb_ ? capture_.registerCallback (func1) : capture_.registerCallback (func2);

      {
        boost::unique_lock<boost::mutex> lock(data_ready_mutex_);
        
        try 
        { 
          capture_.start ();
          while (!exit_ && !final_view_.wasStopped())
          {   
            /*pcl::PCDGrabberBase *ispcd = dynamic_cast<pcl::PCDGrabberBase*>(&capture_);
            if(ispcd)
              ispcd ->trigger();*/

         
            bool has_data = cloud ? true : data_ready_cond_.timed_wait(lock, boost::posix_time::millisec(100));
            if(has_data)
            {                                       
              pcl::ScopeTime frame_time("frame_time");

              if (cloud_cb_)
                people_detector_.process(cloud_host_.makeShared());
              else
                people_detector_.process(depth_device_, image_device_);
      
              ++counter_;              
            }            
           
            if(has_data)
              visualizeAndWrite();
          }
          final_view_.spinOnce (3);                  
        }
        catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; }
        catch (const std::exception& /*e*/) { cout << "Exception" << endl; }

        capture_.stop ();
      }
      c.disconnect();
    }

    boost::mutex data_ready_mutex_;
    boost::condition_variable data_ready_cond_;

    pcl::Grabber& capture_;

    bool cloud_cb_;
    bool exit_;
    int counter_;
    PeopleDetector people_detector_;
    PeopleDetector::Image cmap_device_;
    pcl::PointCloud<pcl::RGB> cmap_host_;

    PeopleDetector::Depth depth_device_;
    PeopleDetector::Image image_device_;
    
    pcl::PointCloud<pcl::RGB> rgba_host_;
    std::vector<unsigned char> rgb_host_;

    PointCloud<PointXYZRGB> cloud_host_;        

    ImageViewer final_view_;
    //ImageViewer image_view_;    
};

void print_help()
{
  cout << "\nPeople tracking app options (help):" << endl;
  cout << "\t -numTrees    \t<int> \tnumber of trees to load" << endl;
  cout << "\t -tree0       \t<path_to_tree_file>" << endl;
  cout << "\t -tree1       \t<path_to_tree_file>" << endl;
  cout << "\t -tree2       \t<path_to_tree_file>" << endl;
  cout << "\t -tree3       \t<path_to_tree_file>" << endl;
  cout << "\t -gpu         \t<GPU_device_id>" << endl;
  cout << "\t -dev         \t<Kinect_device_id>" << endl;  
  cout << "\t -pcd         \t<path_to_pcd_file>" << endl;
  cout << "\t -oni         \t<path_to_oni_file>" << endl;  
  cout << "\t -pcd_folder  \t<path_to_folder_with_pcf_files>" << endl;
}

int main(int argc, char** argv)
{
  // answering for help 
  cout << "People tracking on PCD files version 0.1" << std::endl;
  if(pc::find_switch (argc, argv, "--help") || pc::find_switch (argc, argv, "-h"))
    return print_help(), 0;
  
  // selecting GPU and pringing info
  int device = 0;
  pc::parse_argument (argc, argv, "-gpu", device);
  pcl::gpu::setDevice (device);
  pcl::gpu::printShortCudaDeviceInfo (device);
  
  // selecting data source
  boost::shared_ptr<pcl::Grabber> capture;
  std::string openni_device, oni_file, pcd_file, pcd_folder;  

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

  bool signel_cloud = false;
  try
  {
    if (pc::parse_argument (argc, argv, "-dev", openni_device) > 0)
    {
      capture.reset( new pcl::OpenNIGrabber(openni_device) );
    }
    else
    if (pc::parse_argument (argc, argv, "-oni", oni_file) > 0)
    {    
      capture.reset( new pcl::ONIGrabber(oni_file, true, true) );
    }    
    else
    if (pc::parse_argument (argc, argv, "-pcd", pcd_file) > 0)
    {       
      if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcd_file, *cloud) == -1)
        return cout << "Couldn't read cloud file" << endl, -1;
      
      //vector<string> single_file(1, pcd_file);
      //capture.reset( new pcl::PCDGrabber<PointXYZRGBA>(single_file, 0, true) );            
    }    
    else
    if (pc::parse_argument (argc, argv, "-pcd_folder", pcd_folder) > 0)
    {        
      capture.reset( new pcl::PCDGrabber<PointXYZRGBA>(pcd_folder, 0, true) );
    }    
    else
    {
      capture.reset( new pcl::OpenNIGrabber() );
      //capture.reset( new pcl::PCDGrabber("d:/3", 0, true) );
      //capture.reset( new pcl::ONIGrabber("d:/onis/20111013-224932.oni", true, true) );    
  
      const char* f1 = "d:/3/0008.pcd";
      const char* f2 = "d:/git/pcl/gpu/people/tools/test.pcd";     

      if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (f1, *cloud) == -1)
        return cout << "Couldn't read cloud file" << endl, -1;
            
      //capture.reset( new pcl::PCDGrabber<PointXYZRGBA>(vector<string>(1, f1), 0, true) );
      //capture.reset( new pcl::PCDGrabber<PointXYZRGBA>("d:/3", 0, true) );
    }
  }
  catch (const pcl::PCLException& /*e*/) { return cout << "Can't opencv depth source" << endl, -1; }
    
  //selecting tree files
  vector<string> tree_files;
  tree_files.push_back("Data/forest1/tree_20.txt");
  tree_files.push_back("Data/forest2/tree_20.txt");
  tree_files.push_back("Data/forest3/tree_20.txt");
  tree_files.push_back("Data/forest4/tree_20.txt");    
  
  pc::parse_argument (argc, argv, "-tree0", tree_files[0]);
  pc::parse_argument (argc, argv, "-tree1", tree_files[1]);
  pc::parse_argument (argc, argv, "-tree2", tree_files[2]);
  pc::parse_argument (argc, argv, "-tree3", tree_files[3]);
  
  int num_trees = (int)tree_files.size();
  pc::parse_argument (argc, argv, "-numTrees", num_trees);
    
  tree_files.resize(num_trees);
  if (num_trees == 0 || num_trees > 4)
    return cout << "Invalid number of trees" << endl, -1;
  
  try
  {
    // loading trees
    typedef pcl::gpu::people::RDFBodyPartsDetector RDFBodyPartsDetector;
    RDFBodyPartsDetector::Ptr rdf(new RDFBodyPartsDetector(tree_files));
    std::cout << "Loaded files into rdf" << std::endl;

    // Create the app
    PeoplePCDApp app(*capture);  
    app.people_detector_.rdf_detector_ = rdf;
    
    // executing
    app.startMainLoop (cloud);
  }
  catch (const pcl::PCLException& /*e*/) { cout << "PCLException" << endl; }
  catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; }
  catch (const std::runtime_error& e) { cout << e.what() << endl; }
  catch (const std::exception& /*e*/) { cout << "Exception" << endl; }
  

  return 0;
}  
  

