/**
 * @brief This file is the execution node of the Human Tracking 
 * @copyright Copyright (2011) Willow Garage
 * @authors Koen Buys, Anatoly Baksheev
 **/

////// ALL INCLUDES /////////////
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_labeled_clusters.h>
#include <pcl/segmentation/seeded_hue_segmentation.h>
#include <pcl/people/conversion/conversions.h>
#include <opencv2/opencv.hpp>
#include <pcl/people/label_skeleton/conversion.h>
#include <pcl/people/trees/tree_live.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/common/time.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

class PeopleTrackingApp
{
  public:
    PeopleTrackingApp () : viewer ("PCL People Tracking App") {}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
    {
      if (!viewer.wasStopped())
        viewer.showCloud (cloud);
      ////////////////////CALLBACK IMPL/////////////////////

      /// @todo rewrite this to a pointcloud::Ptr
      pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
      pcl::PointCloud<pcl::PointXYZRGB> cloud_in_filt;
      cloud_in = *cloud;

      cv::Mat dmat(cloud_in.height, cloud_in.width, CV_16U);

      // Project pointcloud back into the imageplane
      // TODO: do this directly in GPU?
      pcl::people::label_skeleton::makeDepthImage16FromPointCloud(dmat, cloud_in);

      // Process the depthimage (CUDA)
      m_proc->process(dmat, m_lmap);

      ////////////////////END CALLBACK IMPL/////////////////
    }

    void run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
        boost::bind (&PeopleTrackingApp::cloud_cb_, this, _1);

      interface->registerCallback (f);
      interface->start ();

      while (!viewer.wasStopped())
      {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      }
      interface->stop ();
    }

    pcl::visualization::CloudViewer         viewer;
    pcl::people::trees::MultiTreeLiveProc*  m_proc;
    cv::Mat                                 m_lmap;
    cv::Mat                                 m_cmap;
    cv::Mat                                 cmap;
    cv::Mat                                 m_bmap;
};

int print_help()
{
  cout << "\nPeople tracking app options:" << std::endl;
  cout << "\t -numTrees \t<int> \tnumber of trees to load" << std::endl;
  cout << "\t -tree0 \t<path_to_tree_file>" << std::endl;
  cout << "\t -tree1 \t<path_to_tree_file>" << std::endl;
  cout << "\t -tree2 \t<path_to_tree_file>" << std::endl;
  cout << "\t -tree3 \t<path_to_tree_file>" << std::endl;
  return 0;
}

int main(int argc, char** argv)
{
  if(pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
    return print_help();

  std::string treeFilenames[4];
  int         numTrees;
  pcl::console::parse_argument (argc, argv, "-numTrees", numTrees);
  pcl::console::parse_argument (argc, argv, "-tree0", treeFilenames[0]);
  pcl::console::parse_argument (argc, argv, "-tree1", treeFilenames[1]);
  pcl::console::parse_argument (argc, argv, "-tree2", treeFilenames[2]);
  pcl::console::parse_argument (argc, argv, "-tree3", treeFilenames[3]);
  //Don't know if this assert is still needed with pcl::console?
  assert(numTrees > 0 );
  assert(numTrees <= 4 );

  /// Create the app
  PeopleTrackingApp app;

  /// Load the first tree
  std::ifstream fin0(treeFilenames[0].c_str() );
  assert(fin0.is_open() );
  app.m_proc = new pcl::people::trees::MultiTreeLiveProc(fin0);
  fin0.close();

  /// Load the other tree files
  for(int ti=1;ti<numTrees;++ti) {
    std::ifstream fin(treeFilenames[ti].c_str() );
    assert(fin.is_open() );
    app.m_proc->addTree(fin);
    fin.close();
  }
  /// Run the app
  app.run();
  return 0;
}
