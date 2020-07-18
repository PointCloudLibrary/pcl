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
#include <pcl/people/label_skeleton/conversion.h>
#include <pcl/people/trees/tree_live.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <opencv2/opencv.hpp>

#include <functional>
#include <thread>

using namespace std::chrono_literals;

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

      std::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
        [this] (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) { cloud_cb_ (cloud); };

      interface->registerCallback (f);
      interface->start ();

      while (!viewer.wasStopped())
      {
        std::this_thread::sleep_for(1s);
      }
      interface->stop ();
    }

    void load_tree(std::string treeFilenames[4], int numTrees)
    {
      std::ifstream fin0 (treeFilenames[0]);
      assert(fin0.is_open());
      m_proc = std::make_unique<pcl::people::trees::MultiTreeLiveProc> (fin0);

      /// Load the other tree files
      for (const auto& file : treeFilenames)
      {
        std::ifstream fin (file);
        assert (fin.is_open());
        m_proc->addTree(fin);
      }
    }

    pcl::visualization::CloudViewer                        viewer;
    std::unique_ptr<pcl::people::trees::MultiTreeLiveProc> m_proc;
    cv::Mat                                                m_lmap;
    cv::Mat                                                m_cmap;
    cv::Mat                                                cmap;
    cv::Mat                                                m_bmap;
};

int print_help()
{
  std::cout << "\nPeople tracking app options:" << std::endl;
  std::cout << "\t -numTrees \t<int> \tnumber of trees to load" << std::endl;
  std::cout << "\t -tree0 \t<path_to_tree_file>" << std::endl;
  std::cout << "\t -tree1 \t<path_to_tree_file>" << std::endl;
  std::cout << "\t -tree2 \t<path_to_tree_file>" << std::endl;
  std::cout << "\t -tree3 \t<path_to_tree_file>" << std::endl;
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
  app.load_tree(treeFilenames, numTrees);

  /// Run the app
  app.run();
  return 0;
}
