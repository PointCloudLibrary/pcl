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
#include <pcl/people/trees/tree_live.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/time.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

class PeopleTrackingApp
{
  public:
    PeopleTrackingApp () : viewer ("PCL People Tracking App") {}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
      if (!viewer.wasStopped())
        viewer.showCloud (cloud);
    }

    void run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
        boost::bind (&PeopleTrackingApp::cloud_cb_, this, _1);

      interface->registerCallback (f);
      interface->start ();

      while (!viewer.wasStopped())
      {
        sleep (1);
      }
      interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
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

  PeopleTrackingApp a;
  a.run();

/*
  /// load the first tree file
  std::ifstream fin0(treeFilenames[0].c_str() );
  assert(fin0.is_open() );
  m_proc = new MultiTreeLiveProc(fin0);
  fin0.close();

  /// Load the other tree files
  for(int ti=1;ti<numTrees;++ti) {
    std::ifstream fin(treeFilenames[ti].c_str() );
    assert(fin.is_open() );
    m_proc->addTree(fin);
    fin.close();
  }
*/
  return 0;
}
