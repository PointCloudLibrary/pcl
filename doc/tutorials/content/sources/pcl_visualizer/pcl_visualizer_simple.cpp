/* \author Geoffrey Biggs */


#include <iostream>
using namespace std;

#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/common/common_headers.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/console/parse.h>

using namespace pcl;
using namespace pcl::visualization;
typedef PointXYZ PointType;

// --------------
// -----Help-----
// --------------
void printUsage (const char* progName)
{
  cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
       << "Options:\n"
       << "-------------------------------------------\n"
       << "-h           this help\n"
       << "\n\n";
}

// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }

  // ------------------------------------------------------------------
  // -----Read pcd file or create example point cloud if not given-----
  // ------------------------------------------------------------------
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  if (!pcd_filename_indices.empty ())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
    {
      cerr << "Was not able to open file \""<<filename<<"\".\n";
      printUsage (argv[0]);
      return 0;
    }
  }
  else
  {
    cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
    // We're going to make an ellipse extruded along the z-axis.
    for (float z(-1.0); z <= 1.0; z += 0.05)
    {
        for (float angle(0.0); angle <= 360.0; angle += 5.0)
        {
            PointType point;
            point.x = 0.5 * cosf(deg2rad(angle));
            point.y = sinf(deg2rad(angle));
            point.z = z;
            point_cloud.points.push_back (point);
        }
    }
    point_cloud.width = point_cloud.points.size ();
    point_cloud.height = 1;
  }

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);
  viewer.addPointCloud (point_cloud_ptr, "sample cloud");
  viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters ();

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
