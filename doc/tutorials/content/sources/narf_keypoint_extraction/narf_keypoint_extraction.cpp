/* \author Bastian Steder */

#include <iostream>
using namespace std;

#include <boost/thread/thread.hpp>
#include "pcl/range_image/range_image.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/range_image_visualizer.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/features/range_image_border_extractor.h"
#include "pcl/keypoints/narf_keypoint.h"
#include <pcl/console/parse.h>

using namespace pcl;
using namespace pcl::visualization;
typedef PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
float support_size = 0.2f;
RangeImage::CoordinateFrame coordinate_frame = RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

// --------------
// -----Help-----
// --------------
void printUsage(const char* progName)
{
  cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
       << "Options:\n"
       << "-------------------------------------------\n"
       << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
       << "-c <int>     coordinate frame (default "<<(int)coordinate_frame<<")\n"
       << "-m           Treat all unseen points as maximum range readings\n"
       << "-s <float>   support size for the interest points (diameter of the used sphere) (default "<<support_size<<")\n"
       << "-h           this help\n"
       << "\n\n";
}

// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  // ------------------------------------------------------------------
  // -----Read pcd file or create example point cloud if not given-----
  // ------------------------------------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  if (pcl::console::find_argument (argc, argv, "-m") >= 0)
  {
    setUnseenToMaxRange = true;
    cout << "Setting unseen values in range image to maximum range readings.\n";
  }
  int tmp_coordinate_frame;
  if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
  {
    coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
    cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
  }
  if (pcl::console::parse (argc, argv, "-s", support_size) >= 0)
    cout << "Setting support size to "<<support_size<<".\n";
  if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
    cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
  angular_resolution = deg2rad (angular_resolution);
  
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
  PointCloud<PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
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
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
    std::string far_ranges_filename = getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
    if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
      std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
  }
  else
  {
    cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.points.push_back (point);
      }
    }
    point_cloud.width = point_cloud.points.size ();  point_cloud.height = 1;
  }
  
  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<RangeImage> range_image_ptr(new RangeImage);
  RangeImage& range_image = *range_image_ptr;   
  range_image.createFromPointCloud (point_cloud, angular_resolution, deg2rad (360.0f), deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();
  
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (1, 1, 1);
  viewer.addCoordinateSystem (1.0f);
  PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 0, 0, 0);
  viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  //PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 150, 150, 150);
  //viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
  //viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "range image");
  
  // --------------------------
  // -----Show range image-----
  // --------------------------
  RangeImageVisualizer range_image_widget("Range image");
  range_image_widget.setRangeImage(range_image);
  
  // --------------------------------
  // -----Extract NARF keypoints-----
  // --------------------------------
  RangeImageBorderExtractor range_image_border_extractor;
  NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage(&range_image);
  narf_keypoint_detector.getParameters().support_size = support_size;
  //narf_keypoint_detector.getParameters().add_points_on_straight_edges = true;
  //narf_keypoint_detector.getParameters().distance_for_additional_points = 0.5;
  
  PointCloud<int> keypoint_indices;
  narf_keypoint_detector.compute(keypoint_indices);
  std::cout << "Found "<<keypoint_indices.points.size()<<" key points.\n";

  // ----------------------------------------------
  // -----Show keypoints in range image widget-----
  // ----------------------------------------------
  for (size_t i=0; i<keypoint_indices.points.size(); ++i)
    range_image_widget.markPoint(keypoint_indices.points[i]%range_image.width, keypoint_indices.points[i]/range_image.width);
  
  // -------------------------------------
  // -----Show keypoints in 3D viewer-----
  // -------------------------------------
  PointCloud<PointXYZ> keypoints;
  keypoints.points.resize(keypoint_indices.points.size());
  for (size_t i=0; i<keypoint_indices.points.size(); ++i)
    keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
  viewer.addPointCloud(keypoints.makeShared(), "keypoints");
  viewer.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  
  //--------------------
  // -----Main loop-----
  //--------------------
  while(!viewer.wasStopped() || range_image_widget.isShown())
  {
    ImageWidgetWX::spinOnce();  // process GUI events
    viewer.spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
