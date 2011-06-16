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
#include "pcl/features/narf_descriptor.h"

using namespace pcl;
typedef PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
float support_size = 0.2f;
RangeImage::CoordinateFrame coordinate_frame = RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
bool rotation_invariant = true;

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
       << "-m           Treat all unseen points to max range\n"
       << "-s <float>   support size for the interest points (diameter of the used sphere in meters) (default "<<support_size<<")\n"
       << "-o <0/1>     switch rotational invariant version of the feature on/off (default "<<(int)rotation_invariant<<")\n"
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
  for (char c; (c = getopt(argc, argv, "r:c:ms:o:h")) != -1; ) {
    switch (c) {
      case 'r':
      {
        angular_resolution = strtod(optarg, NULL);
        cout << "Setting angular resolution to "<<angular_resolution<<".\n";
        break;
      }
      case 'c':
      {
        coordinate_frame = (RangeImage::CoordinateFrame)strtol(optarg, NULL, 0);
        cout << "Using coordinate frame "<<(int)coordinate_frame<<".\n";
        break;
      }
      case 'm':
      {
        setUnseenToMaxRange = true;
        break;
      }
      case 's':
      {
        support_size = strtod(optarg, NULL);
        cout << "Setting support size to "<<support_size<<".\n";
        break;
      }
      case 'o':
      {
        rotation_invariant = strtod(optarg, NULL);
        cout << "Switching rotation invariant feature version "<<(rotation_invariant ? "on" : "off")<<".\n";
        break;
      }
      case 'h':
        printUsage(argv[0]);
        exit(0);
    }
  }
  angular_resolution = deg2rad(angular_resolution);
  
  // ------------------------------------------------------------------
  // -----Read pcd file or create example point cloud if not given-----
  // ------------------------------------------------------------------
  // Read/Generate point cloud
  pcl::PointCloud<PointType> point_cloud;
  PointCloud<PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
  if (optind < argc)
  {
    sensor_msgs::PointCloud2 point_cloud_data;
    if (pcl::io::loadPCDFile(argv[optind], point_cloud_data) == -1)
    {
      cerr << "Was not able to open file \""<<argv[optind]<<"\".\n";
      printUsage(argv[0]);
      exit(0);
    }
    fromROSMsg(point_cloud_data, point_cloud);
    RangeImage::extractFarRanges(point_cloud_data, far_ranges);
    if (pcl::getFieldIndex(point_cloud_data, "vp_x")>=0)
    {
      cout << "Scene point cloud has viewpoint information.\n";
      PointCloud<PointWithViewpoint> tmp_pc;  fromROSMsg(point_cloud_data, tmp_pc);
      Eigen::Vector3f average_viewpoint = RangeImage::getAverageViewPoint(tmp_pc);
      scene_sensor_pose = Eigen::Translation3f(average_viewpoint) * scene_sensor_pose;
    }
  }
  else
  {
    cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.points.push_back(point);
      }
    }
    point_cloud.width = point_cloud.points.size();  point_cloud.height = 1;
  }

  
  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  RangeImage range_image;
  range_image.createFromPointCloud(point_cloud, angular_resolution, deg2rad(360.0f), deg2rad(180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges(far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange();

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  visualization::PCLVisualizer viewer("3D Viewer");
  viewer.addCoordinateSystem(1.0f);
  viewer.addPointCloud(point_cloud.makeShared(), "original point cloud");
  //viewer.addPointCloud(range_image, "range image");
  
  // --------------------------
  // -----Show range image-----
  // --------------------------
  visualization::RangeImageVisualizer range_image_widget("Range image");
  range_image_widget.setRangeImage(range_image);
  
  // --------------------------------
  // -----Extract NARF keypoints-----
  // --------------------------------
  RangeImageBorderExtractor range_image_border_extractor;
  NarfKeypoint narf_keypoint_detector;
  narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage(&range_image);
  narf_keypoint_detector.getParameters().support_size = support_size;
  
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
  viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  
  // ------------------------------------------------------
  // -----Extract NARF descriptors for interest points-----
  // ------------------------------------------------------
  vector<int> keypoint_indices2;
  keypoint_indices2.resize(keypoint_indices.points.size());
  for (unsigned int i=0; i<keypoint_indices.size(); ++i) // This step is necessary to get the right vector type
    keypoint_indices2[i]=keypoint_indices.points[i];
  NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);
  narf_descriptor.getParameters().support_size = support_size;
  narf_descriptor.getParameters().rotation_invariant = rotation_invariant;
  PointCloud<Narf36> narf_descriptors;
  narf_descriptor.compute(narf_descriptors);
  cout << "Extracted "<<narf_descriptors.size()<<" descriptors for "<<keypoint_indices.points.size()<< " keypoints.\n";
  
  //--------------------
  // -----Main loop-----
  //--------------------
  while(!viewer.wasStopped() || range_image_widget.isShown())
  {
    visualization::ImageWidgetWX::spinOnce();  // process GUI events
    viewer.spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
