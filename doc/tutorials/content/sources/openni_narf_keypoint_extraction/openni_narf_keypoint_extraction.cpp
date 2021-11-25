/* \author Bastian Steder */

/* ---[ */
#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/common/time.h> // for pcl::getTime
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>

#include <mutex>

std::string device_id = "#1";

float angular_resolution = 0.5;
float support_size = 0.2f;
bool set_unseen_to_max_range = true;
int max_no_of_threads = 1;
float min_interest_value = 0.5;

std::mutex depth_image_mutex,
             ir_image_mutex,
             image_mutex;
pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud_ptr;
openni_wrapper::DepthImage::Ptr depth_image_ptr;
openni_wrapper::IRImage::Ptr ir_image_ptr;
openni_wrapper::Image::Ptr image_ptr;

bool received_new_depth_data = false,
     received_new_ir_image   = false,
     received_new_image   = false;
struct EventHelper
{
  void
  depth_image_cb (const openni_wrapper::DepthImage::Ptr& depth_image)
  {
    if (depth_image_mutex.try_lock ())
    {
      depth_image_ptr = depth_image;
      depth_image_mutex.unlock ();
      received_new_depth_data = true;
    }
  }
};


void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] [scene.pcd] <model.pcl> [model_2.pcl] ... [model_n.pcl]\n\n"
       << "Options:\n"
       << "-------------------------------------------\n"
       << "-d <device_id>  set the device id (default \""<<device_id<<"\")\n"
       << "-r <float>      angular resolution in degrees (default "<<angular_resolution<<")\n"
       << "-s <float>      support size for the interest points (diameter of the used sphere in meters)"
       <<                 " (default "<<support_size<<")\n"
       << "-i <float>      minimum interest value (0-1) (default "<<min_interest_value<<")"
       << "-t <int>        maximum number of threads to use (default "<< max_no_of_threads<<")\n"
       << "-h              this help\n"
       << "\n\n";
}

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
  if (pcl::console::parse (argc, argv, "-d", device_id) >= 0)
    std::cout << "Using device id \""<<device_id<<"\".\n";
  if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
    std::cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
  angular_resolution = pcl::deg2rad (angular_resolution);
  if (pcl::console::parse (argc, argv, "-s", support_size) >= 0)
    std::cout << "Setting support size to "<<support_size<<"m.\n";
  if (pcl::console::parse (argc, argv, "-i", min_interest_value) >= 0)
    std::cout << "Setting minimum interest value to "<<min_interest_value<<".\n";
  if (pcl::console::parse (argc, argv, "-t", max_no_of_threads) >= 0)
    std::cout << "Setting maximum number of threads to "<<max_no_of_threads<<".\n";
  
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range Image");
  
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.addCoordinateSystem (1.0f, "global");
  viewer.setBackgroundColor (1, 1, 1);
  
  viewer.initCameraParameters ();
  viewer.setCameraPosition (0.0, -0.3, -2.0,
                            0.0, -0.3, 1.0,
                            0.0, -1.0, 0.0);
  
  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      std::cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx)
           << ", product: " << driver.getProductName (deviceIdx) << ", connected: "
           << (int) driver.getBus (deviceIdx) << " @ " << (int) driver.getAddress (deviceIdx)
           << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'\n";
    }
  }
  else
  {
    std::cout << "\nNo devices connected.\n\n";
    return 1;
  }
  
  pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id);
  EventHelper event_helper;
  
  std::function<void (const openni_wrapper::DepthImage::Ptr&) > f_depth_image =
    [&] (const openni_wrapper::DepthImage::Ptr& depth) { event_helper.depth_image_cb (depth); };
  boost::signals2::connection c_depth_image = interface->registerCallback (f_depth_image);
  
  std::cout << "Starting grabber\n";
  interface->start ();
  std::cout << "Done\n";
  
  pcl::RangeImagePlanar::Ptr range_image_planar_ptr (new pcl::RangeImagePlanar);
  pcl::RangeImagePlanar& range_image_planar = *range_image_planar_ptr;
  
  pcl::RangeImageBorderExtractor range_image_border_extractor;
  pcl::NarfKeypoint narf_keypoint_detector;
  narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
  narf_keypoint_detector.getParameters ().support_size = support_size;
  narf_keypoint_detector.getParameters ().max_no_of_threads = max_no_of_threads;
  narf_keypoint_detector.getParameters ().min_interest_value = min_interest_value;
  //narf_keypoint_detector.getParameters ().calculate_sparse_interest_image = false;
  //narf_keypoint_detector.getParameloadters ().add_points_on_straight_edges = true;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& keypoints_cloud = *keypoints_cloud_ptr;
  
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    range_image_widget.spinOnce ();  // process GUI events
    pcl_sleep (0.01);
    
    bool got_new_range_image = false;
    if (received_new_depth_data && depth_image_mutex.try_lock ())
    {
      received_new_depth_data = false;
      
      //unsigned long time_stamp = depth_image_ptr->getTimeStamp ();
      //int frame_id = depth_image_ptr->getFrameID ();
      const unsigned short* depth_map = depth_image_ptr->getDepthMetaData ().Data ();
      int width = depth_image_ptr->getWidth (), height = depth_image_ptr->getHeight ();
      float center_x = width/2, center_y = height/2;
      float focal_length_x = depth_image_ptr->getFocalLength (), focal_length_y = focal_length_x;
      // float original_angular_resolution = asinf (0.5f*float (width)/float (focal_length_x)) / (0.5f*float (width));
      float desired_angular_resolution = angular_resolution;
      range_image_planar.setDepthImage (depth_map, width, height, center_x, center_y,
                                        focal_length_x, focal_length_y, desired_angular_resolution);
      depth_image_mutex.unlock ();
      got_new_range_image = !range_image_planar.empty ();
    }
    
    if (!got_new_range_image)
      continue;
    
    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    if (set_unseen_to_max_range)
      range_image_planar.setUnseenToMaxRange ();
    narf_keypoint_detector.setRangeImage (&range_image_planar);
    pcl::PointCloud<int> keypoint_indices;
    double keypoint_extraction_start_time = pcl::getTime();
    narf_keypoint_detector.compute (keypoint_indices);
    double keypoint_extraction_time = pcl::getTime()-keypoint_extraction_start_time;
    std::cout << "Found "<<keypoint_indices.size ()<<" key points. "
              << "This took "<<1000.0*keypoint_extraction_time<<"ms.\n";
    
    // ----------------------------------------------
    // -----Show keypoints in range image widget-----
    // ----------------------------------------------
    range_image_widget.showRangeImage (range_image_planar, 0.5f, 10.0f);
    //for (std::size_t i=0; i<keypoint_indices.size (); ++i)
      //range_image_widget.markPoint (keypoint_indices[i]%range_image_planar.width,
                                    //keypoint_indices[i]/range_image_planar.width,
                                    //pcl::visualization::Vector3ub (0,255,0));
    
    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> color_handler_cloud
      (range_image_planar_ptr, 0, 0, 0);
    if (!viewer.updatePointCloud<pcl::PointWithRange> (range_image_planar_ptr, color_handler_cloud, "range image"))
      viewer.addPointCloud<pcl::PointWithRange> (range_image_planar_ptr, color_handler_cloud, "range image");
    
    keypoints_cloud.resize (keypoint_indices.size ());
    for (std::size_t i=0; i<keypoint_indices.size (); ++i)
      keypoints_cloud[i].getVector3fMap () =
        range_image_planar[keypoint_indices[i]].getVector3fMap ();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler_keypoints
      (keypoints_cloud_ptr, 0, 255, 0);
    if (!viewer.updatePointCloud (keypoints_cloud_ptr, color_handler_keypoints, "keypoints"))
      viewer.addPointCloud (keypoints_cloud_ptr, color_handler_keypoints, "keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  }

  interface->stop ();
}
