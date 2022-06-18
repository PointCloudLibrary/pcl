/* \author Bastian Steder */

/* ---[ */
#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <mutex>

std::string device_id = "#1";

float angular_resolution = -1.0f;

std::mutex depth_image_mutex;
openni_wrapper::DepthImage::Ptr depth_image_ptr;
bool received_new_depth_data = false;

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
  
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range Image");
  
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.addCoordinateSystem (1.0f, "global");
  viewer.setBackgroundColor (1, 1, 1);
  
  // Set the viewing pose so that the openni cloud is visible
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
  
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();  // process 3D Viewer events
    range_image_widget.spinOnce ();  // process Image Viewer events
    pcl_sleep (0.01);
    
    bool got_new_range_image = false;
    if (received_new_depth_data && depth_image_mutex.try_lock ())
    {
      received_new_depth_data = false;

      int frame_id = depth_image_ptr->getFrameID ();
      std::cout << "Visualizing frame "<<frame_id<<"\n";
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
    
    // Show range image in the image widget
    range_image_widget.showRangeImage (range_image_planar, 0.5f, 10.0f);
    
    // Show range image in the 3D viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> color_handler_cloud
      (range_image_planar_ptr, 0, 0, 0);
    if (!viewer.updatePointCloud<pcl::PointWithRange> (range_image_planar_ptr, color_handler_cloud, "range image"))
      viewer.addPointCloud<pcl::PointWithRange> (range_image_planar_ptr, color_handler_cloud, "range image");
  }

  interface->stop ();
}
