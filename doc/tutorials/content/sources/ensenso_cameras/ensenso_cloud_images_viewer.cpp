/**
 * @file pcl_ensenso_cloud_images_viewer.cpp
 * @brief Display both Ensenso images & cloud using the PCL Ensenso grabber (and OpenCV)
 * @author Victor Lamoine
 * @date November 2014
 */

#include <algorithm>
#include <iostream>
#include <thread>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/memory.h>
#include <pcl/console/print.h>
#include <pcl/io/ensenso_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std::chrono_literals;

/** @brief Convenience typedef */
typedef pcl::PointXYZ PointT;

/** @brief Convenience typedef */
typedef pcl::PointCloud<PointT> PointCloudT;

/** @brief Convenience typdef for the Ensenso grabber callback */
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;
typedef pcl::shared_ptr<PairOfImages> PairOfImagesPtr;

/** @brief CloudViewer pointer */
pcl::visualization::CloudViewer::Ptr viewer_ptr;

/** @brief Pair of Ensenso images */
pcl::EnsensoGrabber::Ptr ensenso_ptr;

/** @brief Get OpenCV image type corresponding to the parameters given
 * @param channels number of channels in the image
 * @param bpe bytes per element
 * @param isFlt is float
 * @returns the OpenCV type
 */
int
getOpenCVType (const std::string &type)
{
  if (type == "CV_32FC1")
    return CV_32FC1;
  else if (type == "CV_32FC2")
    return CV_32FC2;
  else if (type == "CV_32FC3")
    return CV_32FC3;
  else if (type == "CV_32FC4")
    return CV_32FC4;
  else if (type == "CV_64FC1")
    return CV_64FC1;
  else if (type == "CV_64FC2")
    return CV_64FC2;
  else if (type == "CV_64FC3")
    return CV_64FC3;
  else if (type == "CV_64FC4")
    return CV_64FC4;
  else if (type == "CV_8UC1")
    return CV_8UC1;
  else if (type == "CV_8UC2")
    return CV_8UC2;
  else if (type == "CV_8UC3")
    return CV_8UC3;
  else if (type == "CV_8UC4")
    return CV_8UC4;
  else if (type == "CV_16UC1")
    return CV_16UC1;
  else if (type == "CV_16UC2")
    return CV_16UC2;
  else if (type == "CV_16UC3")
    return CV_16UC3;
  else if (type == "CV_16UC4")
    return CV_16UC4;
  else if (type == "CV_32SC1")
    return CV_32SC1;
  else if (type == "CV_32SC2")
    return CV_32SC2;
  else if (type == "CV_32SC3")
    return CV_32SC3;
  else if (type == "CV_32SC4")
    return CV_32SC4;

  return (-1);
}

/** @brief Process and/or display Ensenso grabber images
 * @param[in] cloud The Ensenso point cloud
 * @param[in] images Pair of Ensenso images (raw or with overlay)
 * @warning Image type changes if a calibration pattern is discovered/lost;
 * check @c images->first.encoding */
void
grabberCallback (const PointCloudT::Ptr& cloud,
                 const PairOfImagesPtr& images)
{
  viewer_ptr->showCloud (cloud);
  unsigned char *l_image_array = reinterpret_cast<unsigned char *> (&images->first.data[0]);
  unsigned char *r_image_array = reinterpret_cast<unsigned char *> (&images->second.data[0]);

  std::cout << "Encoding: " << images->first.encoding << std::endl;
  int type = getOpenCVType (images->first.encoding);
  cv::Mat l_image (images->first.height, images->first.width, type, l_image_array);
  cv::Mat r_image (images->first.height, images->first.width, type, r_image_array);
  cv::Mat im (images->first.height, images->first.width * 2, type);

  im.adjustROI (0, 0, 0, -images->first.width);
  l_image.copyTo (im);
  im.adjustROI (0, 0, -images->first.width, images->first.width);
  r_image.copyTo (im);
  im.adjustROI (0, 0, images->first.width, 0);
  cv::imshow ("Ensenso images", im);
  cv::waitKey (10);
}

/** @brief Main function
 * @return Exit status */
int
main (void)
{
  viewer_ptr.reset (new pcl::visualization::CloudViewer ("3D Viewer"));
  ensenso_ptr.reset (new pcl::EnsensoGrabber);
  ensenso_ptr->openDevice (0);
  ensenso_ptr->openTcpPort ();
  //ensenso_ptr->initExtrinsicCalibration (5); // Disable projector if you want good looking images.
  // You won't be able to detect a calibration pattern with the projector enabled!

  std::function<void (const PointCloudT::Ptr&, const PairOfImagesPtr&)> f = grabberCallback;
  ensenso_ptr->registerCallback (f);

  cv::namedWindow ("Ensenso images", cv::WINDOW_AUTOSIZE);
  ensenso_ptr->start ();

  while (!viewer_ptr->wasStopped ())
  {
    PCL_INFO("FPS: %f\n", ensenso_ptr->getFramesPerSecond ());
    std::this_thread::sleep_for(500ms);
  }

  ensenso_ptr->stop ();
  //cv::destroyAllWindows (); // Doesn't work

  ensenso_ptr->closeDevice ();
  ensenso_ptr->closeTcpPort ();
  return 0;
}
