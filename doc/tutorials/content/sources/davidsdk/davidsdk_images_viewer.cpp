#include <iostream>
#include <pcl/io/davidsdk_grabber.h>
#include <pcl/console/print.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/** @brief A pointer to the PCL DavidSDKGrabber object */
pcl::DavidSDKGrabber::Ptr davidsdk_ptr;

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

/** @brief Process and/or display DavidSDKGrabber image
 * @param[in] image davidSDK image */
void
grabberCallback (const pcl::PCLImage::Ptr& image)
{
  unsigned char *image_array = reinterpret_cast<unsigned char *> (&image->data[0]);

  int type = getOpenCVType (image->encoding);
  cv::Mat cv_image (image->height, image->width, type, image_array);

  cv::imshow ("davidSDK images", cv_image);
  cv::waitKey (5);
}

/** @brief Asks the user to press enter to continue
 * @param[in] str Message to display */
void
waitForUser (std::string str = "Press enter to continue")
{
  PCL_WARN (str.c_str ());
  std::cout.flush ();
  getc (stdin);
}

/** @brief Main function
 * @param argc
 * @param argv
 * @return Exit status */
int
main (int argc,
      char *argv[])
{
  if (argc != 2)
  {
    PCL_ERROR ("Usage:\n%s 192.168.100.65\n", argv[0]);
    return (-1);
  }

  davidsdk_ptr.reset (new pcl::DavidSDKGrabber);
  davidsdk_ptr->connect (argv[1]);

  if (!davidsdk_ptr->isConnected ())
    return (-1);
  PCL_WARN ("davidSDK connected\n");

  std::function<void (const pcl::PCLImage::Ptr&)> f = grabberCallback;
  davidsdk_ptr->registerCallback (f);
  davidsdk_ptr->start ();
  waitForUser ("Press enter to quit");

  return (0);
}
