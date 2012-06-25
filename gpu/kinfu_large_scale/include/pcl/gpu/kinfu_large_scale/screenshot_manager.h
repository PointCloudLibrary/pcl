#ifndef PCL_SCREENSHOT_MANAGER_H_
#define PCL_SCREENSHOT_MANAGER_H_

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/containers/kernel_containers.h>
#include <pcl/gpu/kinfu_large_scale/pixel_rgb.h> 
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp> 

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/flann/miniflann.hpp> 

namespace pcl
{
  namespace gpu
  {  
    /** \brief Screenshot Manager saves a screenshot with the corresponding camera pose from Kinfu. Please create a folder named "KinFuSnapshots" in the folder where you call kinfu.
      * \author Francisco Heredia
      */
    class ScreenshotManager
    {
      public:

        typedef pcl::gpu::PixelRGB PixelRGB;

        /** Constructor */
        ScreenshotManager();

        /** Destructor */
        ~ScreenshotManager(){}

        /**Save Screenshot*/
        void
        saveImage(Eigen::Affine3f camPose, PtrStepSz<const PixelRGB> rgb24);

      private:

        /**Write camera pose to file*/
        void 
        writePose(Eigen::Vector3f teVecs, Eigen::Matrix<float, 3, 3, Eigen::RowMajor> erreMats, std::string filename_pose);

        /**Counter of the number of screenshots taken*/
        int screenshot_counter;
    };
  }
}

#endif // PCL_SCREENSHOT_MANAGER_H_
