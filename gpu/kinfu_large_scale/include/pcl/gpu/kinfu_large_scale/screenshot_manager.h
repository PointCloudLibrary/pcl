#ifndef PCL_SCREENSHOT_MANAGER_H_
#define PCL_SCREENSHOT_MANAGER_H_

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/pcl_exports.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/containers/kernel_containers.h>
#include <pcl/gpu/kinfu_large_scale/pixel_rgb.h> 
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp> 

#include <pcl/io/png_io.h>


namespace pcl
{
  namespace gpu
  {  
    /** \brief Screenshot Manager saves a screenshot with the corresponding camera pose from Kinfu. Please create a folder named "KinFuSnapshots" in the folder where you call kinfu.
      * \author Francisco Heredia
      */
    class PCL_EXPORTS ScreenshotManager
    {
      public:

        typedef pcl::gpu::PixelRGB PixelRGB;

        /** Constructor */
        ScreenshotManager();

        /** Destructor */
        ~ScreenshotManager(){}
        
        /** \brief Sets Depth camera intrinsics
          * \param[in] fx focal length x 
          * \param[in] fy focal length y
          * \param[in] cx principal point x
          * \param[in] cy principal point y
          */
        void
        setDepthIntrinsics (float fx = 575.816f, float fy = 575.816f, float cx = 319.5f, float cy = 239.5f);

        /**Save Screenshot*/
        void
        saveImage(const Eigen::Affine3f &camPose, PtrStepSz<const PixelRGB> rgb24);

      private:

        /**Write camera pose to file*/
        void 
        writePose(const std::string &filename_pose, const Eigen::Vector3f &teVecs, const Eigen::Matrix<float, 3, 3, Eigen::RowMajor> &erreMats);

        /**Counter of the number of screenshots taken*/
        int screenshot_counter;
        
        /** \brief Intrinsic parameters of depth camera. */
        float fx_, fy_, cx_, cy_;
    };
  }
}

#endif // PCL_SCREENSHOT_MANAGER_H_
