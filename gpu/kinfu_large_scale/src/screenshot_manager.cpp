/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Francisco Heredia, Technical University Eindhoven, (f.j.mysurname.soriano < aT > tue.nl)
 */



#ifndef PCL_SCREENSHOT_MANAGER_CPP_
#define PCL_SCREENSHOT_MANAGER_CPP_
#include <pcl/gpu/kinfu_large_scale/screenshot_manager.h>

namespace pcl
{
  namespace kinfuLS
  {
      ScreenshotManager::ScreenshotManager()
      {
        boost::filesystem::path p ("KinFuSnapshots"); 
        boost::filesystem::create_directory (p);
        screenshot_counter = 0;
        setCameraIntrinsics();
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      void
      ScreenshotManager::saveImage(const Eigen::Affine3f &camPose, pcl::gpu::PtrStepSz<const PixelRGB> rgb24)
      {

        PCL_WARN ("[o] [o] [o] [o] Saving screenshot [o] [o] [o] [o]\n");

        std::string file_extension_image = ".png";
        std::string file_extension_pose = ".txt";
        std::string filename_image = "KinFuSnapshots/";
        std::string filename_pose = "KinFuSnapshots/";

        // Get Pose
        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> erreMats = camPose.linear ();
                    Eigen::Vector3f teVecs = camPose.translation ();

                    // Create filenames
                    filename_pose += std::to_string(screenshot_counter) + file_extension_pose;
                    filename_image += std::to_string(screenshot_counter) + file_extension_image;

                    // Write files
                    writePose (filename_pose, teVecs, erreMats);
          
        // Save Image
        pcl::io::saveRgbPNGFile (filename_image, (unsigned char*)rgb24.data, 640,480);
          
        screenshot_counter++;
      }
      
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      void
      ScreenshotManager::setCameraIntrinsics (float focal, float height, float width)
      {
        focal_ = focal;
        height_ = height;
        width_ = width;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      void 
      ScreenshotManager::writePose(const std::string &filename_pose, const Eigen::Vector3f &teVecs, const Eigen::Matrix<float, 3, 3, Eigen::RowMajor> &erreMats) const
      {
          std::ofstream poseFile;
          poseFile.open (filename_pose.c_str());

          if (poseFile.is_open())
          {
            poseFile << "TVector" << std::endl << teVecs << std::endl << std::endl 
                    << "RMatrix" << std::endl << erreMats << std::endl << std::endl 
                    << "Camera Intrinsics: focal height width" << std::endl << focal_ << " " << height_ << " " << width_ << std::endl << std::endl;
            poseFile.close ();
          }
          else
          {
            PCL_WARN ("Unable to open/create output file for camera pose!\n");
          }
        }  

  } // namespace kinfuLS
} //namespace pcl

#endif // PCL_SCREENSHOT_MANAGER_CPP_
