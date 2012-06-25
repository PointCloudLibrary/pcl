#ifndef PCL_SCREENSHOT_MANAGER_IMPL_HPP_
#define PCL_SCREENSHOT_MANAGER_IMPL_HPP_

#include <pcl/gpu/kinfu_large_scale/screenshot_manager.h>

namespace pcl
{
  namespace gpu
  {
     ScreenshotManager::ScreenshotManager()
     {
       boost::filesystem::path p("KinFuSnapshots"); 
       boost::filesystem::create_directory(p);
       screenshot_counter = 0;
     }

     void
     ScreenshotManager::saveImage(Eigen::Affine3f camPose, PtrStepSz<const PixelRGB> rgb24)
     {

       std::string file_extension_image = ".png";
       std::string file_extension_pose = ".txt";
       std::string filename_image = "KinFuSnapshots/";
       std::string filename_pose = "KinFuSnapshots/";

       //Get Pose
		    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> erreMats = camPose.linear();
		    Eigen::Vector3f teVecs = camPose.translation();

		    //Create filenames
		    filename_pose = filename_pose + boost::lexical_cast<std::string>(screenshot_counter) + file_extension_pose;
		    filename_image = filename_image + boost::lexical_cast<std::string>(screenshot_counter) + file_extension_image;

		    //Write files
		    writePose(teVecs, erreMats, filename_pose);

    		//Save Image				
        IplImage* image = cvCreateImage(cvSize(640, 480), 8, 3);
        memcpy(image->imageData, rgb24.data, 640 * 480 * 3); 
        cv::Mat mat(image);         
        cv::cvtColor(mat, mat, CV_BGR2RGB);

        char buf2[4096]; 
        sprintf (buf2, "KinFuSnapshots/%d.jpg", screenshot_counter++); 
        cvSaveImage(buf2, image);
        cvReleaseImage(&image); 

     }

     void 
     ScreenshotManager::writePose(Eigen::Vector3f teVecs, Eigen::Matrix<float, 3, 3, Eigen::RowMajor> erreMats, std::string filename_pose)
     {
        std::ofstream poseFile;
        poseFile.open (filename_pose.c_str());

        if (poseFile.is_open())
        {
          poseFile << "TVector" << std::endl << teVecs << std::endl << std::endl << "RMatrix" << std::endl << erreMats << std::endl << std::endl;
          poseFile.close();
        }
        else
        {
          std::cout << "unable to open output pose file!" << std::endl;
        }
      }  

  } // namespace gpu
} //namespace pcl

#endif // PCL_SCREENSHOT_MANAGER_IMPL_HPP_
