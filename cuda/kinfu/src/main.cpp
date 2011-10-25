#define _CRT_SECURE_NO_DEPRECATE

#include <iostream>
//#include <opencv2/opencv.hpp>
#include "pcl/point_cloud.h"
#include "pcl/gpu/kinfu/openni_capture.hpp"
#include "pcl/gpu/kinfu/kinfu.hpp"
//#include "pcl/gpu/kinfu/video_recorder.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>

using namespace std;
using namespace pcl;
using namespace pcl::gpu;

int main()
{
    //return just_capture(), 0;    
    try
    {
        //BufferedRecorder recorder;
        CaptureOpenNI cap(0);
        //CaptureOpenNI cap("d:/onis/20111013-224932.oni");
        //CaptureOpenNI cap("d:/onis/20111013-224551.oni");
        //CaptureOpenNI cap("d:/onis/20111013-224719.oni");

        KinfuTracker kinfu;
        kinfu.fx = kinfu.fy = cap.depth_focal_length_VGA;

        KinfuTracker::DepthMap depth_device;
        KinfuTracker::View view_device;

        pcl::visualization::ImageViewer viewer;
        pcl::visualization::ImageViewer viewer3d;
        vector<uchar3> view3d_host;
                                
        for(int i = 0;;++i)
        {                        
            cout << i << endl;
            
            PtrStepSz<const unsigned short> depth;
            PtrStepSz<const uchar3> rgb24;
            if (!cap.grab(depth, rgb24))
            {
                cout << "Can't grab" << endl;
                break;
            }

            viewer.showRGBImage ((unsigned char*)rgb24.data, rgb24.cols, rgb24.rows);
            viewer.spinOnce(3);

            /*double maxval;
            cv::minMaxLoc(depth, 0, &maxval);
            depth.convertTo(du8, CV_8U, 255.0/maxval);
            imshow("depth", du8);*/

            depth_device.upload(depth.data, depth.step, depth.rows, depth.cols);

            kinfu(depth_device, view_device);
            if (!view_device.empty())
            {
                int cols;
                view_device.download(view3d_host, cols);
                viewer3d.showRGBImage((unsigned char*)&view3d_host[0], view_device.cols(), view_device.rows());
            }
            
            viewer3d.spinOnce(3);
           
            //recorder.push_back(bgr, du8, res);            
        }

        //recorder.save("video.avi");
    }
    catch(const std::exception& /*e*/) { cout << "exception" << endl; }	

    return 0;
}