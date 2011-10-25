#define _CRT_SECURE_NO_DEPRECATE

#include <iostream>
#include <opencv2/opencv.hpp>
#include "pcl/gpu/kinfu/openni_capture.hpp"
#include "pcl/gpu/kinfu/kinfu.hpp"
#include "pcl/gpu/kinfu/video_recorder.hpp"


using namespace std;
using namespace cv;
using namespace pcl;
using namespace pcl::gpu;

void just_capture()
{
    try
    {
        //CaptureOpenNI cap("d:/onis/20111013-224932.oni");
        CaptureOpenNI cap(0);		

        Mat bgr, du8;
        for(;;)
        {
            Mat depth, rgb24;
            if (!cap.grab(depth, rgb24))
            {
                cout << "Can't grab" << endl;
                break;
            }

            if (!depth.empty())
            {
                depth.convertTo(du8, CV_8U, 0.1);
                imshow("depth", du8);				
            }

            if(!rgb24.empty())
            {
                cvtColor(rgb24, bgr, CV_RGB2BGR);									
                imshow("image", bgr);
            }

            int key = cv::waitKey(3);
            if (key == 27)
                break;
        }
    }
    catch(const std::exception& /*e*/) { cout << "exception" << endl; }	
}

int main()
{
    //return just_capture(), 0;    
    
    try
    {
        BufferedRecorder recorder;

        //CaptureOpenNI cap(0);
        CaptureOpenNI cap("d:/onis/20111013-224932.oni");
        //CaptureOpenNI cap("d:/onis/20111013-224551.oni");
        //CaptureOpenNI cap("d:/onis/20111013-224719.oni");

        KinfuTracker kinfu;
        kinfu.fx = kinfu.fy = cap.depth_focal_length_VGA;

        Mat bgr, du8;
        Mat depth, rgb24;
        for(int i = 0;;++i)
        {
            cout << i << endl;

            if (!cap.grab(depth, rgb24))
            {
                cout << "Can't grab" << endl;
                break;
            }

            cvtColor(rgb24, bgr, CV_RGB2BGR);
            imshow("image", bgr);

            double maxval;
            cv::minMaxLoc(depth, 0, &maxval);
            depth.convertTo(du8, CV_8U, 255.0/maxval);
            imshow("depth", du8);

            Mat res = kinfu(depth, bgr);
            if (!res.empty())
                imshow("3d", res);

            int key = cv::waitKey(3);
            if (key == 27)
                break;

            recorder.push_back(bgr, du8, res);            
        }

        recorder.save("video.avi");
    }
    catch(const std::exception& /*e*/) { cout << "exception" << endl; }	

    return 0;
}