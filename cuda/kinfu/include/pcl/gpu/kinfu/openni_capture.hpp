#pragma once

#include<pcl/gpu/containers/device_array.hpp>
#include<opencv2/core/core.hpp>
#include<string>

namespace pcl
{	
    namespace gpu
    {
        class CaptureOpenNI
        {
        public:				
            CaptureOpenNI(int device);
            CaptureOpenNI(const std::string& filename);
            ~CaptureOpenNI();

            bool grab(cv::Mat& depth, cv::Mat& rgb24);		

            //parameters taken from camera/oni
            float depth_focal_length_VGA;		
            float baseline; // mm
            int shadow_value;
            int no_sample_value;
            double pixelSize; //mm

            unsigned short max_depth; //mm
        private:
            struct Impl;
            cv::Ptr<Impl> impl;
            void getParams();
        };	
    }
};