#pragma once

#include<pcl/gpu/containers/device_array.hpp>
#include<pcl/gpu/containers/kernel_containers.hpp>

#include<boost/shared_ptr.hpp>
#include<string>

#include "vector_types.h"

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

            bool grab(PtrStepSz<const unsigned short>& depth, PtrStepSz<const uchar3>& rgb24);		

            //parameters taken from camera/oni
            float depth_focal_length_VGA;		
            float baseline; // mm
            int shadow_value;
            int no_sample_value;
            double pixelSize; //mm

            unsigned short max_depth; //mm
        private:
            struct Impl;
            boost::shared_ptr<Impl> impl;
            void getParams();
        };	
    }
};