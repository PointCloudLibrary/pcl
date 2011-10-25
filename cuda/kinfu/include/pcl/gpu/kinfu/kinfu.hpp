#pragma once

#include<opencv2/core/core.hpp>
#include<opencv2/gpu/gpu.hpp>
#include<pcl/gpu/containers/device_array.hpp>
#include<vector>

namespace pcl
{
    namespace gpu
    {


        class KinfuTracker
        {
        public:	
            typedef DeviceArray2D<float> MapArr;
            typedef DeviceArray2D<ushort> DepthMap;

            KinfuTracker();

            float fx, fy, cx, cy;		

            int icp_iterations_number; //somewhere 20 I think
            float distThres;
            float normalThres;

            cv::Matx31f volume_size; // sizeof volume in mm
            cv::Matx33f init_Rcam; // init camera rotaion in volume coo space
            cv::Matx31f init_tcam; // init camera pos in volume coo space


            float light_x, light_y, light_z;

            cv::Mat operator()(const cv::Mat& depth, const cv::Mat& image);
        private:
            DeviceArray2D<float> volume;
            cv::gpu::GpuMat image;

            MapArr vmap_curr, nmap_curr;
            MapArr vmap_g_curr, nmap_g_curr;
            MapArr vmap_g_prev, nmap_g_prev;
            DeviceArray2D<unsigned short> depth_curr;
            DeviceArray2D<float> gbuf;
            DeviceArray<float> sumbuf; 

            std::vector<cv::Matx33f> rmats;
            std::vector<cv::Matx31f> tvecs;

            int global_time;

            cv::Mat result_host;

            void allocateBufffers(int rows, int cols);
            void estimateTrel(const MapArr& v_dst, const MapArr& n_dst, const MapArr& v_src, cv::Matx33f& Rrel, cv::Matx31f trel);
        };	
    }
};


//debugging fuctions

typedef pcl::gpu::DeviceArray2D<float> MapArr;
typedef pcl::gpu::DeviceArray2D<unsigned short> DepthMap;

cv::Mat convertToMat(const MapArr& map);
cv::Mat cvtCoresp(const pcl::gpu::DeviceArray2D<int>& coresp);

void show_vmap(const MapArr& vmap);

void show_qnan_map(const MapArr& vmap);

void show_vmap(const MapArr& vmap, const MapArr& nmap);

void show_mask(const pcl::gpu::DeviceArray2D<int>& coresp);

void show_qnan_map(const MapArr& map, const char* name);


void show_pa_mask(const MapArr& vmap_prev, const MapArr& nmap_prev, const MapArr& vmap_curr, const MapArr& nmap_curr, const DepthMap& depth);

void drawCoresShow(const cv::Mat& img_prev, const cv::Mat& img_curr, const pcl::gpu::DeviceArray2D<int>& coresp);

void slice_volume(const pcl::gpu::DeviceArray2D<float>& volume);

cv::Mat generateNormalsView(const MapArr& normals);