#pragma once

#include<pcl/gpu/containers/device_array.hpp>
#include<vector>
#include "vector_types.h"

#include <Eigen/Core>

namespace pcl
{
    namespace gpu
    {
        class KinfuTracker
        {
        public:	            
            typedef unsigned short ushort;
            typedef DeviceArray2D<float> MapArr;
            typedef DeviceArray2D<ushort> DepthMap;
            typedef DeviceArray2D<uchar3> View;

            //typedef Eigen::Matrix3f Matrix3f;
            typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3f;
            typedef Eigen::Vector3f Vector3f;
            

            KinfuTracker();

            float fx, fy, cx, cy;		

            int icp_iterations_number; //somewhere 20 I think
            float distThres;
            float normalThres;

            Vector3f volume_size;        // sizeof volume in mm
            Matrix3f init_Rcam; // init camera rotaion in volume coo space
            Vector3f init_tcam;          // init camera pos in volume coo space

            float light_x, light_y, light_z;

            void operator()(const DepthMap& depth, View& view);
        private:
            
            typedef Eigen::Matrix<float, 6, 6> Matrix6f;
            typedef Eigen::Matrix<float, 6, 1> Vector6f;

            DeviceArray2D<float> volume;

            MapArr vmap_curr, nmap_curr;
            MapArr vmap_g_curr, nmap_g_curr;
            MapArr vmap_g_prev, nmap_g_prev;
            DeviceArray2D<unsigned short> depth_curr;
            DeviceArray2D<float> gbuf;
            DeviceArray<float> sumbuf; 

            std::vector<Matrix3f> rmats;
            std::vector<Vector3f> tvecs;

            int global_time;

            void allocateBufffers(int rows, int cols);
            void estimateTrel(const MapArr& v_dst, const MapArr& n_dst, const MapArr& v_src, Matrix3f& Rrel, Vector3f& trel);
        };	
    }
};