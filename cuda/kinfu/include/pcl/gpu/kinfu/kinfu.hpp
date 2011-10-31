/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 *///M*/



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