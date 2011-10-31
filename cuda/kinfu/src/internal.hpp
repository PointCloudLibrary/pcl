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
 */

#pragma once

#include "pcl/gpu/containers/device_array.hpp"
#include "pcl/gpu/utils/safe_call.hpp"

namespace pcl
{
	namespace device
	{
        typedef unsigned short ushort;
        typedef DeviceArray2D<float> MapArr;
        typedef DeviceArray2D<ushort> DepthMap;
               

		enum { VOLUME_X = 512, VOLUME_Y = 512, VOLUME_Z = 512 };

		struct Intr
		{
			float fx, fy, cx, cy;
			Intr() {}
			Intr(float fx_, float fy_, float cx_, float cy_) : fx(fx_), fy(fy_), cx(cx_), cy(cy_) {}

            Intr operator()(int level_index) const 
            { 
                int div = 1 << level_index; 
                return Intr(fx/div, fy/div, cx/div, cy/div);
            }            
		};

		struct Mat33
		{
			float3 data[3];			
		};

		struct LightSource
		{
			float3 pos[1];						
			int number;
		};

        void bilateralFilter(const DepthMap& src, DepthMap& dst);
        void pyrDown(const DepthMap& src, DepthMap& dst);

        void createVMap(const Intr& intr, const DepthMap& depth, MapArr& vmap);
        void createNMap(const MapArr& vmap, MapArr& nmap);
        void compteNormalsEigen(const MapArr& vmap, MapArr& nmap);

        void tranformMaps(const MapArr& vmap_src, const MapArr& nmap_src, const Mat33& Rmat, const float3& tvec, MapArr& vmap_dst, MapArr& nmap_dst);

        struct float8 { float x, y, z, w, f1, f2, f3, f4; };
        template<typename T> void convert(const MapArr& vmap, DeviceArray2D<T>& output);		             
		
        //icp        

        void findCoresp(const MapArr& vmap_g_curr, const MapArr& nmap_g_curr, const Mat33& Rprev_inv, const float3& tprev, const Intr& intr, 
            const MapArr& vmap_g_prev, const MapArr& nmap_g_prev, float distThres, float angleThres, PtrStepSz<short2> coresp);

        typedef float work_type;  
        //typedef double work_type;
		void estimateTransform(const MapArr& v_dst, const MapArr& n_dst, const MapArr& v_src, const PtrStepSz<short2>& coresp,
                DeviceArray2D<work_type>& gbuf, DeviceArray<work_type>& mbuf, work_type* matrixA_host, work_type* vectorB_host);

		//tsdf
		void initVolume(PtrStepSz<short2> array);
		void integrateTsdfVolume(const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size, 
			const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, PtrStep<short2> volume);
		

		//raycast
		void raycast(const Mat33& Rcurr, const float3& tcurr, const Intr& intr, const float3& volume_size, 
				const PtrStep<short2>& volume, MapArr vmap, MapArr nmap);

		void generateImage(const MapArr& vmap, const MapArr& nmap, const LightSource& light, PtrStepSz<uchar3> dst);


        inline bool valid_host(float value)
        {
            return *reinterpret_cast<int*>(&value) != 0x7fffffff;
        }
	}
}