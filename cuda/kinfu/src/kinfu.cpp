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

#include<iostream>
#include<fstream>

#include "pcl/common/time.h"
#include "pcl/gpu/utils/timers_opencv.hpp"
#include "pcl/gpu/kinfu/kinfu.hpp"
#include "internal.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/LU> 

#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

using namespace std;		
using namespace pcl::device;
using namespace Eigen;

template<class D, class Matx> 
D& device_cast(Matx& matx) { return *reinterpret_cast<D*>(matx.data()); }

cudaStream_t pcl::device::stream = 0;

pcl::gpu::KinfuTracker::KinfuTracker(int rows, int cols) : rows_(rows), cols_(cols), global_time(0) 
{
	rmats.reserve(30000);
	tvecs.reserve(30000);

	fx = fy = 525.f; cx = cols/2; cy = rows/2;	

    const int iters[] = {10, 5, 4};
    std::copy(iters, iters + LEVELS, icp_iterations_numbers);    
	
	 distThres = 100; //mm
	angleThres = 20.f * 3.14159254f/180.f;

    tranc_dist = 30;

    volume_size = Vector3f::Constant(2000);
    init_Rcam   = Matrix3f::Identity();
    init_tcam   = volume_size * 0.5f;
    
    init_tcam(2) -= volume_size(2)/2 * 1.5;

    light_pos = volume_size * (-10.f);

    allocateBufffers(rows, cols);
}

void pcl::gpu::KinfuTracker::allocateBufffers(int rows, int cols)
{
    volume.create(device::VOLUME_Y * device::VOLUME_Z, device::VOLUME_X);
    device::initVolume(volume);

    depths_curr.resize(LEVELS);
    vmaps_g_curr.resize(LEVELS);
    nmaps_g_curr.resize(LEVELS);

    vmaps_g_prev.resize(LEVELS);
    nmaps_g_prev.resize(LEVELS);

    vmaps_curr.resize(LEVELS);
    nmaps_curr.resize(LEVELS);

    coresps.resize(LEVELS);
    
    for(int i = 0; i < LEVELS; ++i)
    {
        int pyr_rows = rows >> i;
        int pyr_cols = cols >> i;

        depths_curr[i].create(pyr_rows, pyr_cols);

        vmaps_g_curr[i].create(pyr_rows*3, pyr_cols);
        nmaps_g_curr[i].create(pyr_rows*3, pyr_cols);

        vmaps_g_prev[i].create(pyr_rows*3, pyr_cols);
        nmaps_g_prev[i].create(pyr_rows*3, pyr_cols);

        vmaps_curr[i].create(pyr_rows*3, pyr_cols);
        nmaps_curr[i].create(pyr_rows*3, pyr_cols);

        coresps[i].create(pyr_rows, pyr_cols);
    }
                             
    gbuf.create(27, 20*60);
    sumbuf.create(27);
}

void pcl::gpu::KinfuTracker::estimateTrel(const MapArr& v_dst, const MapArr& n_dst, const MapArr& v_src, const CorespMap& coresp, Matrix3f& Rinc, Vector3f& tinc)
{    
    float A_data[36];
    Matrix<float, 6, 1> b;
                    
    device::estimateTransform(v_dst, n_dst, v_src, coresp, gbuf, sumbuf, A_data, b.data());

    Map<Matrix6f> A(A_data);        

    Matrix<float, 6, 1> res = A.llt().solve(b);
    //Matrix<float, 6, 1> res = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

    //cout << b << endl;
    //cout << A << endl;
    //cout << res << endl;
     
    float alpha = res(0);
	float beta  = res(1);
	float gamma = res(2);
	float tx = res(3);
	float ty = res(4);
	float tz = res(5);

	float r11 =  cos(gamma)*cos(beta);
	float r12 = -sin(gamma)*cos(alpha) + cos(gamma)*sin(beta)*sin(alpha);
	float r13 =  sin(gamma)*sin(alpha) + cos(gamma)*sin(beta)*cos(alpha);

	float r21 =  sin(gamma)*cos(beta);
	float r22 =  cos(gamma)*cos(alpha) + sin(gamma)*sin(beta)*sin(alpha);
	float r23 = -cos(gamma)*sin(alpha) + sin(gamma)*sin(beta)*cos(alpha);

	float r31 = -sin(beta);
	float r32 =  cos(beta)*sin(alpha);
	float r33 =  cos(beta)*cos(alpha);

    float data_rotation[] = { r11, r12, r13, r21, r22, r23, r31, r32, r33 };
    float data_traslation[] = { tx, ty, tz };

    Rinc = Map<Matrix3f>(data_rotation);
    tinc = Map<Vector3f>(data_traslation);	
}

void slice_volume(const PtrStepSz<short2>& volume)
{
    using namespace cv::gpu;
    using namespace cv;
    GpuMat v(volume.rows, volume.cols, CV_32F, volume.data, volume.step);
    Mat slice;

    for(int z = 0; z < 512; ++z)
    {        
        v.rowRange(512 * z, 512 * (z+1)).download(slice);
        Mat image(slice.size(), CV_8UC3);

        for(int y = 0; y < 512; ++y)
        {
            short2 *vol = slice.ptr<short2>(y);
            Vec3b *img = image.ptr<Vec3b>(y);

            for(int x = 0; x < 512; ++x)
            {
                float tsdf = float(vol[x].x)/32767;
                Vec3b& color = img[x];

                if (vol[x].y == 0)
                {
                    color.val[0] = 0;
                    color.val[1] = 255;
                    color.val[2] = 0;
                }
                else
                {
                    int br = static_cast<int>(tsdf * 75 + 75 + 50);// [-1,0,1] -> [50,125,250]

                    if (br < 125)
                    {
                        color.val[0] = 0;
                        color.val[1] = 0;
                        color.val[2] = br;

                        if (br < 55)
                            color.val[1] = 128;

                    }
                    else
                    {
                        color.val[0] = br;
                        color.val[1] = 0;
                        color.val[2] = 0;

                        if (br > 195)
                            color.val[1] = 128;
                    }
                }
            }
        }
        cv::imshow("slice", image);
        cv::waitKey();
    }
}

void pcl::gpu::KinfuTracker::operator()(const DepthMap& depth_raw, View& view)
{				        	
	device::Intr intr(fx, fy, cx, cy);

    {
        ScopeTimerCV time("bilateral"); 
        //depth_raw.copyTo(depths_curr[0]);
        device::bilateralFilter(depth_raw, depths_curr[0]);
    }

    {
        ScopeTimerCV time("pyr-down-all"); 
        for(int i = 1; i < LEVELS; ++i)
            device::pyrDown(depths_curr[i-1], depths_curr[i]);
    }

    {
        ScopeTimerCV time("create-maps-all"); 
        for(int i = 0; i < LEVELS; ++i)
        {
            device::createVMap(intr(i), depths_curr[i], vmaps_curr[i]);
            device::createNMap(vmaps_curr[i], nmaps_curr[i]);
            //compteNormalsEigen(vmap_curr, nmap_curr);
        }
    }
    
	//can't perform more on first frame
	if (global_time == 0)
	{		
		rmats.push_back(init_Rcam);
		tvecs.push_back(init_tcam);		

        Mat33&  device_Rcam = device_cast<Mat33> (init_Rcam);
		float3& device_tcam = device_cast<float3>(init_tcam);
        
        Matrix3f init_Rcam_inv = init_Rcam.inverse();    
        Mat33&  device_Rcam_inv = device_cast<Mat33> (init_Rcam_inv);
        float3 device_volume_size = device_cast<float3>(volume_size);
        	
	    integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcam_inv, device_tcam, tranc_dist, volume);

        //slice_volume(volume);
        
        for(int i = 0; i < LEVELS; ++i)
            device::tranformMaps(vmaps_curr[i], nmaps_curr[i], device_Rcam, device_tcam, vmaps_g_prev[i], nmaps_g_prev[i]);
        		
		++global_time;
		view.release();
        return;
	}
    
	///////////////////////////////////////////////////////////////////////////////////////////
	// Iterative Closest Point 

	cout << "Starting ICP" << endl;
	Matrix3f Rprev = rmats[global_time - 1]; //  [Ri|ti] - pos of camera, i.e.
	Vector3f tprev = tvecs[global_time - 1]; //  tranfrom from camera to global coo space for (i-1)th camera pose
    Matrix3f Rprev_inv = Rprev.inverse(); //Rprev.t();
	
	Mat33&  device_Rprev     = device_cast<Mat33 >(Rprev);
	Mat33&  device_Rprev_inv = device_cast<Mat33 >(Rprev_inv);
	float3& device_tprev     = device_cast<float3>(tprev);
	    
    Matrix3f Rcurr = Rprev; // tranform to global coo for ith camera pose
    Vector3f tcurr = tprev; 


    {
        ScopeTimerCV time("icp-all"); 

        for(int level_index = LEVELS-1; level_index>=0; --level_index)
        {
            int iter_num = icp_iterations_numbers[level_index];

            MapArr& vmap_curr = vmaps_curr[level_index];
            MapArr& nmap_curr = nmaps_curr[level_index];

            MapArr& vmap_g_curr = vmaps_g_curr[level_index];
            MapArr& nmap_g_curr = nmaps_g_curr[level_index];

            MapArr& vmap_g_prev = vmaps_g_prev[level_index];
            MapArr& nmap_g_prev = nmaps_g_prev[level_index];

            CorespMap& coresp = coresps[level_index];
          
            ScopeTimerCV time("  icp-level"); 
            {
	            for(int iter = 0; iter < iter_num; ++iter) 
	            {			
                    ScopeTimerCV time("    icp-iter"); 
		            Mat33&  device_Rcurr = device_cast<Mat33> (Rcurr);
		            float3& device_tcurr = device_cast<float3>(tcurr);

                    device::tranformMaps(vmap_curr, nmap_curr, device_Rcurr, device_tcurr, vmap_g_curr, nmap_g_curr);

                    
                    {
                        //ScopeTimerCV time("      coresp");
                        findCoresp(vmap_g_curr, nmap_g_curr, device_Rprev_inv, device_tprev, intr(level_index), 
                                     vmap_g_prev, nmap_g_prev, distThres, angleThres, coresp);
                    }
                    
                   /* if (global_time == 5)
                    {
                        cv::gpu::GpuMat ma(coresp.rows(), coresp.cols(), CV_32S, coresp.ptr(), coresp.step());
                        cv::Mat cpu;
                        ma.download(cpu);

                        cv::imshow("coresp whi == -1", cpu == -1);
                        cv::waitKey();
                    }*/
                    
                    Matrix3f Rinc;
		            Vector3f tinc;
                    estimateTrel(vmap_g_prev, nmap_g_prev, vmap_g_curr, coresp, Rinc, tinc);

		            //compose
		            tcurr = Rinc * tcurr + tinc;
		            Rcurr = Rinc * Rcurr;        
	            }
            }
        }
    }
    
	//save tranform
	rmats.push_back(Rcurr);
	tvecs.push_back(tcurr);		

    //cout << "FinalR:\n" << Rcurr << endl;
    //cout << "FinalT:\n" << tcurr << endl;
	
	///////////////////////////////////////////////////////////////////////////////////////////
	// Volume integration

	cout << "Starting Volume integration" << endl;

	float3 device_volume_size = device_cast<float3>(volume_size);

    Matrix3f Rcurr_inv = Rcurr.inverse();    
    Mat33&  device_Rcurr_inv = device_cast<Mat33> (Rcurr_inv);
	float3& device_tcurr = device_cast<float3>(tcurr);	
	
    { 
        ScopeTimerCV time("tsdf"); 
	    integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcurr_inv, device_tcurr, tranc_dist, volume);
    }

    //slice_volume(volume);

   
	///////////////////////////////////////////////////////////////////////////////////////////
	// Ray casting     

	cout << "Starting raycasting" << endl;
    Mat33&  device_Rcurr = device_cast<Mat33> (Rcurr);


    cout << "Global time: " << global_time << endl;

    
    { 
        ScopeTimerCV time("ray-cast-all"); 
        for(int i = 0; i < LEVELS; ++i)   
            raycast(intr(i), device_Rcurr, device_tcurr, tranc_dist, device_volume_size, volume, vmaps_g_prev[i], nmaps_g_prev[i]);
    }
        
    /*for(int i = 0; i < LEVELS; ++i)
        device::tranformMaps(vmaps_curr[i], nmaps_curr[i], device_Rcurr, device_tcurr, vmaps_g_prev[i], nmaps_g_prev[i]);*/
                        
	///////////////////////////////////////////////////////////////////////////////////////////
	// image generation
    	 
	cout << "Starting image generation" << endl;
	device::LightSource light;
	light.number = 1;
    light.pos[0] = device_cast<float3>(light_pos);
	    	        
    view.create(rows_, cols_);

    { 
        ScopeTimerCV time("generate-view"); 
	    generateImage(vmaps_g_prev[0], nmaps_g_prev[0], light, view);
    }
	    
    ++global_time;
}



