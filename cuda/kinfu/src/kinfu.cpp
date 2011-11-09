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
#include "pcl/gpu/kinfu/kinfu.hpp"
#include "internal.hpp"
#include "host_map.hpp"

#include "vector_functions.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU> 

#ifdef HAVE_OPENCV
    #include <opencv2/opencv.hpp>
    #include <opencv2/gpu/gpu.hpp>
    #include "pcl/gpu/utils/timers_opencv.hpp"
#endif

using namespace std;		
using namespace pcl::device;
using namespace pcl::gpu;
using namespace Eigen;

template<class D, class Matx> 
D& device_cast(Matx& matx) { return *reinterpret_cast<D*>(matx.data()); }

pcl::gpu::KinfuTracker::KinfuTracker(int rows, int cols) : rows_(rows), cols_(cols), global_time(0) 
{
	rmats.reserve(30000);
	tvecs.reserve(30000);

	fx = fy = 525.f; cx = cols/2; cy = rows/2;	

    const int iters[] = {10, 5, 4};
    std::copy(iters, iters + LEVELS, icp_iterations_numbers);    
	
	 distThres = 100; //mm
	angleThres = sin(20.f * 3.14159254f/180.f);

    tranc_dist = 30;

    volume_size = Vector3f::Constant(3000);
    init_Rcam   = Matrix3f::Identity();// * AngleAxisf(-30.f/180*3.1415926, Vector3f::UnitX());
    init_tcam   = volume_size * 0.5f;
    
    init_tcam(2) -= volume_size(2)/2 * 1.2f;

    light_pos = volume_size * (-10.f);

    allocateBufffers(rows, cols);
    reset();
}

void pcl::gpu::KinfuTracker::reset()
{
     global_time = 0;
     rmats.clear();
     tvecs.clear();
     device::initVolume<volume_elem_type>(volume);
}

void pcl::gpu::KinfuTracker::allocateBufffers(int rows, int cols)
{
    volume.create(device::VOLUME_Y * device::VOLUME_Z, device::VOLUME_X);
    
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
    depthRawScaled.create(rows, cols);

    gbuf.create(27, 20*60);
    sumbuf.create(27);
}

//void findCorespCPU(const HostMap& vmap_g_curr, const HostMap& nmap_g_curr, const Matrix3f& Rprev_inv, const Vector3f& tprev, const Intr& intr, 
//                   const HostMap& vmap_g_prev, const HostMap& nmap_g_prev, float distThres, float angleThres, cv::Mat& coresp)
//{
//    int rows = vmap_g_curr.rows;
//    int cols = vmap_g_curr.cols;
//    coresp.create(rows, cols, CV_32S);
//
//    pcl::ScopeTime time("estiamte-CPU"); 
//
//    Matrix<float, Dynamic, 6> matA(cols * rows, 6);
//    VectorXf matb(cols * rows);
//    matA.setZero();
//    matb.setZero();
//        
//    int count = 0;
//        
//    for(int y = 0; y < rows; ++y)
//        for(int x = 0; x < cols; ++x)
//        {
//            coresp.ptr<short2>(y)[x] = make_short2(-1, -1);
//
//            Vector3f ncurr_g;                
//            ncurr_g(0) = nmap_g_curr.ptrX(y)[x];
//
//            if (!valid_host(ncurr_g(0)))
//                continue;
//                
//            Vector3f vcurr_g;
//            vcurr_g(0) = vmap_g_curr.ptrX(y)[x];
//            vcurr_g(1) = vmap_g_curr.ptrY(y)[x];
//            vcurr_g(2) = vmap_g_curr.ptrZ(y)[x];
//
//            Vector3f vcurr_cp = Rprev_inv * (vcurr_g - tprev); // prev camera coo space
//                
//            int2 ukr; //projection
//            ukr.x = cvRound(vcurr_cp(0) * intr.fx/vcurr_cp(2) + intr.cx); //4
//            ukr.y = cvRound(vcurr_cp(1) * intr.fy/vcurr_cp(2) + intr.cy); //4
//
//            if (ukr.x < 0 || ukr.y < 0 || ukr.x >= cols || ukr.y >= rows)
//                continue;
//
//            Vector3f nprev_g;
//            nprev_g(0) = nmap_g_prev.ptrX(ukr.y)[ukr.x];
//
//            if (!valid_host(nprev_g(0)))
//                continue;                
//
//            Vector3f vprev_g;
//            vprev_g(0) = vmap_g_prev.ptrX(ukr.y)[ukr.x];
//            vprev_g(1) = vmap_g_prev.ptrY(ukr.y)[ukr.x];
//            vprev_g(2) = vmap_g_prev.ptrZ(ukr.y)[ukr.x];
//
//            float dist = (vcurr_g - vprev_g).norm();
//            if (dist > distThres)
//                continue;
//                
//            ncurr_g(1) = nmap_g_curr.ptrY(y)[x];
//            ncurr_g(2) = nmap_g_curr.ptrZ(y)[x];
//                
//            nprev_g(1) = nmap_g_prev.ptrY(ukr.y)[ukr.x];
//            nprev_g(2) = nmap_g_prev.ptrZ(ukr.y)[ukr.x];
//
//            float sine = ncurr_g.cross(nprev_g).norm();
//            
//            if (sine >= 1 || sine >= angleThres)
//                continue;
//                
//            coresp.ptr<short2>(y)[x] = make_short2(ukr.x, ukr.y);                         
//
//
//            Vector3f n; // nprev_g;                                             
//            n(0) = nmap_g_prev.ptrX(ukr.y)[ukr.x];
//            n(1) = nmap_g_prev.ptrY(ukr.y)[ukr.x];
//            n(2) = nmap_g_prev.ptrZ(ukr.y)[ukr.x];
//
//            Vector3f d; // vprev_g
//            d(0) = vmap_g_prev.ptrX(ukr.y)[ukr.x];
//            d(1) = vmap_g_prev.ptrY(ukr.y)[ukr.x];
//            d(2) = vmap_g_prev.ptrZ(ukr.y)[ukr.x];                        
//
//            Vector3f s; // vcurr_g
//            s(0) = vmap_g_curr.ptrX(y)[x];
//            s(1) = vmap_g_curr.ptrY(y)[x];
//            s(2) = vmap_g_curr.ptrZ(y)[x];        
//
//            float b = n.dot(d - s);
//            
//            Vector3f cr = s.cross(n);
//
//            matA(count, 0) = cr(0);
//            matA(count, 1) = cr(1);
//            matA(count, 2) = cr(2);
//
//            matA(count, 3) = n(0);
//            matA(count, 4) = n(1);
//            matA(count, 5) = n(2);
//
//            matb(count) = b;
//
//            ++count;
//        }
//
//        matA.conservativeResize(count, 6);
//        matb.conservativeResize(count);
//
//        Matrix<float, 6, 6> AA = matA.transpose() * matA;
//        Matrix<float, 6, 1> BB = matA.transpose() * matb;  
//
//        Matrix<float, 6, 1> result = AA.llt().solve(BB);
//
//        printf("count %d\n", count);
//        cout << "from CPU = " << result.transpose() << endl ;
//}
//

void pcl::gpu::KinfuTracker::operator()(const DepthMap& depth_raw, View& view)
{				        	
	device::Intr intr(fx, fy, cx, cy);
    //cout << "Global time: " << global_time << endl;

    {
        //ScopeTimerCV time(">>> Bilateral, pyr-down-all, create-maps-all"); 
        {
            //ScopeTimerCV time("bilateral"); 
            //depth_raw.copyTo(depths_curr[0]);
            device::bilateralFilter(depth_raw, depths_curr[0]);
        }

        {
            //ScopeTimerCV time("pyr-down-all"); 
            for(int i = 1; i < LEVELS; ++i)
                device::pyrDown(depths_curr[i-1], depths_curr[i]);
        }

        {
            //ScopeTimerCV time("create-maps-all"); 
            for(int i = 0; i < LEVELS; ++i)
            {
                device::createVMap(intr(i), depths_curr[i], vmaps_curr[i]);
                //device::createNMap(vmaps_curr[i], nmaps_curr[i]);
                compteNormalsEigen(vmaps_curr[i], nmaps_curr[i]);
            }
        }

        pcl::device::sync();
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
        	
	    //integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcam_inv, device_tcam, tranc_dist, volume);
        integrateVolume(depth_raw, intr, device_volume_size, device_Rcam_inv, device_tcam, tranc_dist, volume, depthRawScaled);
                
        for(int i = 0; i < LEVELS; ++i)
            device::tranformMaps(vmaps_curr[i], nmaps_curr[i], device_Rcam, device_tcam, vmaps_g_prev[i], nmaps_g_prev[i]);
        		
		++global_time;
		view.release();
        return;
	}
    
	///////////////////////////////////////////////////////////////////////////////////////////
	// Iterative Closest Point 	
	Matrix3f Rprev = rmats[global_time - 1]; //  [Ri|ti] - pos of camera, i.e.
	Vector3f tprev = tvecs[global_time - 1]; //  tranfrom from camera to global coo space for (i-1)th camera pose
    Matrix3f Rprev_inv = Rprev.inverse(); //Rprev.t();
	
	Mat33&  device_Rprev     = device_cast<Mat33 >(Rprev);
	Mat33&  device_Rprev_inv = device_cast<Mat33 >(Rprev_inv);
	float3& device_tprev     = device_cast<float3>(tprev);
	    
    Matrix3f Rcurr = Rprev; // tranform to global coo for ith camera pose
    Vector3f tcurr = tprev; 

    {
        //ScopeTime time("icp-all"); 

        //static std::string names[] = { "l1", "l2", "l3" };

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
          
            //ScopeTimerCV time("  icp-level"); 
            {
	            for(int iter = 0; iter < iter_num; ++iter) 
	            {			

                    //cout << "==============\niter = " << iter << endl;
                    //ScopeTimerCV time("    icp-iter"); 
		            Mat33&  device_Rcurr = device_cast<Mat33> (Rcurr);
		            float3& device_tcurr = device_cast<float3>(tcurr);

                    device::tranformMaps(vmap_curr, nmap_curr, device_Rcurr, device_tcurr, vmap_g_curr, nmap_g_curr);
                    
                    {
                        //ScopeTimerCV time("      coresp");
                        findCoresp(vmap_g_curr, nmap_g_curr, device_Rprev_inv, device_tprev, intr(level_index), 
                                     vmap_g_prev, nmap_g_prev, distThres, angleThres, coresp);
                    }

                    /*cv::Mat coresp_cpu;
                    findCorespCPU(vmap_g_curr, nmap_g_curr, Rprev_inv, tprev, intr(level_index), vmap_g_prev, nmap_g_prev, distThres, angleThres, coresp_cpu);*/

                    //if (global_time == 5)
                    {
                        //cv::gpu::GpuMat ma(coresp.rows(), coresp.cols(), CV_32S, coresp.ptr(), coresp.step());
                        //cv::Mat cpu;
                        //ma.download(cpu);                        

                        //cv::imshow(names[level_index] + string(" --- coresp white == -1"), cpu == -1);

                        //printf("count GPU %d\n", cv::countNonZero(cpu != -1));

                        /*cv::imshow(names[level_index] + string(" --- coresp DIff == -1"), coresp_cpu != cpu);
                        cv::waitKey(2);*/
                    }
                    

                    ////////////////////////////////////
                    Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A;
                    Matrix<float, 6, 1> b;    
                    device::estimateTransform(vmap_g_prev, nmap_g_prev, vmap_g_curr, coresp, gbuf, sumbuf, A.data(), b.data());    
                    
                    float det = A.determinant();

                    if (det < 1e-15)
                    {
                       reset();
                        return;
                    }
                    //float maxc = A.maxCoeff();

                    Matrix<float, 6, 1> result = A.llt().solve(b);
                    //Matrix<float, 6, 1> res = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
                    //cout << "from GPU = " << result.transpose() << endl;

                    float alpha = result(0);
	                float beta  = result(1);
	                float gamma = result(2);

                    Matrix3f Rinc;
                    Rinc = AngleAxisf(gamma, Vector3f::UnitZ()) * AngleAxisf(beta, Vector3f::UnitY()) * AngleAxisf(alpha, Vector3f::UnitX());
                    Vector3f tinc = result.tail<3>();


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
    	
	///////////////////////////////////////////////////////////////////////////////////////////
	// Volume integration
	
	float3 device_volume_size = device_cast<float3>(volume_size);

    Matrix3f Rcurr_inv = Rcurr.inverse();    
    Mat33&  device_Rcurr_inv = device_cast<Mat33> (Rcurr_inv);
	float3& device_tcurr = device_cast<float3>(tcurr);	
	
    { 
        //ScopeTime time("tsdf"); 
	    //integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcurr_inv, device_tcurr, tranc_dist, volume);
        integrateVolume(depth_raw, intr, device_volume_size, device_Rcurr_inv, device_tcurr, tranc_dist, volume, depthRawScaled);        
    }

	///////////////////////////////////////////////////////////////////////////////////////////
	// Ray casting     
	
    Mat33&  device_Rcurr = device_cast<Mat33> (Rcurr);

    
    device::LightSource light;
	light.number = 1;
    light.pos[0] = device_cast<float3>(light_pos);
    	        
    view.create(rows_, cols_);


    { 
        //ScopeTime time("ray-cast-all, view generation"); 
       
        raycast(intr, device_Rcurr, device_tcurr, tranc_dist, device_volume_size, volume, vmaps_g_prev[0], nmaps_g_prev[0]);        
        for(int i = 1; i < LEVELS; ++i)   
        {
            resizeVMap(vmaps_g_prev[i-1], vmaps_g_prev[i]);
            resizeNMap(nmaps_g_prev[i-1], nmaps_g_prev[i]);
        }

        generateImage(vmaps_g_prev[0], nmaps_g_prev[0], light, view);

        pcl::device::sync();
    }               
                
    ++global_time;
}



