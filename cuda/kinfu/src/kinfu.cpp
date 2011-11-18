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

//#include "vector_functions.h"

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
//using namespace Eigen;

using Eigen::AngleAxisf;
using Eigen::Array3f;

template<class D, class Matx> 
D& device_cast(Matx& matx) { return *reinterpret_cast<D*>(matx.data()); }

pcl::gpu::KinfuTracker::KinfuTracker(int rows, int cols) : rows_(rows), cols_(cols), global_time_(0) 
{
    rmats_.reserve(30000);
    tvecs_.reserve(30000);

    setDepthIntrinsics(525.f, 525.f);  
    setVolumeSize(Vector3f::Constant(3000));

    init_Rcam_   = Eigen::Matrix3f::Identity();// * AngleAxisf(-30.f/180*3.1415926, Vector3f::UnitX());
    init_tcam_   = volume_size_ * 0.5f;    
    init_tcam_(2) -= volume_size_(2)/2 * 1.2f;
    
    const int iters[] = {10, 5, 4};
    std::copy(iters, iters + LEVELS, icp_iterations_);

    distThres_ = 100; //mm
    angleThres_ = sin(20.f * 3.14159254f/180.f);
    tranc_dist_ = 30; //mm

    allocateBufffers(rows, cols);
    reset();
}

void pcl::gpu::KinfuTracker::setDepthIntrinsics(float fx, float fy, float cx, float cy)
{
    fx_ = fx;
    fy_ = fy;
    cx_ = (cx == -1) ? cols_/2 : cx;
    cy_ = (cy == -1) ? rows_/2 : cy;   
}

void pcl::gpu::KinfuTracker::setVolumeSize(const Eigen::Vector3f& volume_size)
{
    volume_size_ = volume_size;
}

void pcl::gpu::KinfuTracker::setInitalCameraPose(const Eigen::Affine3f& pose)
{
    init_Rcam_ = pose.rotation();
    init_tcam_ = pose.translation();
}

int pcl::gpu::KinfuTracker::cols() { return cols_; }
int pcl::gpu::KinfuTracker::rows() { return rows_; }                              

void pcl::gpu::KinfuTracker::reset()
{
    global_time_ = 0;
    rmats_.clear();
    tvecs_.clear();

    rmats_.push_back(init_Rcam_);
    tvecs_.push_back(init_tcam_);

    device::initVolume<volume_elem_type>(volume_);
    cout << "Reset" << endl;
}

void pcl::gpu::KinfuTracker::allocateBufffers(int rows, int cols)
{
    volume_.create(device::VOLUME_Y * device::VOLUME_Z, device::VOLUME_X);

    depths_curr_.resize(LEVELS);
    vmaps_g_curr_.resize(LEVELS);
    nmaps_g_curr_.resize(LEVELS);

    vmaps_g_prev_.resize(LEVELS);
    nmaps_g_prev_.resize(LEVELS);

    vmaps_curr_.resize(LEVELS);
    nmaps_curr_.resize(LEVELS);

    coresps_.resize(LEVELS);

    for(int i = 0; i < LEVELS; ++i)
    {
        int pyr_rows = rows >> i;
        int pyr_cols = cols >> i;

        depths_curr_[i].create(pyr_rows, pyr_cols);

        vmaps_g_curr_[i].create(pyr_rows*3, pyr_cols);
        nmaps_g_curr_[i].create(pyr_rows*3, pyr_cols);

        vmaps_g_prev_[i].create(pyr_rows*3, pyr_cols);
        nmaps_g_prev_[i].create(pyr_rows*3, pyr_cols);

        vmaps_curr_[i].create(pyr_rows*3, pyr_cols);
        nmaps_curr_[i].create(pyr_rows*3, pyr_cols);

        coresps_[i].create(pyr_rows, pyr_cols);
    }                      
    depthRawScaled_.create(rows, cols);

    // see estimate tranform for the magic numbers
    gbuf_.create(27, 20*60);
    sumbuf_.create(27);
}

bool pcl::gpu::KinfuTracker::operator()(const DepthMap& depth_raw)
{				        	
    device::Intr intr(fx_, fy_, cx_, cy_);
    {
        //ScopeTime time(">>> Bilateral, pyr-down-all, create-maps-all");          
        //depth_raw.copyTo(depths_curr[0]);
        device::bilateralFilter(depth_raw, depths_curr_[0]);

        for(int i = 1; i < LEVELS; ++i)
            device::pyrDown(depths_curr_[i-1], depths_curr_[i]);

        for(int i = 0; i < LEVELS; ++i)
        {
            device::createVMap(intr(i), depths_curr_[i], vmaps_curr_[i]);
            //device::createNMap(vmaps_curr_[i], nmaps_curr_[i]);
            compteNormalsEigen(vmaps_curr_[i], nmaps_curr_[i]);
        }
        pcl::device::sync();        
    }

    //can't perform more on first frame
    if (global_time_ == 0)
    {	
        Matrix3frm init_Rcam = rmats_[0]; //  [Ri|ti] - pos of camera, i.e.
        Vector3f   init_tcam = tvecs_[0]; //  tranfrom from camera to global coo space for (i-1)th camera pose

        Mat33&  device_Rcam = device_cast<Mat33> (init_Rcam);
        float3& device_tcam = device_cast<float3>(init_tcam);

        Matrix3frm init_Rcam_inv = init_Rcam.inverse();    
        Mat33&   device_Rcam_inv = device_cast<Mat33> (init_Rcam_inv);
        float3 device_volume_size = device_cast<float3>(volume_size_);

        //integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcam_inv, device_tcam, tranc_dist, volume_);
        integrateVolume(depth_raw, intr, device_volume_size, device_Rcam_inv, device_tcam, tranc_dist_, volume_, depthRawScaled_);

        for(int i = 0; i < LEVELS; ++i)
            device::tranformMaps(vmaps_curr_[i], nmaps_curr_[i], device_Rcam, device_tcam, vmaps_g_prev_[i], nmaps_g_prev_[i]);

        ++global_time_;		
        return false;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Iterative Closest Point 	
    Matrix3frm Rprev = rmats_[global_time_ - 1]; //  [Ri|ti] - pos of camera, i.e.
    Vector3f   tprev = tvecs_[global_time_ - 1]; //  tranfrom from camera to global coo space for (i-1)th camera pose
    Matrix3frm Rprev_inv = Rprev.inverse(); //Rprev.t();

    Mat33&  device_Rprev     = device_cast<Mat33 >(Rprev);
    Mat33&  device_Rprev_inv = device_cast<Mat33 >(Rprev_inv);
    float3& device_tprev     = device_cast<float3>(tprev);

    Matrix3frm Rcurr = Rprev; // tranform to global coo for ith camera pose
    Vector3f   tcurr = tprev; 

    {
        //ScopeTime time("icp-all"); 
        for(int level_index = LEVELS-1; level_index>=0; --level_index)
        {
            int iter_num = icp_iterations_[level_index];

            MapArr& vmap_curr = vmaps_curr_[level_index];
            MapArr& nmap_curr = nmaps_curr_[level_index];

            MapArr& vmap_g_curr = vmaps_g_curr_[level_index];
            MapArr& nmap_g_curr = nmaps_g_curr_[level_index];

            MapArr& vmap_g_prev = vmaps_g_prev_[level_index];
            MapArr& nmap_g_prev = nmaps_g_prev_[level_index];

            CorespMap& coresp = coresps_[level_index];

            for(int iter = 0; iter < iter_num; ++iter) 
            {			
                Mat33&  device_Rcurr = device_cast<Mat33> (Rcurr);
                float3& device_tcurr = device_cast<float3>(tcurr);

                Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A;
                Eigen::Matrix<float, 6, 1> b;
#if 0
                device::tranformMaps(vmap_curr, nmap_curr, device_Rcurr, device_tcurr, vmap_g_curr, nmap_g_curr);                                                                
                findCoresp(vmap_g_curr, nmap_g_curr, device_Rprev_inv, device_tprev, intr(level_index), vmap_g_prev, nmap_g_prev, distThres_, angleThres_, coresp);
                device::estimateTransform(vmap_g_prev, nmap_g_prev, vmap_g_curr, coresp, gbuf_, sumbuf_, A.data(), b.data());    

                //cv::gpu::GpuMat ma(coresp.rows(), coresp.cols(), CV_32S, coresp.ptr(), coresp.step());
                //cv::Mat cpu;
                //ma.download(cpu);                        
                //cv::imshow(names[level_index] + string(" --- coresp white == -1"), cpu == -1);
#else                                                                                                               
                estimateCombined(device_Rcurr, device_tcurr, vmap_curr, nmap_curr, device_Rprev_inv, device_tprev, intr(level_index), 
                    vmap_g_prev, nmap_g_prev, distThres_, angleThres_, gbuf_, sumbuf_, A.data(), b.data());
#endif

                //checking nullspace
                float det = A.determinant();

                if (det < 1e-15 || !pcl::device::valid_host(det))
                {                    
                    if (!valid_host(det)) cout << "qnan" << endl;

                    reset();
                    return false;
                }
                //float maxc = A.maxCoeff();

                Eigen::Matrix<float, 6, 1> result = A.llt().solve(b);
                //Eigen::Matrix<float, 6, 1> result = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

                float alpha = result(0);
                float beta  = result(1);
                float gamma = result(2);

                Eigen::Matrix3f Rinc = (Eigen::Matrix3f)AngleAxisf(gamma, Vector3f::UnitZ()) * AngleAxisf(beta, Vector3f::UnitY()) * AngleAxisf(alpha, Vector3f::UnitX());
                Vector3f tinc = result.tail<3>();

                //compose
                tcurr = Rinc * tcurr + tinc;
                Rcurr = Rinc * Rcurr;        
            }
        }
    }

    //save tranform
    rmats_.push_back(Rcurr);
    tvecs_.push_back(tcurr);		

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Volume integration

    float3 device_volume_size = device_cast<float3>(volume_size_);

    Matrix3frm Rcurr_inv = Rcurr.inverse();    
    Mat33&  device_Rcurr_inv = device_cast<Mat33> (Rcurr_inv);
    float3& device_tcurr = device_cast<float3>(tcurr);	

    { 
        //ScopeTime time("tsdf"); 
        //integrateTsdfVolume(depth_raw, intr, device_volume_size, device_Rcurr_inv, device_tcurr, tranc_dist, volume_);
        integrateVolume(depth_raw, intr, device_volume_size, device_Rcurr_inv, device_tcurr, tranc_dist_, volume_, depthRawScaled_);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Ray casting     

    Mat33&  device_Rcurr = device_cast<Mat33> (Rcurr);        
    { 
        //ScopeTime time("ray-cast-all"); 
        raycast(intr, device_Rcurr, device_tcurr, tranc_dist_, device_volume_size, volume_, vmaps_g_prev_[0], nmaps_g_prev_[0]);
        for(int i = 1; i < LEVELS; ++i)   
        {
            resizeVMap(vmaps_g_prev_[i-1], vmaps_g_prev_[i]);
            resizeNMap(nmaps_g_prev_[i-1], nmaps_g_prev_[i]);
        }        
        pcl::device::sync();        
    }               

    ++global_time_;
    return true;
}

Eigen::Affine3f pcl::gpu::KinfuTracker::getCameraPose(int time)
{    
    if (time > rmats_.size() || time < 0)
        time = rmats_.size() - 1;

    Eigen::Affine3f aff;
    aff.linear() = rmats_[time];
    aff.translation() = tvecs_[time];
    return aff;    
}

void pcl::gpu::KinfuTracker::getImage(View& view) const
{    
    getImage(view, volume_size_ * (-3.f));
}

void pcl::gpu::KinfuTracker::getImage(View& view_arg, const Eigen::Vector3f& light_source_pose) const
{
    device::LightSource light;
    light.number = 1;
    light.pos[0] = device_cast<const float3>(light_source_pose);

    view_arg.create(rows_, cols_);
    generateImage(vmaps_g_prev_[0], nmaps_g_prev_[0], light, view_arg);
}

void pcl::gpu::KinfuTracker::getLastFrameCloud(DeviceArray2D<PointType>& cloud) const
{
    cloud.create(rows_, cols_);
    DeviceArray2D<float4>& c = (DeviceArray2D<float4>&)cloud;
    device::convert(vmaps_g_prev_[0], c);
}

void pcl::gpu::KinfuTracker::getLastFrameNormals(DeviceArray2D<NormalType>& normals) const
{
    normals.create(rows_, cols_);
    DeviceArray2D<float8>& n = (DeviceArray2D<float8>&)normals;
    device::convert(nmaps_g_prev_[0], n);
}

void pcl::gpu::KinfuTracker::getCloudFromVolumeHost(PointCloud<PointType>& cloud, bool connected26)
{
    int cols;
    std::vector<int> volume_host;
    volume_.download(volume_host, cols);

    cloud.points.clear();
    cloud.points.reserve(10000);

    const int DIVISOR = 32767; // SHRT_MAX;

#define FETCH(x, y, z) volume_host[(x) + (y) * VOLUME_X + (z) * VOLUME_Y * VOLUME_X]

    Array3f cell_size = volume_size_.array() / Array3f(VOLUME_X, VOLUME_Y, VOLUME_Z);

    for(int x = 1; x < VOLUME_X-1; ++x)
    {
        for(int y = 1; y < VOLUME_X-1; ++y)
        {
            for(int z = 0; z < VOLUME_Z-1; ++z)
            {
                int tmp = FETCH(x, y, z);                    
                int W = reinterpret_cast<short2*>(&tmp)->y;
                int F = reinterpret_cast<short2*>(&tmp)->x;

                if (W == 0 || F == DIVISOR)
                    continue;

                Vector3f V = ((Array3f(x, y, z) + 0.5f) * cell_size).matrix();

                if (connected26)
                {

                    int dz = 1;                
                    for(int dy = -1; dy < 2; ++dy)
                        for(int dx = -1; dx < 2; ++dx)
                        {
                            int tmp = FETCH(x+dx, y+dy, z+dz);

                            int Wn = reinterpret_cast<short2*>(&tmp)->y;
                            int Fn = reinterpret_cast<short2*>(&tmp)->x;
                            if (Wn == 0 || Fn == DIVISOR)
                                continue;

                            if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
                            {
                                Vector3f Vn = ((Array3f(x+dx, y+dy, z+dz) + 0.5f) * cell_size).matrix();
                                Vector3f point = (V * abs(Fn) + Vn * abs(F))/(abs(F) + abs(Fn));

                                pcl::PointXYZ xyz;
                                xyz.x = point(0);
                                xyz.y = point(1);
                                xyz.z = point(2);

                                cloud.points.push_back(xyz);
                            }
                        }

                        dz = 0;
                        for(int dy = 0; dy < 2; ++dy)
                            for(int dx = -1; dx < dy * 2; ++dx)
                            {
                                int tmp = FETCH(x+dx, y+dy, z+dz);

                                int Wn = reinterpret_cast<short2*>(&tmp)->y;
                                int Fn = reinterpret_cast<short2*>(&tmp)->x;
                                if (Wn == 0 || Fn == DIVISOR)
                                    continue;

                                if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
                                {
                                    Vector3f Vn = ((Array3f(x+dx, y+dy, z+dz) + 0.5f) * cell_size).matrix();
                                    Vector3f point = (V * abs(Fn) + Vn * abs(F))/(abs(F) + abs(Fn));

                                    pcl::PointXYZ xyz;
                                    xyz.x = point(0);
                                    xyz.y = point(1);
                                    xyz.z = point(2);

                                    cloud.points.push_back(xyz);
                                }                        
                            }
                }
                else /* if (connected26) */
                {
                    for(int i = 0; i < 3; ++i)
                    {
                        int ds[] = {0, 0, 0};
                        ds[i] = 1;

                        int dx = ds[0];
                        int dy = ds[1];
                        int dz = ds[2];

                        int tmp = FETCH(x+dx, y+dy, z+dz);

                        int Wn = reinterpret_cast<short2*>(&tmp)->y;
                        int Fn = reinterpret_cast<short2*>(&tmp)->x;
                        if (Wn == 0 || Fn == DIVISOR)
                            continue;

                        if ((F > 0 && Fn < 0) || (F < 0 && Fn > 0))
                        {
                            Vector3f Vn = ((Array3f(x+dx, y+dy, z+dz) + 0.5f) * cell_size).matrix();
                            Vector3f point = (V * abs(Fn) + Vn * abs(F))/(abs(F) + abs(Fn));

                            pcl::PointXYZ xyz;
                            xyz.x = point(0);
                            xyz.y = point(1);
                            xyz.z = point(2);

                            cloud.points.push_back(xyz);
                        }

                    }
                } /* if (connected26) */
            }
        }
    }
#undef FETCH
    cloud.width  = (int)cloud.points.size();
    cloud.height = 1;
}

pcl::gpu::DeviceArray<pcl::gpu::KinfuTracker::PointType> pcl::gpu::KinfuTracker::getCloudFromVolume(DeviceArray<PointType>& cloud_buffer, bool connected26)
{
    pcl::gpu::error("GPU cloud extraction is not implemented", __FILE__, __LINE__);

    if (cloud_buffer.empty())
        cloud_buffer.create(DEFAULT_VOLUME_CLOUD_BUFFER_SIZE);
        
    float3 device_volume_size = device_cast<float3>(volume_size_);
    size_t size = device::extractCloud(volume_, device_volume_size, cloud_buffer, connected26);
    return DeviceArray<PointType>(cloud_buffer.ptr(), size);
}
