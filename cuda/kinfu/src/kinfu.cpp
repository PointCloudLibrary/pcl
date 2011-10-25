#include<iostream>
#include<fstream>

#include "pcl/gpu/kinfu/kinfu.hpp"
#include "internal.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/LU> 

using namespace std;		
using namespace pcl::device;
using namespace Eigen;

template<class D, class Matx> 
D& device_cast(Matx& matx) { return *reinterpret_cast<D*>(matx.data()); }


pcl::gpu::KinfuTracker::KinfuTracker() : global_time(-1) 
{
	rmats.reserve(30000);
	tvecs.reserve(30000);

	fx = fy = 525.f;
	cx = 320;
	cy = 240;	

	icp_iterations_number = 20;

	distThres = 100; //mm
	normalThres = static_cast<float>(20.0 * 3.14159254/180.0);

    volume_size = Vector3f::Constant(2000);
    init_Rcam   = Matrix3f::Identity();
    init_tcam   = volume_size * 0.5f;
    
    init_tcam(2) -= volume_size(2)/2 * 1.5f;

	light_x = -volume_size(0) * 10;
	light_y = -volume_size(1) * 10;
	light_z = -volume_size(2) * 10;
}


void pcl::gpu::KinfuTracker::allocateBufffers(int rows, int cols)
{
    volume.create(device::VOLUME_Y * device::VOLUME_Z, device::VOLUME_X);
    
	vmap_curr.create(rows * 3, cols);
	nmap_curr.create(rows * 3, cols);
	
	vmap_g_curr.create(rows * 3, cols);
	nmap_g_curr.create(rows * 3, cols);

	vmap_g_prev.create(rows * 3, cols); // vertexes seen on prev frame in global coo
	nmap_g_prev.create(rows * 3, cols); 
}

void pcl::gpu::KinfuTracker::estimateTrel(const MapArr& v_dst, const MapArr& n_dst, const MapArr& v_src, Matrix3f& Rrel, Vector3f& trel)
{    
    work_type A_data[36];
    work_type b_data[6];
                
    device::estimateTransform(v_dst, n_dst, v_src, gbuf, sumbuf, A_data, b_data);

#if 0
    int cols;
    vector<float4> sv, dv, nv;
    DeviceArray2D<float4> s, d, n;
    device::convert(v_src, s);
    device::convert(v_dst, d);
    device::convert(n_dst, n);
    s.download(sv, cols);
    d.download(dv, cols);
    n.download(nv, cols);

    Mat totalA(640*480, 6, cv::DataDepth<work_type>::value);
    Mat totalb(640*480, 1, cv::DataDepth<work_type>::value);

    int pos = 0;
    for(size_t i = 0; i < sv.size(); ++i)
    {
        float4 s = sv[i];
        float4 d = dv[i];
        float4 n = nv[i];
        
        if (_isnan(n.x) || _isnan(s.x))
            continue;

        totalA.at<work_type>(pos, 0) = n.z * s.y - n.y * s.z;
        totalA.at<work_type>(pos, 1) = n.x * s.z - n.z * s.x;
        totalA.at<work_type>(pos, 2) = n.y * s.x - n.x * s.y;
        totalA.at<work_type>(pos, 3) = n.x;
        totalA.at<work_type>(pos, 4) = n.y;
        totalA.at<work_type>(pos, 5) = n.z;
        totalb.at<work_type>(pos, 0) = n.x*d.x + n.y*d.y + n.z*d.z - n.x*s.x - n.y*s.y - n.z*s.z;
        ++pos;
    }

    cout << "Total: " << pos << "\tRadtio:" << float(pos)/640/480 * 100 << endl;
    totalA = totalA.rowRange(0, pos);
    totalb = totalb.rowRange(0, pos);

    Mat Acpu = totalA.t() * totalA;
    Mat Bcpu = totalA.t() * totalb;      

    cout << "==\n Acpu -> \n" << Acpu << endl << endl;
    cout << "==\n Bcpu -> \n" << Bcpu << endl << endl;

#endif

    Map<Matrix6f> A(A_data);
    Map<Vector6f> b(b_data);
    
    //Matrix<work_type, 6, 1> res = A.llt().solve(b);
    Matrix<work_type, 6, 1> res = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
     
    work_type alpha = res(0);
	work_type beta  = res(1);
	work_type gamma = res(2);
	work_type tx = res(3);
	work_type ty = res(4);
	work_type tz = res(5);

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

    Rrel = Map<Matrix3f>(data_rotation);
    trel = Map<Vector3f>(data_traslation);

	///*!*/ Rrel = Matrix3f(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	///*!*/ trel = Vector3f(tx, ty, tz);
}

void pcl::gpu::KinfuTracker::operator()(const DepthMap& depth_curr, View& view)
{				        
    if (global_time == -1)
	{
        allocateBufffers(depth_curr.rows(), depth_curr.cols());
        device::initVolume(volume);
		++global_time = 0;
		view.release();
	}

	//depth map conversion
	device::Intr intr(fx, fy, cx, cy);

    device::createVMap(intr, depth_curr, vmap_curr);
    device::createNMap(vmap_curr, nmap_curr);
        
    compteNormalsEigen(vmap_curr, nmap_curr);

	//can't perform more on first frame
	if (global_time == 0)
	{		
		rmats.push_back(init_Rcam);
		tvecs.push_back(init_tcam);		

        Mat33&  device_init_Rcam = device_cast<Mat33> (init_Rcam);
		float3& device_init_tcam = device_cast<float3>(init_tcam);

        device::tranformMaps(vmap_curr, nmap_curr, device_init_Rcam, device_init_tcam, vmap_g_prev, nmap_g_prev);
        		
		++global_time;
		view.release();
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
        
	for(int iter = 0; iter < icp_iterations_number; ++iter) 
	{			
		Mat33&  device_Rcurr = device_cast<Mat33> (Rcurr);
		float3& device_tcurr = device_cast<float3>(tcurr);

        device::tranformMaps(vmap_curr, nmap_curr, device_Rcurr, device_tcurr, vmap_g_curr, nmap_g_curr);		

        Matrix3f Rrel;
		Vector3f trel;
        estimateTrel(vmap_g_prev, nmap_g_prev, vmap_g_curr, Rrel, trel);

		//compose
		tcurr = Rrel * tcurr + trel;
		Rcurr = Rrel * Rcurr;        
	}
        
	//save tranform
	rmats.push_back(Rcurr);
	tvecs.push_back(tcurr);		

	//prepare data for next frame
	Mat33&  device_Rcurr = device_cast<Mat33> (Rcurr);
	float3& device_tcurr = device_cast<float3>(tcurr);
	device::tranformMaps(vmap_curr, nmap_curr, device_Rcurr, device_tcurr, vmap_g_prev, nmap_g_prev);

	//return Mat();

	///////////////////////////////////////////////////////////////////////////////////////////
	// Volume integration

	cout << "Starting Volume integration" << endl;

	float3& device_volume_size = *reinterpret_cast<float3*>(volume_size.data());

	Matrix3f Rcurr_inv = Rcurr.inverse();// Rcurr.t(); //Rcurr.inv();
	Mat33&  device_Rcurr_inv = device_cast<Mat33> (Rcurr_inv);

	integrateTsdfVolume(depth_curr, intr, device_volume_size, device_Rcurr_inv, device_tcurr, volume);

    //slice_volume(volume);

	///////////////////////////////////////////////////////////////////////////////////////////
	// Ray casting     

	cout << "Starting raycasting" << endl;
    raycast(device_Rcurr, device_tcurr, intr, device_volume_size, volume, vmap_g_prev, nmap_g_prev);
                    
	///////////////////////////////////////////////////////////////////////////////////////////
	// image generation

	cout << "Starting image generation" << endl;
	device::LightSource light;
	light.number = 1;
	light.pos[0].x = light_x;
	light.pos[0].y = light_y;
	light.pos[0].z = light_z;
    	    
    view.create(depth_curr.rows(), depth_curr.cols());
	generateImage(vmap_g_prev, nmap_g_prev, light, view);
	   
    //if (global_time < 10)
    //device::tranformMaps(vmap_curr, nmap_curr, device_Rcurr, device_tcurr, vmap_g_prev, nmap_g_prev);

    ++global_time;
}


