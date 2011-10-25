#include<iostream>
#include<fstream>
#include<opencv2/opencv.hpp>
#include<opencv2/gpu/gpu.hpp>

#include "pcl/gpu/kinfu/kinfu.hpp"

#include "internal.hpp"

using namespace std;		
using namespace cv;
using namespace cv::gpu;
using namespace pcl::device;

template<class D, class Matx>
D& device_cast(Matx& matx) { return *reinterpret_cast<D*>(&matx.val[0]); }

template<class T>
pcl::gpu::PtrStepSz<T> device_cast(cv::gpu::GpuMat& image) 
{ 
    return pcl::gpu::PtrStepSz<T>(image.rows, image.cols, image.ptr<T>(), image.step);    
}

pcl::gpu::KinfuTracker::KinfuTracker() : global_time(-1) 
{
	rmats.reserve(30000);
	tvecs.reserve(30000);

	fx = fy = 525.f;
	cx = 320;
	cy = 240;	

	icp_iterations_number = 20;

	distThres = 100; //mm
	normalThres = static_cast<float>(20.0 * CV_PI/180.0);

	volume_size = Matx31f(1500, 1500, 1500);
	init_Rcam   = Matx33f::eye();
    init_tcam   = volume_size * 0.5f;
    
    init_tcam.val[2] -= volume_size.val[2]/2 * 1.5f;

	light_x = -volume_size.val[0] * 10;
	light_y = -volume_size.val[1] * 10;
	light_z = -volume_size.val[2] * 10;
}


void pcl::gpu::KinfuTracker::allocateBufffers(int rows, int cols)
{
    volume.create(device::VOLUME_Y * device::VOLUME_Z, device::VOLUME_X);

    depth_curr.create(rows, cols);
	image.create(rows, cols, CV_8UC3);

	vmap_curr.create(rows * 3, cols);
	nmap_curr.create(rows * 3, cols);
	
	vmap_g_curr.create(rows * 3, cols);
	nmap_g_curr.create(rows * 3, cols);

	vmap_g_prev.create(rows * 3, cols); // vertexes seen on prev frame in global coo
	nmap_g_prev.create(rows * 3, cols); 

    result_host.create(rows, cols, CV_8UC3);	
}

void pcl::gpu::KinfuTracker::estimateTrel(const MapArr& v_dst, const MapArr& n_dst, const MapArr& v_src, Matx33f& Rrel, Matx31f trel)
{
    Matx<float, 6, 6> A;
    Matx<float, 6, 1> b;

    device::estimateTransform(v_dst, n_dst, v_src, gbuf, sumbuf, A.val, b.val);

  /*  Mat vd = convertToMat(v_dst);
    Mat nd = convertToMat(n_dst);
    Mat vs = convertToMat(v_src);
    Mat c = cvtCoresp(corespDstToSrc);

    Mat totalA(c.cols * c.rows, 6, CV_32F);
    Mat totalb(c.cols * c.rows, 1, CV_32F);

    int pos = 0;
    for(int y = 0; y < c.rows; ++y)
        for(int x = 0; x < c.cols; ++x)
            if (c.at<int>(y, x) != -1)
            {
                short2 coo = c.at<short2>(y, x);

                if (y == coo.y && x == coo.x)
                {
                    cout << "Badubin" << endl;

                }

                Point3f d = vd.at<Point3f>(y, x);
                Point3f n = nd.at<Point3f>(y, x);
                Point3f s = vs.at<Point3f>(y, x);

                n *= 1/norm(n);
                
                totalA.at<float>(pos, 0) = n.z * s.y - n.y * s.z;
                totalA.at<float>(pos, 1) = n.x * s.z - n.z * s.x;
                totalA.at<float>(pos, 2) = n.y * s.x - n.x * s.y;
                totalA.at<float>(pos, 3) = n.x;
                totalA.at<float>(pos, 4) = n.y;
                totalA.at<float>(pos, 5) = n.z;
                totalb.at<float>(pos, 0) = n.x*d.x + n.y*d.y + n.z*d.z - n.x*s.x - n.y*s.y - n.z*s.z;
                ++pos;
            }

    totalA = totalA.rowRange(0, pos);
    totalb = totalb.rowRange(0, pos);

    Mat Acpu = totalA.t() * totalA;
    Mat Bcpu = totalA.t() * totalb;

    cout << Mat(b) << " -> " << Bcpu << endl;
    cout << Mat(A) << "\n -> \n" << Acpu << endl;
    cout << norm(Bcpu, b) << endl;
    cout << norm(Acpu, A) << endl;

    cv::waitKey();*/


    //Acpu.copyTo(A);
    //Bcpu.copyTo(b);

    //cv::waitKey();
 				
    Matx<float, 6, 1> res;// = A.solve(b, DECOMP_SVD);
	cv::solve(A, b, res, DECOMP_SVD);
    
    //cout <<"A\n" << Mat(A) << endl;
    //cout <<"b\n" << Mat(b) << endl;

    float alpha = res.val[0];
	float beta  = res.val[1];
	float gamma = res.val[2];
	float tx = res.val[3];
	float ty = res.val[4];
	float tz = res.val[5];
            
	float r11 =  cos(gamma)*cos(beta);
	float r12 = -sin(gamma)*cos(alpha) + cos(gamma)*sin(beta)*sin(alpha);
	float r13 =  sin(gamma)*sin(alpha) + cos(gamma)*sin(beta)*cos(alpha);

	float r21 =  sin(gamma)*cos(beta);
	float r22 =  cos(gamma)*cos(alpha) + sin(gamma)*sin(beta)*sin(alpha);
	float r23 = -cos(gamma)*sin(alpha) + sin(gamma)*sin(beta)*cos(alpha);

	float r31 = -sin(beta);
	float r32 =  cos(beta)*sin(alpha);
	float r33 =  cos(beta)*cos(alpha);

	/*!*/ Rrel = Matx33f(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	/*!*/ trel = Matx31f(tx, ty, tz);
}

cv::Mat pcl::gpu::KinfuTracker::operator()(const Mat& depth_host, const Mat& img)
{				        
    if (global_time == -1)
	{
        allocateBufffers(depth_host.rows, depth_host.cols);
        device::initVolume(volume);
		++global_time = 0;
		return Mat();
	}

	depth_curr.upload(depth_host.data, depth_host.step, depth_host.rows, depth_host.cols);

	//depth map conversion
	device::Intr intr(fx, fy, cx, cy);

    device::createVMap(intr, depth_curr, vmap_curr);
    device::createNMap(vmap_curr, nmap_curr);
        
    //compteNormalsEigen(vmap_curr, nmap_curr);

	//can't perform more on first frame
	if (global_time == 0)
	{		
		rmats.push_back(init_Rcam);
		tvecs.push_back(init_tcam);		

        Mat33&  device_init_Rcam = device_cast<Mat33> (init_Rcam);
		float3& device_init_tcam = device_cast<float3>(init_tcam);

        device::tranformMaps(vmap_curr, nmap_curr, device_init_Rcam, device_init_tcam, vmap_g_prev, nmap_g_prev);
        		
		++global_time;
		return Mat();
	}
    
	///////////////////////////////////////////////////////////////////////////////////////////
	// Iterative Closest Point 

	cout << "Starting ICP" << endl;
	Matx33f Rprev = rmats[global_time - 1]; //  [Ri|ti] - pos of camera, i.e.
	Matx31f tprev = tvecs[global_time - 1]; //  tranfrom from camera to global coo space for (i-1)th camera pose
    Matx33f Rprev_inv = Rprev.inv(); //Rprev.t();
	
	Mat33&  device_Rprev     = device_cast<Mat33 >(Rprev);
	Mat33&  device_Rprev_inv = device_cast<Mat33 >(Rprev_inv);
	float3& device_tprev     = device_cast<float3>(tprev);
	    
    Matx33f Rcurr = Rprev; // tranform to global coo for ith camera pose
    Matx31f tcurr = tprev;   
        
	for(int iter = 0; iter < icp_iterations_number; ++ iter) 
	{			
		Mat33&  device_Rcurr = device_cast<Mat33> (Rcurr);
		float3& device_tcurr = device_cast<float3>(tcurr);

        device::tranformMaps(vmap_curr, nmap_curr, device_Rcurr, device_tcurr, vmap_g_curr, nmap_g_curr);		

        Matx33f Rrel;
		Matx31f trel;
        estimateTrel(vmap_g_prev, nmap_g_prev, vmap_g_curr, Rrel, trel);

		//compose
		Mat(Rrel * tcurr + trel).copyTo(tcurr);
		Mat(Rrel * Rcurr).copyTo(Rcurr);        
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

	float3& device_volume_size = *reinterpret_cast<float3*>(&volume_size.val[0]);

	Matx33f Rcurr_inv = Rcurr.inv();// Rcurr.t(); //Rcurr.inv();
	Mat33&  device_Rcurr_inv = device_cast<Mat33> (Rcurr_inv);

	integrateTsdfVolume(depth_curr, intr, device_volume_size, device_Rcurr_inv, device_tcurr, volume);

    //slice_volume(volume);

	///////////////////////////////////////////////////////////////////////////////////////////
	// Ray casting     

	cout << "Starting raycasting" << endl;
    
    raycast(device_Rcurr, device_tcurr, intr, device_volume_size, volume, vmap_g_prev, nmap_g_prev);
    
    cout << "Done raycasting" << endl;
            
	///////////////////////////////////////////////////////////////////////////////////////////
	// image generation

	cout << "Starting image generation" << endl;
	device::LightSource light;
	light.number = 1;
	light.pos[0].x = light_x;
	light.pos[0].y = light_y;
	light.pos[0].z = light_z;
    	    
	generateImage(vmap_g_prev, nmap_g_prev, light, device_cast<uchar3>(image));
	image.download(result_host);

    if (global_time < 10)
        device::tranformMaps(vmap_curr, nmap_curr, device_Rcurr, device_tcurr, vmap_g_prev, nmap_g_prev);

    ++global_time;
	return result_host;	
}


