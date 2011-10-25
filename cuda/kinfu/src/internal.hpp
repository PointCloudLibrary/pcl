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

		//maps
        void createVMap(const Intr& intr, const DepthMap& depth, MapArr& vmap);
        void createNMap(const MapArr& vmap, MapArr& nmap);
        void compteNormalsEigen(const MapArr& vmap, MapArr& nmap);

        void tranformMaps(const MapArr& vmap_src, const MapArr& nmap_src, const Mat33& Rmat, const float3& tvec, MapArr& vmap_dst, MapArr& nmap_dst);

        struct float8 { float x, y, z, w, f1, f2, f3, f4; };
        template<typename T> void convert(const MapArr& vmap, DeviceArray2D<T>& output);		             

		//icp        
        typedef float work_type;  
        //typedef double work_type;
		void estimateTransform(const MapArr& v_dst, const MapArr& n_dst, const MapArr& v_src, 
                DeviceArray2D<work_type>& gbuf, DeviceArray<work_type>& mbuf, work_type* matrixA_host, work_type* vectorB_host);

		//tsdf
		void initVolume(PtrStepSz<float> array);
		void integrateTsdfVolume(const PtrStepSz<ushort>& depth_curr, const Intr& intr, const float3& volume_size, 
			const Mat33& Rcurr_inv, const float3& tcurr, PtrStep<float> volume);
		

		//raycast
		void raycast(const Mat33& Rcurr, const float3& tcurr, const Intr& intr, const float3& volume_size, 
				const PtrStep<float>& volume, MapArr vmap, MapArr nmap);

		void generateImage(const MapArr& vmap, const MapArr& nmap, const LightSource& light, PtrStepSz<uchar3> dst);


        inline bool valid_host(float value)
        {
            return *reinterpret_cast<int*>(&value) != 0x7fffffff;
        }
	}
}