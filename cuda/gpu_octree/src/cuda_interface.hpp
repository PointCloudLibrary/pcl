#ifndef PCL_GPU_CUDA_OCTREE_
#define PCL_GPU_CUDA_OCTREE_

#include "pcl/gpu/common/device_array.hpp"
#include "cuda_runtime.h"

#include <thrust/device_vector.h>

#include "octree_global.hpp"
#include "tasks_global.hpp"

namespace pcl
{
    namespace gpu
    {
        void get_cc_compiled_for(int& bin, int& ptr);

        class CudaOctree
        {
        public:
            virtual ~CudaOctree() {}

            virtual void setCloud(const pcl::gpu::DeviceArray_<float3>& input_points) = 0 { };           
            virtual void build() = 0 { };
            virtual void radiusSearch(const float3& center, float radius, std::vector<int>& out) const = 0 { };
        };
        
        class Octree_host : public CudaOctree
        {
        public:            
            const static int max_leaf_points = 32;

            struct OctreeData
            {
                std::vector<int> nodes;

	            std::vector<int> begs;
	            std::vector<int> ends;	
                std::vector<int> node_codes;	

                std::vector<int> indices;	
            } octree;            

            void setCloud(const pcl::gpu::DeviceArray_<float3>& input_points);           
            void build();
            void radiusSearch(const float3& center, float radius, std::vector<int>& out) const;            

        private:
            thrust::device_vector<float3> points;
			
			//auxilary
			size_t points_num;
			float3 minp, maxp;
			
			thrust::device_vector<int> indices;
			thrust::device_vector<int> codes;

			//buffers
			thrust::device_vector<float> tmp_x;
			thrust::device_vector<float> tmp_y;
			thrust::device_vector<float> tmp_z;

			thrust::device_vector<float3> tmp;

            std::vector<float3> points_host;
            void calcBoundingBoxOld(int level, int code, float3& res_minp, float3& res_maxp) const;
            void getCodesInds(std::vector<int>& codes, std::vector<int>& inds);
        };


        class Octree2 : public pcl::gpu::CudaOctree
        {
        public:
            Octree2(int number_of_SMs_arg) : number_of_SMs(number_of_SMs_arg) {};
            ~Octree2() {};

            void setCloud(const DeviceArray_<float3>& input_points);           
            void build();
            void radiusSearch(const float3& center, float radius, std::vector<int>& out) const;
            void radiusSearch2(const float3& center, float radius, std::vector<int>& out) const;

            void radiusSearchBatch(const DeviceArray_<float3>& queries, float radius, DeviceArray2D_<int>& out, DeviceArray_<int>& out_sizes);

            size_t points_num;

            //storage
            DeviceArray_<float3> points;

            DeviceArray_<int> codes;
            DeviceArray_<int> indices;

            DeviceArray_<float3> points_sorted;

            TasksGlobalLifeTime tasksGlobal;
            OctreeGlobalLifetime octreeGlobal;    

            //test
            Octree_host::OctreeData host;
            std::vector<float3> points_host;

            int downloaded;
            void internalDownload();

            int number_of_SMs;
        };
    }
}

#endif /* PCL_GPU_CUDA_OCTREE_ */