
#include "utils/global_barrier.hpp"

#include "utils/scan_block.hpp"
#include "utils/lower_bound.hpp"
#include "utils/morton.hpp"
#include "utils/funcattrib.hpp"
#include "utils/copygen.hpp"


#include "pcl/gpu/common/timers_cuda.hpp"

#include "builder/cta_initial.hpp"
#include "builder/cta_syncsstep.hpp"
#include "radius_search/batchRadiusSearch.hpp"

#include <thrust/sequence.h>
#include <thrust/sort.h>

#include "cuda_interface.hpp"
#include "functors.hpp"

using namespace pcl::gpu;
using namespace pcl::cuda;
using namespace thrust;
using namespace std;
using namespace batch_radius_search;

namespace getcc
{
    __global__ void get_cc_kernel(int *data)
    {
        data[threadIdx.x + blockDim.x * blockIdx.x] = threadIdx.x;
    }
}

void pcl::gpu::get_cc_compiled_for(int& bin, int& ptx)
{
    cudaFuncAttributes attrs;
    cudaFuncGetAttributes(&attrs, getcc::get_cc_kernel);  
    bin = attrs.binaryVersion;
    ptx = attrs.ptxVersion;
}


void pcl::gpu::Octree2::setCloud(const pcl::gpu::DeviceArray_<float3>& input_points)
{
    points_num = input_points.size();
    points = input_points;
}

void pcl::gpu::Octree2::build()
{       
    downloaded = false;

    //allocatations
    { 
        ScopeTimer alloc("allocatations");
        
        codes.create(points_num);        
        indices.create(points_num);		
        points_sorted.create(points_num);
        tasksGlobal.create(points_num);
        octreeGlobal.create(points_num);    
    }

    {
        ScopeTimer timer("reduce-morton-sort-permutations"); 
    	
        device_ptr<float3> beg(points.ptr());
        device_ptr<float3> end = beg + points.size();

        octreeGlobal.minp = thrust::reduce(beg, end, make_float3( FLT_MAX,  FLT_MAX,  FLT_MAX), SelectMinPoint());
	    octreeGlobal.maxp = thrust::reduce(beg, end, make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX), SelectMaxPoint());	
    		
        device_ptr<int> codes_beg(codes.ptr());
        device_ptr<int> codes_end = codes_beg + codes.size();
	    thrust::transform(beg, end, codes_beg, CalcMorton(octreeGlobal.minp, octreeGlobal.maxp));

        device_ptr<int> indices_beg(indices.ptr());
        device_ptr<int> indices_end = indices_beg + indices.size();
        thrust::sequence(indices_beg, indices_end);
        thrust::sort_by_key(codes_beg, codes_end, indices_beg );		

        thrust::copy(make_permutation_iterator(beg, indices_beg),
                     make_permutation_iterator(end, indices_end), device_ptr<float3>(points_sorted.ptr()));    
    }
      
    {
        typedef initial::KernelInitialPolicyGlobOctree KernelPolicy;
        //printFuncAttrib(initial::Kernel<KernelPolicy>);        

        ScopeTimer timer("KernelInitial"); 
        initial::Kernel<KernelPolicy><<<KernelPolicy::GRID_SIZE, KernelPolicy::CTA_SIZE>>>(tasksGlobal, octreeGlobal, codes.ptr(), points_num);
        cudaSafeCall( cudaGetLastError() );
        cudaSafeCall( cudaDeviceSynchronize() );
    }

    {

        int grid_size = number_of_SMs;

        typedef syncstep::KernelSyncPolicy KernelPolicy;
        //printFuncAttrib(syncstep::Kernel<KernelPolicy>);

        util::GlobalBarrierLifetime global_barrier;
        global_barrier.Setup(grid_size);                

        ScopeTimer timer("Kernelsync"); 
        syncstep::Kernel<KernelPolicy><<<grid_size, KernelPolicy::CTA_SIZE>>>(tasksGlobal, octreeGlobal, codes.ptr(), global_barrier);
        cudaSafeCall( cudaGetLastError() );
        cudaSafeCall( cudaDeviceSynchronize() );
    }    


}

int getBitsNum(int interger)
{
    int count = 0;
    while(interger > 0)
    {
        if (interger & 1)
            ++count;
        interger>>=1;
    }
    return count;
} 

struct TransversalHelperCPU
{
    const static int MAX_LEVELS_PLUS_ROOT = 11;

    int path[MAX_LEVELS_PLUS_ROOT];
    int lens[MAX_LEVELS_PLUS_ROOT];
    bool children_visited;

    int level;

    TransversalHelperCPU() 
    {
        level = 0;
        path[level] = 0;
        lens[level] = 1;
        children_visited = false;
    }

    int getNode() const { return path[level]; };
    void gotoNextLevel(int first, int len) 
    {
        ++level;
        path[level] = first;
        lens[level] = len;
        children_visited = false;        
    }
    bool gotoSibling()
    {
        ++path[level];
        --lens[level];
        children_visited = false;
        return lens[level] != 0;        
    }

    void gotoParent()
    {
        --level;
        children_visited = true;
    }    
};

void pcl::gpu::Octree2::radiusSearch2(const float3& center, float radius, vector<int>& out) const
{    
    const Octree_host::OctreeData& octree = host;
    out.clear();   

    vector<int> check;

    TransversalHelperCPU helper;
                
    while(helper.level >= 0)
    {        
        //just returned from last child
        if (helper.children_visited)
        {
            if (!helper.gotoSibling())
                helper.gotoParent();
            continue;
        }
                
        // neither chilren nor current node were visited, so test it
        int node_idx = helper.getNode();
        int code = octree.node_codes[node_idx];

        float3 node_minp = octreeGlobal.minp, node_maxp = octreeGlobal.maxp;        
        calcBoundingBox(helper.level, code, node_minp, node_maxp);

        //if true, take nothing, and go to next
        if (checkIfNodeOutsideSphere(node_minp, node_maxp, center, radius))
        {                
            if (!helper.gotoSibling())
                helper.gotoParent();
            continue;
        }

        //if true, take all, and go to next
        if (checkIfNodeInsideSphere(node_minp, node_maxp, center, radius))
        {            
            int beg = octree.begs[node_idx];
            int end = octree.ends[node_idx];

            out.insert(out.end(), octree.indices.begin() + beg, octree.indices.begin() + end);
            
            if (!helper.gotoSibling())
                helper.gotoParent();
            continue;
        }

        /* TEST_CHILDREN */

        int children_mask = octree.nodes[node_idx] & 0xFF;
        int first         = octree.nodes[node_idx] >> 8;
        bool isLeaf = children_mask == 0;

        if (!isLeaf)
        {
            helper.gotoNextLevel(first, getBitsNum(children_mask));
        }
        else
        {
            check.push_back(node_idx);
            if (!helper.gotoSibling())
                helper.gotoParent();
        }        
    }

    for(size_t i = 0; i < check.size(); ++i)
    {
        int beg = octree.begs[check[i]];
        int end = octree.ends[check[i]];

        for(int i = beg; i < end; ++i)
        {
            int index = octree.indices[i];
            const float3& point = points_host[i];

            float dx = (point.x - center.x);
            float dy = (point.y - center.y);
            float dz = (point.z - center.z);
                    
            float dist2 = dx * dx + dy * dy + dz * dz;
                    
            if (dist2 < radius * radius)
                out.push_back(index);
        }        
    }
}


void pcl::gpu::Octree2::radiusSearch(const float3& center, float radius, vector<int>& out) const
{          
    const Octree_host::OctreeData& octree = host;

    out.clear();    
        
    const int MAX_STACK_SIZE = 1024;
    int stack[MAX_STACK_SIZE];
    int pos = (0 << 8) + 0;
    stack[pos] = 0;

    vector<int> check;
    
    while (pos >= 0)
    {        
        int tmp = stack[pos--];
        int level = tmp & 0xFF;
        int node_idx = tmp >> 8;

        float3 node_minp = octreeGlobal.minp, node_maxp = octreeGlobal.maxp;
        int code = octree.node_codes[node_idx];
        calcBoundingBox(level, code, node_minp, node_maxp);

        if (checkIfNodeOutsideSphere(node_minp, node_maxp, center, radius))
            continue;
            
        if (checkIfNodeInsideSphere(node_minp, node_maxp, center, radius))
        {            
            int beg = octree.begs[node_idx];
            int end = octree.ends[node_idx];

            out.insert(out.end(), octree.indices.begin() + beg, octree.indices.begin() + end);
            continue;
        }
        
        int children_mask = octree.nodes[node_idx] & 0xFF;
        if (bool isLeaft = (children_mask == 0)) 
            check.push_back(node_idx);

        int count = getBitsNum(children_mask);
                
        int first = octree.nodes[node_idx] >> 8;
        for(int i = 0; i < count; ++i)        
            stack[++pos] = (first++ << 8) + (level + 1);
    }

    for(size_t i = 0; i < check.size(); ++i)
    {
        int beg = octree.begs[check[i]];
        int end = octree.ends[check[i]];

        for(int i = beg; i < end; ++i)
        {
            int index = octree.indices[i];
            const float3& point = points_host[i];

            float dx = (point.x - center.x);
            float dy = (point.y - center.y);
            float dz = (point.z - center.z);
                    
            float dist2 = dx * dx + dy * dy + dz * dz;
                    
            if (dist2 < radius * radius)
                out.push_back(index);
            
        }        
    }
}

void pcl::gpu::Octree2::internalDownload()
{
    int number;
    DeviceArray_<int>(octreeGlobal.nodes_num, 1).download(&number); 

    printf("Nodes = %d\n", number);    
    DeviceArray_<int>(octreeGlobal.begs,  number).download(host.begs);    
    DeviceArray_<int>(octreeGlobal.ends,  number).download(host.ends);    
    DeviceArray_<int>(octreeGlobal.nodes, number).download(host.nodes);    
    DeviceArray_<int>(octreeGlobal.codes, number).download(host.node_codes); 

    points_sorted.download(points_host);    
    indices.download(host.indices);    

    downloaded = true;
}


void pcl::gpu::Octree2::radiusSearchBatch(const DeviceArray_<float3>& queries, float radius, DeviceArray2D_<int>& output, DeviceArray_<int>& out_sizes)
{
    int query_num  = output.rows();
    int max_points = output.cols();
    
    batch_radius_search::BatchRadiusSearchShared batch;
    
    printf("out_sizes = %d\n", out_sizes.size());
    printf("max_points = %d\n", max_points);

    batch.indices = indices;
    batch.octree = octreeGlobal;
    
    batch.output = batch_radius_search::DevMem2D_<int>(output);

    batch.output_sizes = out_sizes;
    batch.points = points;
    batch.queries = queries;
    batch.radius = radius;

    int block = batch_radius_search::KernelPolicy::CTA_SIZE;
    int grid = (query_num + block - 1) / block;    

    batch_radius_search::Kernel<<<grid, block>>>(batch);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );
}