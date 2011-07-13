#include "cuda_interface.hpp"

#include<thrust/host_vector.h>
#include<thrust/sequence.h>
#include<thrust/transform.h>
#include<thrust/copy.h>
#include<thrust/sort.h>
#include<thrust/device_ptr.h>

#include "functors.hpp"
#include "utils/morton.hpp"
#include "utils/boxutils.hpp"
#include "pcl/gpu/common/timers_cuda.hpp"

#include "cuda.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

using namespace std;
using namespace thrust;
using namespace pcl::gpu;
using namespace pcl::cuda;

void pcl::gpu::Octree_host::setCloud(const DeviceArray_<float3>& input_points)
{
	points_num = input_points.size();

	points.resize(points_num);

    device_ptr<float3> input_points_beg((float3*)input_points.ptr());
    device_ptr<float3> input_points_end = input_points_beg + points_num;

	thrust::copy(input_points_beg, input_points_end, points.begin());	
	tmp.resize(points_num);	
            
	codes.resize(points_num);	
	indices.resize(points_num);		
	thrust::sequence(indices.begin(), indices.end());
}


template<int nbits> int getLevelCode(int code, int level) 
{ 
	return (code >> (nbits - 3 * (level + 1) )) & 7; 
}

template<int nbits> int makeMortonCode(int level_code, int level) 
{
    return level_code << (nbits - 3 * (level + 1));
}


struct CompareByLevel
{
    int level;
	CompareByLevel(int level_arg) : level(level_arg) {}    
	bool operator()(int code1, int code2) const 
    {          
        return getLevelCode<30>(code1, level) < getLevelCode<30>(code2, level);  
    }	
};

void pcl::gpu::Octree_host::build()
{    
	minp = thrust::reduce(points.begin(), points.end(), make_float3( FLT_MAX,  FLT_MAX,  FLT_MAX), SelectMinPoint());
	maxp = thrust::reduce(points.begin(), points.end(), make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX), SelectMaxPoint());	

    /*float enlargeRatio = 1.000001f;
    maxp.x = (maxp.x - minp.x) * enlargeRatio + minp.x
    maxp.x = (maxp.x - minp.x) * enlargeRatio + minp.x
    maxp.x = (maxp.x - minp.x) * enlargeRatio + minp.x*/
                             
	thrust::transform(points.begin(), points.end(), codes.begin(), CalcMorton(minp, maxp));
        
#if 0
    thrust::sort_by_key(codes.begin(), codes.end(), make_zip_iterator( make_tuple( points.begin(), indices.begin() ) ) );		
#else
    {
        ScopeTimer t("sort");
        thrust::sort_by_key(codes.begin(), codes.end(), indices.begin() );

        thrust::copy(make_permutation_iterator(points.begin(), indices.begin()),
                     make_permutation_iterator(points.begin(), indices.end()  ), tmp.begin());

        tmp.swap(points);
    };        
#endif
	
    vector<int> codes_h;
    getCodesInds(codes_h, octree.indices);	

    points_host.resize(points_num);
    thrust::copy(points.begin(), points.end(), points_host.begin());
        

    //init root node
	octree.nodes.push_back(0); 
	octree.begs.push_back(0);        
	octree.ends.push_back(points_num); 
    octree.node_codes.push_back(0); // root code is always 0;    
    int nodes_num = 1;

    vector<int> tasks1;
    vector<int> tasks2;

    vector<int>* tasksIn = &tasks1;
    vector<int>* tasksOut = &tasks2;
    tasksIn->push_back(0);

    int level = 0;

    vector< vector<int > > cell_begs;
    vector< vector<char> > cell_ids;
    vector<int> offsets;

    while(!tasksIn->empty())
    {           
        size_t tasksIn_size = tasksIn->size();        

        cell_begs.resize(tasksIn_size);
        cell_ids.resize(tasksIn_size);

        //to be parraleled
        for(size_t t = 0; t < tasksIn_size; ++t)
        {
            int task = (*tasksIn)[t];

            int beg = octree.begs[task];
            int end = octree.ends[task];

            if (end - beg < max_leaf_points)
            {
                cell_ids[t].clear();
                continue;
            }            

            vector<int>& curr_cells = cell_begs[t];
            vector<char>& curr_ids = cell_ids[t];
            curr_cells.clear();
            curr_ids.clear();
            
            int curr_code = getLevelCode<30>(codes_h[beg], level);
            curr_ids.push_back(curr_code);
            curr_cells.push_back(beg);

            int end1_code = getLevelCode<30>(codes_h[end-1], level);

            if (end1_code == curr_code)
            {
                curr_cells.push_back(end);
                continue;
            }
                        
            for(;;)
            { 
                int search_code = curr_code + 1;
                if (search_code == 8)
                {
                    curr_cells.push_back(end);
                    break;
                }
                
                typedef vector<int>::iterator It;
                It seg_beg = codes_h.begin() + beg;
                It seg_end = codes_h.begin() + end;

                int pos = lower_bound(seg_beg, seg_end, makeMortonCode<30>(search_code, level), CompareByLevel(level)) - codes_h.begin();                                
                if (pos == end)
                {
                    curr_cells.push_back(pos);
                    break;
                }
                curr_cells.push_back(pos);

                curr_code = getLevelCode<30>(codes_h[pos], level);
                curr_ids.push_back(curr_code);
                beg = pos;
            }            
        }

        //sync

        offsets.resize(tasksIn_size);

        offsets[0] = 0;
        for(size_t i = 0; i < tasksIn_size - 1; ++i)
            offsets[i+1] = offsets[i] + cell_ids[i].size();

        //sync
        int total_new = offsets.back() + cell_ids.back().size();

        tasksOut->resize(total_new);

        octree.nodes.resize(nodes_num + total_new);
        octree.begs.resize(nodes_num + total_new);
        octree.ends.resize(nodes_num + total_new);        
        octree.node_codes.resize(nodes_num + total_new);
        
       
        //mask be atomic
        tasksOut->resize(total_new);

        //to be parraleled
        for(size_t t = 0; t < tasksIn_size; ++t)
        {
            int task = (*tasksIn)[t];

            vector<char>& curr_ids = cell_ids[t];
            if (curr_ids.empty())
                continue;

            int parent_code_shifted = octree.node_codes[task] << 3;

            vector<int>& curr_begs = cell_begs[t];                                   
            int offset = offsets[t];

            int mask = 0;
            for(size_t i = 0; i < curr_ids.size(); ++i)
            {
                //octree.nodes[nodes_num + offset] = 0;
                octree.begs[nodes_num + offset + i] = curr_begs[i];
                octree.ends[nodes_num + offset + i] = curr_begs[i+1];
                octree.node_codes[nodes_num + offset + i] = parent_code_shifted + curr_ids[i];
                mask |= (1 << curr_ids[i]);

                (*tasksOut)[offset + i] = nodes_num + offset + i;
            }

            octree.nodes[task] = ((nodes_num + offset) << 8) + mask;
        }
        nodes_num += total_new;

        //sync

        std::swap(tasksOut, tasksIn);
        ++level;        
        printf("Level-%d ", level);        
	}   	

    printf("\tDone - %d\n", octree.nodes.size());
}

void pcl::gpu::Octree_host::calcBoundingBoxOld(int level, int code, float3& res_minp, float3& res_maxp) const
{
    res_minp = minp;
    res_maxp = maxp;

    while(level > 0)
    {   
        --level;
        int split = (code >> (level * 3))& 7;        

        float mid_x = (res_maxp.x + res_minp.x)/2;
        float mid_y = (res_maxp.y + res_minp.y)/2;
        float mid_z = (res_maxp.z + res_minp.z)/2;

        float& modifx = ((split & 1) == 1) ? res_minp.x : res_maxp.x;
        float& modify = ((split & 2) == 2) ? res_minp.y : res_maxp.y;
        float& modifz = ((split & 4) == 4) ? res_minp.z : res_maxp.z;
        
        modifx = mid_x;
        modify = mid_y;
        modifz = mid_z;
    }    
}

void pcl::gpu::Octree_host::radiusSearch(const float3& center, float radius, vector<int>& out) const
{
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

        float3 node_minp = minp, node_maxp = maxp;
        int code = octree.node_codes[node_idx];
        calcBoundingBoxOld(level, code, node_minp, node_maxp);
        
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
        if (bool isLEaft = (children_mask == 0)) 
            check.push_back(node_idx);

        int count = 0;
        while(children_mask > 0)
        {
            if (children_mask & 1)
                ++count;
            children_mask>>=1;
        }
        
        int first = octree.nodes[node_idx] >> 8;
        for(int i = 0; i < count; ++i)        
            stack[++pos] = (first++ << 8) + (level + 1);
    }

    if (check.empty())
        return;    

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

void pcl::gpu::Octree_host::getCodesInds(vector<int>& codes_h, vector<int>& inds)
{
    codes_h.resize(points_num);
    inds.resize(points_num);
    thrust::copy(codes.begin(), codes.end(), codes_h.begin());		

    octree.indices.resize(points_num);
    thrust::copy(indices.begin(), indices.end(), inds.begin());
}

