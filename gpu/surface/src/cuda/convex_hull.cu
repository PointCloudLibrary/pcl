/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 *
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
 */

#include "internal.h"
#include "device.h"
#include <limits>

#include <pcl/gpu/utils/device/algorithm.hpp>
#include <pcl/gpu/utils/device/warp.hpp>
//#include <pcl/gpu/utils/device/funcattrib.hpp>
#include <pcl/gpu/utils/safe_call.hpp>

#include <thrust/tuple.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/transform_reduce.h>
#include <thrust/functional.h>
#include <thrust/sequence.h>
#include "thrust/device_ptr.h"
#include <thrust/transform.h>
#include <thrust/sort.h>
#include <thrust/transform_scan.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/unique.h>
#include <thrust/gather.h>

namespace pcl
{
  namespace device
  { 	  
	  template<bool use_max>
	  struct IndOp
	  {
		  __device__ __forceinline__ thrust::tuple<float, int> operator()(const thrust::tuple<float, int>& e1, const thrust::tuple<float, int>& e2) const
		  {	
			  thrust::tuple<float, int> res;
			  
			  if (use_max)
			    res.get<0>() = fmax(e1.get<0>(), e2.get<0>());			  			  
			  else
				res.get<0>() = fmin(e1.get<0>(), e2.get<0>());			  			  

			  res.get<1>()  = (res.get<0>() == e1.get<0>()) ? e1.get<1>() : e2.get<1>();
			  return res;			  
		  }		 
	  };

	  struct X
	  {			  
		  __device__ __forceinline__ 
		  thrust::tuple<float, int> 
		  operator()(const thrust::tuple<PointType, int>& in) const
		  {
			return thrust::tuple<float, int>(in.get<0>().x, in.get<1>());			  
		  }
	  };

	  struct Y
	  {			  
		  __device__ __forceinline__  float operator()(const PointType& in) const { return in.y; }
	  };

	  struct Z
	  {			  
		  __device__ __forceinline__  float operator()(const PointType& in) const { return in.z; }
	  };
		  
	  struct LineDist
	  {
		  float3 x1, x2;
		  LineDist(const PointType& p1, const PointType& p2) : x1(tr(p1)), x2(tr(p2)) {}
		  
		  __device__ __forceinline__
		  thrust::tuple<float, int> operator()(const thrust::tuple<PointType, int>& in) const
		  {			  
			  float3 x0 = tr(in.get<0>());

			  float dist = norm(cross(x0 - x1, x0 - x2))/norm(x1 - x2);			  
			  return thrust::tuple<float, int>(dist, in.get<1>());
		  }	      
	  };

	  struct PlaneDist
	  {		  
		  float3 x1, n;
		  PlaneDist(const PointType& p1, const PointType& p2, const PointType& p3) : x1(tr(p1))
		  {
			  float3 x2 = tr(p2), x3 = tr(p3);
              n = normalized(cross(x2 - x1, x3 - x1));
		  }
		  
		  __device__ __forceinline__
		  thrust::tuple<float, int> operator()(const thrust::tuple<PointType, int>& in) const
		  {
			  float3 x0 = tr(in.get<0>());
              float dist = std::abs(dot(n, x0 - x1));
			  return thrust::tuple<float, int>(dist, in.get<1>());
		  }
	  };
	  
	  template<typename It, typename Unary, typename Init, typename Binary>
      int transform_reduce_index(It beg, It end, Unary unop, Init init, Binary binary)
	  {
	    thrust::counting_iterator<int> cbeg(0);
		thrust::counting_iterator<int> cend = cbeg + thrust::distance(beg, end);
			 		
	    thrust::tuple<float, int> t = transform_reduce( 
		  make_zip_iterator(thrust::make_tuple(beg, cbeg)), 
		  make_zip_iterator(thrust::make_tuple(end, cend)), 
		  unop, init, binary);
		
		return t.get<1>();
	  }

	  template<typename It, typename Unary>
      int transform_reduce_min_index(It beg, It end, Unary unop)
	  {
		thrust::tuple<float, int> min_tuple(std::numeric_limits<float>::max(), 0);
		return transform_reduce_index(beg, end, unop, min_tuple, IndOp<false>());
	  }

	  template<typename It, typename Unary>
      int transform_reduce_max_index(It beg, It end, Unary unop)
	  {
		thrust::tuple<float, int> max_tuple(std::numeric_limits<float>::min(), 0);
		return transform_reduce_index(beg, end, unop, max_tuple, IndOp<true>());
	  }	 
  }
}

pcl::device::PointStream::PointStream(const Cloud& cloud_) : cloud(cloud_)
{				
  cloud_size = cloud.size();
  facets_dists.create(cloud_size);
  perm.create(cloud_size);

  thrust::device_ptr<int> pbeg(perm.ptr());  
  thrust::sequence(pbeg, pbeg + cloud_size);
}

void pcl::device::PointStream::computeInitalSimplex()
{
  thrust::device_ptr<const PointType> beg(cloud.ptr());  
  thrust::device_ptr<const PointType> end = beg + cloud_size;
     
  int minx = transform_reduce_min_index(beg, end, X());
  int maxx = transform_reduce_max_index(beg, end, X());

  PointType p1 = *(beg + minx);
  PointType p2 = *(beg + maxx);
  	        
  int maxl = transform_reduce_max_index(beg, end, LineDist(p1, p2));

  PointType p3 = *(beg + maxl);
    
  int maxp = transform_reduce_max_index(beg, end, PlaneDist(p1, p2, p3));

  PointType p4 = *(beg + maxp);

  simplex.x1 = tr(p1);  simplex.x2 = tr(p2);  simplex.x3 = tr(p3);  simplex.x4 = tr(p4);
  simplex.i1 = minx;    simplex.i2 = maxx;    simplex.i3 = maxl;    simplex.i4 = maxp;

  float maxy = transform_reduce(beg, end, Y(), std::numeric_limits<float>::min(), thrust::maximum<float>()); 
  float miny = transform_reduce(beg, end, Y(), std::numeric_limits<float>::max(), thrust::minimum<float>()); 

  float maxz = transform_reduce(beg, end, Z(), std::numeric_limits<float>::min(), thrust::maximum<float>()); 
  float minz = transform_reduce(beg, end, Z(), std::numeric_limits<float>::max(), thrust::minimum<float>()); 
		  
  float dx = (p2.x - p1.x);
  float dy = (maxy - miny);
  float dz = (maxz - minz);

  cloud_diag = sqrt(dx*dx + dy*dy + dz*dz);

  simplex.p1 = compute_plane(simplex.x4, simplex.x2, simplex.x3, simplex.x1);
  simplex.p2 = compute_plane(simplex.x3, simplex.x1, simplex.x4, simplex.x2);
  simplex.p3 = compute_plane(simplex.x2, simplex.x1, simplex.x4, simplex.x3);
  simplex.p4 = compute_plane(simplex.x1, simplex.x2, simplex.x3, simplex.x4);  
}

namespace pcl
{
  namespace device
  {
    __global__ void init_fs(int i1, int i2, int i3, int i4, PtrStep<int> verts_inds)
	{	  	  	  
      *(int4*)verts_inds.ptr(0) = make_int4(i2, i1, i1, i1);
      *(int4*)verts_inds.ptr(1) = make_int4(i3, i3, i2, i2);
      *(int4*)verts_inds.ptr(2) = make_int4(i4, i4, i4, i3);
	}

  }
}

void pcl::device::FacetStream::setInitialFacets(const InitalSimplex& s)
{  
  init_fs<<<1, 1>>>(s.i1, s.i2, s.i3, s.i4, verts_inds);  
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );  
  facet_count = 4;
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace device
  {
	struct InitalClassify
	{
      float diag;
      float4 pl1, pl2, pl3, pl4;
	  	  
      InitalClassify(const float4& p1, const float4& p2, const float4& p3, const float4& p4, float diagonal) 
          : diag(diagonal), pl1(p1), pl2(p2), pl3(p3), pl4(p4)
	  {				
        pl1 *= compue_inv_normal_norm(pl1);
        pl2 *= compue_inv_normal_norm(pl2);
        pl3 *= compue_inv_normal_norm(pl3);
        pl4 *= compue_inv_normal_norm(pl4);
	  }
	  	  
	  __device__ __forceinline__
	  std::uint64_t 
	  operator()(const PointType& p) const
	  {                   
		  float4 x = p;
		  x.w = 1;

          float d0 = dot(pl1, x);
          float d1 = dot(pl2, x);
          float d2 = dot(pl3, x);
          float d3 = dot(pl4, x);

          float dists[] = { d0, d1, d2, d3 };
          int negs_inds[4];
          int neg_count = 0;
          
          int idx = std::numeric_limits<int>::max();
          float dist = 0;

          #pragma unroll
          for(int i = 0; i < 4; ++i)
            if (dists[i] < 0)
              negs_inds[neg_count++] = i;

          if (neg_count == 3)
          {
             int i1 = negs_inds[1];
             int i2 = negs_inds[2];
             
             int ir = std::abs(dists[i1]) < std::abs(dists[i2]) ? i2 : i1;
             negs_inds[1] = ir;
             --neg_count;
          }

          if (neg_count == 2)
          {
             int i1 = negs_inds[0];
             int i2 = negs_inds[1];
             
             int ir = std::abs(dists[i1]) < std::abs(dists[i2]) ? i2 : i1;
             negs_inds[0] = ir;
             --neg_count;              
          }

          if (neg_count == 1)
          {
            idx = negs_inds[0];
            dist = diag - std::abs(dists[idx]); // to ensure that sorting order is inverse, i.e. distant points go first
          }

          //if (neg_count == 0)
          //  then internal point ==>> idx = std::numeric_limits<int>::max()

		  std::uint64_t res = idx;
		  res <<= 32;
		  return res + *reinterpret_cast<unsigned int*>(&dist);
	  }		
	};		

    __global__ void initalClassifyKernel(const InitalClassify ic, const PointType* points, int cloud_size, std::uint64_t* output) 
    { 
        int index = threadIdx.x + blockIdx.x * blockDim.x;

        if (index < cloud_size)              
          output[index] = ic(points[index]); 
    }
  }
}

void pcl::device::PointStream::initalClassify()
{        
  //thrust::device_ptr<const PointType> beg(cloud.ptr());
  //thrust::device_ptr<const PointType> end = beg + cloud_size;
  thrust::device_ptr<std::uint64_t> out(facets_dists.ptr());
  
  InitalClassify ic(simplex.p1, simplex.p2, simplex.p3, simplex.p4, cloud_diag);
  //thrust::transform(beg, end, out, ic);
  
  //printFuncAttrib(initalClassifyKernel);

  initalClassifyKernel<<<divUp(cloud_size, 256), 256>>>(ic, cloud, cloud_size, facets_dists);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );

  thrust::device_ptr<int> pbeg(perm.ptr());
  thrust::sort_by_key(out, out + cloud_size, pbeg);
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace device
  {
    __device__ int new_cloud_size;    
	struct SearchFacetHeads
	{		
	  std::uint64_t *facets_dists;
	  int cloud_size;
	  int facet_count;
	  int *perm;
	  const PointType* points;

	  mutable int* head_points;
      //bool logger;
	
	  __device__ __forceinline__
	  void operator()(int facet) const
	  {			
		const std::uint64_t* b = facets_dists;
		const std::uint64_t* e = b + cloud_size;

        bool last_thread = facet == facet_count;

        int search_value = !last_thread ? facet : std::numeric_limits<int>::max();		
		int index = lower_bound(b, e, search_value, LessThanByFacet()) - b;			
        
        if (last_thread)
            new_cloud_size = index;
        else
        {
          bool not_found = index == cloud_size || (facet != (facets_dists[index] >> 32));

          head_points[facet] = not_found ? -1 : perm[index];		
        }
	  }
	};

    __global__ void searchFacetHeadsKernel(const SearchFacetHeads sfh)
    {
        int facet = threadIdx.x + blockDim.x * blockIdx.x;

        if (facet <= sfh.facet_count)
          sfh(facet);
    }
  }
}

int pcl::device::PointStream::searchFacetHeads(std::size_t facet_count, DeviceArray<int>& head_points)
{
	SearchFacetHeads sfh;

	sfh.facets_dists = facets_dists;
	sfh.cloud_size = (int)cloud_size;
	sfh.facet_count = (int)facet_count;
	sfh.perm = perm;
	sfh.points = cloud.ptr();
	sfh.head_points = head_points;  
	
    //thrust::counting_iterator<int> b(0);
    //thrust::counting_iterator<int> e = b + facet_count + 1;  	
    //thrust::for_each(b, e, sfh);

    searchFacetHeadsKernel<<<divUp(facet_count+1, 256), 256>>>(sfh);
    cudaSafeCall( cudaGetLastError() );
    cudaSafeCall( cudaDeviceSynchronize() );        

	int new_size;
	cudaSafeCall( cudaMemcpyFromSymbol(	(void*)&new_size,  pcl::device::new_cloud_size, sizeof(new_size)) );	
	return new_size;
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace device
  {
    struct NotMinus1
	{
	  __device__ __forceinline__
	  int operator()(const int& v) const { return (v == -1) ?  0 : 1; }
	};


	struct Compaction
	{
		enum 
		{
			CTA_SIZE = 256,

			WARPS = CTA_SIZE/ Warp::WARP_SIZE
		};

		int* head_points_in;
		PtrStep<int>  verts_inds_in;
		

		int *scan_buffer;
		int facet_count;

		mutable int* head_points_out;
		mutable PtrStep<int>  verts_inds_out;
		

		mutable PtrStep<int> empty_facets;
		mutable int *empty_count;
		  
		__device__ __forceinline__
		void operator()() const
		{
			int idx = threadIdx.x + blockIdx.x * blockDim.x;

#if CUDART_VERSION >= 9000
      if (__all_sync (__activemask (), idx >= facet_count))
        return;
#else
			if (__all (idx >= facet_count))
				return;
#endif

			int empty = 0;

			if(idx < facet_count)
			{
				int head_idx = head_points_in[idx];
				if (head_idx != -1)
				{
					int offset = scan_buffer[idx];

					head_points_out[offset] = head_idx;
					
					verts_inds_out.ptr(0)[offset] = verts_inds_in.ptr(0)[idx];
					verts_inds_out.ptr(1)[offset] = verts_inds_in.ptr(1)[idx];
					verts_inds_out.ptr(2)[offset] = verts_inds_in.ptr(2)[idx];

                    
					
				}
				else                
				  empty = 1;                
			}

#if CUDART_VERSION >= 9000
      int total = __popc (__ballot_sync (__activemask (), empty));
#else
			int total = __popc (__ballot (empty));
#endif
			if (total > 0)
			{
#if CUDART_VERSION >= 9000
        int offset = Warp::binaryExclScan (__ballot_sync (__activemask (), empty));
#else
				int offset = Warp::binaryExclScan (__ballot (empty));
#endif

				volatile __shared__ int wapr_buffer[WARPS];

				int laneid = Warp::laneId();
				int warpid = Warp::id();
				if (laneid == 0)
				{
					int old = atomicAdd(empty_count, total);
					wapr_buffer[warpid] = old;                    
				}
				int old = wapr_buffer[warpid];

                if (empty)
                {
				  empty_facets.ptr(0)[old + offset] = verts_inds_in.ptr(0)[idx];
				  empty_facets.ptr(1)[old + offset] = verts_inds_in.ptr(1)[idx];
				  empty_facets.ptr(2)[old + offset] = verts_inds_in.ptr(2)[idx];		                  

                  int a1 = verts_inds_in.ptr(0)[idx], a2 = verts_inds_in.ptr(1)[idx], a3 = verts_inds_in.ptr(2)[idx];
                }
			}							
		}
	};

	__global__ void compactionKernel( const Compaction c )  { c(); }
  }
}


void pcl::device::FacetStream::compactFacets()
{
  int old_empty_count;  
  empty_count.download(&old_empty_count); 

  thrust::device_ptr<int> b(head_points.ptr());
  thrust::device_ptr<int> e = b + facet_count;
  thrust::device_ptr<int> o(scan_buffer.ptr());
  
  thrust::transform_exclusive_scan(b, e, o, NotMinus1(), 0, thrust::plus<int>());                                                                                    
  
  Compaction c;

  c.verts_inds_in   = verts_inds;
  c.head_points_in  = head_points;    

  c.scan_buffer = scan_buffer;
  c.facet_count = facet_count;

  c.head_points_out = head_points2;
  c.verts_inds_out = verts_inds2;

  c.empty_facets = empty_facets;
  c.empty_count = empty_count;
 
  int block = Compaction::CTA_SIZE;
  int grid = divUp(facet_count, block);

  compactionKernel<<<grid, block>>>(c);   
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );
    
  verts_inds.swap(verts_inds2);
  head_points.swap(head_points2);

  int new_empty_count;  
  empty_count.download(&new_empty_count); 
  
  facet_count -= new_empty_count - old_empty_count;
}


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace device
  {
	  struct Classify
	  {
		std::uint64_t* facets_dists;
		int* scan_buffer;

		int* head_points;
		int* perm;
		PtrStep<int>  verts_inds;

		const PointType *points;

		float diag;

		int facet_count;

		__device__ __forceinline__ 
		void operator()(int point_idx) const
		{
          int perm_index = perm[point_idx];
          
		  int facet = facets_dists[point_idx] >> 32;
		  facet = scan_buffer[facet];

		  int hi = head_points[facet];

          if (hi == perm_index)
          {
            std::uint64_t res = std::numeric_limits<int>::max();
		    res <<= 32;		                      
            facets_dists[point_idx] = res;
          }
          else            
          {

		    int i1 = verts_inds.ptr(0)[facet];
		    int i2 = verts_inds.ptr(1)[facet];
		    int i3 = verts_inds.ptr(2)[facet];

		    float3 hp = tr( points[ hi ] );
		    float3 v1 = tr( points[ i1 ] );
		    float3 v2 = tr( points[ i2 ] );
		    float3 v3 = tr( points[ i3 ] );
		
		    float4 p0 = compute_plane(hp, v1, v2, /*opposite*/v3); // j
		    float4 p1 = compute_plane(hp, v2, v3, /*opposite*/v1); // facet_count + j
		    float4 p2 = compute_plane(hp, v3, v1, /*opposite*/v2); // facet_count + j*2			

            p0 *= compue_inv_normal_norm(p0);
            p1 *= compue_inv_normal_norm(p1);
            p2 *= compue_inv_normal_norm(p2);

          
		    float4 p = points[perm_index];
		    p.w = 1;

		    float d0 = dot(p, p0);
		    float d1 = dot(p, p1);
		    float d2 = dot(p, p2);

            float dists[] = { d0, d1, d2 };
            int negs_inds[3];
            int neg_count = 0;

            int new_idx = std::numeric_limits<int>::max();
            float dist = 0;

            int indeces[] = { facet, facet + facet_count, facet + facet_count * 2 };

            #pragma unroll
            for(int i = 0; i < 3; ++i)
              if (dists[i] < 0)
               negs_inds[neg_count++] = i;
 
            if (neg_count == 3)
            {
              int i1 = negs_inds[1];
              int i2 = negs_inds[2];
           
              int ir = std::abs(dists[i1]) < std::abs(dists[i2]) ? i2 : i1;
              negs_inds[1] = ir;
              --neg_count;
            }

            if (neg_count == 2)
            {
              int i1 = negs_inds[0];
              int i2 = negs_inds[1];
           
              int ir = std::abs(dists[i1]) < std::abs(dists[i2]) ? i2 : i1;
              negs_inds[0] = ir;
              --neg_count;              
            }

            if (neg_count == 1)
            {
              new_idx = negs_inds[0];
              dist = diag - std::abs(dists[new_idx]); // to ensure that sorting order is inverse, i.e. distant points go first
              new_idx = indeces[new_idx];
            }

            // if (neg_count == 0)
            // new_idx = std::numeric_limits<int>::max() ==>> internal point
                      	       	 	   
            std::uint64_t res = new_idx;
		    res <<= 32;
		    res += *reinterpret_cast<unsigned int*>(&dist);

		    facets_dists[point_idx] = res;

          } /* if (hi == perm_index) */            
        }
	  };    

      __global__ void classifyKernel(const Classify c, int cloud_size)
      {
        int point_idx = threadIdx.x + blockIdx.x * blockDim.x;

        if ( point_idx < cloud_size )
          c(point_idx);
      }
  }
}

void pcl::device::PointStream::classify(FacetStream& fs)
{   
  Classify c;

  c.facets_dists = facets_dists;
  c.scan_buffer = fs.scan_buffer;
  c.head_points = fs.head_points;
  c.perm = perm;

  c.verts_inds = fs.verts_inds;
  c.points = cloud;

  c.diag = cloud_diag;
  c.facet_count = fs.facet_count;

  //thrust::counting_iterator<int> b(0);    
  //thrust::for_each(b, b + cloud_size, c);

  classifyKernel<<<divUp(cloud_size, 256), 256>>>(c, cloud_size);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );
  
  thrust::device_ptr<std::uint64_t> beg(facets_dists.ptr());
  thrust::device_ptr<std::uint64_t> end = beg + cloud_size;
  
  thrust::device_ptr<int> pbeg(perm.ptr());
  thrust::sort_by_key(beg, end, pbeg);
}

namespace pcl
{
  namespace device
  {
    struct SplitFacets
    {
      int* head_points;
      int facet_count;

      mutable PtrStep<int>  verts_inds;

      __device__ __forceinline__ 
      void operator()(int facet) const
      {
        int hi = head_points[facet];
        int i1 = verts_inds.ptr(0)[facet];
        int i2 = verts_inds.ptr(1)[facet];
        int i3 = verts_inds.ptr(2)[facet];
        
        make_facet(hi, i1, i2, facet);
        make_facet(hi, i2, i3, facet + facet_count);
        make_facet(hi, i3, i1, facet + facet_count * 2);
      }

      __device__ __forceinline__
      void make_facet(int i1, int i2, int i3, int out_idx) const
      {
        verts_inds.ptr(0)[out_idx] = i1;
        verts_inds.ptr(1)[out_idx] = i2;
        verts_inds.ptr(2)[out_idx] = i3;
      }
    };

    __global__ void splitFacetsKernel(const SplitFacets sf)
    {
      int facet = threadIdx.x + blockIdx.x * blockDim.x;

      if (facet < sf.facet_count)        
        sf(facet);        
    }
  }
}

void pcl::device::FacetStream::splitFacets()
{
  SplitFacets sf;
  sf.head_points = head_points;
  sf.verts_inds = verts_inds;
  sf.facet_count = facet_count;
    

  //thrust::counting_iterator<int> b(0);    
  //thrust::for_each(b, b + facet_count, sf);

  splitFacetsKernel<<<divUp(facet_count, 256), 256>>>(sf);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );

  facet_count *= 3;
}

size_t pcl::device::remove_duplicates(DeviceArray<int>& indeces)
{
  thrust::device_ptr<int> beg(indeces.ptr());
  thrust::device_ptr<int> end = beg + indeces.size();

  thrust::sort(beg, end);  
  return (std::size_t)(thrust::unique(beg, end) - beg);  
}


namespace pcl
{
  namespace device
  {
    __global__ void gatherKernel(const PtrSz<int> indeces, const PointType* src, PointType* dst)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;

        if (idx < indeces.size)
          dst[idx] = src[indeces.data[idx]];
    }
  }
}


void pcl::device::pack_hull(const DeviceArray<PointType>& points, const DeviceArray<int>& indeces, DeviceArray<PointType>& output)
{
  output.create(indeces.size());

  //thrust::device_ptr<const PointType> in(points.ptr());  
  
  //thrust::device_ptr<const int> mb(indeces.ptr());
  //thrust::device_ptr<const int> me = mb + indeces.size();

  //thrust::device_ptr<PointType> out(output.ptr());  

  //thrust::gather(mb, me, in, out);
  
  gatherKernel<<<divUp(indeces.size(), 256), 256>>>(indeces, points, output);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );
}
