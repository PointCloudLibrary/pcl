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

#ifndef PCL_GPU_SURFACE_INTERNAL_H_
#define PCL_GPU_SURFACE_INTERNAL_H_

#include <pcl/gpu/containers/device_array.h>
#include <cuda_runtime.h>

namespace pcl
{
  namespace device
  {
	  typedef unsigned long long uint64_type;

	  typedef float4 PointType;
	  typedef pcl::gpu::DeviceArray<PointType> Cloud;

	  typedef DeviceArray<uint64_type> FacetsDists;
	  typedef DeviceArray<int> Perm;

	  struct InitalSimplex
	  {
		  float3 x1, x2, x3, x4;
		  int i1, i2, i3, i4;

		  float4 p1, p2, p3, p4;
	  };

	  struct FacetStream
	  {	
	  public:
		  FacetStream(size_t buffer_size);

          // indeces: in each col indeces of vertexes for sigle facet
		  DeviceArray2D<int>  verts_inds;		  

		  DeviceArray<int> head_points;		  
		  size_t facet_count;

		  DeviceArray2D<int>  empty_facets;
		  DeviceArray<int> empty_count;
		  
		  DeviceArray<int>  scan_buffer;

		  void setInitialFacets(const InitalSimplex& simplex);

		  void compactFacets();

		  bool canSplit();
		  void splitFacets();
	  private:
		  
          //for compation (double buffering)
		  DeviceArray2D<int>  verts_inds2;
		  DeviceArray<float4> facet_planes2;
		  DeviceArray<int> head_points2;		  
	  };	
	 
	  struct PointStream
	  {
	  public:
		  PointStream(const Cloud& cloud);
		  
		  const Cloud cloud;
		  FacetsDists facets_dists;
		  Perm perm;

		  size_t cloud_size;

		  InitalSimplex simplex;
		  float cloud_diag;

		  void computeInitalSimplex();

		  void initalClassify();
		  

		  int searchFacetHeads(size_t facet_count, DeviceArray<int>& head_points);

		  void classify(FacetStream& fs);	  		  
	  };	 	  	


	  size_t remove_duplicates(DeviceArray<int>& indeces);
	  void pack_hull(const DeviceArray<PointType>& points, const DeviceArray<int>& indeces, DeviceArray<PointType>& output);
  }
}

#endif /* PCL_GPU_SURFACE_INTERNAL_H_ */ 
