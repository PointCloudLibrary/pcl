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
 *  COPYRIGHT OWNER OR CONTR
 IBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/gpu/surface/convex_hull.h>
#include <pcl/gpu/utils/device/static_check.hpp>
#include "internal.h"
#include <pcl/exceptions.h>

pcl::device::FacetStream::FacetStream(std::size_t buffer_size)
{
  verts_inds.create(3, buffer_size);  
  head_points.create(buffer_size);
  
  scan_buffer.create(buffer_size);
  facet_count = 0;

  verts_inds2.create(3,buffer_size);  
  head_points2.create(buffer_size);

  empty_facets.create(3, buffer_size);

  int zero = 0;  
  empty_count.upload(&zero, 1);
}

bool 
pcl::device::FacetStream::canSplit() const
{
  return static_cast<Eigen::Index>(facet_count * 3) < verts_inds.cols();
}

struct pcl::gpu::PseudoConvexHull3D::Impl
{
    Impl(std::size_t buffer_size) : fs(buffer_size) {}
    ~Impl() {};
    
    device::FacetStream fs;
};

pcl::gpu::PseudoConvexHull3D::PseudoConvexHull3D(std::size_t bsize)
{
  impl_.reset( new Impl(bsize) );
}
pcl::gpu::PseudoConvexHull3D::~PseudoConvexHull3D() {}


void 
pcl::gpu::PseudoConvexHull3D::reconstruct (const Cloud &cloud, DeviceArray2D<int>& vertexes)
{     
  const device::Cloud& c = (const device::Cloud&)cloud;
    
  device::FacetStream& fs = impl_->fs;
  device::PointStream ps(c);

  ps.computeInitalSimplex();

  fs.setInitialFacets(ps.simplex);
  ps.initalClassify();
    
  for(;;)
  {
	//new external points number
    ps.cloud_size = ps.searchFacetHeads(fs.facet_count, fs.head_points);  
	if (ps.cloud_size == 0)
		break;
      
	fs.compactFacets();    
    ps.classify(fs);
    	
	if (!fs.canSplit())
		throw PCLException("Can't split facets, please enlarge default buffer", __FILE__, "", __LINE__);    
		
	fs.splitFacets();
  }
    
  int ecount;
  int fcount = fs.facet_count;
  fs.empty_count.download(&ecount);
  
  vertexes.create(3, fcount + ecount);
  DeviceArray2D<int> subf(3, fcount, vertexes.ptr(),        vertexes.step());
  DeviceArray2D<int> sube(3, ecount, vertexes.ptr()+fcount, vertexes.step());
  
  DeviceArray2D<int>(3, fcount, fs.verts_inds.ptr(), fs.verts_inds.step()).copyTo(subf);  
  DeviceArray2D<int>(3, ecount, fs.empty_facets.ptr(), fs.empty_facets.step()).copyTo(sube);  
}

void
pcl::gpu::PseudoConvexHull3D::reconstruct (const Cloud &points, Cloud &output)
{
  DeviceArray2D<int> vertexes;
  reconstruct(points, vertexes);

  DeviceArray<int> cont(vertexes.cols() * vertexes.rows());
  DeviceArray2D<int> buf(3, vertexes.cols(), cont.ptr(), vertexes.cols() * sizeof(int));
  vertexes.copyTo(buf);
   

  std::size_t new_size = device::remove_duplicates(cont);
  DeviceArray<int> new_cont(cont.ptr(), new_size);
  output.create(new_size);

  const device::Cloud& c = (const device::Cloud&)points;
  device::Cloud& o = (device::Cloud&)output;

  device::pack_hull(c, new_cont, o);
}
