/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/pcl_exports.h>

#include <pcl/cuda/point_cloud.h>
//#include <pcl/cuda/segmentation/connected_components.h>
#include <thrust/transform.h>

//struct ConnectedComponentSegmentation
//{
//  ConnectedComponentSegmentation () {}
//  
//  __device__ __host__ __inline__
//  void operator () {}
//};

namespace pcl
{
  namespace cuda
  {
    struct InlierLabeling
    {
      InlierLabeling (int** stencils, int nr_stencils) 
        : nr_stencils_(nr_stencils)
        , stencils_(stencils)
      {
      }

      __inline__ __host__ __device__
      int operator () (int idx)
      {
        int ret = -1;
        for (int i = 0; i < nr_stencils_; ++i)
        {
          if (stencils_[i][idx] != -1)
          {
            ret = (int) ((i / (float)nr_stencils_) * 255);
          }
        }
        return ret;
      }

      int nr_stencils_;
      int ** stencils_;
    };



    template <template <typename> class Storage>
    void markInliers (const typename PointCloudAOS<Storage>::ConstPtr &input, typename Storage<int>::type &region_mask, std::vector<boost::shared_ptr<typename Storage<int>::type> > inlier_stencils)
    {
      region_mask.resize (input->points.size());
      //int** stencils = new int*[inlier_stencils.size()];
      thrust::host_vector<int*> stencils_host (inlier_stencils.size ());
      for (int i = 0; i < inlier_stencils.size (); ++i)
        stencils_host[i] = thrust::raw_pointer_cast(&(*inlier_stencils[i])[0]);
        //stencils_host[i] = thrust::raw_pointer_cast<int> (&(*inlier_stencils[i])[0]);

      typename Storage<int*>::type stencils_storage = stencils_host;
      int** stencils = thrust::raw_pointer_cast(&stencils_storage[0]);

      thrust::counting_iterator<int> first (0);
      thrust::counting_iterator<int> last = first + region_mask.size ();
      thrust::transform (first, last, region_mask.begin (), InlierLabeling (stencils, (int) inlier_stencils.size ()));
    }

    //struct ConvertToImage1
    //{
    //  ConvertToImage1 () {}
    //  __host__ __device__ __inline__ unsigned char operator () (unsigned char i)
    //  {
    //    return i;
    //  }
    //};
    //
    //struct ConvertToImage3
    //{
    //  __host__ __device__ __inline__ OpenNIRGB operator () (unsigned char i)
    //  {
    //    OpenNIRGB r;
    //    r.r = i;
    //    return r;
    //  }
    //};
    //
    //struct ConvertToImage4
    //{
    //  ConvertToImage4 () {}
    //  __host__ __device__ __inline__ int operator () (unsigned char i)
    //  {
    //    return i;
    //  }
    //};

    struct ConvertToChar4
    {
      ConvertToChar4 () {}
      __host__ __device__ __inline__ char4 operator () (float4 n)
      {
        char4 r;
        r.x = (char) ((1.0f + n.x) * 128);
        r.y = (char) ((1.0f + n.y) * 128);
        r.z = (char) ((1.0f + n.z) * 128);
        r.w = (char) ((1.0f + n.w) * 128);
        return r;
      }
    };

    // createIndicesImage
    template <template <typename> class Storage, typename OutT, typename InT>
    void createIndicesImage (OutT &dst, InT &region_mask)
    {
      thrust::copy (region_mask.begin (), region_mask.end (), dst);
    }

    // createNormalsImage
    template <template <typename> class Storage, typename OutT, typename InT>
    void createNormalsImage (const OutT &dst, InT &normals)
    {
      thrust::transform (normals.begin (), normals.end (), dst, ConvertToChar4 ());
    }

    // findConnectedComponents
    template <template <typename> class Storage>
    void findConnectedComponents (const typename PointCloudAOS<Storage>::ConstPtr &input, typename Storage<int>::type &region_mask)
    {


    }

    // createRegionStencils
    template <template <typename> class Storage> std::vector<typename Storage<int>::type> 
    createRegionStencils (typename Storage<int>::type &parent, 
                          typename Storage<int>::type &rank, 
                          typename Storage<int>::type &size, 
                          int min_size, 
                          float percentage)
    {
      return (std::vector<typename Storage<int>::type> ());
    }


    template PCL_EXPORTS void markInliers<Device> (const typename PointCloudAOS<Device>::ConstPtr &input, Device<int>::type &region_mask, std::vector<boost::shared_ptr<Device<int>::type> > inlier_stencils);
    template PCL_EXPORTS void markInliers<Host>   (const typename PointCloudAOS<Host>  ::ConstPtr &input, Host<int>::type &region_mask, std::vector<boost::shared_ptr<Host<int>::type> > inlier_stencils);


    template PCL_EXPORTS void createIndicesImage<Device,
                                     StoragePointer<Device,unsigned char>::type,
                                     Device<int>::type>
      (StoragePointer<Device,unsigned char>::type &dst, typename Device<int>::type&region_mask);
    template PCL_EXPORTS void createIndicesImage<Host,
                                     typename StoragePointer<Host,unsigned char>::type,
                                     typename Host<int>::type>
      (typename StoragePointer<Host,unsigned char>::type &dst, typename Host<int>::type &region_mask);


    template PCL_EXPORTS void createNormalsImage<Device,
                                     StoragePointer<Device,char4>::type,
                                     Device<float4>::type>
      (const StoragePointer<Device,char4>::type &dst, typename Device<float4>::type&region_mask);
    template PCL_EXPORTS void createNormalsImage<Host,
                                     typename StoragePointer<Host,char4>::type,
                                     typename Host<float4>::type>
      (const typename StoragePointer<Host,char4>::type &dst, typename Host<float4>::type &region_mask);

  } // namespace
} // namespace

