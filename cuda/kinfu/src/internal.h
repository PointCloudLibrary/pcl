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

#ifndef PCL_KINFU_INTERNAL_HPP_
#define PCL_KINFU_INTERNAL_HPP_

#include "pcl/gpu/containers/device_array.hpp"
#include "pcl/gpu/utils/safe_call.hpp"

namespace pcl
{
  namespace device
  {
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Types
    typedef unsigned short ushort;
    typedef DeviceArray2D<float> MapArr;
    typedef DeviceArray2D<ushort> DepthMap;
    typedef float4 PointType;

    enum { VOLUME_X = 512, VOLUME_Y = 512, VOLUME_Z = 512 };

    /** \brief
      */ 
    struct Intr
    {
      float fx, fy, cx, cy;
      Intr () {}
      Intr (float fx_, float fy_, float cx_, float cy_) : fx(fx_), fy(fy_), cx(cx_), cy(cy_) {}

      Intr operator()(int level_index) const
      { 
        int div = 1 << level_index; 
        return (Intr (fx / div, fy / div, cx / div, cy / div));
      }
    };

    /** \brief 3x3 Matrix for device code
      */ 
    struct Mat33
    {
      float3 data[3];
    };

    /** \brief Light source collection
      */ 
    struct LightSource
    {
      float3 pos[1];
      int number;
    };

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Maps
  
    /** \brief Perfoms bilateral filtering of disparity map
      * \param[in] src soruce map
      * \param[out] dst output map
      */
    void 
    bilateralFilter (const DepthMap& src, DepthMap& dst);
    /** \brief Computes depth pyramid
      * \param[in] src source
      * \param[out] dst destination
      */
    void 
    pyrDown (const DepthMap& src, DepthMap& dst);

    /** \brief Computes vertex map
      * \param[in] intr depth camera intrinsics
      * \param[in] depth depth
      * \param[out] vmap vertex map
      */
    void 
    createVMap (const Intr& intr, const DepthMap& depth, MapArr& vmap);
    /** \brief Computes normal map using cross product
      * \param[in] vmap vertex map
      * \param[out] nmap normal map
      */
    void 
    createNMap (const MapArr& vmap, MapArr& nmap);
    /** \brief Computes normal map using Eigen/PCA approach
      * \param[in] vmap vertex map
      * \param[out] nmap normal map
      */
    void 
    computeNormalsEigen (const MapArr& vmap, MapArr& nmap);

    /** \brief Perform affine tranform of vertex and normal maps
      * \param[in] vmap_src
      * \param[in] nmap_src
      * \param[in] Rmat
      * \param[in] tvec
      * \param[out] vmap_dst
      * \param[out] nmap_dst
      */
    void 
    tranformMaps (const MapArr& vmap_src, const MapArr& nmap_src, const Mat33& Rmat, const float3& tvec, MapArr& vmap_dst, MapArr& nmap_dst);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //   ICP 
            
    /** \brief Computes corespondances map
      * \param[in] vmap_g_curr
      * \param[in] nmap_g_curr
      * \param[in] Rprev_inv
      * \param[in] tprev
      * \param[in] intr
      * \param[in] vmap_g_prev
      * \param[in] nmap_g_prev
      * \param[in] distThres
      * \param[in] angleThres
      * \param[out] coresp
      */
    void 
    findCoresp (const MapArr& vmap_g_curr, const MapArr& nmap_g_curr, const Mat33& Rprev_inv, const float3& tprev, const Intr& intr, 
                const MapArr& vmap_g_prev, const MapArr& nmap_g_prev, float distThres, float angleThres, PtrStepSz<short2> coresp);

    /** \brief
      * \param[in] v_dst
      * \param[in] n_dst
      * \param[in] v_src
      * \param[in] coresp
      * \param[out] gbuf
      * \param[out] mbuf
      * \param[out] matrixA_host
      * \param[out] vectorB_host
      */
    void 
    estimateTransform (const MapArr& v_dst, const MapArr& n_dst, const MapArr& v_src, const PtrStepSz<short2>& coresp,
                       DeviceArray2D<float>& gbuf, DeviceArray<float>& mbuf, float* matrixA_host, float* vectorB_host);


    /** \brief
      * \param[in] Rcurr
      * \param[in] tcurr
      * \param[in] vmap_curr
      * \param[in] nmap_curr
      * \param[in] Rprev_inv
      * \param[in] tprev
      * \param[in] intr
      * \param[in] vmap_g_prev
      * \param[in] nmap_g_prev
      * \param[in] distThres
      * \param[in] angleThres
      * \param[out] gbuf
      * \param[out] mbuf
      * \param[out] matrixA_host
      * \param[out] vectorB_host
      */
    void 
    estimateCombined (const Mat33& Rcurr, const float3& tcurr, const MapArr& vmap_curr, const MapArr& nmap_curr, const Mat33& Rprev_inv, const float3& tprev, const Intr& intr, 
                      const MapArr& vmap_g_prev, const MapArr& nmap_g_prev, float distThres, float angleThres, 
                      DeviceArray2D<float>& gbuf, DeviceArray<float>& mbuf, float* matrixA_host, float* vectorB_host);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TSDF volume functions        

    typedef short2 volume_elem_type;
    //typedef ushort2 volume_elem_type;
    template<typename T> void initVolume(PtrStepSz<T> array);

    //first version
    /** \brief
      * \param[in] depth_raw
      * \param[in] intr
      * \param[in] volume_size
      * \param[in] Rcurr_inv
      * \param[in] tcurr
      * \param[in] tranc_dist
      * \param[in] volume
      */
    void 
    integrateTsdfVolume (const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size, 
                         const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, PtrStep<short2> volume);

    //second version
    /** \brief
      * \param[in] depth_raw
      * \param[in] intr
      * \param[in] volume_size
      * \param[in] Rcurr_inv
      * \param[in] tcurr
      * \param[in] tranc_dist
      * \param[in] volume
      * \param[out] depthRawScaled
      */
    void 
    integrateTsdfVolume (const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size, 
                         const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, PtrStep<short2> volume, DeviceArray2D<float>& depthRawScaled);

    //third version (half)
    /** \brief
      * \param[in] depth_raw
      * \param[in] intr
      * \param[in] volume_size
      * \param[in] Rcurr_inv
      * \param[in] tcurr
      * \param[in] tranc_dist
      * \param[in] volume
      * \param[out] depthRawScaled
      */
    void 
    integrateTsdfVolume (const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size, 
                         const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, PtrStep<ushort2> volume, DeviceArray2D<float>& depthRawScaled);

    // Dispatcher
    /** \brief
      * \param[in] depth
      * \param[in] intr
      * \param[in] volume_size
      * \param[in] Rcurr_inv
      * \param[in] tcurr
      * \param[in] tranc_dist
      * \param[in] volume
      * \param[out] depthRawScaled
      */
    inline void 
    integrateVolume (const PtrStepSz<ushort>& depth, const Intr& intr, const float3& volume_size, const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, 
                     DeviceArray2D<int>& volume, DeviceArray2D<float>& depthRawScaled)
    {
      integrateTsdfVolume (depth, intr, volume_size, Rcurr_inv, tcurr, tranc_dist, (PtrStep<volume_elem_type>) volume, depthRawScaled);
    }
    

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Raycast and view generation        
    /** \brief
      * \param[in] intr
      * \param[in] Rcurr
      * \param[in] tcurr
      * \param[in] tranc_dist
      * \param[in] volume_size
      * \param[in] volume
      * \param[out] vmap
      * \param[out] nmap
      */
    void 
    raycast (const Intr& intr, const Mat33& Rcurr, const float3& tcurr, float tranc_dist, const float3& volume_size, 
             const PtrStep<volume_elem_type>& volume, MapArr& vmap, MapArr& nmap);

    /** \brief
      * \param[in] vmap
      * \param[in] nmap
      * \param[in] light
      * \param[out] dst
      */
    void 
    generateImage (const MapArr& vmap, const MapArr& nmap, const LightSource& light, PtrStepSz<uchar3> dst);

    /** \brief
      * \param[in] input
      * \param[out] output
      */
    void 
    resizeVMap (const MapArr& input, MapArr& output);
    
    /** \brief
      * \param[in] input
      * \param[out] output
      */
    void 
    resizeNMap (const MapArr& input, MapArr& output);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Cloud extraction 

    /** \brief
      * \param[in] volume
      * \param[in] volume_size
      * \param[out] output
      */ 
    size_t 
    extractCloud (const PtrStep<volume_elem_type>& volume, const float3& volume_size, PtrSz<PointType> output);

    /** \brief
      * \param[in] volume
      * \param[in] volume_size
      * \param[in] input
      * \param[out] output
      */ 
    template<typename NormalType> void 
    extractNormals (const PtrStep<volume_elem_type>& volume, const float3& volume_size, const PtrSz<PointType>& input, NormalType* output);
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Utility
    struct float8 { float x, y, z, w, f1, f2, f3, f4; };

    /** \brief
      * \param[in] vmap
      * \param[out] output
      */
    template<typename T> void 
    convert (const MapArr& vmap, DeviceArray2D<T>& output);

    /** \brief 
      * \param[in] value
      */
    inline bool 
    valid_host (float value)
    {
      return *reinterpret_cast<int*>(&value) != 0x7fffffff; //QNAN
    }

    /** \brief */
    inline void 
    sync () { cudaSafeCall (cudaDeviceSynchronize ()); }
  }
}

#endif /* PCL_KINFU_INTERNAL_HPP_ */
