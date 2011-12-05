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

    //Tsdf fixed point divisor (if old format is enabled)
    const int DIVISOR = 32767;     // SHRT_MAX;

    enum { VOLUME_X = 512, VOLUME_Y = 512, VOLUME_Z = 512 };

    /** \brief Camera intrinsics structure
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

    /** \brief Performs affine tranform of vertex and normal maps
      * \param[in] vmap_src source vertex map
      * \param[in] nmap_src source vertex map
      * \param[in] Rmat Rotation mat
      * \param[in] tvec translation
      * \param[out] vmap_dst destination vertex map
      * \param[out] nmap_dst destination vertex map
      */
    void 
    tranformMaps (const MapArr& vmap_src, const MapArr& nmap_src, const Mat33& Rmat, const float3& tvec, MapArr& vmap_dst, MapArr& nmap_dst);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //   ICP 
            
    /** \brief (now it's exra code) Computes corespondances map
      * \param[in] vmap_g_curr current vertex map in global coo space
      * \param[in] nmap_g_curr current normals map in global coo space
      * \param[in] Rprev_inv inverse camera rotation at previous pose
      * \param[in] tprev camera translation at previous pose
      * \param[in] intr camera intrinsics
      * \param[in] vmap_g_prev previous vertex map in global coo space
      * \param[in] nmap_g_prev previous vertex map in global coo space
      * \param[in] distThres distance filtering threshold
      * \param[in] angleThres angle filtering threshold. Represents sine of angle between normals
      * \param[out] coresp
      */
    void 
    findCoresp (const MapArr& vmap_g_curr, const MapArr& nmap_g_curr, const Mat33& Rprev_inv, const float3& tprev, const Intr& intr, 
                const MapArr& vmap_g_prev, const MapArr& nmap_g_prev, float distThres, float angleThres, PtrStepSz<short2> coresp);

    /** \brief (now it's exra code) Computation Ax=b for ICP iteration
      * \param[in] v_dst destination vertex map (previous frame cloud)
      * \param[in] n_dst destination normal map (previous frame normals) 
      * \param[in] v_src source normal map (current frame cloud) 
      * \param[in] coresp Corespondances
      * \param[out] gbuf temp buffer for GPU reduction
      * \param[out] mbuf ouput GPU buffer for matrix computed
      * \param[out] matrixA_host A
      * \param[out] vectorB_host b
      */
    void 
    estimateTransform (const MapArr& v_dst, const MapArr& n_dst, const MapArr& v_src, const PtrStepSz<short2>& coresp,
                       DeviceArray2D<float>& gbuf, DeviceArray<float>& mbuf, float* matrixA_host, float* vectorB_host);


    /** \brief Computation Ax=b for ICP iteration
      * \param[in] Rcurr Rotation of current camera pose guess 
      * \param[in] tcurr translation of current camera pose guess 
      * \param[in] vmap_curr current vertex map in camera coo space
      * \param[in] nmap_curr current vertex map in camera coo space
      * \param[in] Rprev_inv inverse camera rotation at previous pose
      * \param[in] tprev camera translation at previous pose
      * \param[in] intr camera intrinsics
      * \param[in] vmap_g_prev previous vertex map in global coo space
      * \param[in] nmap_g_prev previous vertex map in global coo space
      * \param[in] distThres distance filtering threshold
      * \param[in] angleThres angle filtering threshold. Represents sine of angle between normals
      * \param[out] gbuf temp buffer for GPU reduction
      * \param[out] mbuf ouput GPU buffer for matrix computed
      * \param[out] matrixA_host A
      * \param[out] vectorB_host b
      */
    void 
    estimateCombined (const Mat33& Rcurr, const float3& tcurr, const MapArr& vmap_curr, const MapArr& nmap_curr, const Mat33& Rprev_inv, const float3& tprev, const Intr& intr, 
                      const MapArr& vmap_g_prev, const MapArr& nmap_g_prev, float distThres, float angleThres, 
                      DeviceArray2D<float>& gbuf, DeviceArray<float>& mbuf, float* matrixA_host, float* vectorB_host);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TSDF volume functions        

    //switch between two tsdf volume formats
    typedef short2 volume_elem_type;
    //typedef ushort2 volume_elem_type;

    /** \brief Perform tsdf volume initialization
      *  \param[out]
      */
    template<typename T> 
    void 
    initVolume(PtrStepSz<T> array);

    //first version
    /** \brief Performs Tsfg volume uptation (extra obsolete now)
      * \param[in] depth_raw Kinect depth image
      * \param[in] intr camera intrinsics
      * \param[in] volume_size size of volume in mm
      * \param[in] Rcurr_inv inverse rotation for current camera pose
      * \param[in] tcurr translation for current camera pose
      * \param[in] tranc_dist tsdf trancation distance
      * \param[in] volume tsdf volume to be updated
      */
    void 
    integrateTsdfVolume (const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size, 
                         const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, PtrStep<short2> volume);

    //second version
    /** \brief Function that integrates volume if volume element contains: 2 bytes for round(tsdf*SHORT_MAX) and 2 bytes for integer weight.
      * \param[in] depth_raw Kinect depth image
      * \param[in] intr camera intrinsics
      * \param[in] volume_size size of volume in mm
      * \param[in] Rcurr_inv inverse rotation for current camera pose
      * \param[in] tcurr translation for current camera pose
      * \param[in] tranc_dist tsdf trancation distance
      * \param[in] volume tsdf volume to be updated
      * \param[out] depthRawScaled Buffer for scaled depth along ray
      */
    void 
    integrateTsdfVolume (const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size, 
                         const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, PtrStep<short2> volume, DeviceArray2D<float>& depthRawScaled);

    //third version (half)
    /** \brief Function that integrates volume if volume element contains: 2 bytes for half-float(tsdf) and 2 bytes for integer weight.
      * \param[in] depth_raw Kinect depth image
      * \param[in] intr camera intrinsics
      * \param[in] volume_size size of volume in mm
      * \param[in] Rcurr_inv inverse rotation for current camera pose
      * \param[in] tcurr translation for current camera pose
      * \param[in] tranc_dist tsdf trancation distance
      * \param[in] volume tsdf volume to be updated
      * \param[out] depthRawScaled buffer for scaled depth along ray
      */
    void 
    integrateTsdfVolume (const PtrStepSz<ushort>& depth_raw, const Intr& intr, const float3& volume_size, 
                         const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, PtrStep<ushort2> volume, DeviceArray2D<float>& depthRawScaled);

    // Dispatcher
    /** \brief Dispatched function for fast swithing between two tsdf volume element formats
      * \param[in] depth Kinect depth image
      * \param[in] intr camera intrinsics
      * \param[in] volume_size size of volume in mm
      * \param[in] Rcurr_inv inverse rotation for current camera pose
      * \param[in] tcurr translation for current camera pose
      * \param[in] tranc_dist tsdf trancation distance
      * \param[in] volume  tsdf volume to be updated
      * \param[out] depthRawScaled buffer for scaled depth along ray
      */
    inline 
    void 
    integrateVolume (const PtrStepSz<ushort>& depth, const Intr& intr, const float3& volume_size, const Mat33& Rcurr_inv, const float3& tcurr, float tranc_dist, 
                     DeviceArray2D<int>& volume, DeviceArray2D<float>& depthRawScaled)
    {
      integrateTsdfVolume (depth, intr, volume_size, Rcurr_inv, tcurr, tranc_dist, (PtrStep<volume_elem_type>) volume, depthRawScaled);
    }
    

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Raycast and view generation        
    /** \brief Generation vertex and normal maps from volume for current camera pose
      * \param[in] intr camera intrinsices
      * \param[in] Rcurr current rotation
      * \param[in] tcurr current translation
      * \param[in] tranc_dist volume trancation distance
      * \param[in] volume_size volume size in mm
      * \param[in] volume tsdf volume
      * \param[out] vmap output vertex map
      * \param[out] nmap output normals map
      */
    void 
    raycast (const Intr& intr, const Mat33& Rcurr, const float3& tcurr, float tranc_dist, const float3& volume_size, 
             const PtrStep<volume_elem_type>& volume, MapArr& vmap, MapArr& nmap);

    /** \brief Renders 3D image of the scene
      * \param[in] vmap vetex map
      * \param[in] nmap normals map
      * \param[in] light poase of light source
      * \param[out] dst buffer where image is generated
      */
    void 
    generateImage (const MapArr& vmap, const MapArr& nmap, const LightSource& light, PtrStepSz<uchar3> dst);

    /** \brief Performs resize of vertex map to next pyramid level by averaging each four points
      * \param[in] input vertext map
      * \param[out] output resized vertex map
      */
    void 
    resizeVMap (const MapArr& input, MapArr& output);
    
    /** \brief Performs resize of vertex map to next pyramid level by averaging each four normals
      * \param[in] input normal map
      * \param[out] output vertex map
      */
    void 
    resizeNMap (const MapArr& input, MapArr& output);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Cloud extraction 

    /** \brief Perofrm point cloud extraction from tsdf volumer
      * \param[in] volume tsdf volume 
      * \param[in] volume_size size of the volume
      * \param[out] output buffer large enought to store point cloud
      * \return number of point stored to passed buffer
      */ 
    size_t 
    extractCloud (const PtrStep<volume_elem_type>& volume, const float3& volume_size, PtrSz<PointType> output);

    /** \brief Performs normals computation for given poins using tsdf volume
      * \param[in] volume tsdf volume
      * \param[in] volume_size volume size
      * \param[in] input points where normals are computed
      * \param[out] output normals. Could be float4 or float8. If for a point normal can't be computed, such normal is marked as nan.
      */ 
    template<typename NormalType> 
    void 
    extractNormals (const PtrStep<volume_elem_type>& volume, const float3& volume_size, const PtrSz<PointType>& input, NormalType* output);
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Utility
    struct float8 { float x, y, z, w, f1, f2, f3, f4; };

    /** \brief Conversion from SOA to AOS
      * \param[in] vmap SOA map
      * \param[out] output Array of 3D points. Can be float4 or float8.
      */
    template<typename T> 
    void 
    convert (const MapArr& vmap, DeviceArray2D<T>& output);

    /** \brief  Check for qnan (unused now) 
      * \param[in] value
      */
    inline bool 
    valid_host (float value)
    {
      return *reinterpret_cast<int*>(&value) != 0x7fffffff; //QNAN
    }

    /** \brief synchronizes CUDA execution */
    inline 
    void 
    sync () { cudaSafeCall (cudaDeviceSynchronize ()); }
  }
}

#endif /* PCL_KINFU_INTERNAL_HPP_ */
