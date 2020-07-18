/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (C) 2009-2010, NVIDIA Corporation, all rights reserved.
 *  Third party copyrights are property of their respective owners.
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
 * $Id:  $
 * Ported to PCL by Koen Buys : Attention Work in progress!
 */

////////////////////////////////////////////////////////////////////////////////
//
// NVIDIA CUDA implementation of Viola-Jones Object Detection Framework
//
// The algorithm and code are explained in the upcoming GPU Computing Gems
// chapter in detail:
//
//   Anton Obukhov, "Haar Classifiers for Object Detection with CUDA"
//   PDF URL placeholder
//   email: aobukhov@nvidia.com, devsupport@nvidia.com
//
// Credits for help with the code to:
// Alexey Mendelenko, Cyril Crassin, and Mikhail Smirnov.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef PCL_GPU_PEOPLE_NCVHAAROBJECTDETECTION_HPP_
#define PCL_GPU_PEOPLE_NCVHAAROBJECTDETECTION_HPP_

#include <string>
#include "NCV.hpp"

//==============================================================================
//
// Guaranteed size cross-platform classifier structures
//
//==============================================================================

struct HaarFeature64
{
    uint2 _ui2;

#define HaarFeature64_CreateCheck_MaxRectField                  0xFF

    __host__ NCVStatus setRect(Ncv32u rectX, Ncv32u rectY, Ncv32u rectWidth, Ncv32u rectHeight, Ncv32u /*clsWidth*/, Ncv32u /*clsHeight*/)
    {
      ncvAssertReturn(rectWidth <= HaarFeature64_CreateCheck_MaxRectField && rectHeight <= HaarFeature64_CreateCheck_MaxRectField, NCV_HAAR_TOO_LARGE_FEATURES);
      ((NcvRect8u*)&(this->_ui2.x))->x = (Ncv8u)rectX;
      ((NcvRect8u*)&(this->_ui2.x))->y = (Ncv8u)rectY;
      ((NcvRect8u*)&(this->_ui2.x))->width = (Ncv8u)rectWidth;
      ((NcvRect8u*)&(this->_ui2.x))->height = (Ncv8u)rectHeight;
      return NCV_SUCCESS;
    }

    __host__ NCVStatus setWeight(Ncv32f weight)
    {
      ((Ncv32f*)&(this->_ui2.y))[0] = weight;
      return NCV_SUCCESS;
    }

    __device__ __host__ void getRect(Ncv32u *rectX, Ncv32u *rectY, Ncv32u *rectWidth, Ncv32u *rectHeight)
    {
      NcvRect8u tmpRect = *(NcvRect8u*)(&this->_ui2.x);
      *rectX = tmpRect.x;
      *rectY = tmpRect.y;
      *rectWidth = tmpRect.width;
      *rectHeight = tmpRect.height;
    }

    __device__ __host__ Ncv32f getWeight()
    {
      return *(Ncv32f*)(&this->_ui2.y);
    }
};

struct HaarFeatureDescriptor32
{
  private:

#define HaarFeatureDescriptor32_Interpret_MaskFlagTilted        0x80000000
#define HaarFeatureDescriptor32_Interpret_MaskFlagLeftNodeLeaf  0x40000000
#define HaarFeatureDescriptor32_Interpret_MaskFlagRightNodeLeaf 0x20000000
#define HaarFeatureDescriptor32_CreateCheck_MaxNumFeatures      0x1F
#define HaarFeatureDescriptor32_NumFeatures_Shift               24
#define HaarFeatureDescriptor32_CreateCheck_MaxFeatureOffset    0x00FFFFFF

    Ncv32u desc;

  public:

    __host__ NCVStatus create(NcvBool bTilted, NcvBool bLeftLeaf, NcvBool bRightLeaf,
                              Ncv32u numFeatures, Ncv32u offsetFeatures)
    {
      if (numFeatures > HaarFeatureDescriptor32_CreateCheck_MaxNumFeatures)
      {
        return NCV_HAAR_TOO_MANY_FEATURES_IN_CLASSIFIER;
      }
      if (offsetFeatures > HaarFeatureDescriptor32_CreateCheck_MaxFeatureOffset)
      {
        return NCV_HAAR_TOO_MANY_FEATURES_IN_CASCADE;
      }
      this->desc = 0;
      this->desc |= (bTilted ? HaarFeatureDescriptor32_Interpret_MaskFlagTilted : 0);
      this->desc |= (bLeftLeaf ? HaarFeatureDescriptor32_Interpret_MaskFlagLeftNodeLeaf : 0);
      this->desc |= (bRightLeaf ? HaarFeatureDescriptor32_Interpret_MaskFlagRightNodeLeaf : 0);
      this->desc |= (numFeatures << HaarFeatureDescriptor32_NumFeatures_Shift);
      this->desc |= offsetFeatures;
      return NCV_SUCCESS;
    }

    __device__ __host__ NcvBool isTilted() const
    {
      return (this->desc & HaarFeatureDescriptor32_Interpret_MaskFlagTilted) != 0;
    }

    __device__ __host__ NcvBool isLeftNodeLeaf() const
    {
      return (this->desc & HaarFeatureDescriptor32_Interpret_MaskFlagLeftNodeLeaf) != 0;
    }

    __device__ __host__ NcvBool isRightNodeLeaf() const
    {
      return (this->desc & HaarFeatureDescriptor32_Interpret_MaskFlagRightNodeLeaf) != 0;
    }

    __device__ __host__ Ncv32u getNumFeatures() const
    {
      return (this->desc >> HaarFeatureDescriptor32_NumFeatures_Shift) & HaarFeatureDescriptor32_CreateCheck_MaxNumFeatures;
    }

    __device__ __host__ Ncv32u getFeaturesOffset() const
    {
      return this->desc & HaarFeatureDescriptor32_CreateCheck_MaxFeatureOffset;
    }
};

struct HaarClassifierNodeDescriptor32
{
    uint1 _ui1;

    __host__ NCVStatus create(Ncv32f leafValue)
    {
      *(Ncv32f *)&this->_ui1 = leafValue;
      return (NCV_SUCCESS);
    }

    __host__ NCVStatus create(Ncv32u offsetHaarClassifierNode)
    {
      this->_ui1.x = offsetHaarClassifierNode;
      return (NCV_SUCCESS);
    }

    __host__ Ncv32f getLeafValueHost()
    {
      return (*(Ncv32f *)&this->_ui1.x);
    }

    __host__ bool isLeaf() const                                  // TODO: check this hack don't know if is correct
    {
      return ( _ui1.x != 0);
    }

#ifdef __CUDACC__
    __device__ Ncv32f getLeafValue(void)
    {
      return (__int_as_float(this->_ui1.x));
    }
#endif

    __device__ __host__ Ncv32u getNextNodeOffset()
    {
      return (this->_ui1.x);
    }
};

struct HaarClassifierNode128
{
    uint4 _ui4;

    __host__ NCVStatus setFeatureDesc(HaarFeatureDescriptor32 f)
    {
      this->_ui4.x = *(Ncv32u *)&f;
      return NCV_SUCCESS;
    }

    __host__ NCVStatus setThreshold(Ncv32f t)
    {
      this->_ui4.y = *(Ncv32u *)&t;
      return NCV_SUCCESS;
    }

    __host__ NCVStatus setLeftNodeDesc(HaarClassifierNodeDescriptor32 nl)
    {
      this->_ui4.z = *(Ncv32u *)&nl;
      return NCV_SUCCESS;
    }

    __host__ NCVStatus setRightNodeDesc(HaarClassifierNodeDescriptor32 nr)
    {
      this->_ui4.w = *(Ncv32u *)&nr;
      return NCV_SUCCESS;
    }

    __host__ __device__ HaarFeatureDescriptor32 getFeatureDesc()
    {
      return *(HaarFeatureDescriptor32 *)&this->_ui4.x;
    }

    __host__ __device__ Ncv32f getThreshold()
    {
      return *(Ncv32f*)&this->_ui4.y;
    }

    __host__ __device__ HaarClassifierNodeDescriptor32 getLeftNodeDesc()
    {
      return *(HaarClassifierNodeDescriptor32 *)&this->_ui4.z;
    }

    __host__ __device__ HaarClassifierNodeDescriptor32 getRightNodeDesc()
    {
      return *(HaarClassifierNodeDescriptor32 *)&this->_ui4.w;
    }
};

struct HaarStage64
{
#define HaarStage64_Interpret_MaskRootNodes         0x0000FFFF
#define HaarStage64_Interpret_MaskRootNodeOffset    0xFFFF0000
#define HaarStage64_Interpret_ShiftRootNodeOffset   16

    uint2 _ui2;

    __host__ NCVStatus setStageThreshold(Ncv32f t)
    {
      this->_ui2.x = *(Ncv32u *)&t;
      return NCV_SUCCESS;
    }

    __host__ NCVStatus setStartClassifierRootNodeOffset(Ncv32u val)
    {
      if (val > (HaarStage64_Interpret_MaskRootNodeOffset >> HaarStage64_Interpret_ShiftRootNodeOffset))
      {
        return NCV_HAAR_XML_LOADING_EXCEPTION;
      }
      this->_ui2.y = (val << HaarStage64_Interpret_ShiftRootNodeOffset) | (this->_ui2.y & HaarStage64_Interpret_MaskRootNodes);
      return NCV_SUCCESS;
    }

    __host__ NCVStatus setNumClassifierRootNodes(Ncv32u val)
    {
      if (val > HaarStage64_Interpret_MaskRootNodes)
      {
        return NCV_HAAR_XML_LOADING_EXCEPTION;
      }
      this->_ui2.y = val | (this->_ui2.y & HaarStage64_Interpret_MaskRootNodeOffset);
      return NCV_SUCCESS;
    }

    __host__ __device__ Ncv32f getStageThreshold()
    {
      return *(Ncv32f*)&this->_ui2.x;
    }

    __host__ __device__ Ncv32u getStartClassifierRootNodeOffset() const
    {
      return (this->_ui2.y >> HaarStage64_Interpret_ShiftRootNodeOffset);
    }

    __host__ __device__ Ncv32u getNumClassifierRootNodes() const
    {
      return (this->_ui2.y & HaarStage64_Interpret_MaskRootNodes);
    }
};

NCV_CT_ASSERT(sizeof(HaarFeature64) == 8);
NCV_CT_ASSERT(sizeof(HaarFeatureDescriptor32) == 4);
NCV_CT_ASSERT(sizeof(HaarClassifierNodeDescriptor32) == 4);
NCV_CT_ASSERT(sizeof(HaarClassifierNode128) == 16);
NCV_CT_ASSERT(sizeof(HaarStage64) == 8);

/**
 * \brief Classifier cascade descriptor
 */
struct HaarClassifierCascadeDescriptor
{
    Ncv32u NumStages;
    Ncv32u NumClassifierRootNodes;
    Ncv32u NumClassifierTotalNodes;
    Ncv32u NumFeatures;
    NcvSize32u ClassifierSize;
    NcvBool bNeedsTiltedII;
    NcvBool bHasStumpsOnly;
};

//==============================================================================
//
// Functional interface
//
//==============================================================================

enum
{
  NCVPipeObjDet_Default               = 0x000,
  NCVPipeObjDet_UseFairImageScaling   = 0x001,
  NCVPipeObjDet_FindLargestObject     = 0x002,
  NCVPipeObjDet_VisualizeInPlace      = 0x004,
};

NCV_EXPORTS NCVStatus ncvDetectObjectsMultiScale_device(NCVMatrix<Ncv8u> &d_srcImg,
                                                        NcvSize32u srcRoi,
                                                        NCVVector<NcvRect32u> &d_dstRects,
                                                        Ncv32u &dstNumRects,

                                                        HaarClassifierCascadeDescriptor &haar,
                                                        NCVVector<HaarStage64> &h_HaarStages,
                                                        NCVVector<HaarStage64> &d_HaarStages,
                                                        NCVVector<HaarClassifierNode128> &d_HaarNodes,
                                                        NCVVector<HaarFeature64> &d_HaarFeatures,

                                                        NcvSize32u minObjSize,
                                                        Ncv32u minNeighbors,      //default 4
                                                        Ncv32f scaleStep,         //default 1.2f
                                                        Ncv32u pixelStep,         //default 1
                                                        Ncv32u flags,             //default NCVPipeObjDet_Default

                                                        INCVMemAllocator &gpuAllocator,
                                                        INCVMemAllocator &cpuAllocator,
                                                        cudaDeviceProp &devProp,
                                                        cudaStream_t cuStream);

#define OBJDET_MASK_ELEMENT_INVALID_32U     0xFFFFFFFF
#define HAAR_STDDEV_BORDER                  1

NCV_EXPORTS NCVStatus ncvApplyHaarClassifierCascade_device(NCVMatrix<Ncv32u> &d_integralImage,
                                                           NCVMatrix<Ncv32f> &d_weights,
                                                           NCVMatrixAlloc<Ncv32u> &d_pixelMask,
                                                           Ncv32u &numDetections,
                                                           HaarClassifierCascadeDescriptor &haar,
                                                           NCVVector<HaarStage64> &h_HaarStages,
                                                           NCVVector<HaarStage64> &d_HaarStages,
                                                           NCVVector<HaarClassifierNode128> &d_HaarNodes,
                                                           NCVVector<HaarFeature64> &d_HaarFeatures,
                                                           NcvBool bMaskElements,
                                                           NcvSize32u anchorsRoi,
                                                           Ncv32u pixelStep,
                                                           Ncv32f scaleArea,
                                                           INCVMemAllocator &gpuAllocator,
                                                           INCVMemAllocator &cpuAllocator,
                                                           cudaDeviceProp &devProp,
                                                           cudaStream_t cuStream);

NCV_EXPORTS NCVStatus ncvApplyHaarClassifierCascade_host(NCVMatrix<Ncv32u> &h_integralImage,
                                                         NCVMatrix<Ncv32f> &h_weights,
                                                         NCVMatrixAlloc<Ncv32u> &h_pixelMask,
                                                         Ncv32u &numDetections,
                                                         HaarClassifierCascadeDescriptor &haar,
                                                         NCVVector<HaarStage64> &h_HaarStages,
                                                         NCVVector<HaarClassifierNode128> &h_HaarNodes,
                                                         NCVVector<HaarFeature64> &h_HaarFeatures,
                                                         NcvBool bMaskElements,
                                                         NcvSize32u anchorsRoi,
                                                         Ncv32u pixelStep,
                                                         Ncv32f scaleArea);

#define RECT_SIMILARITY_PROPORTION      0.2f

NCV_EXPORTS NCVStatus ncvGrowDetectionsVector_device(NCVVector<Ncv32u> &pixelMask,
                                                     Ncv32u numPixelMaskDetections,
                                                     NCVVector<NcvRect32u> &hypotheses,
                                                     Ncv32u &totalDetections,
                                                     Ncv32u totalMaxDetections,
                                                     Ncv32u rectWidth,
                                                     Ncv32u rectHeight,
                                                     Ncv32f curScale,
                                                     cudaStream_t cuStream);

NCV_EXPORTS NCVStatus ncvGrowDetectionsVector_host(NCVVector<Ncv32u> &pixelMask,
                                                   Ncv32u numPixelMaskDetections,
                                                   NCVVector<NcvRect32u> &hypotheses,
                                                   Ncv32u &totalDetections,
                                                   Ncv32u totalMaxDetections,
                                                   Ncv32u rectWidth,
                                                   Ncv32u rectHeight,
                                                   Ncv32f curScale);

NCV_EXPORTS NCVStatus ncvHaarGetClassifierSize(const std::string &filename, Ncv32u &numStages,
                                               Ncv32u &numNodes, Ncv32u &numFeatures);

NCV_EXPORTS NCVStatus ncvHaarLoadFromFile_host(const std::string &filename,
                                               HaarClassifierCascadeDescriptor &haar,
                                               NCVVector<HaarStage64> &h_HaarStages,
                                               NCVVector<HaarClassifierNode128> &h_HaarNodes,
                                               NCVVector<HaarFeature64> &h_HaarFeatures);

NCV_EXPORTS NCVStatus ncvHaarStoreNVBIN_host(const std::string &filename,
                                             HaarClassifierCascadeDescriptor haar,
                                             NCVVector<HaarStage64> &h_HaarStages,
                                             NCVVector<HaarClassifierNode128> &h_HaarNodes,
                                             NCVVector<HaarFeature64> &h_HaarFeatures);

#endif // PCL_GPU_PEOPLE_NCVHAAROBJECTDETECTION_HPP_
