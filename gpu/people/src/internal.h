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
 * @authors: Cedric Cagniart, Koen Buys, Anatoly Baksheev
 */

#pragma once

#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/gpu/people/label_common.h>
#include <pcl/gpu/people/tree.h>

using pcl::gpu::people::NUM_PARTS;

namespace pcl
{
  namespace device
  {
    using Cloud = DeviceArray2D<float4>;
    using Image = DeviceArray2D<uchar4>;

    using Depth = DeviceArray2D<unsigned short>;
    using Labels = DeviceArray2D<unsigned char>;      
    using HueImage = DeviceArray2D<float>;
    using Mask = DeviceArray2D<unsigned char>;  

    using MultiLabels = DeviceArray2D<char4>;

    /** \brief The intrinsic camera calibration **/
    struct Intr
    {
        float fx, fy, cx, cy;
        Intr () {}
        Intr (float fx_, float fy_, float cx_, float cy_) : fx(fx_), fy(fy_), cx(cx_), cy(cy_) {}

        void setDefaultPPIfIncorrect(int cols, int rows)
        {
          cx = cx > 0 ? cx : cols/2 - 0.5f;
          cy = cy > 0 ? cy : rows/2 - 0.5f;
        }
    };

    void smoothLabelImage(const Labels& src, const Depth& depth, Labels& dst, int num_parts, int  patch_size, int depthThres);
    void colorLMap(const Labels& labels, const DeviceArray<uchar4>& cmap, Image& rgb);
    void mixedColorMap(const Labels& labels, const DeviceArray<uchar4>& map, const Image& rgba, Image& output);

    ////////////// connected components ///////////////////        

    struct ConnectedComponents
    {
        static void initEdges(int rows, int cols, DeviceArray2D<unsigned char>& edges);
        //static void computeEdges(const Labels& labels, const Cloud& cloud, int num_parts, float sq_radius, DeviceArray2D<unsigned char>& edges);
        static void computeEdges(const Labels& labels, const Depth& depth, int num_parts, float sq_radius, DeviceArray2D<unsigned char>& edges);
        static void labelComponents(const DeviceArray2D<unsigned char>& edges, DeviceArray2D<int>& comps);
    };

    void computeCloud(const Depth& depth, const Intr& intr, Cloud& cloud);

    void setZero(Mask& mask);
    void prepareForeGroundDepth(const Depth& depth1, Mask& inverse_mask, Depth& depth2);

    float computeHue(int rgba);
    void  computeHueWithNans(const Image& image, const Depth& depth, HueImage& hue);

    //void shs(const DeviceArray2D<float4> &cloud, float tolerance, const std::vector<int>& indices_in, float delta_hue, Mask& indices_out);

    struct Dilatation
    {
        using Kernel = DeviceArray<unsigned char>;
        enum 
        { 
          KSIZE_X = 5,
          KSIZE_Y = 5,
          ANCH_X = KSIZE_X/2,
          ANCH_Y = KSIZE_Y/2,
        };

        static void prepareRect5x5Kernel(Kernel& kernel);
        static void invoke(const Mask& src, const Kernel& kernel, Mask& dst);
    };

    /** \brief Struct that holds a single RDF tree in GPU **/
    struct CUDATree
    {
        using Node = pcl::gpu::people::trees::Node;
        using Label = pcl::gpu::people::trees::Label;

        int treeHeight;
        int numNodes;

        DeviceArray<Node> nodes_device;
        DeviceArray<Label> leaves_device;

        CUDATree (int treeHeight_, const std::vector<Node>& nodes, const std::vector<Label>& leaves);
    };

    /** \brief Processor using multiple trees */
    class MultiTreeLiveProc
    {
      public:
        /** \brief Constructor with default values, allocates multilmap device memory **/
        MultiTreeLiveProc(int def_rows = 480, int def_cols = 640) : multilmap (def_rows, def_cols) {}
        /** \brief Empty destructor **/
        ~MultiTreeLiveProc() {}

        void
        process (const Depth& dmap, Labels& lmap);

        /** \brief same as process, but runs the trick of declaring as background any neighbor that is more than FGThresh away.**/
        void
        process (const Depth& dmap, Labels& lmap, int FGThresh);

        /** \brief output a probability map from the RDF.**/
        void
        processProb (const Depth& dmap, Labels& lmap, LabelProbability& prob, int FGThresh);

        std::vector<CUDATree> trees;
        MultiLabels multilmap;
    };

    /** \brief Implementation Class to process probability histograms on GPU **/
    class ProbabilityProc
    {
      public:
        /** \brief Default constructor **/
        ProbabilityProc()
        {
          std::cout << "[pcl::device::ProbabilityProc:ProbabilityProc] : (D) : Constructor called" << std::endl;
          //PCL_DEBUG("[pcl::device::ProbabilityProc:ProbabilityProc] : (D) : Constructor called");
        }

        /** \brief Default destructor **/
        ~ProbabilityProc() {}

        /** \brief This will merge the votes from the different trees into one final vote, including probabilistic's **/
        void
        CUDA_SelectLabel ( const Depth& depth, Labels& labels, LabelProbability& probabilities);

        /** \brief This will combine two probabilities according their weight **/
        void
        CUDA_CombineProb ( const Depth& depth, LabelProbability& probIn1, float weight1,
                           LabelProbability& probIn2, float weight2, LabelProbability& probOut);

        /** \brief This will sum a probability multiplied with it's weight **/
        void
        CUDA_WeightedSumProb ( const Depth& depth, LabelProbability& probIn, float weight, LabelProbability& probOut);

        /** \brief This will blur the input labelprobability with the given kernel **/
        int
        CUDA_GaussianBlur( const Depth& depth,
                           LabelProbability& probIn,
                           DeviceArray<float>& kernel,
                           LabelProbability& probOut);
        /** \brief This will blur the input labelprobability with the given kernel, this version avoids extended allocation **/
        int
        CUDA_GaussianBlur( const Depth& depth,
                           LabelProbability& probIn,
                           DeviceArray<float>& kernel,
                           LabelProbability& probTemp,
                           LabelProbability& probOut);
    };
  }
}
