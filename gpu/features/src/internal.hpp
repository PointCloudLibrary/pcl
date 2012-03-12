/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#ifndef PCL_GPU_FEATURES_INTERNAL_HPP_
#define PCL_GPU_FEATURES_INTERNAL_HPP_

#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/octree/device_format.hpp>

#include <cuda_runtime.h>

#undef PI
#ifndef PI
    #define PI 3.1415926535897931f               
#endif

namespace pcl
{
    namespace device
    {   
        using pcl::gpu::DeviceArray;
        using pcl::gpu::DeviceArray2D;
        using pcl::gpu::NeighborIndices;

        typedef float4 PointType;
        typedef float4 NormalType;
        typedef float4 PointXYZRGB;

        typedef DeviceArray< PointType> PointCloud;        
        typedef DeviceArray<NormalType> Normals;
        typedef DeviceArray<int> Indices;

        typedef DeviceArray< PointType> PointXYZRGBCloud;

		template <int N> struct Histogram
		{
			float histogram[N];
		};

		typedef Histogram<125> PFHSignature125;
		typedef Histogram<250> PFHRGBSignature250;
		typedef Histogram<33>  FPFHSignature33;
		typedef Histogram<308> VFHSignature308;

        struct PPFSignature
        {
            float f1, f2, f3, f4;
            float alpha_m;
        };

        struct PPFRGBSignature
        {
            float f1, f2, f3, f4;
            float r_ratio, g_ratio, b_ratio;
            float alpha_m;
        };
	
        struct PrincipalCurvatures
        {
            union
            {
                float principal_curvature[3];
                struct
                {
                    float principal_curvature_x;
                    float principal_curvature_y;
                    float principal_curvature_z;
                };
            };
            float pc1;
            float pc2;
        };

        // normals estimation
        void computeNormals(const PointCloud& cloud, const NeighborIndices& nn_indices, Normals& normals);
        void flipNormalTowardsViewpoint(const PointCloud& cloud, const float3& vp, Normals& normals);        
        void flipNormalTowardsViewpoint(const PointCloud& cloud, const Indices& indices, const float3& vp, Normals& normals);

        // pfh estimation        
        void repackToAosForPfh(const PointCloud& cloud, const Normals& normals, const NeighborIndices& neighbours, DeviceArray2D<float>& data_rpk, int& max_elems_rpk);
        void computePfh125(const DeviceArray2D<float>& data_rpk, int max_elems_rpk, const NeighborIndices& neighbours, DeviceArray2D<PFHSignature125>& features);

        void repackToAosForPfhRgb(const PointCloud& cloud, const Normals& normals, const NeighborIndices& neighbours, DeviceArray2D<float>& data_rpk, int& max_elems_rpk);
        void computePfhRgb250(const DeviceArray2D<float>& data_rpk, int max_elems_rpk, const NeighborIndices& neighbours, DeviceArray2D<PFHRGBSignature250>& features);


        // fpfh estimation
        void computeSPFH(const PointCloud& surface, const Normals& normals, const Indices& indices, const NeighborIndices& neighbours, DeviceArray2D<FPFHSignature33>& spfh33);
        void computeFPFH(const PointCloud& cloud, const NeighborIndices& neighbours, const DeviceArray2D<FPFHSignature33>& spfh, DeviceArray2D<FPFHSignature33>& features);

        void computeFPFH(const PointCloud& cloud, const Indices& indices, const PointCloud& surface, 
            const NeighborIndices& neighbours, DeviceArray<int>& lookup, const DeviceArray2D<FPFHSignature33>& spfh, DeviceArray2D<FPFHSignature33>& features);

        int computeUniqueIndices(size_t surface_size, const NeighborIndices& neighbours, DeviceArray<int>& unique_indices, DeviceArray<int>& lookup);

        // ppf estimation         
        void computePPF(const PointCloud& input, const Normals& normals, const Indices& indices, DeviceArray<PPFSignature>& output);
        void computePPFRGB(const PointXYZRGBCloud& input, const Normals& normals, const Indices& indices, DeviceArray<PPFRGBSignature>& output);        
        void computePPFRGBRegion(const PointXYZRGBCloud& cloud, const Normals& normals, const Indices& indices, 
            const NeighborIndices& nn_indices, DeviceArray<PPFRGBSignature>& output);

        //PrincipalCurvatures estimation
        void computePointPrincipalCurvatures(const Normals& normals, const Indices& indices, const NeighborIndices& neighbours, 
            DeviceArray<PrincipalCurvatures>& output, DeviceArray2D<float>& proj_normals_buf);


        //vfh estimation
        template<typename PointT> void compute3DCentroid(const DeviceArray<PointT>& cloud, float3& centroid);
        template<typename PointT> void compute3DCentroid(const DeviceArray<PointT>& cloud,  const Indices& indices, float3& centroid);

        template<typename PointT> float3 getMaxDistance(const DeviceArray<PointT>& cloud, const float3& pivot);        
        template<typename PointT> float3 getMaxDistance(const DeviceArray<PointT>& cloud, const Indices& indices, const float3& pivot);

        struct VFHEstimationImpl
        {
            float3 xyz_centroid;
            float3 normal_centroid;
            float3 viewpoint;

            Indices indices;
            PointCloud points;
            Normals normals;

            bool normalize_distances;
            bool size_component;
            bool normalize_bins;
       
            void compute(DeviceArray<VFHSignature308>& feature);
        };

		//spinimages estimation
		void computeSpinImagesOrigigNormal(bool radial, bool angular, float support_angle_cos, const Indices& indices, const PointCloud& input_cloud, const Normals& input_normals,
			const PointCloud& surface, const Normals& normals, const NeighborIndices& neighbours, int min_neighb, int image_width, float bin_size, PtrStep<float> output);

		void computeSpinImagesCustomAxes(bool radial, bool angular, float support_angle_cos, const Indices& indices, const PointCloud& input_cloud, const Normals& input_normals,
			const PointCloud& surface, const Normals& normals, const NeighborIndices& neighbours, int min_neighb, int image_width, float bin_size, const float3& rotation_axis, PtrStep<float> output);

		void computeSpinImagesCustomAxesCloud(bool radial, bool angular, float support_angle_cos, const Indices& indices, const PointCloud& input_cloud, const Normals& input_normals,
			const PointCloud& surface, const Normals& normals, const NeighborIndices& neighbours, int min_neighb, int image_width, float bin_size, const Normals& rotation_axes_cloud, PtrStep<float> output);

		void computeMask(const NeighborIndices& neighbours, int min_neighb, DeviceArray<unsigned char>& mask);
    }
}

#endif /* PCL_GPU_FEATURES_INTERNAL_HPP_ */
