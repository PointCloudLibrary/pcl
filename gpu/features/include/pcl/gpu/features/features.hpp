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

#ifndef _PCL_GPU_FEATURES_HPP_
#define _PCL_GPU_FEATURES_HPP_

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/octree/device_format.hpp>
#include <pcl/gpu/octree/octree.hpp>

namespace pcl
{
    namespace gpu
    {
        ////////////////////////////////////////////////////////////////////////////////////////////  
        /** \brief @b Feature represents the base feature class.  */

        struct PCL_EXPORTS Feature
        {
        public:
            using PointType = PointXYZ;
            using NormalType = PointXYZ;

            using PointCloud = DeviceArray<PointType>;
            using Normals = DeviceArray<NormalType>;
            using Indices = DeviceArray<int>;

            Feature();

            void setInputCloud(const PointCloud& cloud);
            void setSearchSurface(const PointCloud& surface);
            void setIndices(const Indices& indices);
            void setRadiusSearch(float radius, int max_results);
        protected:
            PointCloud cloud_;
            PointCloud surface_;
            Indices indices_;
            float radius_;
            int max_results_;

            Octree octree_;
        };

        ////////////////////////////////////////////////////////////////////////////////////////////  
        /** \brief @b Feature represents the base feature class that takes normals as input also.  */

        struct PCL_EXPORTS FeatureFromNormals : public Feature
        {
        public:

            void setInputNormals(const Normals& normals);
        protected:
            Normals normals_;
        };

        ////////////////////////////////////////////////////////////////////////////////////////////  
        /** \brief @b Class for normal estimation.  */
        class PCL_EXPORTS NormalEstimation : public Feature
        {
        public:
            // float x, y, z, curvature; -> sizeof(PointXYZ) = 4 * sizeof(float)            
            using NormalType = Feature::NormalType; 

            NormalEstimation();
            void compute(Normals& normals);
            void setViewPoint(float  vpx, float  vpy, float  vpz);  
            void getViewPoint(float& vpx, float& vpy, float& vpz) const;

            static void computeNormals(const PointCloud& cloud, const NeighborIndices& nn_indices, Normals& normals);
            static void flipNormalTowardsViewpoint(const PointCloud& cloud, float vp_x, float vp_y, float vp_z, Normals& normals);            
            static void flipNormalTowardsViewpoint(const PointCloud& cloud, const Indices& indices, float vp_x, float vp_y, float vp_z, Normals& normals);
        private:              
            float vpx_, vpy_, vpz_;
            NeighborIndices nn_indices_;
        };        

        ////////////////////////////////////////////////////////////////////////////////////////////  
        /** \brief @b Class for PFH estimation.  */
        class PCL_EXPORTS PFHEstimation : public FeatureFromNormals
        {
        public:
            void compute(const PointCloud& cloud, const Normals& normals, const NeighborIndices& neighb_indices, DeviceArray2D<PFHSignature125>& features);
            void compute(DeviceArray2D<PFHSignature125>& features);
        private:
            NeighborIndices nn_indices_;
            DeviceArray2D<float> data_rpk;
            int max_elems_rpk;
        };

        ////////////////////////////////////////////////////////////////////////////////////////////  
        /** \brief @b Class for PFHRGB estimation.  */
        class PCL_EXPORTS PFHRGBEstimation : public FeatureFromNormals
        {
        public:
            using PointType = PointXYZ; //16 bytes for xyzrgb
            void compute(const PointCloud& cloud, const Normals& normals, const NeighborIndices& neighb_indices, DeviceArray2D<PFHRGBSignature250>& features);
            void compute(DeviceArray2D<PFHRGBSignature250>& features);
        private:
            NeighborIndices nn_indices_;
            DeviceArray2D<float> data_rpk;
            int max_elems_rpk;
        };

        ////////////////////////////////////////////////////////////////////////////////////////////  
        /** \brief @b Class for FPFH estimation.  */
        class PCL_EXPORTS FPFHEstimation : public FeatureFromNormals
        {
        public:
            FPFHEstimation();
            virtual ~FPFHEstimation();

            void compute(DeviceArray2D<FPFHSignature33>& features);

            void compute(const PointCloud& cloud, const Normals& normals, const NeighborIndices& neighbours, DeviceArray2D<FPFHSignature33>& features);

        private:
            NeighborIndices nn_indices_, nn_indices2_;

            DeviceArray<int> unique_indices_storage;
            DeviceArray<int> lookup;

            DeviceArray2D<FPFHSignature33> spfh;
        };      

        //////////////////////////////////////////////////////////////////////////////////////////////  
        ///** \brief @b Class for PPF estimation.  */
        class PCL_EXPORTS PPFEstimation : public FeatureFromNormals
        {
        public:
            void compute(DeviceArray<PPFSignature>& features);
        };

        //////////////////////////////////////////////////////////////////////////////////////////////  
        ///** \brief @b Class for PPFRGB estimation.  */

        class PCL_EXPORTS PPFRGBEstimation : public FeatureFromNormals
        {
        public:

            using PointType = PointXYZ; //16 bytes for xyzrgb
            void compute(DeviceArray<PPFRGBSignature>& features);
        };

        //////////////////////////////////////////////////////////////////////////////////////////////  
        ///** \brief @b Class for PPFRGBRegion estimation.  */

        class PCL_EXPORTS PPFRGBRegionEstimation : public FeatureFromNormals
        {
        public:
            using PointType = PointXYZ; //16 bytes for xyzrgb
            void compute(DeviceArray<PPFRGBSignature>& features);

        private:
            NeighborIndices nn_indices_;
        }; 


        //////////////////////////////////////////////////////////////////////////////////////////////
        ///** \brief @b Class for PPFRGBRegion estimation.  */

        class PCL_EXPORTS PrincipalCurvaturesEstimation : public FeatureFromNormals
        {
        public:

            void compute(DeviceArray<PrincipalCurvatures>& features);                    
        private:
            NeighborIndices nn_indices_;
            DeviceArray2D<float> proj_normals_buf;
        }; 


        //////////////////////////////////////////////////////////////////////////////////////////////  
        ///** \brief @b Class for Viewpoint Feature Histogramm estimation.  */

        class PCL_EXPORTS VFHEstimation : public FeatureFromNormals
        {
        public:

            enum
            {
                BINS1_F1 = 45,
                BINT2_F2 = 45,
                BINS3_F3 = 45,
                BINS4_F4 = 45,
                BINS_VP = 128
            };

            VFHEstimation();

            void setViewPoint(float  vpx, float  vpy, float  vpz);  
            void getViewPoint(float& vpx, float& vpy, float& vpz) const;      

            void setUseGivenNormal (bool use);
            void setNormalToUse (const NormalType& normal);
            void setUseGivenCentroid (bool use);
            void setCentroidToUse (const PointType& centroid);

            void setNormalizeBins (bool normalize);
            void setNormalizeDistance (bool normalize);
            void setFillSizeComponent (bool fill_size);

            void compute(DeviceArray<VFHSignature308>& feature);
        private:

            float vpx_, vpy_, vpz_;

            bool use_given_normal_;
            bool use_given_centroid_;
            bool normalize_bins_;
            bool normalize_distances_;
            bool size_component_;

            NormalType normal_to_use_;
            PointType centroid_to_use_;
        }; 


        //////////////////////////////////////////////////////////////////////////////////////////////
        ///** \brief @b Class for SpinImages estimation.  */

        class PCL_EXPORTS SpinImageEstimation : public FeatureFromNormals
        {
        public:  
            using SpinImage = Histogram<153>;

            SpinImageEstimation (unsigned int image_width = 8,
                double support_angle_cos = 0.0,   // when 0, this is bogus, so not applied
                unsigned int min_pts_neighb = 0);
            
            void setImageWidth (unsigned int bin_count);            
            void setSupportAngle (float support_angle_cos);                        
            void setMinPointCountInNeighbourhood (unsigned int min_pts_neighb);            
            void setInputWithNormals (const PointCloud& input, const Normals& normals);                        
            void setSearchSurfaceWithNormals (const PointCloud& surface, const Normals& normals);
            
            void setRotationAxis (const NormalType& axis);
            void setInputRotationAxes (const Normals& axes);            
            void useNormalsAsRotationAxis();
            void setAngularDomain (bool is_angular = true);
            void setRadialStructure (bool is_radial = true);

            void compute(DeviceArray2D<SpinImage>& features, DeviceArray<unsigned char>& mask);
        
        private:            
            Normals input_normals_;
            Normals rotation_axes_cloud_;

            bool is_angular_;

            NormalType rotation_axis_;
            bool use_custom_axis_;
			
			/* use input normals as rotation axes*/
            bool use_custom_axes_cloud_; 

            bool is_radial_;

            unsigned int image_width_;
            float support_angle_cos_;
            unsigned int min_pts_neighb_;

            bool fake_surface_;

			NeighborIndices nn_indices_;
        };
    }
};

#endif /* _PCL_GPU_FEATURES_HPP_ */


