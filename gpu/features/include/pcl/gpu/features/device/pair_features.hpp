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

#ifndef PCL_GPU_FEATURES_DEVICE_PAIR_FEATURES_HPP_
#define PCL_GPU_FEATURES_DEVICE_PAIR_FEATURES_HPP_

#include <pcl/gpu/utils/device/vector_math.hpp>
#include <pcl/gpu/features/device/rodrigues.hpp>

namespace pcl
{
    namespace device
    {
        __device__ __host__ __forceinline__ 
        bool computePairFeatures (const float3& p1, const float3& n1, const float3& p2, const float3& n2, float &f1, float &f2, float &f3, float &f4)
        {
            f1 = f2 = f3 = f4 = 0.0f;

            float3 dp2p1 = p2 - p1;            
            f4 = norm(dp2p1);

            if (f4 == 0.f)
                return false;           

            float3 n1_copy = n1, n2_copy = n2;
            float angle1 = dot(n1_copy, dp2p1) / f4;
            

            float angle2 = dot(n2_copy, dp2p1) / f4;
            if (std::acos (std::abs (angle1)) > std::acos (std::abs (angle2)))
            {
              // switch p1 and p2
              n1_copy = n2;
              n2_copy = n1;
              dp2p1 *= (-1);
              f3 = -angle2;
            }
            else
              f3 = angle1;

            // Create a Darboux frame coordinate system u-v-w
            // u = n1; v = (p_idx - q_idx) x u / || (p_idx - q_idx) x u ||; w = u x v
            float3 v = cross(dp2p1, n1_copy);            
            float v_norm = norm(v);
            if (v_norm == 0.0f)
                return false;
            
            // Normalize v
            v *= 1.f/v_norm;            
                        
            // Do not have to normalize w - it is a unit vector by construction            
            f2 = dot(v, n2_copy);
            
            float3 w = cross(n1_copy, v);
            // Compute f1 = arctan (w * n2, u * n2) i.e. angle of n2 in the x=u, y=w coordinate system            
            f1 = std::atan2 (dot(w, n2_copy), dot(n1_copy, n2_copy)); // @todo optimize this

            return true;
        }

        __device__ __host__ __forceinline__ 
        bool computeRGBPairFeatures (const float3& p1, const float3& n1, const int& colors1, const float3& p2, const float3& n2, const int& colors2,
            float &f1, float &f2, float &f3, float &f4, float &f5, float &f6, float &f7)
        {
            float3 dp2p1 = p2 - p1;            
            f4 = norm(dp2p1);

            if (f4 == 0.0f)
            {
                f1 = f2 = f3 = f4 = f5 = f6 = f7 = 0.0f;
                return false;
            }

            float3 n1_copy = n1, n2_copy = n2;            
            float angle1 = dot(n1_copy, dp2p1) / f4;

            f3 = angle1;

            // Create a Darboux frame coordinate system u-v-w
            // u = n1; v = (p_idx - q_idx) x u / || (p_idx - q_idx) x u ||; w = u x v
            float3 v = cross(dp2p1, n1_copy);            
            float v_norm = norm(v);
            if (v_norm == 0.0f)
            {
                f1 = f2 = f3 = f4 = f5 = f6 = f7 = 0.0f;
                return false;
            }
            // Normalize v
            v *= 1.f/v_norm;

            float3 w = cross(n1_copy, v);
            // Do not have to normalize w - it is a unit vector by construction
            
            f2 = dot(v, n2_copy);

            // Compute f1 = arctan (w * n2, u * n2) i.e. angle of n2 in the x=u, y=w coordinate system
            f1 = std::atan2 (dot(w, n2_copy), dot (n1_copy, n2_copy)); 

            // everything before was standard 4D-Darboux frame feature pair
            // now, for the experimental color stuff            
            
            f5 = ((float) ((colors1      ) & 0xFF)) / ((colors2      ) & 0xFF);
            f6 = ((float) ((colors1 >>  8) & 0xFF)) / ((colors2 >>  8) & 0xFF);
            f7 = ((float) ((colors1 >> 16) & 0xFF)) / ((colors2 >> 16) & 0xFF);

            // make sure the ratios are in the [-1, 1] interval
            if (f5 > 1.f) f5 = - 1.f / f5;
            if (f6 > 1.f) f6 = - 1.f / f6;
            if (f7 > 1.f) f7 = - 1.f / f7;

            return true;
        }

        __device__ __host__ __forceinline__ 
        void computeRGBPairFeatures_RGBOnly (const int& colors1, const int& colors2, float &f5, float &f6, float &f7)
        {
            f5 = ((float) ((colors1      ) & 0xFF)) / ((colors2      ) & 0xFF);
            f6 = ((float) ((colors1 >>  8) & 0xFF)) / ((colors2 >>  8) & 0xFF);
            f7 = ((float) ((colors1 >> 16) & 0xFF)) / ((colors2 >> 16) & 0xFF);

            // make sure the ratios are in the [-1, 1] interval
            if (f5 > 1.f) f5 = - 1.f / f5;
            if (f6 > 1.f) f6 = - 1.f / f6;
            if (f7 > 1.f) f7 = - 1.f / f7;
        }

         __device__ __host__ __forceinline__ bool computePPFPairFeature(const float3& p1, const float3& n1, const float3& p2, const float3& n2,
            float& f1, float& f2, float& f3, float& f4)
        {
            float3 delta = p2 - p1;
            
            f4 = norm (delta);

            delta.x /= f4;
            delta.y /= f4;
            delta.z /= f4;

            f1 = dot(n1, delta);            
            f2 = dot(n2, delta);            
            f3 = dot(n1, n2);

            return true;
        }


         __device__ __host__ __forceinline__ void computeAlfaM(const float3& model_reference_point, const float3& model_reference_normal, 
            const float3& model_point, float& alpha_m)
        {
            float acos_value = std::acos (model_reference_normal.x);

            //float3 cross_vector = cross(model_reference_normal, Eigen::Vector3f::UnitX);
            float3 cross_vector = make_float3(0, model_reference_normal.z, - model_reference_normal.y);
            float3 cross_vector_norm = normalized(cross_vector);

            //Eigen::AngleAxisf rotation_mg (acos_value, cross_vector_norm);
            //Eigen::Affine3f transform_mg = Eigen::Translation3f ( rotation_mg * ((-1) * model_reference_point)) * rotation_mg;

            float3 row1, row2, row3; // == rotation_mg
            AngleAxisf(acos_value, cross_vector_norm, row1, row2, row3);

            float3 translation;
            //translation.x = row1.x * -model_reference_point.x + row1.y * -model_reference_point.y + row1.z * -model_reference_point.z;
            translation.y = row2.x * -model_reference_point.x + row2.y * -model_reference_point.y + row2.z * -model_reference_point.z;
            translation.z = row3.x * -model_reference_point.x + row3.y * -model_reference_point.y + row3.z * -model_reference_point.z;

            float3 model_point_transformed;// = transform_mg * model_point;
            //model_point_transformed.x = translation.x + row1.x * model_point.x + row1.y * model_point.y + row1.z * model_point.z;
            model_point_transformed.y = translation.y + row2.x * model_point.x + row2.y * model_point.y + row2.z * model_point.z;
            model_point_transformed.z = translation.z + row3.x * model_point.x + row3.y * model_point.y + row3.z * model_point.z;


            float angle = std::atan2 ( -model_point_transformed.z, model_point_transformed.y);

            if (sinf(angle) * model_point_transformed.z < 0.0f)
                //if (angle * model_point_transformed.z < 0.ff)
                angle *= -1;
            alpha_m = -angle;
        }     
    }
}

#endif /* PCL_GPU_FEATURES_DEVICE_PAIR_FEATURES_HPP_ */