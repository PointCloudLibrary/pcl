
#if 0

/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2010-2012, Willow Garage, Inc.
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
* $Id: $
* @authors: Anatoly Baskheev
*
*/

#include "internal.h"
#include <pcl/gpu/utils/device/funcattrib.hpp>

namespace pcl
{
    namespace device
    {     
        __device__ __forceinline__ float3 operator*(const Intr& intr, const float3 p)
        {
            float3 r;
            r.x = intr.fx * p.x + intr.cx * p.z;
            r.y = intr.fy * p.y + intr.cy * p.z;
            r.z = p.z;
            return r;
        }

        __device__ __forceinline__
            void getProjectedRadiusSearchBox (int rows, int cols, const device::Intr& intr, const float3& point, float squared_radius, 
            int &minX, int &maxX, int &minY, int &maxY)
        {  
            int min, max;

            float3 q = intr * point;

            // http://www.wolframalpha.com/input/?i=K+%3D+%7B%7Ba%2C+0%2C+b%7D%2C+%7B0%2C+c%2C+d%7D%2C+%7B0%2C+0%2C+1%7D%7D%2C+matrix%5BK+.+transpose%5BK%5D%5D

            float coeff8 = 1;                                   //K_KT_.coeff (8);
            float coeff7 = intr.cy;                             //K_KT_.coeff (7);
            float coeff4 = intr.fy * intr.fy + intr.cy*intr.cy; //K_KT_.coeff (4);

            float coeff6 = intr.cx;                             //K_KT_.coeff (6);
            float coeff0 = intr.fx * intr.fx + intr.cx*intr.cx; //K_KT_.coeff (0);

            float a = squared_radius * coeff8 - q.z * q.z;
            float b = squared_radius * coeff7 - q.y * q.z;
            float c = squared_radius * coeff4 - q.y * q.y;

            // a and c are multiplied by two already => - 4ac -> - ac
            float det = b * b - a * c;

            if (det < 0)
            {
                minY = 0;
                maxY = rows - 1;
            }
            else
            {
                float y1 = (b - sqrt (det)) / a;
                float y2 = (b + sqrt (det)) / a;

                min = min (static_cast<int> (std::floor (y1)), static_cast<int> (std::floor (y2)));
                max = max (static_cast<int> (std::ceil (y1)), static_cast<int> (std::ceil (y2)));
                minY = min (rows - 1, max (0, min));
                maxY = max (min (rows - 1, max), 0);
            }

            b = squared_radius * coeff6 - q.x * q.z;
            c = squared_radius * coeff0 - q.x * q.x;

            det = b * b - a * c;
            if (det < 0)
            {
                minX = 0;
                maxX = cols - 1;
            }
            else
            {
                float x1 = (b - sqrt (det)) / a;
                float x2 = (b + sqrt (det)) / a;

                min = min (static_cast<int> (std::floor (x1)), static_cast<int> (std::floor (x2)));
                max = max (static_cast<int> (std::ceil (x1)), static_cast<int> (std::ceil (x2)));
                minX = min (cols- 1, max (0, min));
                maxX = max (min (cols - 1, max), 0);
            }
        }

        struct Shs
        {
            PtrSz<int> indices;
            PtrStepSz<float8> cloud;
            Intr intr;
            float radius;

            mutable PtrStep<unsigned char> output_mask;
        

            __device__ __forceinline__ void operator()() const
            {
              
            }
        };
    }
}

void pcl::device::shs(const DeviceArray2D<float4> &cloud, float tolerance/*radius*/, const std::vector<int>& indices_in, float delta_hue, Mask& output)
{
    int cols = cloud.cols();
    int rows = cloud.rows();

    output.create(rows, cols);
    device::setZero(output);

    DeviceArray<int> indices_device;
    indices_device.upload(indices_in);
}

#if 0

void optimized_shs5(const PointCloud<PointXYZRGB> &cloud, float tolerance, const PointIndices &indices_in, cv::Mat flowermat, float delta_hue)
{
    int rows = 480;
    int cols = 640;
    device::Intr intr(525, 525, cols/2-0.5f, rows/2-0.5f);

    //FILE *f = fopen("log.txt", "w");

    cv::Mat huebuf(cloud.height, cloud.width, CV_32F);
    float *hue = huebuf.ptr<float>();    

    for(std::size_t i = 0; i < cloud.size(); ++i)
    {
        PointXYZHSV h;
        PointXYZRGB p = cloud[i];
        PointXYZRGBtoXYZHSV(p, h);
        hue[i] = h.h;
    }    
    unsigned char *mask = flowermat.ptr<unsigned char>();


    SearchD search;    
    search.setInputCloud(cloud.makeShared());

    std::vector< std::vector<int> > storage(100);

    //  omp_set_num_threads(1);
    // Process all points in the indices vector
    for (int k = 0; k < static_cast<int> (indices_in.indices.size ()); ++k)
    {
        int i = indices_in.indices[k];
        if (mask[i])
            continue;

        mask[i] = 255;

        //    int id = omp_get_thread_num();
        //std::vector<int>& seed_queue = storage[id];
        std::vector<int> seed_queue;
        seed_queue.clear();
        seed_queue.reserve(cloud.size());
        int sq_idx = 0;
        seed_queue.push_back (i);

        PointXYZRGB p = cloud[i];
        float h = hue[i];

        while (sq_idx < (int)seed_queue.size ())
        {
            int index = seed_queue[sq_idx];
            const PointXYZRGB& q = cloud[index];

            if(!isFinite (q))
                continue;

            // search window
            double squared_radius = tolerance * tolerance;
            //unsigned int left, right, top, bottom;            
            //search.getProjectedRadiusSearchBox (q, squared_radius, left, right, top, bottom);

            int left, right, top, bottom;
            getProjectedRadiusSearchBox(rows, cols, intr, q, squared_radius, left, right, top, bottom);

            //fprintf(f, "%d) %d %d %d %d\n", index, left, right, top, bottom);


            int yEnd  = (bottom + 1) * cloud.width + right + 1;
            int idx  = top * cloud.width + left;
            int skip = cloud.width - right + left - 1;
            int xEnd = idx - left + right + 1;

            for (; xEnd != yEnd; idx += skip, xEnd += cloud.width)
            {
                for (; idx < xEnd; ++idx)
                {
                    if (mask[idx])
                        continue;

                    if (sqnorm(cloud[idx], q) <= squared_radius)
                    {
                        float h_l = hue[idx];

                        if (std::abs(h_l - h) < delta_hue)
                        {
                            if(idx & 1)
                                seed_queue.push_back (idx);
                            mask[idx] = 255;
                        }

                    }
                }
            }
            sq_idx++;

        }        
    }       
}

#endif


#endif