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



void pcl::device::shs(const DeviceArray<float8> &cloud, float tolerance, const std::vector<int>& indices_in, float delta_hue, Mask& indices_out)
{

}

//
//void optimized_shs4(const PointCloud<PointXYZRGB> &cloud, float tolerance, const PointIndices &indices_in, cv::Mat flowermat, float delta_hue)
//{
//    cv::Mat huebuf(cloud.height, cloud.width, CV_32F);
//    float *hue = huebuf.ptr<float>();
//        
//    for(size_t i = 0; i < cloud.points.size(); ++i)
//    {
//        PointXYZHSV h;
//        PointXYZRGB p = cloud.points[i];
//        PointXYZRGBtoXYZHSV(p, h);
//        hue[i] = h.h;
//    }    
//    
//    unsigned char *mask = flowermat.ptr<unsigned char>();
//        
//
//    SearchD search;
//    search.setInputCloud(cloud.makeShared());
//
//    vector< vector<int> > storage(100);
//
//    // Process all points in the indices vector
//#pragma omp parallel for
//    for (int k = 0; k < static_cast<int> (indices_in.indices.size ()); ++k)
//    {
//        int i = indices_in.indices[k];
//        if (mask[i])
//            continue;
//
//        mask[i] = 255;
//
//        int id = omp_get_thread_num();
//        std::vector<int>& seed_queue = storage[id];
//        seed_queue.clear();
//        seed_queue.reserve(cloud.size());
//        int sq_idx = 0;
//        seed_queue.push_back (i);
//
//        PointXYZRGB p = cloud.points[i];
//        float h = hue[i];
//
//        while (sq_idx < (int)seed_queue.size ())
//        {
//            int index = seed_queue[sq_idx];
//            const PointXYZRGB& q = cloud.points[index];
//
//            if(!isFinite (q))
//                continue;
//
//            // search window
//            unsigned left, right, top, bottom;
//            double squared_radius = tolerance * tolerance;
//            search.getProjectedRadiusSearchBox (q, squared_radius, left, right, top, bottom);
//
//            unsigned yEnd  = (bottom + 1) * cloud.width + right + 1;
//            unsigned idx  = top * cloud.width + left;
//            unsigned skip = cloud.width - right + left - 1;
//            unsigned xEnd = idx - left + right + 1;
//
//            for (; xEnd != yEnd; idx += skip, xEnd += cloud.width)
//            {
//                for (; idx < xEnd; ++idx)
//                {
//                    if (mask[idx])
//                        continue;
//
//                    if (sqnorm(cloud.points[idx], q) <= squared_radius)
//                    {
//                        float h_l = hue[idx];
//
//                        if (fabs(h_l - h) < delta_hue)
//                        {
//                            if(idx & 1)
//                              seed_queue.push_back (idx);
//                            mask[idx] = 255;
//                        }
//
//                    }
//                }
//            }
//            sq_idx++;
//
//        }        
//    }    
//}
