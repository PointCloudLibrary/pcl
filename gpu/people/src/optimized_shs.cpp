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
 * @author: Anatoly Baksheev
 */

#ifndef PCL_GPU_PEOPLE_OPTIMIZED_SHS_H_
#define PCL_GPU_PEOPLE_OPTIMIZED_SHS_H_
#include "searchD.h"


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

void optimized_shs(const PointCloud<PointXYZRGB> &cloud, float tolerance, PointIndices &indices_in, PointIndices &indices_out, float delta_hue)
{
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<char> processed (cloud.points.size (), 0);

    double squared_radius = tolerance * tolerance;

    // Process all points in the indices vector
    for (size_t k = 0; k < indices_in.indices.size (); ++k)
    {
        int i = indices_in.indices[k];
        if (processed[i])
            continue;

        processed[i] = 1;

        std::vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back (i);

        PointXYZRGB  p;
        p = cloud.points[i];
        PointXYZHSV h;
        PointXYZRGBtoXYZHSV(p, h);

        while (sq_idx < (int)seed_queue.size ())
        {
            int index = seed_queue[sq_idx];
            int ycoo = index / cloud.width;
            int xcoo = index - cloud.width * ycoo;

            const int winsz = 20;
            const int winsz2 = winsz/2;

            int ybeg = max(0, ycoo - winsz2);
            int yend = min((int)cloud.height-1, ycoo + winsz2) + 1;
            int xend = min((int)cloud.width-1, xcoo + winsz2) + 1;

            for(int y = ybeg; y < yend; ++y)
                for(int x = max(0, xcoo - winsz2); x < xend; ++x)
                {
                    int idx = y * cloud.width + x;
                    if (!processed[idx] && (sqnorm(h, cloud.at(x,y)) < squared_radius))
                    {
                        PointXYZHSV h_l;
                        PointXYZRGB p_l = cloud.at(x, y);
                        PointXYZRGBtoXYZHSV(p_l, h_l);

                        if (fabs(h_l.h - h.h) < delta_hue)
                        {
                            seed_queue.push_back (idx);
                            processed[idx] = 1;
                        }
                    }
                }

            sq_idx++;
        }
        // Copy the seed queue into the output indices
        for (size_t l = 0; l < seed_queue.size (); ++l)
            indices_out.indices.push_back(seed_queue[l]);
    }
    // This is purely esthetical, can be removed for speed purposes
    std::sort (indices_out.indices.begin (), indices_out.indices.end ());
}


void optimized_shs2(const PointCloud<PointXYZRGB> &cloud, float tolerance, PointIndices &indices_in, PointIndices &indices_out, float delta_hue)
{
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<char> processed (cloud.points.size (), 0);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;

    SearchD search;
    search.setInputCloud(cloud.makeShared());

    // Process all points in the indices vector
    for (size_t k = 0; k < indices_in.indices.size (); ++k)
    {
        int i = indices_in.indices[k];
        if (processed[i])
            continue;

        processed[i] = 1;

        std::vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back (i);

        PointXYZRGB  p;
        p = cloud.points[i];
        PointXYZHSV h;
        PointXYZRGBtoXYZHSV(p, h);

        while (sq_idx < (int)seed_queue.size ())
        {
            int index = seed_queue[sq_idx];
            const PointXYZRGB& q = cloud.points[index];

            if(!isFinite (q))
                continue;

            // search window
            unsigned left, right, top, bottom;
            float squared_radius = tolerance * tolerance;
            search.getProjectedRadiusSearchBox (q, squared_radius, left, right, top, bottom);

            unsigned yEnd  = (bottom + 1) * cloud.width + right + 1;
            unsigned idx  = top * cloud.width + left;
            unsigned skip = cloud.width - right + left - 1;
            unsigned xEnd = idx - left + right + 1;

            for (; xEnd != yEnd; idx += skip, xEnd += cloud.width)
            {
                for (; idx < xEnd; ++idx)
                {
                    if (processed[idx])
                        continue;

                    if (sqnorm(cloud.points[idx], q) <= squared_radius)
                    {
                        PointXYZRGB  p_l;
                        p_l = cloud.points[idx];
                        PointXYZHSV h_l;
                        PointXYZRGBtoXYZHSV(p_l, h_l);

                        if (fabs(h_l.h - h.h) < delta_hue)
                        {
                            if(idx & 1)
                              seed_queue.push_back (idx);
                            processed[idx] = 1;
                        }

                    }
                }
            }
            sq_idx++;

        }
        // Copy the seed queue into the output indices
        for (size_t l = 0; l < seed_queue.size (); ++l)
            indices_out.indices.push_back(seed_queue[l]);
    }
    // This is purely esthetical, can be removed for speed purposes
    std::sort (indices_out.indices.begin (), indices_out.indices.end ());
}



void optimized_shs3(const PointCloud<PointXYZRGB> &cloud, float tolerance, PointIndices &indices_in, PointIndices &indices_out, float delta_hue)
{
    cv::Mat huebuf(cloud.height, cloud.width, CV_32F);
    float *hue = huebuf.ptr<float>();

    for(size_t i = 0; i < cloud.points.size(); ++i)
    {
        PointXYZHSV h;
        PointXYZRGB p = cloud.points[i];
        PointXYZRGBtoXYZHSV(p, h);
        hue[i] = h.h;
    }

    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<char> processed (cloud.points.size (), 0);

    SearchD search;
    search.setInputCloud(cloud.makeShared());

    vector< vector<int> > storage(100);

    // Process all points in the indices vector
#pragma omp parallel for
    for (int k = 0; k < static_cast<int> (indices_in.indices.size ()); ++k)
    {
        int i = indices_in.indices[k];
        if (processed[i])
            continue;

        processed[i] = 1;

        std::vector<int> seed_queue;
        seed_queue.reserve(cloud.size());
        int sq_idx = 0;
        seed_queue.push_back (i);

        PointXYZRGB p = cloud.points[i];
        float h = hue[i];

        while (sq_idx < (int)seed_queue.size ())
        {
            int index = seed_queue[sq_idx];
            const PointXYZRGB& q = cloud.points[index];

            if(!isFinite (q))
                continue;

            // search window
            unsigned left, right, top, bottom;
            double squared_radius = tolerance * tolerance;
            search.getProjectedRadiusSearchBox (q, squared_radius, left, right, top, bottom);

            unsigned yEnd  = (bottom + 1) * cloud.width + right + 1;
            unsigned idx  = top * cloud.width + left;
            unsigned skip = cloud.width - right + left - 1;
            unsigned xEnd = idx - left + right + 1;

            for (; xEnd != yEnd; idx += skip, xEnd += cloud.width)
            {
                for (; idx < xEnd; ++idx)
                {
                    if (processed[idx])
                        continue;

                    if (sqnorm(cloud.points[idx], q) <= squared_radius)
                    {
                        float h_l = hue[idx];

                        if (fabs(h_l - h) < delta_hue)
                        {
                            if(idx & 1)
                              seed_queue.push_back (idx);
                            processed[idx] = 1;
                        }

                    }
                }
            }
            sq_idx++;

        }
        // Copy the seed queue into the output indices
        int id = omp_get_thread_num();
        storage[id].insert(storage[id].end(), seed_queue.begin(), seed_queue.end());
    }

    indices_out.indices.clear();
    for(size_t i = 0; i < storage.size(); ++i)
        indices_out.indices.insert(indices_out.indices.begin(), storage[i].begin(), storage[i].end());

    std::sort (indices_out.indices.begin (), indices_out.indices.end ());
    indices_out.indices.erase(std::unique(indices_out.indices.begin (), indices_out.indices.end ()), indices_out.indices.end());
}
#endif // PCL_GPU_PEOPLE_OPTIMIZED_SHS_H_
