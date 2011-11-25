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

#ifndef _PCL_TEST_GPU_OCTREE_DATAGEN_
#define _PCL_TEST_GPU_OCTREE_DATAGEN_

#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/StdVector>
#include <cstdlib>


#if defined (_WIN32) || defined(_WIN64)
    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(pcl::PointXYZ)
#endif

struct DataGenerator
{
    typedef pcl::gpu::Octree::PointType PointType;

    size_t data_size;            
    size_t tests_num;

    float cube_size;
    float max_radius;     

    float shared_radius;

    std::vector<PointType> points;
    std::vector<PointType> queries;
    std::vector<float> radiuses;
    std::vector< std::vector<int> > bfresutls;

    std::vector<int> indices;

    DataGenerator() : data_size(871000), tests_num(10000), cube_size(1024.f)
    {
        max_radius    = cube_size/15.f;
        shared_radius = cube_size/20.f;
    }

    void operator()()
    {             
        srand (0);

        points.resize(data_size);
        for(size_t i = 0; i < data_size; ++i)
        {            
            points[i].x = ((float)rand())/RAND_MAX * cube_size;  
            points[i].y = ((float)rand())/RAND_MAX * cube_size;  
            points[i].z = ((float)rand())/RAND_MAX * cube_size;
        }
        

        queries.resize(tests_num);
        radiuses.resize(tests_num);
        for (size_t i = 0; i < tests_num; ++i)
        {            
            queries[i].x = ((float)rand())/RAND_MAX * cube_size;  
            queries[i].y = ((float)rand())/RAND_MAX * cube_size;  
            queries[i].z = ((float)rand())/RAND_MAX * cube_size;  		
            radiuses[i]  = ((float)rand())/RAND_MAX * max_radius;	
        };        

        for(size_t i = 0; i < tests_num/2; ++i)
            indices.push_back(i*2);
    }

    void bruteForceSearch(bool log = false, float radius = -1.f)
    {        
        if (log)
            std::cout << "BruteForceSearch";

        int value100 = std::min<int>(tests_num, 50);
        int step = tests_num/value100;        

        bfresutls.resize(tests_num);
        for(size_t i = 0; i < tests_num; ++i)
        {            
            if (log && i % step == 0)
            {
                std::cout << ".";
                std::cout.flush();
            }

            std::vector<int>& curr_res = bfresutls[i];
            curr_res.clear();
                        
            float query_radius = radius > 0 ? radius : radiuses[i];
            const PointType& query = queries[i];

            for(size_t ind = 0; ind < points.size(); ++ind)
            {
                const PointType& point = points[ind];

                float dx = query.x - point.x;
                float dy = query.y - point.y;
                float dz = query.z - point.z;

                if (dx*dx + dy*dy + dz*dz < query_radius * query_radius)
                    curr_res.push_back(ind);
            }

            std::sort(curr_res.begin(), curr_res.end());
        }
        if (log)
            std::cout << "Done" << std::endl;
    }

    void printParams() const 
    {        
        std::cout << "Points number  = " << data_size << std::endl;
        std::cout << "Queries number = " << tests_num << std::endl;
        std::cout << "Cube size      = " << cube_size << std::endl;
        std::cout << "Max radius     = " << max_radius << std::endl;
        std::cout << "Shared radius  = " << shared_radius << std::endl;
    }

    template<typename Dst>
    struct ConvPoint
    {    
        Dst operator()(const PointType& src) const 
        {
            Dst dst;
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            return dst;
        }
    };

};

#endif  /* _PCL_TEST_GPU_OCTREE_DATAGEN_ */



