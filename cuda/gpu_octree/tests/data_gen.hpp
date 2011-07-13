#ifndef _PCL_TEST_GPU_OCTREE_DATAGEN_
#define _PCL_TEST_GPU_OCTREE_DATAGEN_

#include <vector>
#include <algorithm>
#include <iostream>
#include <opencv2/core/core.hpp>

struct DataGenerator
{
    typedef pcl::gpu::Octree::PointTypeGpu PointTypeGpu;

    size_t data_size;            
    size_t tests_num;

    float cube_size;
    float max_radius_part;        

    std::vector<PointTypeGpu> points;
    std::vector<PointTypeGpu> queries;
    std::vector<float> radiuses;

    std::vector< std::vector<int> > bfresutls;

    DataGenerator() : data_size(871000), tests_num(10000), cube_size(1024.f), max_radius_part(15.f) 
    {  
        this->operator()();  
    }

    DataGenerator(size_t data_size_arg, size_t tests_num_arg, float cube_size_arg, float max_radius_part_arg) 
        : data_size(data_size_arg), tests_num(tests_num_arg), cube_size(cube_size_arg), max_radius_part(15.f) 
    {  
        this->operator()();  
    }

    void operator()()
    {             
        cv::RNG& rng = cv::theRNG();

        points.resize(data_size);
        for(size_t i = 0; i < data_size; ++i)
        {            
            points[i].x = (float)rng * cube_size;  
            points[i].y = (float)rng * cube_size;  
            points[i].z = (float)rng * cube_size;
        }
        

        queries.resize(tests_num);
        radiuses.resize(tests_num);
        for (size_t i = 0; i < tests_num; ++i)
        {            
            queries[i].x = (float)rng * cube_size;  
            queries[i].y = (float)rng * cube_size;  
            queries[i].z = (float)rng * cube_size;  		
            radiuses[i]  = (float)rng * cube_size / max_radius_part;	
        };        
    }

    void bruteForceSearch(float radius = -1.f)
    {        
        bfresutls.resize(tests_num);
        for(size_t i = 0; i < bfresutls.size(); ++i)
        {            
            std::vector<int>& curr_res = bfresutls[i];
            curr_res.clear();
                        
            float query_radius = radius > 0 ? radius : radiuses[i];
            const PointTypeGpu& query = queries[i];

            for(size_t ind = 0; ind < points.size(); ++ind)
            {
                const PointTypeGpu& point = points[ind];

                float dx = query.x - point.x;
                float dy = query.y - point.y;
                float dz = query.z - point.z;

                if (dx*dx + dy*dy + dz*dz < query_radius * query_radius)
                    curr_res.push_back(ind);
            }

            std::sort(curr_res.begin(), curr_res.end());
        }
    }

    void printParams() const 
    {
        std::cout << "Points number   = " << data_size << std::endl;
        std::cout << "Queries number  = " << tests_num << std::endl;
        std::cout << "Cube size       = " << cube_size << std::endl;
        std::cout << "Max radius part = " << max_radius_part << std::endl;
    }
};

#endif  /* _PCL_TEST_GPU_OCTREE_DATAGEN_ */



