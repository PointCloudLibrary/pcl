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


#ifndef PCL_GPU_FEATURES_TEST_DATA_SORUCE_HPP_
#define PCL_GPU_FEATURES_TEST_DATA_SORUCE_HPP_

#include<string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/gpu/containers/kernel_containers.h>
#include <pcl/search/search.h>

#include <Eigen/StdVector>

#if defined (_WIN32) || defined(_WIN64)
    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(pcl::PointXYZ)
    EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(pcl::Normal)
#endif

#include <algorithm>

namespace pcl
{
    namespace gpu
    {
        struct DataSource
        {               
            const static int k = 32;
            const static int max_elements = 500;

            PointCloud<PointXYZ>::Ptr cloud;
            PointCloud<PointXYZ>::Ptr surface;            
            IndicesPtr indices;

            PointCloud<Normal>::Ptr normals;
            PointCloud<Normal>::Ptr normals_surface;
            float radius;

            std::vector< std::vector<int> > neighbors_all;
            std::vector<int> sizes;
            int max_nn_size;

            DataSource(const std::string& file = "d:/office_chair_model.pcd") 
                : cloud(new PointCloud<PointXYZ>()), surface(new PointCloud<PointXYZ>()), indices( new std::vector<int>() ),
                normals(new PointCloud<Normal>()), normals_surface(new PointCloud<Normal>())
            {                
                PCDReader pcd;
                pcd.read(file, *cloud);

                PointXYZ minp, maxp;
                pcl::getMinMax3D(*cloud, minp, maxp);
                float sz = (maxp.x - minp.x + maxp.y - minp.y + maxp.z - minp.z) / 3;
                radius = sz / 15;
            }

            void generateColor()
            {
                size_t cloud_size = cloud->points.size();
                for(size_t i = 0; i < cloud_size; ++i)
                {
                    PointXYZ& p = cloud->points[i];

                    int r = std::max(1, std::min(255, static_cast<int>((double(rand())/RAND_MAX)*255)));
                    int g = std::max(1, std::min(255, static_cast<int>((double(rand())/RAND_MAX)*255)));
                    int b = std::max(1, std::min(255, static_cast<int>((double(rand())/RAND_MAX)*255)));

                    *reinterpret_cast<int*>(&p.data[3]) = (b << 16) + (g << 8) + r;
                }
            }

            void estimateNormals()
            {
                pcl::NormalEstimation<PointXYZ, Normal> ne;
                ne.setInputCloud (cloud);
                ne.setSearchMethod (pcl::search::KdTree<PointXYZ>::Ptr (new pcl::search::KdTree<PointXYZ>));
                ne.setKSearch (k);
                //ne.setRadiusSearch (radius);
                
                ne.compute (*normals);                
            }

            void runCloudViewer() const
            {
                pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
                viewer.showCloud (cloud);
                while (!viewer.wasStopped ()) {}
            }

            void findKNNeghbors()
            {                                
                KdTreeFLANN<PointXYZ>::Ptr kdtree(new KdTreeFLANN<PointXYZ>);
                kdtree->setInputCloud(cloud);                
                
                size_t cloud_size = cloud->points.size();

                std::vector<float> dists;
                neighbors_all.resize(cloud_size);
                for(size_t i = 0; i < cloud_size; ++i)
                {
                    kdtree->nearestKSearch(cloud->points[i], k, neighbors_all[i], dists);
                    sizes.push_back((int)neighbors_all[i].size());        
                }
                max_nn_size = *max_element(sizes.begin(), sizes.end());
            }

            void findRadiusNeghbors(float radius = -1)
            {
                radius = radius == -1 ? this->radius : radius;

                KdTreeFLANN<PointXYZ>::Ptr kdtree(new KdTreeFLANN<PointXYZ>);
                kdtree->setInputCloud(cloud);                
                
                size_t cloud_size = cloud->points.size();

                std::vector<float> dists;
                neighbors_all.resize(cloud_size);
                for(size_t i = 0; i < cloud_size; ++i)
                {
                    kdtree->radiusSearch(cloud->points[i], radius, neighbors_all[i], dists);
                    sizes.push_back((int)neighbors_all[i].size());        
                }
                max_nn_size = *max_element(sizes.begin(), sizes.end());
            }

            void getNeghborsArray(std::vector<int>& data)
            {   
                data.resize(max_nn_size * neighbors_all.size());
                pcl::gpu::PtrStep<int> ps(&data[0], max_nn_size * sizeof(int));    
                for(size_t i = 0; i < neighbors_all.size(); ++i)
                    copy(neighbors_all[i].begin(), neighbors_all[i].end(), ps.ptr(i));
            }

            void generateSurface()
            {
                surface->points.clear();
                for(size_t i = 0; i < cloud->points.size(); i+= 10)               
                    surface->points.push_back(cloud->points[i]);
                surface->width = surface->points.size();
                surface->height = 1;
                  
                if (!normals->points.empty())
                {
                    normals_surface->points.clear();
                    for(size_t i = 0; i < normals->points.size(); i+= 10)               
                        normals_surface->points.push_back(normals->points[i]);

                    normals_surface->width = surface->points.size();
                    normals_surface->height = 1;
                }                                
            }

            void generateIndices(size_t step = 100)
            {
                indices->clear();
                for(size_t i = 0; i < cloud->points.size(); i += step)
                    indices->push_back(i);                
            }

            struct Normal2PointXYZ
            {
                PointXYZ operator()(const Normal& n) const
                {
                    PointXYZ xyz;
                    xyz.x = n.normal[0];
                    xyz.y = n.normal[1];
                    xyz.z = n.normal[2];
                    return xyz;
                }
            };
        };
    }
}

#endif /* PCL_GPU_FEATURES_TEST_DATA_SORUCE_HPP_ */
