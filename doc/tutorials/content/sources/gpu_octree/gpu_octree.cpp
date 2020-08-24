/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception
 *
 *  All rights reserved
 */

#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>

#include<algorithm>
#include<fstream>
#include<iostream>

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Generate pointcloud data
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size (); ++i)
    {
        (*cloud)[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
        (*cloud)[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
        (*cloud)[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
    }

    const size_t query_size = 10;
    std::vector<pcl::PointXYZ> queries;
    queries.resize(query_size);

    for (std::size_t i = 0; i < query_size; ++i)
    {
        queries[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
        queries[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
        queries[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
    }

    //prepare device cloud
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(cloud->points);

    //upload queries
    pcl::gpu::Octree::Queries queries_device;
    queries_device.upload(queries);

    //gpu build
    pcl::gpu::Octree octree_device;
    octree_device.setCloud(cloud_device);
    octree_device.build();


    //perform k nearest search
    //prepare output buffers on device
    const int k = 1;
    pcl::gpu::NeighborIndices results_knn(query_size, k);
    pcl::gpu::Octree::ResultSqrDists dists_knn;

    //search GPU
    octree_device.nearestKSearchBatch(queries_device, k, results_knn, dists_knn);

    //download results
    std::vector<int> downloaded_knn;
    std::vector<float> dists_downloaded_knn;
    results_knn.data.download(downloaded_knn);
    dists_knn.download(dists_downloaded_knn);

    for(std::size_t i = 0; i < query_size; ++i)
    {
        std::cout << "K nearest neighbor search at (" << queries[i].x
            << " " << queries[i].y
            << " " << queries[i].z << ")" << std::endl;

        const int beg = i * k;
        const int end = beg + k;

        const auto beg_dist2 = dists_downloaded_knn.cbegin() + beg;
        const auto end_dist2 = dists_downloaded_knn.cbegin() + end;

        const std::vector<int> downloaded_knn_cur (downloaded_knn.cbegin() + beg, downloaded_knn.cbegin() + end);
        const std::vector<float> dists_downloaded_knn_cur (beg_dist2, end_dist2);

        for (std::size_t j = 0; j < k; ++j)
        {
            std::cout << "\t" << cloud->points[downloaded_knn_cur[j]].x
                << " " << cloud->points[downloaded_knn_cur[j]].y
                << " " << cloud->points[downloaded_knn_cur[j]].z
                << " (squared distance " << dists_downloaded_knn_cur[j] << ")" << std::endl;
        }
    }
    std::cout <<std::endl;


    //perform radius search
    //prepare radiuses
    std::vector<float> radiuses;
    for (std::size_t i = 0; i < query_size; ++i)
        radiuses.push_back (200.0f * rand () / (RAND_MAX + 1.0f));

    pcl::gpu::Octree::Radiuses radiuses_device;
    radiuses_device.upload(radiuses);

    //prepare output buffers on device
    const int max_results = 1000;
    pcl::gpu::NeighborIndices results_radius(query_size, max_results);
    pcl::gpu::Octree::ResultSqrDists dists_radius;

    //search GPU
    octree_device.radiusSearch(queries_device, radiuses_device, max_results, results_radius, dists_radius);

    //download results
    std::vector<int> downloaded_radius;
    std::vector<float> dists_downloaded_radius;
    std::vector<int> sizes;
    results_radius.data.download(downloaded_radius);
    dists_radius.download(dists_downloaded_radius);
    results_radius.sizes.download(sizes);

    for(std::size_t i = 0; i < query_size; ++i)
    {
        std::cout << "Radius search at (" << queries[i].x
            << " " << queries[i].y
            << " " << queries[i].z
            << ") within radius " << radiuses[i]
            << ". Found " << sizes[i] << " results." << std::endl;

        const int beg = i * max_results;
        const int end = beg + sizes[i];

        const auto beg_dist2 = dists_downloaded_radius.cbegin() + beg;
        const auto end_dist2 = dists_downloaded_radius.cbegin() + end;

        const std::vector<int> downloaded_radius_cur (downloaded_radius.cbegin() + beg, downloaded_radius.cbegin() + end);
        const std::vector<float> dists_downloaded_radius_cur (beg_dist2, end_dist2);

        for (std::size_t j = 0; j < sizes[i]; ++j)
        {
            std::cout << "\t" << cloud->points[downloaded_radius_cur[j]].x
                << " " << cloud->points[downloaded_radius_cur[j]].y
                << " " << cloud->points[downloaded_radius_cur[j]].z
                << " (squared distance " << dists_downloaded_radius_cur[j] << ")" << std::endl;
        }
    }
    std::cout <<std::endl;


    //perform approximate nearest search
    //prepare output buffers on device
    pcl::gpu::NeighborIndices results_approx(query_size, 1);
    pcl::gpu::Octree::ResultSqrDists dists_approx;

    //search GPU
    octree_device.approxNearestSearch(queries_device, results_approx, dists_approx);

    //download results
    std::vector<int> downloaded_approx;
    std::vector<float> dists_downloaded_approx;
    results_approx.data.download(downloaded_approx);
    dists_approx.download(dists_downloaded_approx);

    for(std::size_t i = 0; i < query_size; ++i)
    {
        std::cout << "Approximate nearest neighbor search at (" << queries[i].x
            << " " << queries[i].y
            << " " << queries[i].z << ")" << std::endl;

        std::cout << "\t" << cloud->points[downloaded_approx[i]].x
            << " " << cloud->points[downloaded_approx[i]].y
            << " " << cloud->points[downloaded_approx[i]].z
            << " (squared distance " << dists_downloaded_approx[i] << ")" << std::endl;
    }

    return 0;
}
