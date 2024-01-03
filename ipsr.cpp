/*
Copyright (c) 2022, Fei Hou and Chiyu Wang, Institute of Software, Chinese Academy of Sciences.
All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <vector>
#include <cstdlib>
#include <ctime>
#include <string>
#include <queue>
#include <algorithm>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/uniform_sampling.h>

using namespace std;
using namespace pcl;

typedef PointXYZRGBNormal PointT;
typedef PointCloud<PointT> PointCloudT;

double squaredEuclideanDistance(const PointT& p1, const PointT& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return dx * dx + dy * dy + dz * dz;
}

void ipsr(const std::string& input_name, const std::string& output_name, int iters, double pointweight, int depth, int k_neighbors)
{
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPLYFile(input_name, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", input_name.c_str());
        return;
    }
 
    PointCloudT::Ptr sampled_cloud(new PointCloudT);
    UniformSampling<PointT> uniform_sampler;
    uniform_sampler.setInputCloud(cloud);
    uniform_sampler.setRadiusSearch(0.05);
    uniform_sampler.filter(*sampled_cloud);

    KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(sampled_cloud);

    PointCloudT::Ptr reconstructed_cloud(new PointCloudT);
    int epoch = 0;
    while (epoch < iters)
    {
        ++epoch;
        printf("Iter: %d\n", epoch);

        // Poisson reconstruction
        Poisson<PointT> poisson;
        poisson.setDepth(depth);
        poisson.setPointWeight(pointweight);
        poisson.setInputCloud(sampled_cloud);
        PolygonMesh mesh;
  
        pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
        pcl::copyPointCloud(*reconstructed_cloud, cloud_xyz);

        pcl::toPCLPointCloud2(cloud_xyz, mesh.cloud);
        poisson.reconstruct(mesh);

        // Compute face normals and map them to sample points
        PointCloud<Normal>::Ptr face_normals(new PointCloud<Normal>);
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setInputCloud(reconstructed_cloud);
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        ne.setSearchMethod(tree);
        ne.setKSearch(k_neighbors);
        ne.compute(*face_normals);

        // Update sample point normals
        for (size_t i = 0; i < sampled_cloud->points.size(); ++i)
        {
            // Find the k nearest faces of the sample point
            std::vector<int> k_nearest_faces;
            std::vector<float> k_nearest_distances;
            kdtree.nearestKSearch(sampled_cloud->points[i], k_neighbors, k_nearest_faces, k_nearest_distances);

            // Compute the average normal of the k nearest faces
            Eigen::Vector3f average_normal(0.0f, 0.0f, 0.0f);
            for (size_t j = 0; j < k_nearest_faces.size(); ++j)
            {
                average_normal += face_normals->points[k_nearest_faces[j]].getNormalVector3fMap();
            }
            average_normal.normalize();

            // Update the sample point normal
            sampled_cloud->points[i].normal_x = average_normal[0];
            sampled_cloud->points[i].normal_y = average_normal[1];
            sampled_cloud->points[i].normal_z = average_normal[2];
        }

        // Compute the average normal variation of the top 1/1000 points
        size_t heap_size = std::ceil(sampled_cloud->points.size() / 1000.0);
        std::priority_queue<double, std::vector<double>, std::greater<double>> min_heap;
        for (size_t i = 0; i < sampled_cloud->points.size(); ++i)
        {
                double diff = squaredEuclideanDistance(sampled_cloud->points[i], reconstructed_cloud->points[i]);
                if (min_heap.size() < heap_size)
                {
                    min_heap.push(diff);
                }
                else if (diff > min_heap.top())
                {
                    min_heap.pop();
                    min_heap.push(diff);
                }
        }

        heap_size = min_heap.size();
        double ave_max_diff = 0;
        while (!min_heap.empty())
        {
            ave_max_diff += std::sqrt(min_heap.top());
            min_heap.pop();
        }
        ave_max_diff /= heap_size;
        printf("normals variation %f\n", ave_max_diff);
        if (ave_max_diff < 0.175)
        {
            break;
        }
    }

    Poisson<PointT> final_poisson;
    final_poisson.setDepth(depth);
    final_poisson.setPointWeight(pointweight);
    final_poisson.setInputCloud(sampled_cloud);
    PolygonMesh mesh;

    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::copyPointCloud(*reconstructed_cloud, cloud_xyz);

    pcl::toPCLPointCloud2(cloud_xyz, mesh.cloud);
    final_poisson.reconstruct(mesh);

    pcl::io::savePLYFileBinary(output_name, *reconstructed_cloud);
}

int main(int argc, char* argv[])
{
	string input_name, output_name;
	int iters = 30;
	double pointweight = 10;
	int depth = 10;
	int k_neighbors = 10;
	for (int i = 1; i < argc; i += 2) {
		if (strcmp(argv[i], "--in") == 0) {
			input_name = argv[i + 1];
		}
		else if (strcmp(argv[i], "--out") == 0) {
			output_name = argv[i + 1];
		}
		else if (strcmp(argv[i], "--iters") == 0) {
			iters = std::atoi(argv[i + 1]);
		}
		else if (strcmp(argv[i], "--pointWeight") == 0) {
			pointweight = std::atof(argv[i + 1]);
		}
		else if (strcmp(argv[i], "--depth") == 0) {
			depth = std::atoi(argv[i + 1]);
		}
		else if (strcmp(argv[i], "--neighbors") == 0) {
			k_neighbors = std::atoi(argv[i + 1]);
		}
		else {
			std::cout << "Unknown parameter: " << argv[i] << std::endl;
			return 1;
		}
	}

	if (argc <= 1 || input_name.empty() || output_name.empty())
	{
		printf("Parameters:\n");
		printf("--in                      input .ply model\n");
		printf("--out                     output .ply model\n");
		printf("--iters (optional)        maximum number of iterations, default 30\n");
		printf("--pointWeight (optional)  screened weight of SPSR, default 10\n");
		printf("--depth (optional)        maximum depth of the octree, default 10\n");
		printf("--neighbors (optional)    number of the nearest neighbors to search, default 10\n");
		return 0;
	}

	printf("Iterative Poisson Surface Reconstruction (iPSR)\n");
	printf("Parameters:\n");
	printf("--in          %s\n", input_name.c_str());
	printf("--out         %s\n", output_name.c_str());
	printf("--iters       %d\n", iters);
	printf("--pointWeight %f\n", pointweight);
	printf("--depth       %d\n", depth);
	printf("--neighbors   %d\n\n", k_neighbors);

	ipsr(input_name, output_name, iters, pointweight, depth, k_neighbors);

	return 0;
}
