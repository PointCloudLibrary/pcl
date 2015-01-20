#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// The GPU specific stuff here
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>

#include <time.h>

int 
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDWriter writer;
  reader.read (argv[1], *cloud_filtered);

/////////////////////////////////////////////
/// CPU VERSION
/////////////////////////////////////////////

  std::cout << "INFO: PointCloud_filtered still has " << cloud_filtered->points.size() << " Points " << std::endl;
  clock_t tStart = clock();
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud( cloud_filtered);
  ec.extract (cluster_indices);
  
  printf("CPU Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

/////////////////////////////////////////////
/// GPU VERSION
/////////////////////////////////////////////

  std::cout << "INFO: starting with the GPU version" << std::endl;

  tStart = clock();

  pcl::gpu::Octree::PointCloud cloud_device;
  cloud_device.upload(cloud_filtered->points);
  
  pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
  octree_device->setCloud(cloud_device);
  octree_device->build();

  std::vector<pcl::PointIndices> cluster_indices_gpu;
  pcl::gpu::EuclideanClusterExtraction gec;
  gec.setClusterTolerance (0.02); // 2cm
  gec.setMinClusterSize (100);
  gec.setMaxClusterSize (25000);
  gec.setSearchMethod (octree_device);
  gec.setHostCloud( cloud_filtered);
  gec.extract (cluster_indices_gpu);
//  octree_device.clear();

  printf("GPU Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  std::cout << "INFO: stopped with the GPU version" << std::endl;

  j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin (); it != cluster_indices_gpu.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_gpu (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster_gpu->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster_gpu->width = cloud_cluster_gpu->points.size ();
    cloud_cluster_gpu->height = 1;
    cloud_cluster_gpu->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster_gpu->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "gpu_cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster_gpu, false); //*
    j++;
  }

  return (0);
}

