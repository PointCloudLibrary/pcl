#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width    = 500;
  cloud.height   = 200;
  cloud.is_dense = false;

  for (size_t w = 0; w < cloud.width; ++w)
  {
    for (size_t h = 0; h < cloud.height; ++h)
    {
      pcl::PointXYZ p;
      p.x = w; p.y = h; p.z = 1;
      cloud.points.push_back(p);
    }
  }

  pcl::io::savePCDFileASCII ("input.pcd", cloud);
  std::cout << "INFO: Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;
  
  pcl::gpu::Octree::PointCloud cloud_device;
  cloud_device.upload(cloud.points);
  
  pcl::gpu::Octree octree_device;
  octree_device.setCloud(cloud_device);
  octree_device.build();
 
  // Create two query points
  std::vector<pcl::PointXYZ> query_host;
  query_host.resize (3);
  query_host[0].x = 250;
  query_host[0].y = 100;
  query_host[0].z = 1;
  query_host[1].x = 0;
  query_host[1].y = 0;
  query_host[1].z = 1;
  query_host[2].x = 500;
  query_host[2].y = 200;
  
  pcl::gpu::Octree::Queries queries_device;
  queries_device.upload(query_host);

  // Take two identical radiuses
  std::vector<float> radius;
  radius.push_back(10.0);
  radius.push_back(10.0);
  radius.push_back(10.0);

  pcl::gpu::Octree::Radiuses radiuses_device;
  radiuses_device.upload(radius);

  const int max_answers = 500*200;
  
  // Output buffer on the device
  pcl::gpu::NeighborIndices result_device(queries_device.size(), max_answers);
  
  // Do the actual search
  octree_device.radiusSearch(queries_device, radiuses_device, max_answers, result_device);

  std::vector<int> sizes, data;
  result_device.sizes.download(sizes);
  result_device.data.download(data);

  std::cout << "INFO: Data generated" << std::endl;
  
  std::cout<< "INFO: found : " << data.size() << " data.size" << std::endl;
  std::cout<< "INFO: found : " << sizes.size() << " sizes.size" << std::endl;

  for (size_t i = 0; i < sizes.size (); ++i)
  {
    std::cout << "INFO: sizes : " << i << " size " << sizes[i] << std::endl;
    if(sizes[i] != 0)
    {
      pcl::PointCloud<pcl::PointXYZ> cloud_result;
      // Fill in the cloud data
      cloud_result.height   = 1;
      cloud_result.is_dense = false;

      for (size_t j = 0; j < sizes[i] ; ++j)
      {
        cloud_result.points.push_back(cloud.points[data[j + i * max_answers]]);
        std::cout << "INFO: data : " << j << " " << j + i * max_answers << " data " << data[j+ i * max_answers] << std::endl;
      }
      std::stringstream ss;
      ss << "cloud_cluster_" << i << ".pcd";
      cloud_result.width    = cloud_result.points.size();
      pcl::io::savePCDFileASCII (ss.str(), cloud_result);
      std::cout << "INFO: Saved " << cloud_result.points.size () << " data points to " << ss.str() << std::endl;
    }
  }
  return 0;
}
