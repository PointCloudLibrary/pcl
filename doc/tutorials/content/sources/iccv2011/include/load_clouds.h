#ifndef IO_H_
#define IO_H_

#include "typedefs.h"

#include <pcl/io/pcd_io.h>
  
template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT> >
loadPointCloud (std::string filename, std::string suffix)
{
  boost::shared_ptr<pcl::PointCloud<PointT> > output (new pcl::PointCloud<PointT>);
  filename.append (suffix);
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

PointCloudPtr
loadPoints (std::string filename)
{
  PointCloudPtr output (new PointCloud);
  filename.append ("_points.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

SurfaceNormalsPtr
loadSurfaceNormals(std::string filename)
{
  SurfaceNormalsPtr output (new SurfaceNormals);
  filename.append ("_normals.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

PointCloudPtr
loadKeypoints (std::string filename)
{
  PointCloudPtr output (new PointCloud);
  filename.append ("_keypoints.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

LocalDescriptorsPtr
loadLocalDescriptors (std::string filename)
{
  LocalDescriptorsPtr output (new LocalDescriptors);
  filename.append ("_localdesc.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

GlobalDescriptorsPtr
loadGlobalDescriptors (std::string filename)
{
  GlobalDescriptorsPtr output (new GlobalDescriptors);
  filename.append ("_globaldesc.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}


#endif
