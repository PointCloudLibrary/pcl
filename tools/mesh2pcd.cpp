/*
 * mesh2pcd.cpp
 *
 *  Created on: Aug 23, 2011
 *      Author: aitor
 */

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include "vtkSmartPointer.h"
#include "vtkPLYReader.h"
#include "vtkPolyData.h"
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv) {
  vtkSmartPointer < vtkPLYReader > readerQuery = vtkSmartPointer<vtkPLYReader>::New ();
  readerQuery->SetFileName (argv[1]);
  vtkSmartPointer < vtkPolyData > polydata1 = readerQuery->GetOutput ();
  polydata1->Update ();

  int tesselated_sphere_level = atoi(argv[2]);
  int resolution = atoi(argv[3]);
  double leaf_size = atof(argv[4]);

  bool INTER_VIS = false;

  pcl::visualization::PCLVisualizer vis;
  vis.addModelFromPolyData (polydata1, "mesh1", 0);
  vis.setRepresentationToSurfaceForAllActors ();

  std::vector < pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > views_xyz;
  std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
  std::vector<float> enthropies;
  vis.renderViewTesselatedSphere (resolution, resolution, views_xyz, poses, enthropies, tesselated_sphere_level);

  //take views and fuse them together
  Eigen::Matrix4f first_pose;
  first_pose = poses[0];

  std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr > aligned_clouds;

  for(size_t i=0; i < views_xyz.size(); i++) {
    pcl::PointCloud < pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud < pcl::PointXYZ >());
    Eigen::Matrix4f pose_inverse;
    pose_inverse = poses[i].inverse();
    pcl::transformPointCloud(views_xyz[i], *cloud, pose_inverse);
    aligned_clouds.push_back(cloud);
  }

  if(INTER_VIS) {
    pcl::visualization::PCLVisualizer vis2("visualize");

    for(size_t i=0; i < aligned_clouds.size(); i++) {
      std::stringstream name;
      name << "cloud_" << i;
      vis2.addPointCloud(aligned_clouds[i],name.str());
      vis2.spin();
    }
  }

  //fuse clouds
  pcl::PointCloud < pcl::PointXYZ >::Ptr big_boy(new pcl::PointCloud < pcl::PointXYZ >());
  for(size_t i=0; i < aligned_clouds.size(); i++) {
    *big_boy += *aligned_clouds[i];
  }

  {
    pcl::visualization::PCLVisualizer vis2("visualize");
    vis2.addPointCloud(big_boy);
    vis2.spin();
  }

  //voxelgrid
  pcl::VoxelGrid<pcl::PointXYZ> grid_;
  grid_.setInputCloud (big_boy);
  grid_.setLeafSize (leaf_size, leaf_size, leaf_size);
  grid_.filter (*big_boy);

  {
    pcl::visualization::PCLVisualizer vis2("visualize");
    vis2.addPointCloud(big_boy);
    vis2.spin();
  }

  //save
  pcl::io::savePCDFileASCII("out.pcd",*big_boy);
}
