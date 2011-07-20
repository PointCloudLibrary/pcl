#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int
 main(int argc, char** argv)
 {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width    = 500;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
    if( i % 8 == 0)
      cloud->points[i].z = 1024*rand () / (RAND_MAX + 1.0);
    else
      cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
  }
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model);
  ransac.setDistanceThreshold (.01);
  ransac.computeModel();

  std::vector<int> inliers;
  ransac.getInliers(inliers);

  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
  if (pcl::console::find_argument (argc, argv, "-f") >= 0)
    viewer = simpleVis(final);
  else
    viewer = simpleVis(cloud);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;
 }
