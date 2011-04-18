#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

using pcl::PointCloud;
using pcl::PointXYZRGB;
using pcl::Normal;
using pcl::PointXYZRGBNormal;

using pcl::PCDReader;
using pcl::PassThrough;
using pcl::NormalEstimation;
using pcl::KdTreeFLANN;

int 
  main (int argc, char **argv)
{
  srand (time (0));

  PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);

  PCDReader pcd;
  if (pcd.read (argv[1], *cloud) == -1)
    return (-1);

  // Filter the data
  std::cerr << "Filtering..." << std::endl;
  PassThrough<PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  PointCloud<PointXYZRGB>::Ptr cloud_filtered (new PointCloud<PointXYZRGB>);
  pass.filter (*cloud_filtered);

  // Estimate surface normals
  std::cerr << "Estimating normals..." << std::endl;
  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (20);
  KdTreeFLANN<PointXYZRGB>::Ptr tree (new KdTreeFLANN<PointXYZRGB>);
  ne.setSearchMethod (tree);
  PointCloud<Normal> normals;
  ne.compute (normals);

  // Concatenate points and normals
  PointCloud<PointXYZRGBNormal> cloud_normals;
  pcl::concatenateFields (cloud_filtered, normals, cloud_normals);

  // Start the visualizer
  pcl::visualization::PCLVisualizer p ("test");
  p.setBackgroundColor (1, 1, 1);
  p.addCoordinateSystem (0.1);

  pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal> color_handler (cloud_normals);
  // Geometry handler demo
  {
    std::cerr << "PointCloudGeometryHandlerSurfaceNormal demo." << std::endl;
    pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<PointXYZRGBNormal> geometry_handler (cloud_normals);
    p.addPointCloud (cloud_normals, geometry_handler, "cloud_normal");
    p.spin ();
    p.removePointCloud ("cloud_normal");

    p.addPointCloud (cloud_normals, color_handler, geometry_handler, "cloud_normal");
    p.spin ();
    p.removePointCloud ("cloud_normal");
  }

  {
    std::cerr << "PointCloudGeometryHandlerXYZ demo." << std::endl;
    pcl::visualization::PointCloudGeometryHandlerXYZ<PointXYZRGBNormal> geometry_handler (cloud_normals);
    p.addPointCloud (cloud_normals, color_handler, geometry_handler, "cloud_xyz");
    p.spin ();
    p.removePointCloud ("cloud_xyz");
  }

  {
    std::cerr << "PointCloudGeometryHandlerXYZ demo." << std::endl;
    pcl::visualization::PointCloudGeometryHandlerCustom<PointXYZRGBNormal> geometry_handler (cloud_normals, "x", "y", "z");
    p.addPointCloud (cloud_normals, color_handler, geometry_handler, "cloud_xyz");
    p.spin ();
    p.removePointCloud ("cloud_xyz");
    
    geometry_handler = pcl::visualization::PointCloudGeometryHandlerCustom<PointXYZRGBNormal> (cloud_normals, "x", "x", "y");
    p.addPointCloud (cloud_normals, color_handler, geometry_handler, "cloud_xyz");
    p.spin ();
    p.removePointCloud ("cloud_xyz");
   }
  p.spin ();
}
