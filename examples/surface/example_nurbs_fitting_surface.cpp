#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

typedef pcl::PointXYZ Point;

void
PointCloud2Vector3d (pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
  for (unsigned i = 0; i < cloud->size(); i++)
  {
    Point &p = cloud->at(i);
    data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
  }
}

int
main (int argc, char *argv[])
{
  std::string pcd_file;

  if (argc < 2) {
    printf("\nUsage: pcl_example_nurbs_fitting_surface pcd<PointXYZ>-file\n\n");
    exit(0);
  }

  pcd_file = argv[1];

  unsigned refinement (5);
  unsigned iterations (10);

  pcl::visualization::PCLVisualizer viewer ("Test: NURBS surface fitting");
  viewer.setSize (800, 600);

  // ############################################################################
  // load point cloud
  pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>);
  sensor_msgs::PointCloud2 cloud2;
  pcl::on_nurbs::NurbsDataSurface data;

  printf("  loading %s\n", pcd_file.c_str());
  if (pcl::io::loadPCDFile(pcd_file, cloud2) == -1)
    throw std::runtime_error("  PCD file not found.");

  fromROSMsg(cloud2, *cloud);
  PointCloud2Vector3d(cloud, data.interior);
  viewer.addPointCloud<Point> (cloud, "cloud_cylinder");
  printf("  %d points in data set\n", cloud->size());

  // ############################################################################
  // fit NURBS surface
  ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (3, &data);
  pcl::on_nurbs::FittingSurface fit (&data, nurbs);
  fit.setQuiet (false);

  pcl::on_nurbs::FittingSurface::Parameter params;
  params.interior_smoothness = 0.2;
  params.interior_weight = 1.0;
  params.boundary_smoothness = 0.2;
  params.boundary_weight = 0.0;

  // NURBS refinement
  for (unsigned i = 0; i < refinement; i++)
  {
    fit.refine (0);
    fit.refine (1);
  }

  // fitting iterations
  for (unsigned i = 0; i < iterations; i++)
  {
    fit.assemble (params);
    fit.solve ();
  }

  // ############################################################################
  // triangulate NURBS surface
  nurbs = fit.m_nurbs;
  pcl::PolygonMesh mesh;
  std::string mesh_id = "mesh_nurbs";
  pcl::on_nurbs::Triangulation::convert (nurbs, mesh, 128);
  viewer.addPolygonMesh (mesh, mesh_id);

  viewer.spin ();
  return 0;
}
