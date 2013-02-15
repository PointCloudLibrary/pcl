#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

typedef pcl::PointXYZ Point;

void
CreateCylinderPoints (pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data, unsigned npoints,
                      double alpha, double h, double r)
{
  for (unsigned i = 0; i < npoints; i++)
  {
    double da = alpha * double (rand ()) / RAND_MAX;
    double dh = h * (double (rand ()) / RAND_MAX - 0.5);

    Point p;
    p.x = float (r * cos (da));
    p.y = float (r * sin (da));
    p.z = float (dh);

    data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
    cloud->push_back (p);
  }
}

int
main ()
{
  unsigned npoints (200);
  unsigned refinement (2);
  unsigned iterations (10);

  pcl::visualization::PCLVisualizer viewer ("Test: NURBS surface fitting");
  viewer.setSize (800, 600);

  // create point cloud
  pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>);
  pcl::on_nurbs::NurbsDataSurface data;
  CreateCylinderPoints (cloud, data.interior, npoints, M_PI, 1.0, 0.5);
  viewer.addPointCloud<Point> (cloud, "cloud_cylinder");

  // fit NURBS surface
  ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (3, &data);
  pcl::on_nurbs::FittingSurface fit (&data, nurbs);
//  fit.setQuiet (false);

  pcl::on_nurbs::FittingSurface::Parameter params;
  params.interior_smoothness = 0.1;
  params.interior_weight = 1.0;
  params.boundary_smoothness = 0.1;
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

  // triangulate NURBS surface
  nurbs = fit.m_nurbs;
  pcl::PolygonMesh mesh;
  std::string mesh_id = "mesh_nurbs";
  pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (nurbs, mesh, 128);
  viewer.addPolygonMesh (mesh, mesh_id);

  viewer.spin ();
  return 0;
}
