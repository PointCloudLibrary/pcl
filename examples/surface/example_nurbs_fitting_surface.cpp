#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_sdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

typedef pcl::PointXYZ Point;

void
PointCloud2Vector3d (pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
  for (unsigned i = 0; i < cloud->size (); i++)
  {
    Point &p = cloud->at (i);
    if (!isnan (p.x) && !isnan (p.y) && !isnan (p.z))
      data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
  }
}

int
main (int argc, char *argv[])
{
  std::string pcd_file;

  if (argc < 2)
  {
    printf ("\nUsage: pcl_example_nurbs_fitting_surface pcd<PointXYZ>-file\n\n");
    exit (0);
  }

  pcd_file = argv[1];

  unsigned order (3);
  unsigned refinement (5);
  unsigned iterations (10);
  unsigned mesh_resolution (256);

  pcl::visualization::PCLVisualizer viewer ("Test: NURBS surface fitting");
  viewer.setSize (800, 600);

  // ############################################################################
  // load point cloud
  printf ("  loading %s\n", pcd_file.c_str ());
  pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>);
  sensor_msgs::PointCloud2 cloud2;
  pcl::on_nurbs::NurbsDataSurface data;

  if (pcl::io::loadPCDFile (pcd_file, cloud2) == -1)
    throw std::runtime_error ("  PCD file not found.");

  fromROSMsg (cloud2, *cloud);
  PointCloud2Vector3d (cloud, data.interior);
  pcl::visualization::PointCloudColorHandlerCustom<Point> handler (cloud, 0, 255, 0);
  viewer.addPointCloud<Point> (cloud, handler, "cloud_cylinder");
  printf ("  %d points in data set\n", cloud->size ());

  // ############################################################################
  // fit NURBS surface
  printf ("  surface fitting ...\n");
  ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data);
  pcl::on_nurbs::FittingSurface fit (&data, nurbs);
  //  fit.setQuiet (false);

  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> mesh_vertices;

  std::string mesh_id = "mesh_nurbs";
  pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (fit.m_nurbs, mesh, mesh_resolution);
  viewer.addPolygonMesh (mesh, mesh_id);

  pcl::on_nurbs::FittingSurface::Parameter params;
  params.interior_smoothness = 0.15;
  params.interior_weight = 1.0;
  params.boundary_smoothness = 0.15;
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
    viewer.spinOnce ();
    fit.assemble (params);
    fit.solve ();
    pcl::on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
    viewer.updatePolygonMesh<pcl::PointXYZ> (mesh_cloud, mesh_vertices, mesh_id);
  }

  // ############################################################################
  // fit NURBS curve
  printf ("  boundary fitting ...\n");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::on_nurbs::FittingCurve2dSDM::FitParameter curve_params;
  curve_params.addCPsAccuracy = 5e-2;
  curve_params.addCPsIteration = 2;
  curve_params.maxCPs = 200;
  curve_params.fitMaxError = 5e-2;
  curve_params.fitAvgError = 1e-4;
  curve_params.fitMaxSteps = 20;
  curve_params.refinement = 2;

  curve_params.param.closest_point_resolution = 0;
  curve_params.param.closest_point_weight = 1.0;
  curve_params.param.closest_point_sigma2 = 0.1;
  curve_params.param.interior_sigma2 = 0.00005;
  curve_params.param.smooth_concavity = 1.0;
  curve_params.param.smoothness = 0.5;

  pcl::on_nurbs::NurbsDataCurve2d curve_data;
  curve_data.interior = data.interior_param;
  curve_data.interior_weight_function.push_back (true);

  ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2d::initNurbsCurve2D (order, curve_data.interior);

  pcl::on_nurbs::FittingCurve2dSDM curve_fit (&curve_data, curve_nurbs);
  //  curve_fit.setQuiet (false);

  curve_fit.fitting (curve_params);

  // visualisation
  //  pcl::on_nurbs::Triangulation::convertCurve2PointCloud (curve_fit.m_nurbs, fit.m_nurbs, curve_cloud, 8);
  //  viewer.addPointCloud (curve_cloud);
  //  for (std::size_t i = 0; i < curve_cloud->size () - 1; i++)
  //  {
  //    pcl::PointXYZRGB &p1 = curve_cloud->at (i);
  //    pcl::PointXYZRGB &p2 = curve_cloud->at (i + 1);
  //    std::ostringstream os;
  //    os << "line" << i;
  //    viewer.addLine<pcl::PointXYZRGB> (p1, p2, 1.0, 0.0, 0.0, os.str ());
  //  }

  // ############################################################################
  // triangulation of trimmed surface
  printf ("  triangulate trimmed surface ...\n");
  viewer.removePolygonMesh (mesh_id);
  pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh (fit.m_nurbs, curve_fit.m_nurbs, mesh,
                                                                   mesh_resolution);
  viewer.addPolygonMesh (mesh, mesh_id);

  printf ("  ... done.\n");

  viewer.spin ();
  return 0;
}
