#include <pcl/surface/on_nurbs/fitting_curve_2d_pdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer viewer ("Boundary Fitting PDM");

void
PointCloud2Vector2d (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec2d &data)
{
  for (unsigned i = 0; i < cloud->size (); i++)
  {
    pcl::PointXYZ &p = cloud->at (i);
    if (!isnan (p.x) && !isnan (p.y))
      data.push_back (Eigen::Vector2d (p.x, p.y));
  }
}

void
VisualizeCurve (ON_NurbsCurve &curve)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::on_nurbs::Triangulation::convertCurve2PointCloud (curve, cloud, 8);

  for (std::size_t i = 0; i < cloud->size () - 1; i++)
  {
    pcl::PointXYZRGB &p1 = cloud->at (i);
    pcl::PointXYZRGB &p2 = cloud->at (i + 1);
    std::ostringstream os;
    os << "line" << i;
    viewer.addLine<pcl::PointXYZRGB> (p1, p2, 1.0, 0.0, 0.0, os.str ());
  }
}

int
main (int argc, char *argv[])
{
  std::string pcd_file;

  if (argc > 1)
  {
    pcd_file = argv[1];
  }
  else
  {
    printf ("\nUsage: boundaryFittingPDM pcd-file \n\n");
    printf ("  pcd-file    point-cloud file containing the boundary points (xy)\n");
    exit (0);
  }

  // #################### LOAD FILE #########################
  printf ("  loading %s\n", pcd_file.c_str ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 cloud2;

  if (pcl::io::loadPCDFile (pcd_file, cloud2) == -1)
    throw std::runtime_error ("  PCD file not found.");

  fromROSMsg (cloud2, *cloud);

  viewer.setSize (800, 600);
  viewer.addPointCloud<pcl::PointXYZ> (cloud, "cloud");

  pcl::on_nurbs::NurbsDataCurve2d data;
  PointCloud2Vector2d (cloud, data.interior);

  // #################### CURVE PARAMETERS #########################
  unsigned order (3);
  unsigned n_control_points (20);

  pcl::on_nurbs::FittingCurve2d::FitParameter curve_params;
  curve_params.addCPsAccuracy = 1000;   // no control points added
  curve_params.addCPsIteration = 1000;  // no control points added
  curve_params.maxCPs = 1000;
  curve_params.fitMaxError = 1e-6;
  curve_params.fitAvgError = 1e-8;
  curve_params.fitMaxSteps = 50;
  curve_params.refinement = 0;

  curve_params.param.closest_point_resolution = 0;
  curve_params.param.closest_point_weight = 0.0;
  curve_params.param.closest_point_sigma2 = 0.0;
  curve_params.param.interior_sigma2 = 0.0;
  curve_params.param.smooth_concavity = 0.0;
  curve_params.param.smoothness = 0.000001;

  data.interior_weight_function.push_back (false);

  // #################### CURVE FITTING #########################
  ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2d::initNurbsCurve2D (order, data.interior, n_control_points);
  pcl::on_nurbs::FittingCurve2d curve_fit (&data, curve);
  curve_fit.fitting (curve_params);

  VisualizeCurve (curve_fit.m_nurbs);

  viewer.spin ();
  return 0;
}

