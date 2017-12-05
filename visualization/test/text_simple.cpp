#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char** argv)
{
  pcl::visualization::PCLVisualizer viz ("Visualizator");
  viz.addCoordinateSystem (1.0);

  viz.addText3D ("Following text", pcl::PointXYZ(0.0, 0.0, 0.0),
                 1.0, 1.0, 0.0, 0.0, "id_following");
  viz.spin ();
  double orientation[3] = {0., 0., 0.};
  viz.addText3D ("Fixed text", pcl::PointXYZ(0.0, 0.0, 0.0), orientation,
                 1.0, 0.0, 1.0, 0.0, "id_fixed");
  viz.spin ();
  viz.removeText3D ("id_following");
  viz.spin ();
  viz.removeText3D ("id_fixed");
  viz.spin ();

  return (0);
}
