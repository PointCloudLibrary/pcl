#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char** argv)
{
  pcl::visualization::PCLVisualizer viz ("Visualizator");
  int leftPort (0);
  int rightPort (0);

  viz.createViewPort (0, 0, 0.5, 1, leftPort);
  viz.createViewPort (0.5, 0, 1, 1, rightPort);

  viz.addCoordinateSystem (1.0);

  viz.addText3D ("Following text", pcl::PointXYZ(0.0, 0.0, 0.0),
                 1.0, 1.0, 0.0, 0.0, "id_following", leftPort);
  viz.spin ();
  double orientation[3] = {0., 0., 0.};
  viz.addText3D ("Fixed text", pcl::PointXYZ(0.0, 0.0, 0.0), orientation,
                 1.0, 0.0, 1.0, 0.0, "id_fixed", rightPort);
  viz.spin ();
  viz.removeText3D ("id_following", leftPort);
  viz.spin ();
  viz.removeText3D ("id_fixed", rightPort);
  viz.spin ();

  return (0);
}
