#include <pcl_visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

using pcl::PointCloud;
using pcl::PointXYZ;

int 
  main (int argc, char **argv)
{
  srand (time (0));

  PointCloud<PointXYZ> cloud;

  cloud.points.resize (5);
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = i; cloud.points[i].y = i / 2; cloud.points[i].z = 0;
  }

  // Start the visualizer
  pcl_visualization::PCLVisualizer p ("test_shapes");
  p.setBackgroundColor (1, 1, 1);
  p.addCoordinateSystem (0.1);

  //p.addPolygon (cloud, "polygon");
  p.addPolygon (cloud, 1.0, 0.0, 0.0, "polygon", 0);
  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "polygon");
  
  p.addLine<PointXYZ, PointXYZ> (cloud.points[0], cloud.points[1], 0.0, 1.0, 0.0);
  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 50, "line");

  p.addSphere<PointXYZ> (cloud.points[0], 1, 0.0, 1.0, 0.0);
  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "sphere");
//  p.removePolygon ("poly");

  p.spin ();
}
