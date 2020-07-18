#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

using pcl::PointCloud;
using pcl::PointXYZ;

int 
main (int , char **)
{
  srand (unsigned (time (0)));

  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

  cloud->points.resize (5);
  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    (*cloud)[i].x = float (i); 
    (*cloud)[i].y = float (i / 2);
    (*cloud)[i].z = 0.0f;
  }

  // Start the visualizer
  pcl::visualization::PCLVisualizer p ("test_shapes");
  p.setBackgroundColor (1, 1, 1);
  p.addCoordinateSystem (1.0, "first");

  //p.addPolygon (cloud, "polygon");
  p.addPolygon<PointXYZ> (cloud, 1.0, 0.0, 0.0, "polygon", 0);
  p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "polygon");
  
  p.addLine<PointXYZ, PointXYZ> ((*cloud)[0], (*cloud)[1], 0.0, 1.0, 0.0);
  p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, "line");

  p.addSphere<PointXYZ> ((*cloud)[0], 1, 0.0, 1.0, 0.0);
  p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "sphere");
//  p.removePolygon ("poly");

  p.addText ("text", 200, 200, 1.0, 0, 0, "text");
  
  p.addText3D ("text3D", (*cloud)[0], 1.0, 1.0, 0.0, 0.0);
  p.spin ();
  p.removeCoordinateSystem ("first", 0);
  p.spin ();
  p.addCoordinateSystem (1.0, 5, 3, 1, "second");
  p.spin ();
  p.removeCoordinateSystem ("second", 0);
  p.spin ();
}
