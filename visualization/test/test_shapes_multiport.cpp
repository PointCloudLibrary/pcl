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
  int leftPort(0);
  int rightPort(0);
  p.createViewPort(0, 0, 0.5, 1, leftPort);
  p.createViewPort(0.5, 0, 1, 1, rightPort);
  p.setBackgroundColor (1, 1, 1);
  p.addCoordinateSystem (1.0, "first");

  //p.addPolygon (cloud, "polygon");
  p.addPolygon<PointXYZ> (cloud, 1.0, 0.0, 0.0, "polygon", leftPort);
  p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "polygon", leftPort);
  
  p.addLine<PointXYZ, PointXYZ> ((*cloud)[0], (*cloud)[1], 0.0, 1.0, 0.0, "line", leftPort);
  p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 50, "line", leftPort);

  p.addSphere<PointXYZ> ((*cloud)[0], 1, 0.0, 1.0, 0.0, "sphere", leftPort);
  p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "sphere", leftPort);
//  p.removePolygon ("poly");

  p.addText ("text", 200, 200, 1.0, 0, 0, "text", leftPort);
  
  p.addText3D ("text3D", (*cloud)[0], 1.0, 1.0, 0.0, 0.0, "", rightPort);
  p.spin ();
  p.removeCoordinateSystem ("first", 0);
  p.spin ();
  p.addCoordinateSystem (1.0, 5, 3, 1, "second");
  p.spin ();
  p.removeCoordinateSystem ("second", 0);
  p.spin ();
  p.addText3D ("text3D_to_remove", (*cloud)[1], 1.0, 0.0, 1.0, 0.0, "", rightPort);
  p.spin ();
  p.removeText3D ("text3D_to_remove", rightPort);
  p.spin ();
}
