#include <string>
#include <pcl/ros/register_point_struct.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include <vtkMath.h>
#include <vtkGeneralTransform.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkCellLocator.h>
#include <vtkPolyData.h>
#include <boost/random.hpp>


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

using namespace pcl;


int
main (int argc, char ** argv)
{
  pcl::PCDReader reader;
  sensor_msgs::PointCloud2 cloud;
  reader.read (argv[1], cloud);

  pcl::PointCloud<pcl::PointXYZ> xyz;
  pcl::fromROSMsg (cloud, xyz);

  pcl::visualization::ImageViewer depth_image_viewer_;
  float* img = new float[cloud.width * cloud.height];
  for (int i = 0; i < xyz.points.size (); ++i)
    img[i] = xyz.points[i].z;

  depth_image_viewer_.showFloatImage (img,
                                      cloud.width, cloud.height,
                                      std::numeric_limits<float>::min (),
                                      // Scale so that the colors look brigher on screen
                                      std::numeric_limits<float>::max () / 10,
                                      true);
  depth_image_viewer_.spin ();
  return (0);
}
