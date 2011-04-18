#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGB Point;

int 
  main (int argc, char **argv)
{
  if (argc < 2)
  {
    std::cerr << "Needs a PCD file as input." << std::endl;
    return (-1);
  }

  srand (time (0));

  pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>);

  pcl::PCDReader pcd;
  if (pcd.read (argv[1], *cloud) == -1)
    return (-1);

  pcl::visualization::PCLVisualizer p ("test");
  p.setBackgroundColor (1, 1, 1);

  // Handler random color demo
  {
    std::cerr << "PointCloudColorHandlerRandom demo." << std::endl;
    pcl::visualization::PointCloudColorHandlerRandom<Point> handler (cloud);
    
    p.addPointCloud<Point> (cloud, "cloud_random");      // no need to add the handler, we use a random handler by default
    p.spin ();
    p.removePointCloud ("cloud_random");

    p.addPointCloud (cloud, handler, "cloud_random");
    p.spin ();
    p.removePointCloud ("cloud_random");
  }

  // Handler custom demo
  {
    std::cerr << "PointCloudColorHandlerCustom demo." << std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<Point> handler (cloud, 255, 0, 0);
    
    p.addPointCloud (cloud, handler);             // the default id is "cloud"
    p.spin ();
    p.removePointCloud ();                        // the default id is "cloud"

    handler = pcl::visualization::PointCloudColorHandlerCustom<Point> (cloud, 255, 0, 0);
    p.addPointCloud (cloud, handler, "cloud");
    p.spin ();
    p.removePointCloud ("cloud");
  }

  // Handler RGB demo
  {
    std::cerr << "PointCloudColorHandlerRGBField demo." << std::endl;
    pcl::visualization::PointCloudColorHandlerRGBField<Point> handler (cloud);

    p.addPointCloud (cloud, handler, "cloud_rgb");
    p.spin ();
    p.removePointCloud ("cloud_rgb");
   }
  
  // Handler generic field demo
  {
    std::cerr << "PointCloudColorHandlerGenericField demo." << std::endl;
    pcl::visualization::PointCloudColorHandlerGenericField<Point> handler_z (cloud, "z");
    pcl::visualization::PointCloudColorHandlerGenericField<Point> handler_x (cloud, "x");
    
    p.addPointCloud (cloud, handler_x, "cloud_x");
    p.spin ();
    p.removePointCloud ("cloud_x");
    
    p.addPointCloud (cloud, handler_z, "cloud_z");
    p.spin ();
    p.removePointCloud ("cloud_z");
  }

  p.addCoordinateSystem (0.1);
  
  // Demonstrate usage of spinOnce()
  p.resetStoppedFlag();
  while (!p.wasStopped())
  {
    static int counter = 0;
    cout << "spinOnce was called "<<++counter<<" times.\n";
    p.spinOnce(1000);  // Give the GUI 1000ms to handle events, then return
  }

  //p.removePointCloud ("cloud");
  //p.spin ();
}
