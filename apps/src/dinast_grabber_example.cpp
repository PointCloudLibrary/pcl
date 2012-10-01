#include <string>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <cstdio>

#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/dinast_grabber.h>

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

void
savePGM (const unsigned char *image, const std::string& filename)
{
  FILE *file = fopen (filename.c_str (), "w");
  if (!file)
  {
    std::cerr << "Unable to open file '" << filename << "' for writing." << std::endl;
    return;
  }

  // PGM header
  fprintf (file, "P2\n%d %d\n255\n", IMAGE_WIDTH, IMAGE_HEIGHT);

  // Write data as ASCII
  for (int i = 0; i < IMAGE_HEIGHT; ++i)
  {
    for (int j = 0; j < IMAGE_WIDTH; ++j)
    {
      fprintf (file, "%3d ", (int)*image++);
    }
    fprintf (file, "\n");
  }

  fclose (file);
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* data)
{
  static int NUM_IMAGES = 0;
  if (event.getKeySym () == "s" && event.keyDown ())
  {
    char filename[16];
    snprintf (filename, sizeof(filename), "image%.2d.pgm", NUM_IMAGES++);
    unsigned char *image = reinterpret_cast<unsigned char*> (data);
    savePGM (image, filename);
    printf ("Wrote %s\n", filename);
  }
}

int
main (int argc, char** argv) 
{

  pcl::DinastGrabber grabber;
  
  grabber.findDevice (1);
  
  grabber.openDevice();

  std::cerr << "Device version/revision number: " << grabber.getDeviceVersion () << std::endl;
  
  grabber.start ();

  pcl::visualization::ImageViewer vis_img ("Dinast Image Viewer");
  pcl::visualization::PCLVisualizer vis_cld (argc, argv, "Dinast Cloud Viewer");

  unsigned char *image = (unsigned char*)malloc (IMAGE_SIZE);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  
  while (true)
  {

    grabber.getData(image, cloud);
    
    
    FPS_CALC ("grabber + visualization");
    vis_img.showMonoImage (image, IMAGE_WIDTH, IMAGE_HEIGHT);
    
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler (cloud, "intensity");
    if (!vis_cld.updatePointCloud (cloud, handler, "DinastCloud"))
    {
      vis_cld.addPointCloud (cloud, handler, "DinastCloud");
      vis_cld.resetCameraViewpoint ("DinastCloud");
    }

    vis_img.spinOnce ();
    vis_cld.spinOnce ();
    
  }
  
  grabber.stop ();
  grabber.closeDevice ();

}
