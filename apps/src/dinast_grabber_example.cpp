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

void
convertImageToCloud (const unsigned char *image, pcl::PointCloud<pcl::PointXYZI> &cloud)
{
  cloud.points.resize (IMAGE_WIDTH * IMAGE_HEIGHT);
  cloud.width = IMAGE_WIDTH;
  cloud.height = IMAGE_HEIGHT;
  cloud.is_dense = false;

  int depth_idx = 0;
  int pxl[9];

  for (int x = 0; x < cloud.width; ++x)
  {
    for (int y = 0; y < cloud.height; ++y, ++depth_idx)
    {
      float xc = (float)(x - 160);
      float yc = (float)(y - 120);
      double r1 = sqrt (xc * xc + yc * yc);
      double r2 = r1 * r1;
      double r3 = r1 * r2;
      double A = -2e-5 * r3 + 0.004 * r2 + 0.1719 * r1 + 350.03;
      double B = -2e-9 * r3 + 3e-7 * r2 - 1e-5 * r1 - 0.01;

      // Low pass filtering
      /// @todo Try a bilateral filter to avoid blurring over depth boundaries
      int measure = 0;
      if ((y > 0) && (y < (IMAGE_HEIGHT - 1)) && (x > 0) && (x < (IMAGE_WIDTH - 1)))
      {
        int ipx = x + IMAGE_WIDTH * y;
#if 1
        pxl[0] = image[ipx];
        pxl[1] = image[ipx-1];
        pxl[2] = image[ipx+1];
        pxl[3] = image[ipx - IMAGE_WIDTH];
        pxl[4] = image[ipx - IMAGE_WIDTH - 1];
        pxl[5] = image[ipx - IMAGE_WIDTH + 1];
        pxl[6] = image[ipx + IMAGE_WIDTH];
        pxl[7] = image[ipx + IMAGE_WIDTH - 1];
        pxl[8] = image[ipx + IMAGE_WIDTH + 1];

        for (int ii = 0; ii < 9; ii++) 
          measure += pxl[ii];
        measure /= 9;
#else
        // No blurring
        measure = image[ipx];
#endif
      }
      if (measure > 255)
        measure = 255;  // saturation for display

      unsigned char pixel = measure;//image[depth_idx];
      if (pixel < 1)
      {
        cloud.points[depth_idx].x = std::numeric_limits<float>::quiet_NaN ();
        cloud.points[depth_idx].y = std::numeric_limits<float>::quiet_NaN ();
        cloud.points[depth_idx].z = std::numeric_limits<float>::quiet_NaN ();
        cloud.points[depth_idx].intensity = pixel;
        continue;
      }

      if (pixel > A)
        pixel = A;

      float dy = y*0.1;
      double dist = (log((double)pixel/A)/B-dy)*(7E-07*r3 - 0.0001*r2 + 0.004*r1 + 0.9985)*1.5;
      double dist_2d = r1;

      static const double dist_max_2d = 1 / 160.0; /// @todo Why not 200?
      //static const double dist_max_2d = 1 / 200.0;
      static const double FOV = 64.0 * M_PI / 180.0; // diagonal FOV?
  
      double theta_colati = FOV * r1 * dist_max_2d;
      double c_theta = cos (theta_colati);
      double s_theta = sin (theta_colati);
      double c_ksai = ((double)(x - 160.)) / r1;
      double s_ksai = ((double)(y - 120.)) / r1;

      cloud.points[depth_idx].x = (dist * s_theta * c_ksai) / 500.0 + 0.5; //cartesian x
      cloud.points[depth_idx].y = (dist * s_theta * s_ksai) / 500.0 + 0.5; //cartesian y
      cloud.points[depth_idx].z = (dist * c_theta);                        //cartesian z
      /// @todo This looks weird, can it cause artifacts?
      if (cloud.points[depth_idx].z < 0.01)
#if 1
        cloud.points[depth_idx].z = 0.01;
#else
        cloud.points[depth_idx].x = std::numeric_limits<float>::quiet_NaN ();
        cloud.points[depth_idx].y = std::numeric_limits<float>::quiet_NaN ();
        cloud.points[depth_idx].z = std::numeric_limits<float>::quiet_NaN ();
        cloud.points[depth_idx].intensity = pixel;
        continue;
#endif

      cloud.points[depth_idx].z /= 500.0;
      cloud.points[depth_idx].intensity = pixel;
    }
  }
}

/* --[ */ 

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

  unsigned char *img1 = (unsigned char*)malloc (IMAGE_SIZE);
  unsigned char *img2 = (unsigned char*)malloc (IMAGE_SIZE);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  
  while (true)
  {
    if (grabber.readImage ( img1, img2) == 0)
      continue;

     convertImageToCloud (img1, *cloud);
    
    
    FPS_CALC ("grabber + visualization");
    vis_img.showMonoImage (img1, IMAGE_WIDTH, IMAGE_HEIGHT);
    
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
