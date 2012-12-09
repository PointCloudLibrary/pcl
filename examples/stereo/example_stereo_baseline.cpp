
#include <pcl/stereo/stereo_matching.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/io/pcd_io.h>

/** \brief Stereo Matching demo
  *
  * This demo loads a stereo image pair and computes the disparity map and related organized point cloud
  * using the PCL AdaptiveCost-2SO Stereo Matching algorithm or the PCL Block-based stereo matching algorithm (you have to comment it out, see the code)
  * Input pcds should be "stereo_left.pcd" and "stereo_right.pcd" which can be found in the test subfolder within trunk.
  * A rescaled version of the disparity map is displayed, as well as the point cloud.
  *
  * \author Federico Tombari (federico.tombari@unibo.it)
  * \ingroup stereo
  */

int 
main(int argc, char **argv)
{

  if (argc < 3)
  {
    std::cerr << "Needs two pcd input files." << std::endl;
    return (-1);
  }

  pcl::PointCloud<pcl::RGB>::Ptr left_cloud (new pcl::PointCloud<pcl::RGB>);
  pcl::PointCloud<pcl::RGB>::Ptr right_cloud (new pcl::PointCloud<pcl::RGB>);

  //Read pcd files
  pcl::PCDReader pcd;
  
  if (pcd.read (argv[1], *left_cloud) == -1)
    return (-1);

  if (pcd.read (argv[2], *right_cloud) == -1)
    return (-1);

  if (!left_cloud->isOrganized () || !right_cloud->isOrganized () || left_cloud->width != right_cloud->width || left_cloud->height != right_cloud->height)
  {
    std::cout << "Wrong stereo pair; please check input pcds .." << std::endl; 
    return 0;
  }

  //choice between the two algorithms:
  //pcl::AdaptiveCostSOStereoMatching stereo;
  pcl::BlockBasedStereoMatching stereo;

  stereo.setMaxDisparity(60);
  stereo.setXOffset(0);
  stereo.setRadius(5);

  //only needed for AdaptiveCostSOStereoMatching:
  //stereo.setSmoothWeak(20);
  //stereo.setSmoothStrong(100);
  //stereo.setGammaC(25);
  //stereo.setGammaS(10);

  stereo.setRatioFilter(20);
  stereo.setPeakFilter(0);

  stereo.setLeftRightCheck(true);
  stereo.setLeftRightCheckThreshold(1);

  stereo.setPreProcessing(true);

  stereo.compute(*left_cloud,  *right_cloud);

  stereo.medianFilter(4);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud( new pcl::PointCloud<pcl::PointXYZRGB> );

  stereo.getPointCloud(318.112200, 224.334900, 368.534700, 0.8387445, out_cloud, left_cloud);

  pcl::PointCloud<pcl::RGB>::Ptr vmap( new pcl::PointCloud<pcl::RGB> );
  stereo.getVisualMap(vmap);

  pcl::visualization::ImageViewer iv ("My viewer");
  iv.addRGBImage<pcl::RGB> (vmap); 
  //iv.addRGBImage<pcl::RGB> (left_cloud);
  //iv.spin (); // press 'q' to exit

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  
  //viewer->addPointCloud<pcl::PointXYZRGB> (out_cloud, "stereo");
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> intensity(out_cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (out_cloud, intensity, "stereo");

  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "stereo");
  viewer->initCameraParameters ();
  //viewer->spin();
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    iv.spinOnce (100); // press 'q' to exit
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
