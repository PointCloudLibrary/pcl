#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/recognition/implicit_shape_model.h>
#include <pcl/recognition/impl/implicit_shape_model.hpp>

int
main (int argc, char** argv)
{
  if (argc == 0 || argc % 2 == 0)
    return (-1);

  unsigned int number_of_training_clouds = (argc - 3) / 2;

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setRadiusSearch (25.0);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> training_clouds;
  std::vector<pcl::PointCloud<pcl::Normal>::Ptr> training_normals;
  std::vector<unsigned int> training_classes;

  for (unsigned int i_cloud = 0; i_cloud < number_of_training_clouds - 1; i_cloud++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tr_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[i_cloud * 2 + 1], *tr_cloud) == -1 )
      return (-1);

    pcl::PointCloud<pcl::Normal>::Ptr tr_normals (new pcl::PointCloud<pcl::Normal>);
    normal_estimator.setInputCloud (tr_cloud);
    normal_estimator.compute (*tr_normals);

    unsigned int tr_class = static_cast<unsigned int> (strtol (argv[i_cloud * 2 + 2], 0, 10));

    training_clouds.push_back (tr_cloud);
    training_normals.push_back (tr_normals);
    training_classes.push_back (tr_class);
  }

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >::Ptr fpfh
    (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >);
  fpfh->setRadiusSearch (30.0);
  pcl::Feature< pcl::PointXYZ, pcl::Histogram<153> >::Ptr feature_estimator(fpfh);

  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal> ism;
  ism.setFeatureEstimator(feature_estimator);
  ism.setTrainingClouds (training_clouds);
  ism.setTrainingNormals (training_normals);
  ism.setTrainingClasses (training_classes);
  ism.setSamplingSize (2.0f);

  pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model (new pcl::features::ISMModel);
  ism.trainISM (model);

  std::string file ("trained_ism_model.txt");
  model->saveModelToFile (file);

  model->loadModelFromfile (file);

  unsigned int testing_class = static_cast<unsigned int> (strtol (argv[argc - 1], 0, 10));
  pcl::PointCloud<pcl::PointXYZ>::Ptr testing_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[argc - 2], *testing_cloud) == -1 )
    return (-1);

  pcl::PointCloud<pcl::Normal>::Ptr testing_normals (new pcl::PointCloud<pcl::Normal>);
  normal_estimator.setInputCloud (testing_cloud);
  normal_estimator.compute (*testing_normals);

  pcl::features::ISMVoteList<pcl::PointXYZ>::Ptr vote_list = ism.findObjects (
    model,
    testing_cloud,
    testing_normals,
    testing_class);

  double radius = model->sigmas_[testing_class] * 10.0;
  double sigma = model->sigmas_[testing_class];
  std::vector<pcl::ISMPeak, Eigen::aligned_allocator<pcl::ISMPeak> > strongest_peaks;
  vote_list->findStrongestPeaks (strongest_peaks, testing_class, radius, sigma);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  colored_cloud->height = 0;
  colored_cloud->width = 1;

  pcl::PointXYZRGB point;
  point.r = 255;
  point.g = 255;
  point.b = 255;

  for (std::size_t i_point = 0; i_point < testing_cloud->size (); i_point++)
  {
    point.x = (*testing_cloud)[i_point].x;
    point.y = (*testing_cloud)[i_point].y;
    point.z = (*testing_cloud)[i_point].z;
    colored_cloud->points.push_back (point);
  }
  colored_cloud->height += testing_cloud->size ();

  point.r = 255;
  point.g = 0;
  point.b = 0;
  for (std::size_t i_vote = 0; i_vote < strongest_peaks.size (); i_vote++)
  {
    point.x = strongest_peaks[i_vote].x;
    point.y = strongest_peaks[i_vote].y;
    point.z = strongest_peaks[i_vote].z;
    colored_cloud->points.push_back (point);
  }
  colored_cloud->height += strongest_peaks.size ();

  pcl::visualization::CloudViewer viewer ("Result viewer");
  viewer.showCloud (colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  return (0);
}
