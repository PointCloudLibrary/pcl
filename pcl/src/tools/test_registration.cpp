#include <fstream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/correspondence_estimation.h>

#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_reciprocal.h>
#include <pcl/registration/correspondence_rejection_sample_consensuns.h>

#include <pcl/registration/transformation_estimation_svd.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::registration::Correspondence Correspondence;


int main(int argc, char** argv)
{
  if (argc != 3)
  {
    ROS_ERROR ("Syntax is: %s <source.pcd> <target.pcd>", argv[0]);
    ROS_INFO ("Example: %s `rospack find pcl`/test/bun0.pcd `rospack find pcl`/test/bun4.pcd", argv[0]);
    return (-1);
  }

  ros::init (argc, argv, "pcd_inc_reg");
  ros::Time::init();

  PointCloud cloud_input, cloud_target;
  pcl::io::loadPCDFile(argv[1], cloud_input);
  pcl::io::loadPCDFile(argv[2], cloud_target);

  PointCloud::ConstPtr input_cloud_ptr = cloud_input.makeShared();
  PointCloud::ConstPtr target_cloud_ptr = cloud_target.makeShared();


  /////////////////////////////////////////////////////////////////////////////
  // determine correspondences
  float registration_distance_threshold = 0.01;
  std::vector<Correspondence> correspondences;
  pcl::registration::CorrespondenceEstimation<Point, Point> corr_est;
  corr_est.setInputTarget(target_cloud_ptr);
  corr_est.setMaxCorrespondenceDistance(registration_distance_threshold); // TODO: currently ignored
  corr_est.setInputCloud(input_cloud_ptr); // + setIndices(...)
  corr_est.determineCorrespondences(correspondences);
  printf("%d\t original correspondences\n", (int)correspondences.size());

  pcl::registration::CorrespondencesConstPtr correspondences_ptr = boost::make_shared< const std::vector<pcl::registration::Correspondence> >(correspondences);

  std::vector<Correspondence> correspondeces_reciprocal;
  corr_est.determineReciprocalCorrespondences(correspondeces_reciprocal);
  printf("%d\t reciprocal correspondences\n", (int)correspondeces_reciprocal.size());


  /////////////////////////////////////////////////////////////////////////////
  // sort out probably "false" correspondences

  float max_cor_dist = 0.0001;
  std::vector<Correspondence> correspondeces_dist;
  pcl::registration::CorrespondenceRejectorDistance cor_rej_dist;
  cor_rej_dist.setMaximumDistance(0.0001);
  cor_rej_dist.setInputCorrespondences(correspondences_ptr);
  cor_rej_dist.getCorrespondeces(correspondeces_dist);
  printf("%d\t filtered correspondences (max_cost_dist=%0.4f)\n", (int)correspondeces_dist.size(), max_cor_dist);

  float ratio = 0.5;
  std::vector<Correspondence> correspondeces_trimmed;
  pcl::registration::CorrespondenceRejectorTrimmed cor_rej_trimmed;
  cor_rej_trimmed.setOverlapRadio(ratio);
  cor_rej_trimmed.setInputCorrespondences(correspondences_ptr);
  cor_rej_trimmed.getCorrespondeces(correspondeces_trimmed);
  printf("%d\t trimmed correspondences (ratio=%0.4f)\n", (int)correspondeces_trimmed.size(), ratio);

  std::vector<Correspondence> correspondeces_one_to_one;
  pcl::registration::CorrespondenceRejectorOneToOne cor_rej_one_to_one;
  cor_rej_one_to_one.setInputCorrespondences(correspondences_ptr);
  cor_rej_one_to_one.getCorrespondeces(correspondeces_one_to_one);
  printf("%d\t ono-to-one correspondences\n", (int)correspondeces_one_to_one.size());


  double sac_threshold = 0.005;
  int sac_max_iterations = 100;
  std::vector<pcl::registration::Correspondence> correspondeces_sac;
  pcl::registration::CorrespondenceRejectorSampleConsensus<Point> cor_rej_sac;
  cor_rej_sac.setInputCloud(input_cloud_ptr);
  cor_rej_sac.setTargetCloud(target_cloud_ptr);
  cor_rej_sac.setInlierThreshold(sac_threshold);
  cor_rej_sac.setMaxIterations(sac_max_iterations);
  cor_rej_sac.setInputCorrespondences(correspondences_ptr);
  cor_rej_sac.getCorrespondeces(correspondeces_sac);

  printf("%d\t correspondences after SAC rejection\n", (int)correspondeces_sac.size());

  Eigen::Matrix4f transform_sac = cor_rej_sac.getBestTransformation();

  std::vector<int> indices_not_corresponding;
  cor_rej_sac.getRejectedQueryIndices(correspondeces_sac, indices_not_corresponding);
  printf("%d\t correspondences rejected by SAC\n", (int)indices_not_corresponding.size());


  pcl::registration::TransformationEstimationSVD<Point, Point> trans_est;
  Eigen::Matrix4f transform_svd;
  trans_est.estimateRigidTransformation(cloud_input, cloud_target, correspondeces_sac, transform_svd);

  std::cout << "Transform from SAC:" << std::endl;
  std::cout << transform_sac << std::endl;
  std::cout << "Transform from SVD:" << std::endl;
  std::cout << transform_svd << std::endl;




  /////////////////////////////////////////////////////////////////////////////
  // write files
  std::ofstream file_output;

  file_output.open("correspondences_original.dat");
  for (unsigned int i = 0; i < correspondences.size(); ++i)
    file_output << correspondences[i] << std::endl;
  file_output.close();

  file_output.open("correspondences_dist.dat");
  for (unsigned int i = 0; i < correspondeces_dist.size(); ++i)
    file_output << correspondeces_dist[i] << std::endl;
  file_output.close();

  file_output.open("correspondences_trimmed.dat");
  for (unsigned int i = 0; i < correspondeces_trimmed.size(); ++i)
    file_output << correspondeces_trimmed[i] << std::endl;
  file_output.close();

  file_output.open("correspondences_one_to_one.dat");
  for (unsigned int i = 0; i < correspondeces_one_to_one.size(); ++i)
    file_output << correspondeces_one_to_one[i] << std::endl;
  file_output.close();

  file_output.open("correspondences_reciprocal.dat");
  for (unsigned int i = 0; i < correspondeces_reciprocal.size(); ++i)
    file_output << correspondeces_reciprocal[i] << std::endl;
  file_output.close();

  return 0;
}
