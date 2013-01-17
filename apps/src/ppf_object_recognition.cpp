#include <pcl/features/ppf.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ppf_registration.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

using namespace pcl;
using namespace std;

const Eigen::Vector4f subsampling_leaf_size (0.02f, 0.02f, 0.02f, 0.0f);
const float normal_estimation_search_radius = 0.05f;


PointCloud<PointNormal>::Ptr
subsampleAndCalculateNormals (PointCloud<PointXYZ>::Ptr cloud)
{
  PointCloud<PointXYZ>::Ptr cloud_subsampled (new PointCloud<PointXYZ> ());
  VoxelGrid<PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud (cloud);
  subsampling_filter.setLeafSize (subsampling_leaf_size);
  subsampling_filter.filter (*cloud_subsampled);

  PointCloud<Normal>::Ptr cloud_subsampled_normals (new PointCloud<Normal> ());
  NormalEstimation<PointXYZ, Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_subsampled);
  search::KdTree<PointXYZ>::Ptr search_tree (new search::KdTree<PointXYZ>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch (normal_estimation_search_radius);
  normal_estimation_filter.compute (*cloud_subsampled_normals);

  PointCloud<PointNormal>::Ptr cloud_subsampled_with_normals (new PointCloud<PointNormal> ());
  concatenateFields (*cloud_subsampled, *cloud_subsampled_normals, *cloud_subsampled_with_normals);

  PCL_INFO ("Cloud dimensions before / after subsampling: %u / %u\n", cloud->points.size (), cloud_subsampled->points.size ());
  return cloud_subsampled_with_normals;
}

int
main (int argc, char** argv)
{
  if (argc != 3)
  {
    PCL_ERROR ("Syntax: ./ppf_object_recognition pcd_model_list pcd_scene\n");
    return -1;
  }


  /// read point clouds from HDD
  PCL_INFO ("Reading scene ...\n");
  PointCloud<PointXYZ>::Ptr cloud_scene (new PointCloud<PointXYZ> ());
  PCDReader reader;
  reader.read (argv[2], *cloud_scene);
  PCL_INFO ("Scene read: %s\n", argv[2]);

  PCL_INFO ("Reading models ...\n");
  vector<PointCloud<PointXYZ>::Ptr > cloud_models;
  ifstream pcd_file_list (argv[1]);
  while (!pcd_file_list.eof())
  {
    char str[512];
    pcd_file_list.getline (str, 512);
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
    reader.read (str, *cloud);
    cloud_models.push_back (cloud);
    PCL_INFO ("Model read: %s\n", str);
  }

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.05);
  extract.setNegative (true);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  unsigned nr_points = unsigned (cloud_scene->points.size ());
  while (cloud_scene->points.size () > 0.3 * nr_points)
  {
    seg.setInputCloud (cloud_scene);
    seg.segment (*inliers, *coefficients);
    PCL_INFO ("Plane inliers: %u\n", inliers->indices.size ());
    if (inliers->indices.size () < 50000) break;

    extract.setInputCloud (cloud_scene);
    extract.setIndices (inliers);
    extract.filter (*cloud_scene);
  }

  PointCloud<PointNormal>::Ptr cloud_scene_input = subsampleAndCalculateNormals (cloud_scene);
  vector<PointCloud<PointNormal>::Ptr > cloud_models_with_normals;

  PCL_INFO ("Training models ...\n");
  vector<PPFHashMapSearch::Ptr> hashmap_search_vector;
  for (size_t model_i = 0; model_i < cloud_models.size (); ++model_i)
  {
    PointCloud<PointNormal>::Ptr cloud_model_input = subsampleAndCalculateNormals (cloud_models[model_i]);
    cloud_models_with_normals.push_back (cloud_model_input);

    PointCloud<PPFSignature>::Ptr cloud_model_ppf (new PointCloud<PPFSignature> ());
    PPFEstimation<PointNormal, PointNormal, PPFSignature> ppf_estimator;
    ppf_estimator.setInputCloud (cloud_model_input);
    ppf_estimator.setInputNormals (cloud_model_input);
    ppf_estimator.compute (*cloud_model_ppf);

    PPFHashMapSearch::Ptr hashmap_search (new PPFHashMapSearch (12.0f / 180.0f * float (M_PI),
                                                                 0.05f));
    hashmap_search->setInputFeatureCloud (cloud_model_ppf);
    hashmap_search_vector.push_back (hashmap_search);
  }


  visualization::PCLVisualizer viewer ("PPF Object Recognition - Results");
  viewer.setBackgroundColor (0, 0, 0);
  viewer.addPointCloud (cloud_scene);
  viewer.spinOnce (10);
  PCL_INFO ("Registering models to scene ...\n");
  for (size_t model_i = 0; model_i < cloud_models.size (); ++model_i)
  {

    PPFRegistration<PointNormal, PointNormal> ppf_registration;
    // set parameters for the PPF registration procedure
    ppf_registration.setSceneReferencePointSamplingRate (10);
    ppf_registration.setPositionClusteringThreshold (0.2f);
    ppf_registration.setRotationClusteringThreshold (30.0f / 180.0f * float (M_PI));
    ppf_registration.setSearchMethod (hashmap_search_vector[model_i]);
    ppf_registration.setInputSource (cloud_models_with_normals[model_i]);
    ppf_registration.setInputTarget (cloud_scene_input);

    PointCloud<PointNormal> cloud_output_subsampled;
    ppf_registration.align (cloud_output_subsampled);

    PointCloud<PointXYZ>::Ptr cloud_output_subsampled_xyz (new PointCloud<PointXYZ> ());
    for (size_t i = 0; i < cloud_output_subsampled.points.size (); ++i)
      cloud_output_subsampled_xyz->points.push_back ( PointXYZ (cloud_output_subsampled.points[i].x, cloud_output_subsampled.points[i].y, cloud_output_subsampled.points[i].z));


    Eigen::Matrix4f mat = ppf_registration.getFinalTransformation ();
    Eigen::Affine3f final_transformation (mat);


    //  io::savePCDFileASCII ("output_subsampled_registered.pcd", cloud_output_subsampled);

    PointCloud<PointXYZ>::Ptr cloud_output (new PointCloud<PointXYZ> ());
    pcl::transformPointCloud (*cloud_models[model_i], *cloud_output, final_transformation);


    stringstream ss; ss << "model_" << model_i;
    visualization::PointCloudColorHandlerRandom<PointXYZ> random_color (cloud_output->makeShared ());
    viewer.addPointCloud (cloud_output, random_color, ss.str ());
//    io::savePCDFileASCII (ss.str ().c_str (), *cloud_output);
    PCL_INFO ("Showing model %s\n", ss.str ().c_str ());
  }

  PCL_INFO ("All models have been registered!\n");


  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return 0;
}
