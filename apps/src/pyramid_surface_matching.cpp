#include <pcl/features/normal_3d.h>
#include <pcl/features/ppf.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/pyramid_feature_matching.h>

using namespace pcl;

#include <iostream>

const Eigen::Vector4f subsampling_leaf_size(0.02f, 0.02f, 0.02f, 0.0f);
const float normal_estimation_search_radius = 0.05f;

void
subsampleAndCalculateNormals(PointCloud<PointXYZ>::Ptr& cloud,
                             PointCloud<PointXYZ>::Ptr& cloud_subsampled,
                             PointCloud<Normal>::Ptr& cloud_subsampled_normals)
{
  cloud_subsampled = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
  VoxelGrid<PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud(cloud);
  subsampling_filter.setLeafSize(subsampling_leaf_size);
  subsampling_filter.filter(*cloud_subsampled);

  cloud_subsampled_normals = PointCloud<Normal>::Ptr(new PointCloud<Normal>());
  NormalEstimation<PointXYZ, Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud(cloud_subsampled);
  search::KdTree<PointXYZ>::Ptr search_tree(new search::KdTree<PointXYZ>);
  normal_estimation_filter.setSearchMethod(search_tree);
  normal_estimation_filter.setRadiusSearch(normal_estimation_search_radius);
  normal_estimation_filter.compute(*cloud_subsampled_normals);

  std::cerr << "Before -> After subsampling: " << cloud->size() << " -> "
            << cloud_subsampled->size() << std::endl;
}

int
main(int argc, char** argv)
{
  if (argc != 3) {
    PCL_ERROR("Syntax: ./pyramid_surface_matching model_1 model_2\n");
    return -1;
  }

  /// read point clouds from HDD
  PCL_INFO("Reading scene ...\n");
  PointCloud<PointXYZ>::Ptr cloud_a(new PointCloud<PointXYZ>()),
      cloud_b(new PointCloud<PointXYZ>());
  PCDReader reader;
  reader.read(argv[1], *cloud_a);
  reader.read(argv[2], *cloud_b);

  PointCloud<PointXYZ>::Ptr cloud_a_subsampled, cloud_b_subsampled;
  PointCloud<Normal>::Ptr cloud_a_subsampled_normals, cloud_b_subsampled_normals;
  subsampleAndCalculateNormals(cloud_a, cloud_a_subsampled, cloud_a_subsampled_normals);
  subsampleAndCalculateNormals(cloud_b, cloud_b_subsampled, cloud_b_subsampled_normals);

  PCL_INFO("Finished subsampling the clouds ...\n");

  PointCloud<PPFSignature>::Ptr ppf_signature_a(new PointCloud<PPFSignature>()),
      ppf_signature_b(new PointCloud<PPFSignature>());
  PointCloud<PointNormal>::Ptr cloud_subsampled_with_normals_a(
      new PointCloud<PointNormal>()),
      cloud_subsampled_with_normals_b(new PointCloud<PointNormal>());
  concatenateFields(*cloud_a_subsampled,
                    *cloud_a_subsampled_normals,
                    *cloud_subsampled_with_normals_a);
  concatenateFields(*cloud_b_subsampled,
                    *cloud_b_subsampled_normals,
                    *cloud_subsampled_with_normals_b);

  PPFEstimation<PointNormal, PointNormal, PPFSignature> ppf_estimator;
  ppf_estimator.setInputCloud(cloud_subsampled_with_normals_a);
  ppf_estimator.setInputNormals(cloud_subsampled_with_normals_a);
  ppf_estimator.compute(*ppf_signature_a);
  ppf_estimator.setInputCloud(cloud_subsampled_with_normals_b);
  ppf_estimator.setInputNormals(cloud_subsampled_with_normals_b);
  ppf_estimator.compute(*ppf_signature_b);

  PCL_INFO("Feature cloud sizes: %zu , %zu\n",
           static_cast<std::size_t>(ppf_signature_a->size()),
           static_cast<std::size_t>(ppf_signature_b->size()));

  PCL_INFO("Finished calculating the features ...\n");
  std::vector<std::pair<float, float>> dim_range_input, dim_range_target;
  for (std::size_t i = 0; i < 3; ++i)
    dim_range_input.emplace_back(float(-M_PI), float(M_PI));
  dim_range_input.emplace_back(0.0f, 1.0f);
  for (std::size_t i = 0; i < 3; ++i)
    dim_range_target.emplace_back(float(-M_PI) * 10.0f, float(M_PI) * 10.0f);
  dim_range_target.emplace_back(0.0f, 50.0f);

  PyramidFeatureHistogram<PPFSignature>::Ptr pyramid_a(
      new PyramidFeatureHistogram<PPFSignature>());
  pyramid_a->setInputCloud(ppf_signature_a);
  pyramid_a->setInputDimensionRange(dim_range_input);
  pyramid_a->setTargetDimensionRange(dim_range_target);
  pyramid_a->compute();
  PCL_INFO("Done with the first pyramid\n");

  PyramidFeatureHistogram<PPFSignature>::Ptr pyramid_b(
      new PyramidFeatureHistogram<PPFSignature>());
  pyramid_b->setInputCloud(ppf_signature_b);
  pyramid_b->setInputDimensionRange(dim_range_input);
  pyramid_b->setTargetDimensionRange(dim_range_target);
  pyramid_b->compute();
  PCL_INFO("Done with the second pyramid\n");

  float value = PyramidFeatureHistogram<PPFSignature>::comparePyramidFeatureHistograms(
      pyramid_a, pyramid_b);
  PCL_INFO(
      "Surface comparison value between %s and %s is: %f\n", argv[1], argv[2], value);

  return 0;
}
