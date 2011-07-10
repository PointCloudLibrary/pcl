#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/ppf.h>
#include <pcl/registration/pyramid_feature_matching.h>

using namespace pcl;

#include <iostream>
using namespace std;

const Eigen::Vector4f subsampling_leaf_size (0.02, 0.02, 0.02, 0.0);
const float normal_estimation_search_radius = 0.05;

//const Eigen::Vector4f subsampling_leaf_size (5, 5, 5, 0.0);
//const float normal_estimation_search_radius = 11;

void
subsampleAndCalculateNormals (PointCloud<PointXYZ>::Ptr &cloud,
                              PointCloud<PointXYZ>::Ptr &cloud_subsampled,
                              PointCloud<Normal>::Ptr &cloud_subsampled_normals)
{
  cloud_subsampled = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ> ());
  VoxelGrid<PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud (cloud);
  subsampling_filter.setLeafSize (subsampling_leaf_size);
  subsampling_filter.filter (*cloud_subsampled);

  cloud_subsampled_normals = PointCloud<Normal>::Ptr (new PointCloud<Normal> ());
  NormalEstimation<PointXYZ, Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_subsampled);
  KdTreeFLANN<PointXYZ>::Ptr search_tree (new KdTreeFLANN<PointXYZ>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch (normal_estimation_search_radius);
  normal_estimation_filter.compute (*cloud_subsampled_normals);

  cerr << "Before -> After subsampling: " << cloud->points.size () << " -> " << cloud_subsampled->points.size () << endl;
}

/*
class PFHPyramidFeatureMatching : public PyramidFeatureMatching<PFHSignature125>
{
public:
  PFHPyramidFeatureMatching (size_t a_dimensions,
                             std::vector<std::pair<float, float> > a_dimension_range)
    : PyramidFeatureMatching<PFHSignature125> (a_dimensions, a_dimension_range)
      {}
  void
  convertFeatureToVector (const PFHSignature125& feature,
                          std::vector<float>& feature_vector)
  {
    //feature_vector.clear ();
    for (size_t i = 0; i < 125; ++i)
      feature_vector.push_back (feature.histogram[i]);
  }
};
*/

class PPFPyramidFeatureMatching : public PyramidFeatureMatching<PPFSignature>
{
public:
  PPFPyramidFeatureMatching (size_t a_dimensions,
                             std::vector<std::pair<float, float> > a_dimension_range)
    : PyramidFeatureMatching<PPFSignature> (a_dimensions, a_dimension_range)
      {}
  void
  convertFeatureToVector (const PPFSignature& feature,
                          std::vector<float>& feature_vector)
  {
    // rescale features for better bin sizes inside the histogram
    feature_vector.push_back (feature.f1 * 10.0f);
    feature_vector.push_back (feature.f2 * 10.0f);
    feature_vector.push_back (feature.f3 * 10.0f);
    feature_vector.push_back (feature.f4 * 50.0f);
  }
};

int
main (int argc, char **argv)
{
  if (argc != 3)
  {
    PCL_ERROR ("Syntax: ./pyramid_surface_matching model_1 model_2\n");
    return -1;
  }


  /// read point clouds from HDD
  PCL_INFO ("Reading scene ...\n");
  PointCloud<PointXYZ>::Ptr cloud_a (new PointCloud<PointXYZ> ()),
      cloud_b (new PointCloud<PointXYZ> ());
  PCDReader reader;
  reader.read (argv[1], *cloud_a);
  reader.read (argv[2], *cloud_b);

  PointCloud<PointXYZ>::Ptr cloud_a_subsampled, cloud_b_subsampled;
  PointCloud<Normal>::Ptr cloud_a_subsampled_normals, cloud_b_subsampled_normals;
  subsampleAndCalculateNormals (cloud_a, cloud_a_subsampled, cloud_a_subsampled_normals);
  subsampleAndCalculateNormals (cloud_b, cloud_b_subsampled, cloud_b_subsampled_normals);

  cerr << "Finished subsampling the clouds..." << endl;


  PointCloud<PPFSignature>::Ptr ppf_signature_a (new PointCloud<PPFSignature> ()),
      ppf_signature_b (new PointCloud<PPFSignature> ());
  PointCloud<PointNormal>::Ptr cloud_subsampled_with_normals_a (new PointCloud<PointNormal> ()),
      cloud_subsampled_with_normals_b (new PointCloud<PointNormal> ());
  concatenateFields (*cloud_a_subsampled, *cloud_a_subsampled_normals, *cloud_subsampled_with_normals_a);
  concatenateFields (*cloud_b_subsampled, *cloud_b_subsampled_normals, *cloud_subsampled_with_normals_b);

  PPFEstimation<PointNormal, PointNormal, PPFSignature> ppf_estimator;
  ppf_estimator.setInputCloud (cloud_subsampled_with_normals_a);
  ppf_estimator.setInputNormals (cloud_subsampled_with_normals_a);
  ppf_estimator.compute (*ppf_signature_a);
  ppf_estimator.setInputCloud (cloud_subsampled_with_normals_b);
  ppf_estimator.setInputNormals (cloud_subsampled_with_normals_b);
  ppf_estimator.compute (*ppf_signature_b);

  cerr << "Feature cloud sizes: " << ppf_signature_a->points.size () << " " << ppf_signature_b->points.size () << endl;

  cerr << "Finished calculating the features..." << endl;
  size_t dimensions = 4;
  vector<pair<float, float> > dim_range;
  for (size_t i = 0; i < 3; ++i) dim_range.push_back (pair<float, float> (-M_PI * 10.0f, M_PI * 10.0f));
  dim_range.push_back (pair<float, float> (0.0f, 50.0f));

  PPFPyramidFeatureMatching pyramid_matching (dimensions, dim_range);
  PyramidHistogram::Ptr pyramid_a, pyramid_b;
  pyramid_matching.computePyramidHistogram (ppf_signature_a, pyramid_a);
  cerr << "Done with the first pyramid" << endl;
  pyramid_matching.computePyramidHistogram (ppf_signature_b, pyramid_b);
  cerr << "Done with the second pyramid" << endl;


  float value = pyramid_matching.comparePyramidHistograms (pyramid_a, pyramid_b);

  cerr << "Surface comparison value between " << argv[1] << " and " << argv[2] << " is: " << value << endl;



  return 0;
}
