/**
 * @file don_segmentation.cpp
 * Difference of Normals Example for PCL Segmentation Tutorials.
 *
 * @author Yani Ioannou
 * @date 2012-09-24
 */
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>

using namespace pcl;

int
main (int argc, char *argv[])
{
  ///The smallest scale to use in the DoN filter.
  double scale1;

  ///The largest scale to use in the DoN filter.
  double scale2;

  ///The minimum DoN magnitude to threshold by
  double threshold;

  ///segment scene into clusters with given distance tolerance using euclidean clustering
  double segradius;

  if (argc < 6)
  {
    std::cerr << "usage: " << argv[0] << " inputfile smallscale largescale threshold segradius" << std::endl;
    exit (EXIT_FAILURE);
  }

  /// the file to read from.
  std::string infile = argv[1];
  /// small scale
  std::istringstream (argv[2]) >> scale1;
  /// large scale
  std::istringstream (argv[3]) >> scale2;
  std::istringstream (argv[4]) >> threshold;   // threshold for DoN magnitude
  std::istringstream (argv[5]) >> segradius;   // threshold for radius segmentation

  // Load cloud in blob format
  pcl::PCLPointCloud2 blob;
  pcl::io::loadPCDFile (infile.c_str (), blob);
  pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
  pcl::fromPCLPointCloud2 (blob, *cloud);

  // Create a search tree, use KDTreee for non-organized data.
  pcl::search::Search<PointXYZRGB>::Ptr tree;
  if (cloud->isOrganized ())
  {
    tree.reset (new pcl::search::OrganizedNeighbor<PointXYZRGB> ());
  }
  else
  {
    tree.reset (new pcl::search::KdTree<PointXYZRGB> (false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud (cloud);

  if (scale1 >= scale2)
  {
    std::cerr << "Error: Large scale must be > small scale!" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // calculate normals with the small scale
  std::cout << "Calculating normals for scale..." << scale1 << std::endl;
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  std::cout << "Calculating normals for scale..." << scale2 << std::endl;
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  copyPointCloud (*cloud, *doncloud);

  std::cout << "Calculating DoN... " << std::endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);

  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute DoN
  don.computeFeature (*doncloud);

  // Save DoN features
  pcl::PCDWriter writer;
  writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 

  // Filter by magnitude
  std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;

  // Build the condition for filtering
  pcl::ConditionOr<PointNormal>::Ptr range_cond (
    new pcl::ConditionOr<PointNormal> ()
    );
  range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                               new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                             );
  // Build the filter
  pcl::ConditionalRemoval<PointNormal> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (doncloud);

  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);

  doncloud = doncloud_filtered;

  // Save filtered output
  std::cout << "Filtered Pointcloud: " << doncloud->size () << " data points." << std::endl;

  writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

  // Filter by magnitude
  std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << std::endl;

  pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
  segtree->setInputCloud (doncloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointNormal> ec;

  ec.setClusterTolerance (segradius);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (segtree);
  ec.setInputCloud (doncloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
    for (const auto& idx : cluster.indices)
    {
      cloud_cluster_don->points.push_back ((*doncloud)[idx]);
    }

    cloud_cluster_don->width = cloud_cluster_don->size ();
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;

    //Save cluster
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "don_cluster_" << j << ".pcd";
    writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
    ++j;
  }
}
