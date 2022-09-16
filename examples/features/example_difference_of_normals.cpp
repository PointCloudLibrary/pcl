/**
 * @file pcl_don.cpp
 * Difference of normals implementation using PCL.
 *
 * @author Yani Ioannou
 * @date 2012-03-11
 */

#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/don.h>

#ifdef PCL_ONLY_CORE_POINT_TYPES
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#endif

using namespace pcl;

using PointT = pcl::PointXYZRGB;
using PointNT = pcl::PointNormal;
using PointOutT = pcl::PointNormal;
using SearchPtr = pcl::search::Search<PointT>::Ptr;

int main (int argc, char *argv[])
{
  ///The smallest scale to use in the DoN filter.
  constexpr double scale1 = 0.2;

  ///The largest scale to use in the DoN filter.
  constexpr double scale2 = 2.0;

  ///The minimum DoN magnitude to threshold by
  constexpr double threshold = 0.25;

  ///segment scene into clusters with given distance tolerance using euclidean clustering
  double segradius = 0.2;

  //voxelization factor of pointcloud to use in approximation of normals
  bool approx = false;
  constexpr double decimation = 100;

  if(argc < 2){
    std::cerr << "Expected 2 arguments: inputfile outputfile" << std::endl;
  }

  ///The file to read from.
  std::string infile = argv[1];

  ///The file to output to.
  std::string outfile = argv[2];

  // Load cloud in blob format
  pcl::PCLPointCloud2 blob;
  pcl::io::loadPCDFile (infile, blob);

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        std::cout << "Loading point cloud...";
        pcl::fromPCLPointCloud2 (blob, *cloud);
        std::cout << "done." << std::endl;

  SearchPtr tree;

  if (cloud->isOrganized ())
  {
    tree.reset (new pcl::search::OrganizedNeighbor<PointT> ());
  }
  else
  {
      tree.reset (new pcl::search::KdTree<PointT> (false));
  }

  tree->setInputCloud (cloud);

  PointCloud<PointT>::Ptr small_cloud_downsampled;
  PointCloud<PointT>::Ptr large_cloud_downsampled;

  // If we are using approximation
  if(approx){
    std::cout << "Downsampling point cloud for approximation" << std::endl;

    // Create the downsampling filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setDownsampleAllData (false);
    sor.setInputCloud (cloud);

    // Create downsampled point cloud for DoN NN search with small scale
    small_cloud_downsampled = PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    float smalldownsample = static_cast<float> (scale1 / decimation);
    sor.setLeafSize (smalldownsample, smalldownsample, smalldownsample);
    sor.filter (*small_cloud_downsampled);
    std::cout << "Using leaf size of " << smalldownsample << " for small scale, " << small_cloud_downsampled->size() << " points" << std::endl;

    // Create downsampled point cloud for DoN NN search with large scale
    large_cloud_downsampled = PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    const float largedownsample = float (scale2/decimation);
    sor.setLeafSize (largedownsample, largedownsample, largedownsample);
    sor.filter (*large_cloud_downsampled);
    std::cout << "Using leaf size of " << largedownsample << " for large scale, " << large_cloud_downsampled->size() << " points" << std::endl;
  }

  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<PointT, PointNT> ne;
  ne.setInputCloud (cloud);
        ne.setSearchMethod (tree);

  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
  ne.setViewPoint(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());

  if(scale1 >= scale2){
    std::cerr << "Error: Large scale must be > small scale!" << std::endl;
    exit(EXIT_FAILURE);
  }

  //the normals calculated with the small scale
  std::cout << "Calculating normals for scale..." << scale1 << std::endl;
  pcl::PointCloud<PointNT>::Ptr normals_small_scale (new pcl::PointCloud<PointNT>);

  if(approx){
    ne.setSearchSurface(small_cloud_downsampled);
    }

  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  std::cout << "Calculating normals for scale..." << scale2 << std::endl;
  //the normals calculated with the large scale
  pcl::PointCloud<PointNT>::Ptr normals_large_scale (new pcl::PointCloud<PointNT>);

  if(approx){
    ne.setSearchSurface(large_cloud_downsampled);
  }
  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointOutT>::Ptr doncloud (new pcl::PointCloud<PointOutT>);
  copyPointCloud (*cloud, *doncloud);

  std::cout << "Calculating DoN... " << std::endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointT, PointNT, PointOutT> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge(normals_large_scale);
  don.setNormalScaleSmall(normals_small_scale);

  if(!don.initCompute ()){
    std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
    exit(EXIT_FAILURE);
  }

  //Compute DoN
  don.computeFeature(*doncloud);

        pcl::PCDWriter writer;

        // Save DoN features
        writer.write<PointOutT> (outfile, *doncloud, false);

  //Filter by magnitude
  std::cout << "Filtering out DoN mag <= "<< threshold <<  "..." << std::endl;

  // build the condition
  pcl::ConditionOr<PointOutT>::Ptr range_cond (new
  pcl::ConditionOr<PointOutT> ());
  range_cond->addComparison (pcl::FieldComparison<PointOutT>::ConstPtr (new
  pcl::FieldComparison<PointOutT> ("curvature", pcl::ComparisonOps::GT, threshold)));
  // build the filter
  pcl::ConditionalRemoval<PointOutT> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (doncloud);

  pcl::PointCloud<PointOutT>::Ptr doncloud_filtered (new pcl::PointCloud<PointOutT>);

  // apply filter
  condrem.filter (*doncloud_filtered);

  doncloud = doncloud_filtered;

  // Save filtered output
  std::cout << "Filtered Pointcloud: " << doncloud->size () << " data points." << std::endl;
  std::stringstream ss;
  ss << outfile.substr(0,outfile.length()-4) << "_threshold_"<< threshold << "_.pcd";
  writer.write<PointOutT> (ss.str (), *doncloud, false);

  //Filter by magnitude
  std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= "<< segradius <<  "..." << std::endl;

  pcl::search::KdTree<PointOutT>::Ptr segtree (new pcl::search::KdTree<PointOutT>);
  segtree->setInputCloud (doncloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointOutT> ec;

  ec.setClusterTolerance (segradius);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (segtree);
  ec.setInputCloud (doncloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<PointOutT>::Ptr cloud_cluster_don (new pcl::PointCloud<PointOutT>);
    for (const auto &index : cluster.indices){
      cloud_cluster_don->points.push_back ((*doncloud)[index]);
    }

    cloud_cluster_don->width = cloud_cluster_don->size ();
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << outfile.substr(0,outfile.length()-4) << "_threshold_"<< threshold << "_cluster_" << j << ".pcd";
    writer.write<PointOutT> (ss.str (), *cloud_cluster_don, false);
    ++j;
  }
}

