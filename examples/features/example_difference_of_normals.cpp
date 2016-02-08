/**
 * @file pcl_don.cpp
 * Difference of normals implementation using PCL.
 *
 * @author Yani Ioannou
 * @date 2012-03-11
 */
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/don.h>

#ifdef PCL_ONLY_CORE_POINT_TYPES
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#endif

using namespace pcl;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointNormal PointOutT;
typedef pcl::search::Search<PointT>::Ptr SearchPtr;

int main (int argc, char *argv[])
{
  ///The smallest scale to use in the DoN filter.
  double scale1 = 0.2;

  ///The largest scale to use in the DoN filter.
  double scale2 = 2.0;

  ///The minimum DoN magnitude to threshold by
  double threshold = 0.25;

  ///segment scene into clusters with given distance tolerance using euclidean clustering
  double segradius = 0.2;

  //voxelization factor of pointcloud to use in approximation of normals
  bool approx = false;
  double decimation = 100;

  if(argc < 2){
    cerr << "Expected 2 arguments: inputfile outputfile" << endl;
  }

  ///The file to read from.
  string infile = argv[1];

  ///The file to output to.
  string outfile = argv[2];

  // Load cloud in blob format
  pcl::PCLPointCloud2 blob;
  pcl::io::loadPCDFile (infile.c_str(), blob);

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        cout << "Loading point cloud...";
        pcl::fromPCLPointCloud2 (blob, *cloud);
        cout << "done." << endl;

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
    cout << "Downsampling point cloud for approximation" << endl;

    // Create the downsampling filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setDownsampleAllData (false);
    sor.setInputCloud (cloud);

    // Create downsampled point cloud for DoN NN search with small scale
    small_cloud_downsampled = PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    float smalldownsample = static_cast<float> (scale1 / decimation);
    sor.setLeafSize (smalldownsample, smalldownsample, smalldownsample);
    sor.filter (*small_cloud_downsampled);
    cout << "Using leaf size of " << smalldownsample << " for small scale, " << small_cloud_downsampled->size() << " points" << endl;

    // Create downsampled point cloud for DoN NN search with large scale
    large_cloud_downsampled = PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    const float largedownsample = float (scale2/decimation);
    sor.setLeafSize (largedownsample, largedownsample, largedownsample);
    sor.filter (*large_cloud_downsampled);
    cout << "Using leaf size of " << largedownsample << " for large scale, " << large_cloud_downsampled->size() << " points" << endl;
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
    cerr << "Error: Large scale must be > small scale!" << endl;
    exit(EXIT_FAILURE);
  }

  //the normals calculated with the small scale
  cout << "Calculating normals for scale..." << scale1 << endl;
  pcl::PointCloud<PointNT>::Ptr normals_small_scale (new pcl::PointCloud<PointNT>);

  if(approx){
    ne.setSearchSurface(small_cloud_downsampled);
    }

  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  cout << "Calculating normals for scale..." << scale2 << endl;
  //the normals calculated with the large scale
  pcl::PointCloud<PointNT>::Ptr normals_large_scale (new pcl::PointCloud<PointNT>);

  if(approx){
    ne.setSearchSurface(large_cloud_downsampled);
  }
  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointOutT>::Ptr doncloud (new pcl::PointCloud<PointOutT>);
  copyPointCloud<PointT, PointOutT>(*cloud, *doncloud);

  cout << "Calculating DoN... " << endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointT, PointNT, PointOutT> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge(normals_large_scale);
  don.setNormalScaleSmall(normals_small_scale);

  if(!don.initCompute ()){
    std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
    exit(EXIT_FAILURE);
  }

  //Compute DoN
  don.computeFeature(*doncloud);

        pcl::PCDWriter writer;

        // Save DoN features
        writer.write<PointOutT> (outfile.c_str (), *doncloud, false);

  //Filter by magnitude
  cout << "Filtering out DoN mag <= "<< threshold <<  "..." << endl;

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
  std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;
  std::stringstream ss;
  ss << outfile.substr(0,outfile.length()-4) << "_threshold_"<< threshold << "_.pcd";
  writer.write<PointOutT> (ss.str (), *doncloud, false);

  //Filter by magnitude
  cout << "Clustering using EuclideanClusterExtraction with tolerance <= "<< segradius <<  "..." << endl;

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
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
  {
    pcl::PointCloud<PointOutT>::Ptr cloud_cluster_don (new pcl::PointCloud<PointOutT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster_don->points.push_back (doncloud->points[*pit]);
    }

    cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << outfile.substr(0,outfile.length()-4) << "_threshold_"<< threshold << "_cluster_" << j << ".pcd";
    writer.write<PointOutT> (ss.str (), *cloud_cluster_don, false);
  }
}

