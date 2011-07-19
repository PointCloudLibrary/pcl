#include <pcl/features/statistical_multiscale_interest_region_extraction.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;
using namespace std;

const float subsampling_leaf_size = 4;
const float base_scale = 5;


int
main (int argc, char **argv)
{
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());

  PCDReader reader;
  reader.read (argv[1], *cloud);
  PCL_INFO ("Cloud read: %s\n", argv[1]);
  cerr << "cloud has #points: " << cloud->points.size () << endl;

  PointCloud<PointXYZ>::Ptr cloud_subsampled (new PointCloud<PointXYZ> ());
  VoxelGrid<PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud (cloud);
  subsampling_filter.setLeafSize (subsampling_leaf_size, subsampling_leaf_size, subsampling_leaf_size);
  subsampling_filter.filter (*cloud_subsampled);
  cerr << "subsampled cloud has #points: " << cloud_subsampled->points.size () << endl;

  StatisticalMultiscaleInterestRegionExtraction<PointXYZ> region_extraction;
  std::vector<float> scale_vector;
  PCL_INFO ("Scale values that will be used: ");
  float base_scale_aux = base_scale;
  for (size_t scales = 0; scales < 3; ++scales)
  {
    PCL_INFO ("%f ", base_scale_aux);
    scale_vector.push_back (base_scale_aux);
    base_scale_aux *= 1.6f;
  }
  PCL_INFO ("\n");
  region_extraction.setInputCloud (cloud_subsampled);
  region_extraction.setScalesVector (scale_vector);
  PointCloud<PointXYZ> result;
  region_extraction.computeRegionsOfInterest (result);

  io::savePCDFileASCII ("regions_of_interest.pcd", result);
  io::savePCDFileASCII ("subsampled_input.pcd", *cloud_subsampled);

  return 0;
}
