#include <pcl/features/statistical_multiscale_interest_region_extraction.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


using namespace pcl;
using namespace std;

const float subsampling_leaf_size = 0.003;
const float base_scale = 0.005;


int
main (int, char **argv)
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
  for (size_t scales = 0; scales < 7; ++scales)
  {
    PCL_INFO ("%f ", base_scale_aux);
    scale_vector.push_back (base_scale_aux);
    base_scale_aux *= 1.6f;
  }
  PCL_INFO ("\n");
  region_extraction.setInputCloud (cloud_subsampled);
  region_extraction.setScalesVector (scale_vector);
  std::list<IndicesPtr> rois;
  region_extraction.computeRegionsOfInterest (rois);

  PCL_INFO ("Regions of interest found: %d\n", rois.size ());
  pcl::ExtractIndices<PointXYZ> extract_indices_filter;
  unsigned int roi_count = 0;
  for (std::list<IndicesPtr>::iterator l_it = rois.begin (); l_it != rois.end (); ++l_it)
  {
    PointCloud<PointXYZ> roi_points;
    extract_indices_filter.setInputCloud (cloud_subsampled);
    extract_indices_filter.setIndices (*l_it);
    extract_indices_filter.filter (roi_points);

    char filename[512];
    sprintf (filename, "roi_%03d.pcd", ++roi_count);
    io::savePCDFileASCII (filename, roi_points);
  }

  io::savePCDFileASCII ("subsampled_input.pcd", *cloud_subsampled);

  return 0;
}
