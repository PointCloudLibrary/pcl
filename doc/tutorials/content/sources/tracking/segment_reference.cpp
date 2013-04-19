#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include "segment_reference.h"

using namespace pcl::tracking;

pcl::visualization::CloudViewer* viewer_;
CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr plane_cloud_;
CloudPtr nonplane_cloud_;
CloudPtr cloud_hull_;
CloudPtr segmented_cloud_;
CloudPtr reference_;
std::vector<pcl::Vertices> hull_vertices_;

std::string device_id_;
boost::mutex mtx_;
bool new_cloud_;
boost::shared_ptr<ParticleFilter> tracker_;
int counter_;
double downsampling_grid_size_;

void extractNonPlanePoints (const CloudConstPtr &cloud,
			    const CloudConstPtr &cloud_hull,
			    Cloud &result)
{
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGBA> polygon_extract;
  pcl::PointIndices::Ptr inliers_polygon (new pcl::PointIndices ());
  polygon_extract.setHeightLimits (0.01, 10.0);
  polygon_extract.setInputPlanarHull (cloud_hull);
  polygon_extract.setInputCloud (cloud);
  polygon_extract.segment (*inliers_polygon);
  {
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract_positive;
    extract_positive.setNegative (false);
    extract_positive.setInputCloud (cloud);
    extract_positive.setIndices (inliers_polygon);
    extract_positive.filter (result);
  }
}

//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
{
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 10.0);
  pass.setKeepOrganized (false);
  pass.setInputCloud (cloud);
  pass.filter (result);
}


//Compute and get cluster_indies
void euclideanSegment (const CloudConstPtr &cloud,
		       std::vector<pcl::PointIndices> &cluster_indices)
{
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  KdTreePtr tree (new KdTree ());
    
  ec.setClusterTolerance (0.05); // 2cm
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
}

//
void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}
  
//plane segment
void planeSegmentation (const CloudConstPtr &cloud,
			pcl::ModelCoefficients &coefficients,
			pcl::PointIndices &inliers)
{
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud);
  seg.segment (inliers, coefficients);
}


void planeProjection (const CloudConstPtr &cloud,
		      Cloud &result,
		      const pcl::ModelCoefficients::ConstPtr &coefficients)
{
  pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (result);
}


void extractSegmentCluster (const CloudConstPtr &cloud,
			    const std::vector<pcl::PointIndices> cluster_indices,
			    const int segment_index,
			    Cloud &result)
{
  pcl::PointIndices segmented_indices = cluster_indices[segment_index];
  for (size_t i = 0; i < segmented_indices.indices.size (); i++)
    {
      pcl::PointXYZRGBA point = cloud->points[segmented_indices.indices[i]];
      result.points.push_back (point);
    }
  result.width = uint32_t (result.points.size ());
  result.height = 1;
  result.is_dense = true;
}

//Create convexHull
void convexHull (const CloudConstPtr &cloud,
		 Cloud &,
		 std::vector<pcl::Vertices> &hull_vertices)
{
  pcl::ConvexHull<pcl::PointXYZRGBA> chull;
  chull.setInputCloud (cloud);
  chull.reconstruct (*cloud_hull_, hull_vertices);
}

//Decide model reference
void initialize_segemnted_reference(CloudPtr target_cloud, CloudPtr& transed_ref_downsampled, Eigen::Affine3f& trans){

  std::vector<pcl::PointIndices> cluster_indices;
  euclideanSegment (target_cloud, cluster_indices);
  if (cluster_indices.size () > 0)
    {
      // select the cluster to track
      CloudPtr temp_cloud (new Cloud);
      extractSegmentCluster (target_cloud, cluster_indices, 0, *temp_cloud);
      Eigen::Vector4f c;
      pcl::compute3DCentroid<RefPointType> (*temp_cloud, c);
      int segment_index = 0;
      double segment_distance = c[0] * c[0] + c[1] * c[1];
      for (size_t i = 1; i < cluster_indices.size (); i++)
	{
	  temp_cloud.reset (new Cloud);
	  extractSegmentCluster (target_cloud, cluster_indices, int (i), *temp_cloud);
	  pcl::compute3DCentroid<RefPointType> (*temp_cloud, c);
	  double distance = c[0] * c[0] + c[1] * c[1];
	  if (distance < segment_distance)
	    {
	      segment_index = int (i);
	      segment_distance = distance;
	    }
	}
          
      segmented_cloud_.reset (new Cloud);
      extractSegmentCluster (target_cloud, cluster_indices, segment_index, *segmented_cloud_);								
      RefCloudPtr transed_ref (new RefCloud);
      pcl::compute3DCentroid<RefPointType> (*segmented_cloud_, c);
      trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
      pcl::transformPointCloud<RefPointType> (*segmented_cloud_, *transed_ref, trans.inverse());
					
      gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
      reference_ = transed_ref;
    }
  else
    {
      PCL_WARN ("euclidean segmentation failed\n");
    }	
}

//plane extracting
void extractPlanes(CloudPtr &target_cloud){
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  planeSegmentation (cloud_pass_downsampled_, *coefficients, *inliers);
  if (inliers->indices.size () > 3)
    {
      CloudPtr cloud_projected (new Cloud);
      cloud_hull_.reset (new Cloud);
      nonplane_cloud_.reset (new Cloud);
          
      planeProjection (cloud_pass_downsampled_, *cloud_projected, coefficients);
      convexHull (cloud_projected, *cloud_hull_, hull_vertices_);
      extractNonPlanePoints (cloud_pass_downsampled_, cloud_hull_, *nonplane_cloud_);
      target_cloud = nonplane_cloud_;
    }
  else
    {
      PCL_WARN ("cannot segment plane\n");
    }
}




