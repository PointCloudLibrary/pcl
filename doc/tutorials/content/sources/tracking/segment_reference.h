#ifndef __SEGMENT_REFERENCE_H__
#define __SEGMENT_REFERENCE_H__
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>


using namespace pcl::tracking;

typedef pcl::PointXYZRGBA RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef pcl::PointCloud<RefPointType> RefCloud;
typedef RefCloud::Ptr RefCloudPtr;
typedef RefCloud::ConstPtr RefCloudConstPtr;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
typedef ParticleFilter::CoherencePtr CoherencePtr;
typedef pcl::search::KdTree<pcl::PointXYZRGBA> KdTree;
typedef KdTree::Ptr KdTreePtr;

extern pcl::visualization::CloudViewer* viewer_;
extern CloudPtr cloud_pass_;
extern CloudPtr cloud_pass_downsampled_;
extern CloudPtr plane_cloud_;
extern CloudPtr nonplane_cloud_;
extern CloudPtr cloud_hull_;
extern CloudPtr segmented_cloud_;
extern CloudPtr reference_;
extern std::vector<pcl::Vertices> hull_vertices_;
  
extern std::string device_id_;
extern boost::mutex mtx_;
extern bool new_cloud_;
extern boost::shared_ptr<ParticleFilter> tracker_;
extern int counter_;
extern double downsampling_grid_size_;
  

void extractNonPlanePoints (const CloudConstPtr &cloud,
          const CloudConstPtr &cloud_hull,
          Cloud &result);

//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, Cloud &result);

//Compute and get cluster_indies
void euclideanSegment (const CloudConstPtr &cloud,
           std::vector<pcl::PointIndices> &cluster_indices);

//
void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01);
  
//plane segment
void planeSegmentation (const CloudConstPtr &cloud,
      pcl::ModelCoefficients &coefficients,
      pcl::PointIndices &inliers);


void planeProjection (const CloudConstPtr &cloud,
          Cloud &result,
          const pcl::ModelCoefficients::ConstPtr &coefficients);

void extractSegmentCluster (const CloudConstPtr &cloud,
          const std::vector<pcl::PointIndices> cluster_indices,
          const int segment_index,
          Cloud &result);

//Create convex hull
void convexHull (const CloudConstPtr &cloud,
     Cloud &,
     std::vector<pcl::Vertices> &hull_vertices);

//Decide model reference
void initialize_segemnted_reference(CloudPtr target_cloud, CloudPtr& transed_ref_downsampled, Eigen::Affine3f& trans);

//plane extracting
void extractPlanes(CloudPtr &target_cloud);


#endif // __SEGMENT_REFERENCE_H__
