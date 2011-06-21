#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <pcl/apps/dominant_plane_segmentation.h>

template<typename PointType>
void
pcl::apps::DominantPlaneSegmentation<PointType>::compute (std::vector<CloudPtr, Eigen::aligned_allocator<CloudPtr> > & clusters)
{

  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[DominantPlaneSegmentation] No input dataset given!\n");
    return;
  }

  CloudConstPtr cloud_;
  CloudPtr cloud_filtered_(new Cloud());
  CloudPtr cloud_downsampled_(new Cloud());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_(new pcl::PointCloud<pcl::Normal>());
  pcl::PointIndices::Ptr table_inliers_(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr table_coefficients_(new pcl::ModelCoefficients());
  CloudPtr table_projected_(new Cloud());
  CloudPtr table_hull_(new Cloud());
  CloudPtr cloud_objects_(new Cloud());
  CloudPtr cluster_object_(new Cloud());

  KdTreePtr normals_tree_(new pcl::KdTreeFLANN<PointType>);
  KdTreePtr clusters_tree_(new pcl::KdTreeFLANN<PointType>);
  clusters_tree_->setEpsilon (1);

  // Normal estimation parameters
  n3d_.setKSearch (k_);
  n3d_.setSearchMethod (normals_tree_);

  // Table model fitting parameters
  seg_.setDistanceThreshold (sac_distance_threshold_);
  seg_.setMaxIterations (2000);
  seg_.setNormalDistanceWeight (0.1);
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setProbability (0.99);

  proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  bb_cluster_proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);

  prism_.setHeightLimits (object_min_height_, object_max_height_);

  // Clustering parameters
  cluster_.setClusterTolerance (object_cluster_tolerance_);
  cluster_.setMinClusterSize (object_cluster_min_size_);
  cluster_.setSearchMethod (clusters_tree_);

  // ---[ PassThroughFilter
  pass_.setFilterLimits (min_z_bounds_, max_z_bounds_);
  pass_.setFilterFieldName ("z");
  pass_.setInputCloud (input_);
  pass_.filter (*cloud_filtered_);

  if ((int)cloud_filtered_->points.size () < k_)
  {
    PCL_WARN ("[DominantPlaneSegmentation] Filtering returned %d points! Aborting.",
        (int)cloud_filtered_->points.size ());
    return;
  }

  // Downsample the point cloud
  grid_.setLeafSize (downsample_leaf_, downsample_leaf_, downsample_leaf_);
  grid_.setDownsampleAllData (false);
  grid_.setInputCloud (cloud_filtered_);
  grid_.filter (*cloud_downsampled_);

  PCL_INFO ("[DominantPlaneSegmentation] Number of points left after filtering (%f -> %f): %d out of %d\n",
  min_z_bounds_, max_z_bounds_, (int)cloud_downsampled_->points.size (), (int)input_->points.size ());

  // ---[ Estimate the point normals
  n3d_.setInputCloud (cloud_downsampled_);
  n3d_.compute (*cloud_normals_);

  PCL_INFO ("[DominantPlaneSegmentation] %d normals estimated. \n", (int)cloud_normals_->points.size ());

  // ---[ Perform segmentation
  seg_.setInputCloud (cloud_downsampled_);
  seg_.setInputNormals (cloud_normals_);
  seg_.segment (*table_inliers_, *table_coefficients_);

  if (table_inliers_->indices.size () == 0)
  {
    PCL_WARN ("[DominantPlaneSegmentation] No Plane Inliers points! Aborting.");
    return;
  }

  // ---[ Extract the plane
  proj_.setInputCloud (cloud_downsampled_);
  proj_.setIndices (table_inliers_);
  proj_.setModelCoefficients (table_coefficients_);
  proj_.filter (*table_projected_);

  // ---[ Estimate the convex hull
  std::vector<pcl::Vertices> polygons;
  CloudPtr table_hull (new Cloud ());
  hull_.setInputCloud (table_projected_);
  hull_.reconstruct (*table_hull, polygons);

  // Compute the plane coefficients
  Eigen::Vector4f model_coefficients;
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;

  model_coefficients[0] = table_coefficients_->values[0];
  model_coefficients[1] = table_coefficients_->values[1];
  model_coefficients[2] = table_coefficients_->values[2];
  model_coefficients[3] = table_coefficients_->values[3];

  // Need to flip the plane normal towards the viewpoint
  Eigen::Vector4f vp (0, 0, 0, 0);
  // See if we need to flip any plane normals
  vp -= table_hull->points[0].getVector4fMap ();
  vp[3] = 0;
  // Dot product between the (viewpoint - point) and the plane normal
  float cos_theta = vp.dot (model_coefficients);
  // Flip the plane normal
  if (cos_theta < 0)
  {
    model_coefficients *= -1;
    model_coefficients[3] = 0;
    // Hessian form (D = nc . p_plane (centroid here) + p)
    model_coefficients[3] = -1 * (model_coefficients.dot (table_hull->points[0].getVector4fMap ()));
  }

  //Set table_coeffs
  table_coeffs_ = model_coefficients;

  // ---[ Get the objects on top of the table
  pcl::PointIndices cloud_object_indices;
  prism_.setInputCloud (cloud_downsampled_);
  prism_.setInputPlanarHull (table_hull);
  prism_.segment (cloud_object_indices);

  pcl::ExtractIndices<PointType> extract_object_indices;
  extract_object_indices.setInputCloud (cloud_downsampled_);
  extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
  extract_object_indices.filter (*cloud_objects_);

  if (cloud_objects_->points.size () == 0)
    return;

  // ---[ Split the objects into Euclidean clusters
  std::vector<pcl::PointIndices> clusters2;
  cluster_.setInputCloud (cloud_objects_);
  cluster_.extract (clusters2);

  PCL_INFO ("[DominantPlaneSegmentation] Number of clusters found matching the given constraints: %d.\n",
      (int)clusters2.size ());

  clusters.resize (clusters2.size ());
  for (size_t i = 0; i < clusters2.size (); ++i)
  {
    clusters[i] = (CloudPtr)(new Cloud());
    clusters[i]->points.resize (clusters2[i].indices.size ());
    for (size_t j = 0; j < clusters[i]->points.size (); ++j)
    {
      clusters[i]->points[j].x = cloud_objects_->points[clusters2[i].indices[j]].x;
      clusters[i]->points[j].y = cloud_objects_->points[clusters2[i].indices[j]].y;
      clusters[i]->points[j].z = cloud_objects_->points[clusters2[i].indices[j]].z;
    }
  }
}

#define PCL_INSTANTIATE_DominantPlaneSegmentation(T) template class PCL_EXPORTS pcl::apps::DominantPlaneSegmentation<T>;

