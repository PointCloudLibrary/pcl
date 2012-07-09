/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <boost/make_shared.hpp>
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <boost/graph/incremental_components.hpp>

typedef pcl::PointXYZ PointT;

/** \brief comparePoints will compare the plane equations of two points.
 * It requires a point type that includes normal information.
 * The normal (a,b,c) and range (d) components of the normal are compared separately, so separate thresholds are available.
 *
 * \author Alexander J. B. Trevor
 */
template<typename PointInT> bool
comparePoints (PointInT p1, PointInT p2, double p1_d, double p2_d, float ang_thresh, float dist_thresh)
{
  float dot_p = p1.normal[0] * p2.normal[0] + p1.normal[1] * p2.normal[1] + p1.normal[2] * p2.normal[2];
  float dist = fabsf (float (p1_d - p2_d));
  if ((dot_p > ang_thresh) && (dist < dist_thresh))
    return (true);
  else
    return (false);
}


/** \brief extractPlanesByFloodFill finds all planes present in the input cloud, and outputs a vector of plane equations, as well as a vector of point clouds corresponding to the inliers of each detected plane.
 * Only planes with more than min_inliers points are detected.
 *
 * \author Alexander J. B. Trevor
 */
template<typename PointInT, typename PointOutT> void
extractPlanesByFloodFill (
    const pcl::PointCloud<PointInT> & cloud_in, 
    std::vector<pcl::ModelCoefficients>& normals_out, 
    std::vector<pcl::PointCloud<PointOutT> >& inliers, 
    unsigned min_inliers, 
    float ang_thresh, 
    float dist_thresh)
{
  ang_thresh = cosf (ang_thresh);

  pcl::PointCloud<PointOutT> normal_cloud;
  pcl::copyPointCloud (cloud_in, normal_cloud);

  //estimate the normals
  double normal_start = pcl::getTime ();
  pcl::IntegralImageNormalEstimation<PointOutT, PointOutT> ne;
  ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor (0.02f);
  ne.setNormalSmoothingSize (10.0f);
  ne.setInputCloud (boost::make_shared<pcl::PointCloud<PointOutT> >(normal_cloud));
  ne.compute (normal_cloud);
  double normal_end = pcl::getTime ();
  std::cout << "Normal Estimation took: " << double(normal_end - normal_start) << std::endl;

  //Calculate range part of planes' hessian normal form
  double plane_d_start = pcl::getTime ();
  float *plane_d = new float[normal_cloud.points.size ()];
  for (unsigned int i = 0; i < normal_cloud.points.size (); ++i)
  {
    plane_d[i] = normal_cloud[i].x * normal_cloud[i].normal[0] + normal_cloud[i].y * normal_cloud[i].normal[1] + normal_cloud[i].z * normal_cloud[i].normal[2];
  }
  double plane_d_end = pcl::getTime ();
  std::cout << "Plane_d calc took: " << double(plane_d_end - plane_d_start) << std::endl;

  //Connected Components
  //Set up for union-find
  double clust_start = pcl::getTime ();
  std::vector<int> rank (normal_cloud.points.size ());
  std::vector<int> parent (normal_cloud.points.size ());
  boost::disjoint_sets<int*, int*> ds (&rank[0], &parent[0]);

  int *clusts = new int[normal_cloud.points.size ()];
  memset (clusts, -1, normal_cloud.points.size () * sizeof(int));
  int clust_id = 0;

  //First row
  for (size_t ri = 1; ri < normal_cloud.height; ++ri)
  {
    if (comparePoints (normal_cloud[ri], normal_cloud[ri - 1], plane_d[ri], plane_d[ri - 1], ang_thresh, dist_thresh))
    {
      clusts[ri] = clusts[ri - 1];
    }
    else
    {
      clusts[ri] = clust_id++;
      ds.make_set (clusts[ri]);
    }
  }

  //Everything else
  unsigned int index = 0;
  unsigned int last_row_idx = 0;
  std::vector<std::vector<int> > same;
  for (size_t ri = 1; ri < normal_cloud.height; ++ri)
  {

    index = unsigned (ri) * normal_cloud.width;
    last_row_idx = unsigned (ri - 1) * normal_cloud.width;

    //First pixel
    if (comparePoints (normal_cloud[index], normal_cloud[last_row_idx], plane_d[index], plane_d[last_row_idx], ang_thresh, dist_thresh))
    {
      clusts[index] = clusts[last_row_idx];
    }
    else
    {
      clusts[index] = clust_id++;
      ds.make_set (clusts[index]);
    }

    //Rest of row
    for (size_t ci = 1; ci < normal_cloud.width; ++ci, ++index)
    {
      int label = -1;

      if (comparePoints (normal_cloud[index], normal_cloud[index - 1], plane_d[index], plane_d[index - 1], ang_thresh, dist_thresh))
      {
        label = clusts[index - 1];
      }

      if (comparePoints (normal_cloud[index], normal_cloud[last_row_idx + ci], plane_d[index], plane_d[last_row_idx + ci], ang_thresh, dist_thresh))
      {
        if ((label > 0) && (label != clusts[last_row_idx + ci]))
        {
          ds.union_set (label, clusts[last_row_idx + ci]);
        }
        else
        {
          label = clusts[last_row_idx + ci];
        }
      }

      if (label >= 0)
      {
        clusts[index] = label;
      }
      else
      {
        clusts[index] = clust_id++;
        ds.make_set (clusts[index]);
      }
    }
  }

  //Second pass
  index = 0;
  std::vector<pcl::PointIndices> clust_inds;
  clust_inds.resize (10);
  for (unsigned int ri = 0; ri < normal_cloud.height; ++ri)
  {
    for (unsigned int ci = 0; ci < normal_cloud.width; ++ci, ++index)
    {
      clusts[index] = ds.find_set (clusts[index]);
      if (clusts[index] >= int (clust_inds.size ()))
      {
        clust_inds.resize (clusts[index] * 2, pcl::PointIndices ());
      }
      clust_inds[clusts[index]].indices.push_back (index);
    }
  }

  double clust_end = pcl::getTime ();
  std::cout << "Clust took: " << double(clust_end - clust_start) << std::endl;

  //Fit planes to each cluster
  double fitting_start = pcl::getTime ();
  for (size_t i = 0; i < clust_inds.size (); i++)
  {
    if (clust_inds[i].indices.size () > min_inliers)
    {
      pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
      pcl::PointCloud<pcl::PointXYZRGBNormal> clust_inliers;
      extract.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal> >(normal_cloud));
      extract.setIndices (boost::make_shared<pcl::PointIndices>(clust_inds[i]));
      extract.filter (clust_inliers);

      Eigen::Vector4f clust_centroid;
      Eigen::Matrix3f clust_cov;
      pcl::computeMeanAndCovarianceMatrix (normal_cloud, clust_inds[i].indices, clust_cov, clust_centroid);
      Eigen::Vector4f plane_params;
      float curvature;
      pcl::solvePlaneParameters (clust_cov, clust_centroid, plane_params, curvature);
      pcl::flipNormalTowardsViewpoint (normal_cloud[clust_inds[i].indices[0]], 0.0f, 0.0f, 0.0f, plane_params);
      pcl::ModelCoefficients model;
      model.values.push_back(plane_params[0]);
      model.values.push_back(plane_params[1]);
      model.values.push_back(plane_params[2]);
      model.values.push_back(plane_params[3]);
      normals_out.push_back(model);
      //normals_out.push_back (plane_params);
      inliers.push_back (clust_inliers);
    }
  }
  double fitting_end = pcl::getTime ();
  std::cout << "Fitting took: " << double(fitting_end - fitting_start) << std::endl;
}


//modified extractEuclideanClusters from pcl/segmentation/extract_clusters
//we can save on fabs(acos()) in inner loop by doing cos(eps_angle) instead.
template<typename PointT, typename Normal> void
extractEuclideanClustersModified (
  const pcl::PointCloud<PointT> &cloud, const pcl::PointCloud<Normal> &normals,
  float tolerance, const boost::shared_ptr<pcl::search::KdTree<PointT> > &tree,
  std::vector<pcl::PointIndices> &clusters,
  double eps_angle,
  unsigned int min_pts_per_cluster,
  unsigned int max_pts_per_cluster)
{
  if (tree->getInputCloud ()->points.size () != cloud.points.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
    return;
  }
  if (cloud.points.size () != normals.points.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%zu) different than normals (%zu)!\n", cloud.points.size (), normals.points.size ());
    return;
  }

  //Allows us to avoid acos in the loop
  eps_angle = cos (eps_angle);

  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.points.size (), false);

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  // Process all points in the indices vector
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    if (processed[i])
      continue;

    std::vector<unsigned int> seed_queue;
    int sq_idx = 0;
    seed_queue.push_back (i);

    processed[i] = true;

    while (sq_idx < int (seed_queue.size ()))
    {
      // Search for sq_idx
      if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
      {
        sq_idx++;
        continue;
      }

      for (size_t j = 1; j < nn_indices.size (); ++j)                 // nn_indices[0] should be sq_idx
      {
        if (processed[nn_indices[j]])                               // Has this point been processed before ?
          continue;

        //processed[nn_indices[j]] = true;
        // [-1;1]
        double dot_p = normals.points[i].normal[0] * normals.points[nn_indices[j]].normal[0] +
                       normals.points[i].normal[1] * normals.points[nn_indices[j]].normal[1] +
                       normals.points[i].normal[2] * normals.points[nn_indices[j]].normal[2];

        //if ( fabs (acos (dot_p)) < eps_angle )
        if (dot_p > eps_angle)
        {
          processed[nn_indices[j]] = true;
          seed_queue.push_back (nn_indices[j]);
        }
      }

      sq_idx++;
    }

    // If this queue is satisfactory, add to the clusters
    if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
    {
      pcl::PointIndices r;
      r.indices.resize (seed_queue.size ());
      for (size_t j = 0; j < seed_queue.size (); ++j)
        r.indices[j] = seed_queue[j];

      std::sort (r.indices.begin (), r.indices.end ());
      r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

      r.header = cloud.header;
      clusters.push_back (r);     // We could avoid a copy by working directly in the vector
    }
  }
}

/** \brief extractPlanesByClustering makes use of the version of extractEuclideanClusters that also clusters on normal information in order to find all planes in the scene.
 *
 * \author Alexander J. B. Trevor
 */
template<typename PointInT, typename PointOutT> void
extractPlanesByClustering (const pcl::PointCloud<PointInT> & cloud_in, std::vector<Eigen::Vector4f>& normals_out, std::vector<pcl::PointCloud<PointOutT> >& inliers, int min_inliers, double ang_thresh, double dist_thresh)
{
  //TODO: Check that PointOutT is valid -- it must have a spot for normals

  //Some parameters -- perhaps some of these should be args
  double voxel_grid_res = 0.01;

  pcl::PointCloud<PointOutT> normal_cloud;
  pcl::copyPointCloud (cloud_in, normal_cloud); //TODO: is there a better way to do this?

  //Estimate Normals
  double normal_start = pcl::getTime ();
  pcl::IntegralImageNormalEstimation<PointOutT, PointOutT> ne;
  ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor (0.02f);
  ne.setNormalSmoothingSize (10.0f);
  ne.setInputCloud (boost::make_shared<pcl::PointCloud<PointOutT> >(normal_cloud));
  ne.compute (normal_cloud);
  double normal_end = pcl::getTime ();
  std::cout << "Normal Estimation took: " << double(normal_end - normal_start) << std::endl;

  //Downsample the cloud with Normals
  double downsample_start = pcl::getTime ();
  pcl::PointCloud<PointOutT> downsampled_cloud;
  pcl::VoxelGrid<PointOutT> grid;
  grid.setInputCloud (boost::make_shared<pcl::PointCloud<PointOutT> >(normal_cloud));
  grid.setLeafSize (voxel_grid_res, voxel_grid_res, voxel_grid_res);
  grid.filter (downsampled_cloud);
  double downsample_end = pcl::getTime ();
  std::cout << "Downsampling took: " << double(downsample_end - downsample_start) << std::endl;

  //Cluster the data
  double cluster_start = pcl::getTime ();
  std::vector<pcl::PointIndices> clusters;
  typename pcl::search::KdTree<PointOutT>::Ptr tree (new pcl::search::KdTree<PointOutT>());
  tree->setInputCloud (boost::make_shared<pcl::PointCloud<PointOutT> >(downsampled_cloud));
  extractEuclideanClustersModified (downsampled_cloud, downsampled_cloud, dist_thresh, tree, clusters, ang_thresh, min_inliers, 400000);
  double cluster_end = pcl::getTime ();
  std::cout << "Clustering took: " << double(cluster_end - cluster_start) << std::endl;

  //Fit planes to each cluster
  double fitting_start = pcl::getTime ();
  for (int i = 0; i < clusters.size (); i++)
  {
    pcl::ExtractIndices<PointOutT> extract;
    pcl::PointCloud<PointOutT> clust_inliers;
    extract.setInputCloud (boost::make_shared<pcl::PointCloud<PointOutT> >(downsampled_cloud));
    extract.setIndices (boost::make_shared<pcl::PointIndices>(clusters[i]));
    extract.filter (clust_inliers);

    Eigen::Vector4f clust_centroid;
    pcl::compute3DCentroid (clust_inliers, clust_centroid);
    Eigen::Matrix3f clust_cov;
    pcl::computeCovarianceMatrix (clust_inliers, clust_centroid, clust_cov);
    Eigen::Vector4f plane_params;
    float curvature;
    pcl::solvePlaneParameters (clust_cov, clust_centroid, plane_params, curvature);

    pcl::flipNormalTowardsViewpoint (clust_inliers.points[0], 0.0f, 0.0f, 0.0f, plane_params);

    normals_out.push_back (plane_params);
    inliers.push_back (clust_inliers);
  }
  double fitting_end = pcl::getTime ();
  std::cout << "Fitting took: " << double(fitting_end - fitting_start) << std::endl;
}

class PlaneSegTest
{
private:
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
pcl::PointCloud<PointT>::ConstPtr prev_cloud;
unsigned int text_id;
boost::mutex cloud_mutex;
boost::mutex viewer_mutex;

public:
PlaneSegTest()
{
  text_id = 0;
}
~PlaneSegTest(){
}

boost::shared_ptr<pcl::visualization::PCLVisualizer>
cloudViewer (pcl::PointCloud<PointT>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (cloud, 0, 255, 0);
  viewer->addPointCloud<PointT> (cloud, single_color, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void
cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  if (!viewer->wasStopped ())
  {
    cloud_mutex.lock ();
    prev_cloud = cloud;
    cloud_mutex.unlock ();
  }
}

void
run ()
{
  pcl::Grabber* interface = new pcl::OpenNIGrabber ();

  boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&PlaneSegTest::cloud_cb_, this, _1);

  //make a viewer
  pcl::PointCloud<PointT>::Ptr init_cloud_ptr (new pcl::PointCloud<PointT>);
  viewer = cloudViewer (init_cloud_ptr);

  boost::signals2::connection c = interface->registerCallback (f);

  interface->start ();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);

    if (prev_cloud && cloud_mutex.try_lock ())
    {
      printf ("Extracting planes...\n");
      //std::vector<Eigen::Vector4f> plane_normals; //This doesn't work on Win, using the following instead:
      std::vector<pcl::ModelCoefficients> plane_normals;
      std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > plane_inliers;

      double start = pcl::getTime ();
      //extractPlanesByClustering(*prev_cloud,plane_normals,plane_inliers,1000,0.174,0.04);
      extractPlanesByFloodFill (*prev_cloud, plane_normals, plane_inliers, 10000, 0.017453f * 3.0f, 0.02f);
      double end = pcl::getTime ();
      std::cout << "Full Plane Extraction using Flood Fill for frame took: " << double(end - start) << std::endl << std::endl;

      //Do some visualization
      viewer_mutex.lock ();
      viewer->removeAllPointClouds ();

      //Display cloud
      pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
      float grid_size = 0.01f;
      pcl::VoxelGrid<pcl::PointXYZ> grid;
      grid.setInputCloud (prev_cloud);
      grid.setLeafSize (grid_size, grid_size, grid_size);
      grid.filter (downsampled_cloud);

      //Display normals
      pcl::PointCloud<pcl::Normal> normals;

      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(downsampled_cloud));
      ne.setSearchMethod (tree);
      ne.setRadiusSearch (0.05); //(0.1);//0.5
      ne.compute (normals);

      viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(downsampled_cloud), boost::make_shared<pcl::PointCloud<pcl::Normal> >(normals), 10, 0.05f, "normals");

      viewer->removeAllShapes ();

      //Draw planar normals
      for (size_t i = 0; i < plane_inliers.size (); i++)
      {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (plane_inliers[i], centroid);
        pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
        pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * plane_normals[i].values[0]),
                                           centroid[1] + (0.5f * plane_normals[i].values[1]),
                                           centroid[2] + (0.5f * plane_normals[i].values[2]));
        char normal_name[500];
        sprintf (normal_name, "normal_%d", unsigned (i));
        viewer->addArrow (pt2, pt1, 1.0, 0, 0, std::string (normal_name));
      }

      viewer->addPointCloud<pcl::PointXYZ>(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(downsampled_cloud), "cloud");

      //Draw a couple clusters
      if (plane_inliers.size () > 0)
      {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clust0 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal> >(plane_inliers[0]);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> color0 (clust0, 255, 0, 0);
        viewer->addPointCloud (clust0, color0, "clust0");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "clust0");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "clust0");
      }
      if (plane_inliers.size () > 1)
      {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clust1 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal> >(plane_inliers[1]);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> color1 (clust1, 0, 255, 0);
        viewer->addPointCloud (clust1, color1, "clust1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "clust1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "clust1");
      }
      if (plane_inliers.size () > 2)
      {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clust2 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal> >(plane_inliers[2]);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> color2 (clust2, 0, 0, 255);
        viewer->addPointCloud (clust2, color2, "clust2");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "clust2");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "clust2");
      }

      cloud_mutex.unlock ();
      viewer_mutex.unlock ();
    }

    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }

  interface->stop ();

}

};

int
main ()
{
  PlaneSegTest v;
  v.run ();
  return 0;
}
