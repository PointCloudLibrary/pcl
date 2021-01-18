/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id: cvfh.hpp 5311 2012-03-26 22:02:04Z aaldoma $
 *
 */

#ifndef PCL_FEATURES_IMPL_OURCVFH_H_
#define PCL_FEATURES_IMPL_OURCVFH_H_

#include <pcl/features/our_cvfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/io.h> // for copyPointCloud
#include <pcl/common/common.h> // for getMaxDistance
#include <pcl/common/transforms.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::OURCVFHEstimation<PointInT, PointNT, PointOutT>::compute (PointCloudOut &output)
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    output.width = output.height = 0;
    output.clear ();
    return;
  }
  // Resize the output dataset
  // Important! We should only allocate precisely how many elements we will need, otherwise
  // we risk at pre-allocating too much memory which could lead to bad_alloc
  // (see http://dev.pointclouds.org/issues/657)
  output.width = output.height = 1;
  output.resize (1);

  // Perform the actual feature computation
  computeFeature (output);

  Feature<PointInT, PointOutT>::deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::OURCVFHEstimation<PointInT, PointNT, PointOutT>::extractEuclideanClustersSmooth (const pcl::PointCloud<pcl::PointNormal> &cloud,
                                                                                        const pcl::PointCloud<pcl::PointNormal> &normals,
                                                                                        float tolerance,
                                                                                        const pcl::search::Search<pcl::PointNormal>::Ptr &tree,
                                                                                        std::vector<pcl::PointIndices> &clusters, double eps_angle,
                                                                                        unsigned int min_pts_per_cluster,
                                                                                        unsigned int max_pts_per_cluster)
{
  if (tree->getInputCloud ()->size () != cloud.size ())
  {
    PCL_ERROR("[pcl::extractEuclideanClusters] Tree built for a different point cloud "
              "dataset (%zu) than the input cloud (%zu)!\n",
              static_cast<std::size_t>(tree->getInputCloud()->size()),
              static_cast<std::size_t>(cloud.size()));
    return;
  }
  if (cloud.size () != normals.size ())
  {
    PCL_ERROR("[pcl::extractEuclideanClusters] Number of points in the input point "
              "cloud (%zu) different than normals (%zu)!\n",
              static_cast<std::size_t>(cloud.size()),
              static_cast<std::size_t>(normals.size()));
    return;
  }

  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.size (), false);

  pcl::Indices nn_indices;
  std::vector<float> nn_distances;
  // Process all points in the indices vector
  for (std::size_t i = 0; i < cloud.size (); ++i)
  {
    if (processed[i])
      continue;

    std::vector<std::size_t> seed_queue;
    std::size_t sq_idx = 0;
    seed_queue.push_back (i);

    processed[i] = true;

    while (sq_idx < seed_queue.size ())
    {
      // Search for sq_idx
      if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
      {
        sq_idx++;
        continue;
      }

      for (std::size_t j = 1; j < nn_indices.size (); ++j) // nn_indices[0] should be sq_idx
      {
        if (processed[nn_indices[j]]) // Has this point been processed before ?
          continue;

        //processed[nn_indices[j]] = true;
        // [-1;1]

        double dot_p = normals[seed_queue[sq_idx]].normal[0] * normals[nn_indices[j]].normal[0]
            + normals[seed_queue[sq_idx]].normal[1] * normals[nn_indices[j]].normal[1] + normals[seed_queue[sq_idx]].normal[2]
            * normals[nn_indices[j]].normal[2];

        if (std::abs (std::acos (dot_p)) < eps_angle)
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
      for (std::size_t j = 0; j < seed_queue.size (); ++j)
        r.indices[j] = seed_queue[j];

      std::sort (r.indices.begin (), r.indices.end ());
      r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

      r.header = cloud.header;
      clusters.push_back (r); // We could avoid a copy by working directly in the vector
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::OURCVFHEstimation<PointInT, PointNT, PointOutT>::filterNormalsWithHighCurvature (const pcl::PointCloud<PointNT> & cloud,
                                                                                        pcl::Indices &indices_to_use,
                                                                                        pcl::Indices &indices_out, pcl::Indices &indices_in,
                                                                                        float threshold)
{
  indices_out.resize (cloud.size ());
  indices_in.resize (cloud.size ());

  std::size_t in, out;
  in = out = 0;

  for (const auto &index : indices_to_use)
  {
    if (cloud[index].curvature > threshold)
    {
      indices_out[out] = index;
      out++;
    }
    else
    {
      indices_in[in] = index;
      in++;
    }
  }

  indices_out.resize (out);
  indices_in.resize (in);
}

template<typename PointInT, typename PointNT, typename PointOutT> bool
pcl::OURCVFHEstimation<PointInT, PointNT, PointOutT>::sgurf (Eigen::Vector3f & centroid, Eigen::Vector3f & normal_centroid,
                                                               PointInTPtr & processed, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & transformations,
                                                               PointInTPtr & grid, pcl::PointIndices & indices)
{

  Eigen::Vector3f plane_normal;
  plane_normal[0] = -centroid[0];
  plane_normal[1] = -centroid[1];
  plane_normal[2] = -centroid[2];
  Eigen::Vector3f z_vector = Eigen::Vector3f::UnitZ ();
  plane_normal.normalize ();
  Eigen::Vector3f axis = plane_normal.cross (z_vector);
  double rotation = -asin (axis.norm ());
  axis.normalize ();

  Eigen::Affine3f transformPC (Eigen::AngleAxisf (static_cast<float> (rotation), axis));

  grid->resize (processed->size ());
  for (std::size_t k = 0; k < processed->size (); k++)
    (*grid)[k].getVector4fMap () = (*processed)[k].getVector4fMap ();

  pcl::transformPointCloud (*grid, *grid, transformPC);

  Eigen::Vector4f centroid4f (centroid[0], centroid[1], centroid[2], 0);
  Eigen::Vector4f normal_centroid4f (normal_centroid[0], normal_centroid[1], normal_centroid[2], 0);

  centroid4f = transformPC * centroid4f;
  normal_centroid4f = transformPC * normal_centroid4f;

  Eigen::Vector3f centroid3f (centroid4f[0], centroid4f[1], centroid4f[2]);

  Eigen::Vector4f farthest_away;
  pcl::getMaxDistance (*grid, indices.indices, centroid4f, farthest_away);
  farthest_away[3] = 0;

  float max_dist = (farthest_away - centroid4f).norm ();

  pcl::demeanPointCloud (*grid, centroid4f, *grid);

  Eigen::Matrix4f center_mat;
  center_mat.setIdentity (4, 4);
  center_mat (0, 3) = -centroid4f[0];
  center_mat (1, 3) = -centroid4f[1];
  center_mat (2, 3) = -centroid4f[2];

  Eigen::Matrix3f scatter;
  scatter.setZero ();
  float sum_w = 0.f;

  for (const auto &index : indices.indices)
  {
    Eigen::Vector3f pvector = (*grid)[index].getVector3fMap ();
    float d_k = (pvector).norm ();
    float w = (max_dist - d_k);
    Eigen::Vector3f diff = (pvector);
    Eigen::Matrix3f mat = diff * diff.transpose ();
    scatter += mat * w;
    sum_w += w;
  }

  scatter /= sum_w;

  Eigen::JacobiSVD <Eigen::MatrixXf> svd (scatter, Eigen::ComputeFullV);
  Eigen::Vector3f evx = svd.matrixV ().col (0);
  Eigen::Vector3f evy = svd.matrixV ().col (1);
  Eigen::Vector3f evz = svd.matrixV ().col (2);
  Eigen::Vector3f evxminus = evx * -1;
  Eigen::Vector3f evyminus = evy * -1;
  Eigen::Vector3f evzminus = evz * -1;

  float s_xplus, s_xminus, s_yplus, s_yminus;
  s_xplus = s_xminus = s_yplus = s_yminus = 0.f;

  //disambiguate rf using all points
  for (const auto& point: grid->points)
  {
    Eigen::Vector3f pvector = point.getVector3fMap ();
    float dist_x, dist_y;
    dist_x = std::abs (evx.dot (pvector));
    dist_y = std::abs (evy.dot (pvector));

    if ((pvector).dot (evx) >= 0)
      s_xplus += dist_x;
    else
      s_xminus += dist_x;

    if ((pvector).dot (evy) >= 0)
      s_yplus += dist_y;
    else
      s_yminus += dist_y;

  }

  if (s_xplus < s_xminus)
    evx = evxminus;

  if (s_yplus < s_yminus)
    evy = evyminus;

  //select the axis that could be disambiguated more easily
  float fx, fy;
  float max_x = static_cast<float> (std::max (s_xplus, s_xminus));
  float min_x = static_cast<float> (std::min (s_xplus, s_xminus));
  float max_y = static_cast<float> (std::max (s_yplus, s_yminus));
  float min_y = static_cast<float> (std::min (s_yplus, s_yminus));

  fx = (min_x / max_x);
  fy = (min_y / max_y);

  Eigen::Vector3f normal3f = Eigen::Vector3f (normal_centroid4f[0], normal_centroid4f[1], normal_centroid4f[2]);
  if (normal3f.dot (evz) < 0)
    evz = evzminus;

  //if fx/y close to 1, it was hard to disambiguate
  //what if both are equally easy or difficult to disambiguate, namely fy == fx or very close

  float max_axis = std::max (fx, fy);
  float min_axis = std::min (fx, fy);

  if ((min_axis / max_axis) > axis_ratio_)
  {
    PCL_WARN ("Both axes are equally easy/difficult to disambiguate\n");

    Eigen::Vector3f evy_copy = evy;
    Eigen::Vector3f evxminus = evx * -1;
    Eigen::Vector3f evyminus = evy * -1;

    if (min_axis > min_axis_value_)
    {
      //combination of all possibilities
      evy = evx.cross (evz);
      Eigen::Matrix4f trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
      transformations.push_back (trans);

      evx = evxminus;
      evy = evx.cross (evz);
      trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
      transformations.push_back (trans);

      evx = evy_copy;
      evy = evx.cross (evz);
      trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
      transformations.push_back (trans);

      evx = evyminus;
      evy = evx.cross (evz);
      trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
      transformations.push_back (trans);

    }
    else
    {
      //1-st case (evx selected)
      evy = evx.cross (evz);
      Eigen::Matrix4f trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
      transformations.push_back (trans);

      //2-nd case (evy selected)
      evx = evy_copy;
      evy = evx.cross (evz);
      trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
      transformations.push_back (trans);
    }
  }
  else
  {
    if (fy < fx)
    {
      evx = evy;
      fx = fy;
    }

    evy = evx.cross (evz);
    Eigen::Matrix4f trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
    transformations.push_back (trans);

  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::OURCVFHEstimation<PointInT, PointNT, PointOutT>::computeRFAndShapeDistribution (PointInTPtr & processed, PointCloudOut & output,
                                                                                     std::vector<pcl::PointIndices> & cluster_indices)
{
  PointCloudOut ourcvfh_output;

  cluster_axes_.clear ();
  cluster_axes_.resize (centroids_dominant_orientations_.size ());

  for (std::size_t i = 0; i < centroids_dominant_orientations_.size (); i++)
  {

    std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
    PointInTPtr grid (new pcl::PointCloud<PointInT>);
    sgurf (centroids_dominant_orientations_[i], dominant_normals_[i], processed, transformations, grid, cluster_indices[i]);

    // Make a note of how many transformations correspond to each cluster
    cluster_axes_[i] = transformations.size ();
    
    for (const auto &transformation : transformations)
    {

      pcl::transformPointCloud (*processed, *grid, transformation);
      transforms_.push_back (transformation);
      valid_transforms_.push_back (true);

      std::vector < Eigen::VectorXf > quadrants (8);
      int size_hists = 13;
      int num_hists = 8;
      for (int k = 0; k < num_hists; k++)
        quadrants[k].setZero (size_hists);

      Eigen::Vector4f centroid_p;
      centroid_p.setZero ();
      Eigen::Vector4f max_pt;
      pcl::getMaxDistance (*grid, centroid_p, max_pt);
      max_pt[3] = 0;
      double distance_normalization_factor = (centroid_p - max_pt).norm ();

      float hist_incr;
      if (normalize_bins_)
        hist_incr = 100.0f / static_cast<float> (grid->size () - 1);
      else
        hist_incr = 1.0f;

      float * weights = new float[num_hists];
      float sigma = 0.01f; //1cm
      float sigma_sq = sigma * sigma;

      for (const auto& point: grid->points)
      {
        Eigen::Vector4f p = point.getVector4fMap ();
        p[3] = 0.f;
        float d = p.norm ();

        //compute weight for all octants
        float wx = 1.f - std::exp (-((p[0] * p[0]) / (2.f * sigma_sq))); //how is the weight distributed among two semi-cubes
        float wy = 1.f - std::exp (-((p[1] * p[1]) / (2.f * sigma_sq)));
        float wz = 1.f - std::exp (-((p[2] * p[2]) / (2.f * sigma_sq)));

        //distribute the weights using the x-coordinate
        if (p[0] >= 0)
        {
          for (std::size_t ii = 0; ii <= 3; ii++)
            weights[ii] = 0.5f - wx * 0.5f;

          for (std::size_t ii = 4; ii <= 7; ii++)
            weights[ii] = 0.5f + wx * 0.5f;
        }
        else
        {
          for (std::size_t ii = 0; ii <= 3; ii++)
            weights[ii] = 0.5f + wx * 0.5f;

          for (std::size_t ii = 4; ii <= 7; ii++)
            weights[ii] = 0.5f - wx * 0.5f;
        }

        //distribute the weights using the y-coordinate
        if (p[1] >= 0)
        {
          for (std::size_t ii = 0; ii <= 1; ii++)
            weights[ii] *= 0.5f - wy * 0.5f;
          for (std::size_t ii = 4; ii <= 5; ii++)
            weights[ii] *= 0.5f - wy * 0.5f;

          for (std::size_t ii = 2; ii <= 3; ii++)
            weights[ii] *= 0.5f + wy * 0.5f;

          for (std::size_t ii = 6; ii <= 7; ii++)
            weights[ii] *= 0.5f + wy * 0.5f;
        }
        else
        {
          for (std::size_t ii = 0; ii <= 1; ii++)
            weights[ii] *= 0.5f + wy * 0.5f;
          for (std::size_t ii = 4; ii <= 5; ii++)
            weights[ii] *= 0.5f + wy * 0.5f;

          for (std::size_t ii = 2; ii <= 3; ii++)
            weights[ii] *= 0.5f - wy * 0.5f;

          for (std::size_t ii = 6; ii <= 7; ii++)
            weights[ii] *= 0.5f - wy * 0.5f;
        }

        //distribute the weights using the z-coordinate
        if (p[2] >= 0)
        {
          for (std::size_t ii = 0; ii <= 7; ii += 2)
            weights[ii] *= 0.5f - wz * 0.5f;

          for (std::size_t ii = 1; ii <= 7; ii += 2)
            weights[ii] *= 0.5f + wz * 0.5f;

        }
        else
        {
          for (std::size_t ii = 0; ii <= 7; ii += 2)
            weights[ii] *= 0.5f + wz * 0.5f;

          for (std::size_t ii = 1; ii <= 7; ii += 2)
            weights[ii] *= 0.5f - wz * 0.5f;
        }

        int h_index = (d <= 0) ? 0 : std::ceil (size_hists * (d / distance_normalization_factor)) - 1;
        /* from http://www.pcl-users.org/OUR-CVFH-problem-td4028436.html
           h_index will be 13 when d is computed on the farthest away point.

          adding the following after computing h_index fixes the problem:
        */
        if(h_index > 12)
          h_index = 12;
        for (int j = 0; j < num_hists; j++)
          quadrants[j][h_index] += hist_incr * weights[j];

      }

      //copy to the cvfh signature
      PointCloudOut vfh_signature;
      vfh_signature.resize (1);
      vfh_signature.width = vfh_signature.height = 1;
      for (int d = 0; d < 308; ++d)
        vfh_signature[0].histogram[d] = output[i].histogram[d];

      int pos = 45 * 3;
      for (int k = 0; k < num_hists; k++)
      {
        for (int ii = 0; ii < size_hists; ii++, pos++)
        {
          vfh_signature[0].histogram[pos] = quadrants[k][ii];
        }
      }

      ourcvfh_output.push_back (vfh_signature[0]);
      ourcvfh_output.width = ourcvfh_output.size ();
      delete[] weights;
    }
  }

  if (!ourcvfh_output.empty ())
  {
    ourcvfh_output.height = 1;
  }
  output = ourcvfh_output;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT> void
pcl::OURCVFHEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  if (refine_clusters_ <= 0.f)
    refine_clusters_ = 1.f;

  // Check if input was set
  if (!normals_)
  {
    PCL_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.clear ();
    return;
  }
  if (normals_->size () != surface_->size ())
  {
    PCL_ERROR ("[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.clear ();
    return;
  }

  centroids_dominant_orientations_.clear ();
  clusters_.clear ();
  transforms_.clear ();
  dominant_normals_.clear ();

  // ---[ Step 0: remove normals with high curvature
  pcl::Indices indices_out;
  pcl::Indices indices_in;
  filterNormalsWithHighCurvature (*normals_, *indices_, indices_out, indices_in, curv_threshold_);

  pcl::PointCloud<pcl::PointNormal>::Ptr normals_filtered_cloud (new pcl::PointCloud<pcl::PointNormal> ());
  normals_filtered_cloud->width = indices_in.size ();
  normals_filtered_cloud->height = 1;
  normals_filtered_cloud->points.resize (normals_filtered_cloud->width);

  pcl::Indices indices_from_nfc_to_indices;
  indices_from_nfc_to_indices.resize (indices_in.size ());

  for (std::size_t i = 0; i < indices_in.size (); ++i)
  {
    (*normals_filtered_cloud)[i].x = (*surface_)[indices_in[i]].x;
    (*normals_filtered_cloud)[i].y = (*surface_)[indices_in[i]].y;
    (*normals_filtered_cloud)[i].z = (*surface_)[indices_in[i]].z;
    //(*normals_filtered_cloud)[i].getNormalVector4fMap() = (*normals_)[indices_in[i]].getNormalVector4fMap();
    indices_from_nfc_to_indices[i] = indices_in[i];
  }

  std::vector<pcl::PointIndices> clusters;

  if (normals_filtered_cloud->size () >= min_points_)
  {
    //recompute normals and use them for clustering
    {
      KdTreePtr normals_tree_filtered (new pcl::search::KdTree<pcl::PointNormal> (false));
      normals_tree_filtered->setInputCloud (normals_filtered_cloud);
      pcl::NormalEstimation<PointNormal, PointNormal> n3d;
      n3d.setRadiusSearch (radius_normals_);
      n3d.setSearchMethod (normals_tree_filtered);
      n3d.setInputCloud (normals_filtered_cloud);
      n3d.compute (*normals_filtered_cloud);
    }

    KdTreePtr normals_tree (new pcl::search::KdTree<pcl::PointNormal> (false));
    normals_tree->setInputCloud (normals_filtered_cloud);

    extractEuclideanClustersSmooth (*normals_filtered_cloud, *normals_filtered_cloud, cluster_tolerance_, normals_tree, clusters,
                                    eps_angle_threshold_, static_cast<unsigned int> (min_points_));

    std::vector<pcl::PointIndices> clusters_filtered;
    int cluster_filtered_idx = 0;
    for (const auto &cluster : clusters)
    {

      pcl::PointIndices pi;
      pcl::PointIndices pi_cvfh;
      pcl::PointIndices pi_filtered;

      clusters_.push_back (pi);
      clusters_filtered.push_back (pi_filtered);

      Eigen::Vector4f avg_normal = Eigen::Vector4f::Zero ();
      Eigen::Vector4f avg_centroid = Eigen::Vector4f::Zero ();

      for (const auto &index : cluster.indices)
      {
        avg_normal += (*normals_filtered_cloud)[index].getNormalVector4fMap ();
        avg_centroid += (*normals_filtered_cloud)[index].getVector4fMap ();
      }

      avg_normal /= static_cast<float> (cluster.indices.size ());
      avg_centroid /= static_cast<float> (cluster.indices.size ());
      avg_normal.normalize ();

      Eigen::Vector3f avg_norm (avg_normal[0], avg_normal[1], avg_normal[2]);
      Eigen::Vector3f avg_dominant_centroid (avg_centroid[0], avg_centroid[1], avg_centroid[2]);

      for (const auto &index : cluster.indices)
      {
        //decide if normal should be added
        double dot_p = avg_normal.dot ((*normals_filtered_cloud)[index].getNormalVector4fMap ());
        if (std::abs (std::acos (dot_p)) < (eps_angle_threshold_ * refine_clusters_))
        {
          clusters_[cluster_filtered_idx].indices.push_back (indices_from_nfc_to_indices[index]);
          clusters_filtered[cluster_filtered_idx].indices.push_back (index);
        }
      }

      //remove last cluster if no points found...
      if (clusters_[cluster_filtered_idx].indices.empty ())
      {
        clusters_.pop_back ();
        clusters_filtered.pop_back ();
      }
      else
        cluster_filtered_idx++;
    }

    clusters = clusters_filtered;

  }

  pcl::VFHEstimation<PointInT, PointNT, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (surface_);
  vfh.setInputNormals (normals_);
  vfh.setIndices (indices_);
  vfh.setSearchMethod (this->tree_);
  vfh.setUseGivenNormal (true);
  vfh.setUseGivenCentroid (true);
  vfh.setNormalizeBins (normalize_bins_);
  output.height = 1;

  // ---[ Step 1b : check if any dominant cluster was found
  if (!clusters.empty ())
  { // ---[ Step 1b.1 : If yes, compute CVFH using the cluster information
    for (const auto &cluster : clusters) //for each cluster
    {
      Eigen::Vector4f avg_normal = Eigen::Vector4f::Zero ();
      Eigen::Vector4f avg_centroid = Eigen::Vector4f::Zero ();

      for (const auto &index : cluster.indices)
      {
        avg_normal += (*normals_filtered_cloud)[index].getNormalVector4fMap ();
        avg_centroid += (*normals_filtered_cloud)[index].getVector4fMap ();
      }

      avg_normal /= static_cast<float> (cluster.indices.size ());
      avg_centroid /= static_cast<float> (cluster.indices.size ());
      avg_normal.normalize ();

      //append normal and centroid for the clusters
      dominant_normals_.emplace_back (avg_normal[0], avg_normal[1], avg_normal[2]);
      centroids_dominant_orientations_.emplace_back (avg_centroid[0], avg_centroid[1], avg_centroid[2]);
    }

    //compute modified VFH for all dominant clusters and add them to the list!
    output.resize (dominant_normals_.size ());
    output.width = dominant_normals_.size ();

    for (std::size_t i = 0; i < dominant_normals_.size (); ++i)
    {
      //configure VFH computation for CVFH
      vfh.setNormalToUse (dominant_normals_[i]);
      vfh.setCentroidToUse (centroids_dominant_orientations_[i]);
      pcl::PointCloud<pcl::VFHSignature308> vfh_signature;
      vfh.compute (vfh_signature);
      output[i] = vfh_signature[0];
    }

    //finish filling the descriptor with the shape distribution
    PointInTPtr cloud_input (new pcl::PointCloud<PointInT>);
    pcl::copyPointCloud (*surface_, *indices_, *cloud_input);
    computeRFAndShapeDistribution (cloud_input, output, clusters_); //this will set transforms_
  }
  else
  { // ---[ Step 1b.1 : If no, compute a VFH using all the object points

    PCL_WARN("No clusters were found in the surface... using VFH...\n");
    Eigen::Vector4f avg_centroid;
    pcl::compute3DCentroid (*surface_, avg_centroid);
    Eigen::Vector3f cloud_centroid (avg_centroid[0], avg_centroid[1], avg_centroid[2]);
    centroids_dominant_orientations_.push_back (cloud_centroid);

    //configure VFH computation using all object points
    vfh.setCentroidToUse (cloud_centroid);
    vfh.setUseGivenNormal (false);

    pcl::PointCloud<pcl::VFHSignature308> vfh_signature;
    vfh.compute (vfh_signature);

    output.resize (1);
    output.width = 1;

    output[0] = vfh_signature[0];
    Eigen::Matrix4f id = Eigen::Matrix4f::Identity ();
    transforms_.push_back (id);
    valid_transforms_.push_back (false);
  }
}

#define PCL_INSTANTIATE_OURCVFHEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::OURCVFHEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_OURCVFH_H_
