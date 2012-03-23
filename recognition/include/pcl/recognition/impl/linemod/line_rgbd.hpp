/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#ifndef PCL_RECOGNITION_LINEMOD_LINE_RGBD_IMPL_HPP_
#define PCL_RECOGNITION_LINEMOD_LINE_RGBD_IMPL_HPP_

//#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/io/pcd_io.h>
#include <fcntl.h>
#include <Eigen/Dense>
#ifdef _WIN32
# include <io.h>
# include <windows.h>
# define pcl_open                    _open
# define pcl_close(fd)               _close(fd)
# define pcl_lseek(fd,offset,origin) _lseek(fd,offset,origin)
#else
#include <unistd.h>
# include <sys/mman.h>
# define pcl_open                    open
# define pcl_close(fd)               close(fd)
# define pcl_lseek(fd,offset,origin) lseek(fd,offset,origin)
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointXYZT, typename PointRGBT> bool
pcl::LineRGBD<PointXYZT, PointRGBT>::readLTMHeader (int fd, pcl::io::TARHeader &header)
{
  // Read in the header
  int result = static_cast<int> (::read (fd, reinterpret_cast<char*> (&header.file_name[0]), 512));
  if (result == -1)
    return (false);

  // We only support regular files for now. 
  // Addional file types in TAR include: hard links, symbolic links, device/special files, block devices, 
  // directories, and named pipes.
  if (header.file_type[0] != '0' && header.file_type[0] != '\0')
    return (false);

  // We only support USTAR version 0 files for now
  if (std::string (header.ustar).substr (0, 5) != "ustar")
    return (false);

  if (header.getFileSize () == 0)
    return (false);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointXYZT, typename PointRGBT> bool
pcl::LineRGBD<PointXYZT, PointRGBT>::loadTemplates (const std::string &file_name, const size_t object_id)
{
  // Open the file
  int ltm_fd = pcl_open (file_name.c_str (), O_RDONLY);
  if (ltm_fd == -1)
    return (false);
  
  int ltm_offset = 0;

  pcl::io::TARHeader ltm_header;
  PCDReader pcd_reader;

  std::string pcd_ext (".pcd");
  std::string sqmmt_ext (".sqmmt");

  // While there still is an LTM header to be read
  while (readLTMHeader (ltm_fd, ltm_header))
  {
    ltm_offset += 512;

    // Search for extension
    std::string chunk_name (ltm_header.file_name);
    std::transform (chunk_name.begin (), chunk_name.end (), chunk_name.begin (), tolower);
    std::string::size_type it;

    if ((it = chunk_name.find (pcd_ext)) != std::string::npos &&
        (pcd_ext.size () - (chunk_name.size () - it)) == 0)
    {
      PCL_DEBUG ("[pcl::LineRGBD::loadTemplates] Reading and parsing %s as a PCD file.\n", chunk_name.c_str ());
      // Read the next PCD file
      template_point_clouds_.resize (template_point_clouds_.size () + 1);
      pcd_reader.read (file_name, template_point_clouds_[template_point_clouds_.size () - 1], ltm_offset);

      // Increment the offset for the next file
      ltm_offset += (ltm_header.getFileSize ()) + (512 - ltm_header.getFileSize () % 512);
    }
    else if ((it = chunk_name.find (sqmmt_ext)) != std::string::npos &&
             (sqmmt_ext.size () - (chunk_name.size () - it)) == 0)
    {
      PCL_DEBUG ("[pcl::LineRGBD::loadTemplates] Reading and parsing %s as a SQMMT file.\n", chunk_name.c_str ());

      unsigned int fsize = ltm_header.getFileSize ();
      char *buffer = new char[fsize];
      int result = static_cast<int> (::read (ltm_fd, reinterpret_cast<char*> (&buffer[0]), fsize));
      if (result == -1)
      {
        delete [] buffer;
        PCL_ERROR ("[pcl::LineRGBD::loadTemplates] Error reading SQMMT template from file!\n");
        break;
      }

      // Read a SQMMT file
      std::stringstream stream (std::stringstream::in | std::stringstream::out | std::stringstream::binary);
      stream.write (buffer, fsize);
      SparseQuantizedMultiModTemplate sqmmt;
      sqmmt.deserialize (stream);
      PCL_DEBUG ("%zu %d %d %d %d\n", 
                 sqmmt.features.size (), 
                 sqmmt.region.x, sqmmt.region.y, sqmmt.region.width, sqmmt.region.height);
      linemod_.addTemplate (sqmmt);
      object_ids_.push_back (object_id);

      // Increment the offset for the next file
      ltm_offset += (ltm_header.getFileSize ()) + (512 - ltm_header.getFileSize () % 512);

      delete [] buffer;
    }

    if (static_cast<int> (pcl_lseek (ltm_fd, ltm_offset, SEEK_SET)) < 0)
      break;
  }

  // Close the file
  pcl_close (ltm_fd);

  // Compute 3D bounding boxes from the template point clouds
  bounding_boxes_.resize (template_point_clouds_.size ());
  for (size_t i = 0; i < template_point_clouds_.size (); ++i)
  {
    const PointCloud<PointXYZRGB> & template_point_cloud = template_point_clouds_[i];
    BoundingBoxXYZ & bb = bounding_boxes_[i];
    bb.x = bb.y = bb.z = std::numeric_limits<float>::max ();
    bb.width = bb.height = bb.depth = 0.0f;

    float center_x = 0.0f;
    float center_y = 0.0f;
    float center_z = 0.0f;
    for (size_t j = 0; j < template_point_cloud.size (); ++j)
    {
      const PointXYZRGB & p = template_point_cloud.points[j];
      bb.x = std::min (bb.x, p.x);
      bb.y = std::min (bb.y, p.y);
      bb.z = std::min (bb.z, p.z);
      bb.width = std::max (bb.width, p.x - bb.x);
      bb.height = std::max (bb.height, p.y - bb.y);
      bb.depth = std::max (bb.depth, p.z - bb.z);

      center_x += p.x;
      center_y += p.y;
      center_z += p.z;
    }

    //bb.x = (bb.width / 2.0f - center_x / template_point_cloud.size ()) - bb.width / 2.0f;
    //bb.y = (bb.height / 2.0f - center_y / template_point_cloud.size ()) - bb.height / 2.0f;
    //bb.z = (bb.depth / 2.0f - center_z / template_point_cloud.size ()) - bb.depth / 2.0f;

    bb.x = - bb.width / 2.0f;
    bb.y = - bb.height / 2.0f;
    bb.z = - bb.depth / 2.0f;
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointXYZT, typename PointRGBT> void 
pcl::LineRGBD<PointXYZT, PointRGBT>::detect (
    std::vector<typename pcl::LineRGBD<PointXYZT, PointRGBT>::Detection> & detections)
{
  std::vector<pcl::QuantizableModality*> modalities;
  modalities.push_back (&color_gradient_mod_);
  modalities.push_back (&surface_normal_mod_);

  std::vector<pcl::LINEMODDetection> linemod_detections;
  linemod_.detectTemplates (modalities, linemod_detections);

  detections_.clear ();
  detections_.reserve (linemod_detections.size ());
  detections.clear ();
  detections.reserve (linemod_detections.size ());
  PCL_DEBUG ("%zu\n", linemod_detections.size ());
  for (size_t detection_id = 0; detection_id < linemod_detections.size (); ++detection_id)
  {
    pcl::LINEMODDetection & linemod_detection = linemod_detections[detection_id];

    pcl::LineRGBD<PointXYZT, PointRGBT>::Detection detection;
    detection.template_id = linemod_detection.template_id;
    detection.object_id = object_ids_[linemod_detection.template_id];
    detection.detection_id = detection_id;
    detection.response = linemod_detection.score;

    // compute bounding box:
    // we assume that the bounding boxes of the templates are relative to the center of mass 
    // of the template points; so we also compute the center of mass of the points
    // covered by the 

    const pcl::SparseQuantizedMultiModTemplate & linemod_template = linemod_.getTemplate (linemod_detection.template_id);

    const size_t start_x = std::max (linemod_detection.x, 0);
    const size_t start_y = std::max (linemod_detection.y, 0);
    const size_t end_x = std::min (static_cast<size_t> (start_x + linemod_template.region.width), static_cast<size_t> (cloud_xyz_->width));
    const size_t end_y = std::min (static_cast<size_t> (start_y + linemod_template.region.height), static_cast<size_t> (cloud_xyz_->height));

    float center_x = 0.0f;
    float center_y = 0.0f;
    float center_z = 0.0f;
    size_t counter = 0;
    for (size_t row_index = start_y; row_index < end_y; ++row_index)
    {
      for (size_t col_index = start_x; col_index < end_x; ++col_index)
      {
        const PointXYZT & point = (*cloud_xyz_) (col_index, row_index);

        if (pcl_isfinite (point.x) && pcl_isfinite (point.y) && pcl_isfinite (point.z))
        {
          center_x += point.x;
          center_y += point.y;
          center_z += point.z;
          ++counter;
        }
      }
    }
    const float inv_counter = 1.0f / static_cast<float> (counter);
    center_x *= inv_counter;
    center_y *= inv_counter;
    center_z *= inv_counter;

    pcl::BoundingBoxXYZ template_bounding_box = bounding_boxes_[detection.template_id];

    detection.bounding_box = template_bounding_box;
    detection.bounding_box.x += center_x;
    detection.bounding_box.y += center_y;
    detection.bounding_box.z += center_z;

    detections_.push_back (detection);
  }

  // remove overlaps
  removeOverlappingDetections ();

  for (size_t detection_index = 0; detection_index < detections_.size (); ++detection_index)
  {
    detections.push_back (detections_[detection_index]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointXYZT, typename PointRGBT> void 
pcl::LineRGBD<PointXYZT, PointRGBT>::computeTransformedTemplatePoints (
    const size_t detection_id, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
  if (detection_id >= detections_.size ())
    std::cerr << "ERROR pcl::LineRGBD::computeTransformedTemplatePoints - detection_id is out of bounds" << std::endl;

  const size_t template_id = detections_[detection_id].template_id;
  pcl::PointCloud<pcl::PointXYZRGB> & template_point_cloud = template_point_clouds_[template_id];

  const pcl::BoundingBoxXYZ & template_bounding_box = bounding_boxes_[template_id];
  const pcl::BoundingBoxXYZ & detection_bounding_box = detections_[detection_id].bounding_box;

  const float translation_x = detection_bounding_box.x - template_bounding_box.x;
  const float translation_y = detection_bounding_box.y - template_bounding_box.y;
  const float translation_z = detection_bounding_box.z - template_bounding_box.z;

  const size_t nr_points = template_point_cloud.size ();
  cloud.resize (nr_points);
  for (size_t point_index = 0; point_index < nr_points; ++point_index)
  {
    pcl::PointXYZRGB point = template_point_cloud.points[point_index];

    point.x += translation_x;
    point.y += translation_y;
    point.z += translation_z;

    cloud.points[point_index] = point;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointXYZT, typename PointRGBT> void 
pcl::LineRGBD<PointXYZT, PointRGBT>::removeOverlappingDetections ()
{
  // compute overlap between each detection
  const size_t nr_detections = detections_.size ();
  Eigen::MatrixXf overlaps (nr_detections, nr_detections);
  for (size_t detection_index_1 = 0; detection_index_1 < nr_detections; ++detection_index_1)
  {
    for (size_t detection_index_2 = detection_index_1+1; detection_index_2 < nr_detections; ++detection_index_2)
    {
      const float bounding_box_volume = detections_[detection_index_1].bounding_box.width
                                      * detections_[detection_index_1].bounding_box.height
                                      * detections_[detection_index_1].bounding_box.depth;

      if (detections_[detection_index_1].object_id != detections_[detection_index_2].object_id)
        overlaps (detection_index_1, detection_index_2) = 0.0f;
      else
        overlaps (detection_index_1, detection_index_2) = computeBoundingBoxIntersectionVolume (
          detections_[detection_index_1].bounding_box, 
          detections_[detection_index_2].bounding_box) / bounding_box_volume;
    }
  }

  // create clusters of detections
  std::vector<int> detection_to_cluster_mapping (nr_detections, -1);
  std::vector<std::vector<size_t> > clusters;
  for (size_t detection_index = 0; detection_index < nr_detections; ++detection_index)
  {
    if (detection_to_cluster_mapping[detection_index] != -1)
      continue; // already part of a cluster

    std::vector<size_t> cluster;
    const size_t cluster_id = clusters.size ();

    cluster.push_back (detection_index);
    detection_to_cluster_mapping[detection_index] = cluster_id;

    // check for detections which have sufficient overlap with a detection in the cluster
    for (size_t cluster_index = 0; cluster_index < cluster.size (); ++cluster_index)
    {
      const size_t detection_index_1 = cluster[cluster_index];

      for (size_t detection_index_2 = detection_index_1+1; detection_index_2 < nr_detections; ++detection_index_2)
      {
        if (detection_to_cluster_mapping[detection_index_2] != -1)
          continue; // already part of a cluster

        if (overlaps (detection_index_1, detection_index_2) < intersection_volume_threshold_)
          continue; // not enough overlap

        cluster.push_back (detection_index_2);
        detection_to_cluster_mapping[detection_index_2] = cluster_id;
      }
    }

    clusters.push_back (cluster);
  }

  // compute detection representatives for every cluster
  std::vector<LineRGBD<PointXYZT, PointRGBT>::Detection> clustered_detections;

  const size_t nr_clusters = clusters.size ();
  for (size_t cluster_id = 0; cluster_id < nr_clusters; ++cluster_id)
  {
    std::vector<size_t> & cluster = clusters[cluster_id];
    
    float average_center_x = 0.0f;
    float average_center_y = 0.0f;
    float average_center_z = 0.0f;
    float weight_sum = 0.0f;

    float best_response = 0.0f;
    size_t best_detection_id = 0;

    const size_t elements_in_cluster = cluster.size ();
    for (size_t cluster_index = 0; cluster_index < elements_in_cluster; ++cluster_index)
    {
      const size_t detection_id = cluster[cluster_index];

      const float weight = detections_[detection_id].response;

      if (weight > best_response)
      {
        best_response = weight;
        best_detection_id = detection_id;
      }

      const float p_center_x = detections_[detection_id].bounding_box.x + detections_[detection_id].bounding_box.width/2.0f;
      const float p_center_y = detections_[detection_id].bounding_box.y + detections_[detection_id].bounding_box.height/2.0f;
      const float p_center_z = detections_[detection_id].bounding_box.z + detections_[detection_id].bounding_box.depth/2.0f;

      average_center_x += p_center_x * weight;
      average_center_y += p_center_y * weight;
      average_center_z += p_center_z * weight;
      weight_sum += weight;
    }

    LineRGBD<PointXYZT, PointRGBT>::Detection detection;
    detection.template_id = detections_[best_detection_id].template_id;
    detection.object_id = detections_[best_detection_id].object_id;
    detection.detection_id = cluster_id;
    detection.response = best_response;

    const float inv_weight_sum = 1.0f / weight_sum;
    const float best_detection_bb_width  = detections_[best_detection_id].bounding_box.width;
    const float best_detection_bb_height = detections_[best_detection_id].bounding_box.height;
    const float best_detection_bb_depth  = detections_[best_detection_id].bounding_box.depth;

    detection.bounding_box.x = average_center_x * inv_weight_sum - best_detection_bb_width/2.0f;
    detection.bounding_box.y = average_center_y * inv_weight_sum - best_detection_bb_height/2.0f;
    detection.bounding_box.z = average_center_z * inv_weight_sum - best_detection_bb_depth/2.0f;
    detection.bounding_box.width  = best_detection_bb_width;
    detection.bounding_box.height = best_detection_bb_height;
    detection.bounding_box.depth  = best_detection_bb_depth;

    clustered_detections.push_back (detection);
  }

  detections_ = clustered_detections;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointXYZT, typename PointRGBT> float 
pcl::LineRGBD<PointXYZT, PointRGBT>::computeBoundingBoxIntersectionVolume (
  const BoundingBoxXYZ &box1, const BoundingBoxXYZ &box2)
{
  const float x1_min = box1.x;
  const float y1_min = box1.y;
  const float z1_min = box1.z;
  const float x1_max = box1.x + box1.width;
  const float y1_max = box1.y + box1.height;
  const float z1_max = box1.z + box1.depth;

  const float x2_min = box2.x;
  const float y2_min = box2.y;
  const float z2_min = box2.z;
  const float x2_max = box2.x + box2.width;
  const float y2_max = box2.y + box2.height;
  const float z2_max = box2.z + box2.depth;
  
  const float xi_min = std::max (x1_min, x2_min);
  const float yi_min = std::max (y1_min, y2_min);
  const float zi_min = std::max (z1_min, z2_min);

  const float xi_max = std::min (x1_max, x2_max);
  const float yi_max = std::min (y1_max, y2_max);
  const float zi_max = std::min (z1_max, z2_max);

  const float intersection_width  = xi_max - xi_min;
  const float intersection_height = yi_max - yi_min;
  const float intersection_depth  = zi_max - zi_min;

  if (intersection_width <= 0.0f || intersection_height <= 0.0f || intersection_depth <= 0.0f)
    return 0.0f;

  const float intersection_volume = intersection_width * intersection_height * intersection_depth;

  return (intersection_volume);
}


#endif        // PCL_RECOGNITION_LINEMOD_LINE_RGBD_IMPL_HPP_ 

