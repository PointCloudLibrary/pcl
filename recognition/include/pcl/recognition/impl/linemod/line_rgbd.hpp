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
#include <pcl/point_cloud.h>
#include <limits>
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

    std::transform (chunk_name.begin (), chunk_name.end (), chunk_name.begin (), ::tolower);
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
    PointCloud<PointXYZRGBA> & template_point_cloud = template_point_clouds_[i];
    BoundingBoxXYZ & bb = bounding_boxes_[i];
    bb.x = bb.y = bb.z = std::numeric_limits<float>::max ();
    bb.width = bb.height = bb.depth = 0.0f;

    float center_x = 0.0f;
    float center_y = 0.0f;
    float center_z = 0.0f;
    float min_x = std::numeric_limits<float>::max ();
    float min_y = std::numeric_limits<float>::max ();
    float min_z = std::numeric_limits<float>::max ();
    float max_x = -std::numeric_limits<float>::max ();
    float max_y = -std::numeric_limits<float>::max ();
    float max_z = -std::numeric_limits<float>::max ();
    size_t counter = 0;
    for (size_t j = 0; j < template_point_cloud.size (); ++j)
    {
      const PointXYZRGBA & p = template_point_cloud.points[j];

      if (!pcl_isfinite (p.x) || !pcl_isfinite (p.y) || !pcl_isfinite (p.z))
        continue;

      min_x = std::min (min_x, p.x);
      min_y = std::min (min_y, p.y);
      min_z = std::min (min_z, p.z);
      max_x = std::max (max_x, p.x);
      max_y = std::max (max_y, p.y);
      max_z = std::max (max_z, p.z);

      center_x += p.x;
      center_y += p.y;
      center_z += p.z;

      ++counter;
    }

    center_x /= static_cast<float> (counter);
    center_y /= static_cast<float> (counter);
    center_z /= static_cast<float> (counter);

    bb.width  = max_x - min_x;
    bb.height = max_y - min_y;
    bb.depth  = max_z - min_z;

    bb.x = (min_x + bb.width / 2.0f) - center_x - bb.width / 2.0f;
    bb.y = (min_y + bb.height / 2.0f) - center_y - bb.height / 2.0f;
    bb.z = (min_z + bb.depth / 2.0f) - center_z - bb.depth / 2.0f;

    for (size_t j = 0; j < template_point_cloud.size (); ++j)
    {
      PointXYZRGBA p = template_point_cloud.points[j];

      if (!pcl_isfinite (p.x) || !pcl_isfinite (p.y) || !pcl_isfinite (p.z))
        continue;

      p.x -= center_x;
      p.y -= center_y;
      p.z -= center_z;

      template_point_cloud.points[j] = p;
    }
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointXYZT, typename PointRGBT> int
pcl::LineRGBD<PointXYZT, PointRGBT>::createAndAddTemplate (
  pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
  const size_t object_id,
  const MaskMap & mask_xyz,
  const MaskMap & mask_rgb,
  const RegionXY & region)
{
  // add point cloud
  template_point_clouds_.resize (template_point_clouds_.size () + 1);
  pcl::copyPointCloud (cloud, template_point_clouds_[template_point_clouds_.size () - 1]);

  // add template
  object_ids_.push_back (object_id);

  // Compute 3D bounding boxes from the template point clouds
  bounding_boxes_.resize (template_point_clouds_.size ());
  {
    const size_t i = template_point_clouds_.size () - 1;

    PointCloud<PointXYZRGBA> & template_point_cloud = template_point_clouds_[i];
    BoundingBoxXYZ & bb = bounding_boxes_[i];
    bb.x = bb.y = bb.z = std::numeric_limits<float>::max ();
    bb.width = bb.height = bb.depth = 0.0f;

    float center_x = 0.0f;
    float center_y = 0.0f;
    float center_z = 0.0f;
    float min_x = std::numeric_limits<float>::max ();
    float min_y = std::numeric_limits<float>::max ();
    float min_z = std::numeric_limits<float>::max ();
    float max_x = -std::numeric_limits<float>::max ();
    float max_y = -std::numeric_limits<float>::max ();
    float max_z = -std::numeric_limits<float>::max ();
    size_t counter = 0;
    for (size_t j = 0; j < template_point_cloud.size (); ++j)
    {
      const PointXYZRGBA & p = template_point_cloud.points[j];

      if (!pcl_isfinite (p.x) || !pcl_isfinite (p.y) || !pcl_isfinite (p.z))
        continue;

      min_x = std::min (min_x, p.x);
      min_y = std::min (min_y, p.y);
      min_z = std::min (min_z, p.z);
      max_x = std::max (max_x, p.x);
      max_y = std::max (max_y, p.y);
      max_z = std::max (max_z, p.z);

      center_x += p.x;
      center_y += p.y;
      center_z += p.z;

      ++counter;
    }

    center_x /= static_cast<float> (counter);
    center_y /= static_cast<float> (counter);
    center_z /= static_cast<float> (counter);

    bb.width  = max_x - min_x;
    bb.height = max_y - min_y;
    bb.depth  = max_z - min_z;

    bb.x = (min_x + bb.width / 2.0f) - center_x - bb.width / 2.0f;
    bb.y = (min_y + bb.height / 2.0f) - center_y - bb.height / 2.0f;
    bb.z = (min_z + bb.depth / 2.0f) - center_z - bb.depth / 2.0f;

    for (size_t j = 0; j < template_point_cloud.size (); ++j)
    {
      PointXYZRGBA p = template_point_cloud.points[j];

      if (!pcl_isfinite (p.x) || !pcl_isfinite (p.y) || !pcl_isfinite (p.z))
        continue;

      p.x -= center_x;
      p.y -= center_y;
      p.z -= center_z;

      template_point_cloud.points[j] = p;
    }
  }

  std::vector<pcl::QuantizableModality*> modalities;
  modalities.push_back (&color_gradient_mod_);
  modalities.push_back (&surface_normal_mod_);

  std::vector<MaskMap*> masks;
  masks.push_back (const_cast<MaskMap*> (&mask_rgb));
  masks.push_back (const_cast<MaskMap*> (&mask_xyz));

  return (linemod_.createAndAddTemplate (modalities, masks, region));
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointXYZT, typename PointRGBT> bool
pcl::LineRGBD<PointXYZT, PointRGBT>::addTemplate (const SparseQuantizedMultiModTemplate & sqmmt, pcl::PointCloud<pcl::PointXYZRGBA> & cloud, size_t object_id)
{
  // add point cloud
  template_point_clouds_.resize (template_point_clouds_.size () + 1);
  pcl::copyPointCloud (cloud, template_point_clouds_[template_point_clouds_.size () - 1]);

  // add template
  linemod_.addTemplate (sqmmt);
  object_ids_.push_back (object_id);

  // Compute 3D bounding boxes from the template point clouds
  bounding_boxes_.resize (template_point_clouds_.size ());
  {
    const size_t i = template_point_clouds_.size () - 1;

    PointCloud<PointXYZRGBA> & template_point_cloud = template_point_clouds_[i];
    BoundingBoxXYZ & bb = bounding_boxes_[i];
    bb.x = bb.y = bb.z = std::numeric_limits<float>::max ();
    bb.width = bb.height = bb.depth = 0.0f;

    float center_x = 0.0f;
    float center_y = 0.0f;
    float center_z = 0.0f;
    float min_x = std::numeric_limits<float>::max ();
    float min_y = std::numeric_limits<float>::max ();
    float min_z = std::numeric_limits<float>::max ();
    float max_x = -std::numeric_limits<float>::max ();
    float max_y = -std::numeric_limits<float>::max ();
    float max_z = -std::numeric_limits<float>::max ();
    size_t counter = 0;
    for (size_t j = 0; j < template_point_cloud.size (); ++j)
    {
      const PointXYZRGBA & p = template_point_cloud.points[j];

      if (!pcl_isfinite (p.x) || !pcl_isfinite (p.y) || !pcl_isfinite (p.z))
        continue;

      min_x = std::min (min_x, p.x);
      min_y = std::min (min_y, p.y);
      min_z = std::min (min_z, p.z);
      max_x = std::max (max_x, p.x);
      max_y = std::max (max_y, p.y);
      max_z = std::max (max_z, p.z);

      center_x += p.x;
      center_y += p.y;
      center_z += p.z;

      ++counter;
    }

    center_x /= static_cast<float> (counter);
    center_y /= static_cast<float> (counter);
    center_z /= static_cast<float> (counter);

    bb.width  = max_x - min_x;
    bb.height = max_y - min_y;
    bb.depth  = max_z - min_z;

    bb.x = (min_x + bb.width / 2.0f) - center_x - bb.width / 2.0f;
    bb.y = (min_y + bb.height / 2.0f) - center_y - bb.height / 2.0f;
    bb.z = (min_z + bb.depth / 2.0f) - center_z - bb.depth / 2.0f;

    for (size_t j = 0; j < template_point_cloud.size (); ++j)
    {
      PointXYZRGBA p = template_point_cloud.points[j];

      if (!pcl_isfinite (p.x) || !pcl_isfinite (p.y) || !pcl_isfinite (p.z))
        continue;

      p.x -= center_x;
      p.y -= center_y;
      p.z -= center_z;

      template_point_cloud.points[j] = p;
    }
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
  for (size_t detection_id = 0; detection_id < linemod_detections.size (); ++detection_id)
  {
    pcl::LINEMODDetection & linemod_detection = linemod_detections[detection_id];

    typename pcl::LineRGBD<PointXYZT, PointRGBT>::Detection detection;
    detection.template_id = linemod_detection.template_id;
    detection.object_id = object_ids_[linemod_detection.template_id];
    detection.detection_id = detection_id;
    detection.response = linemod_detection.score;

    // compute bounding box:
    // we assume that the bounding boxes of the templates are relative to the center of mass 
    // of the template points; so we also compute the center of mass of the points
    // covered by the 

    const pcl::SparseQuantizedMultiModTemplate & linemod_template = 
      linemod_.getTemplate (linemod_detection.template_id);

    const size_t start_x = std::max (linemod_detection.x, 0);
    const size_t start_y = std::max (linemod_detection.y, 0);
    const size_t end_x = std::min (static_cast<size_t> (start_x + linemod_template.region.width),
                                   static_cast<size_t> (cloud_xyz_->width));
    const size_t end_y = std::min (static_cast<size_t> (start_y + linemod_template.region.height),
                                   static_cast<size_t> (cloud_xyz_->height));

    detection.region.x = linemod_detection.x;
    detection.region.y = linemod_detection.y;
    detection.region.width  = linemod_template.region.width;
    detection.region.height = linemod_template.region.height;

    //std::cerr << "detection region: " << linemod_detection.x << ", "
    //  << linemod_detection.y << ", "
    //  << linemod_template.region.width << ", "
    //  << linemod_template.region.height << std::endl;

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

  // refine detections along depth
  refineDetectionsAlongDepth ();
  //applyprojectivedepthicpondetections();

  // remove overlaps
  removeOverlappingDetections ();

  for (size_t detection_index = 0; detection_index < detections_.size (); ++detection_index)
  {
    detections.push_back (detections_[detection_index]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointXYZT, typename PointRGBT> void 
pcl::LineRGBD<PointXYZT, PointRGBT>::detectSemiScaleInvariant (
    std::vector<typename pcl::LineRGBD<PointXYZT, PointRGBT>::Detection> & detections,
    const float min_scale,
    const float max_scale,
    const float scale_multiplier)
{
  std::vector<pcl::QuantizableModality*> modalities;
  modalities.push_back (&color_gradient_mod_);
  modalities.push_back (&surface_normal_mod_);

  std::vector<pcl::LINEMODDetection> linemod_detections;
  linemod_.detectTemplatesSemiScaleInvariant (modalities, linemod_detections, min_scale, max_scale, scale_multiplier);

  detections_.clear ();
  detections_.reserve (linemod_detections.size ());
  detections.clear ();
  detections.reserve (linemod_detections.size ());
  for (size_t detection_id = 0; detection_id < linemod_detections.size (); ++detection_id)
  {
    pcl::LINEMODDetection & linemod_detection = linemod_detections[detection_id];

    typename pcl::LineRGBD<PointXYZT, PointRGBT>::Detection detection;
    detection.template_id = linemod_detection.template_id;
    detection.object_id = object_ids_[linemod_detection.template_id];
    detection.detection_id = detection_id;
    detection.response = linemod_detection.score;

    // compute bounding box:
    // we assume that the bounding boxes of the templates are relative to the center of mass 
    // of the template points; so we also compute the center of mass of the points
    // covered by the 

    const pcl::SparseQuantizedMultiModTemplate & linemod_template = 
      linemod_.getTemplate (linemod_detection.template_id);

    const size_t start_x = std::max (linemod_detection.x, 0);
    const size_t start_y = std::max (linemod_detection.y, 0);
    const size_t end_x = std::min (static_cast<size_t> (start_x + linemod_template.region.width * linemod_detection.scale),
                                   static_cast<size_t> (cloud_xyz_->width));
    const size_t end_y = std::min (static_cast<size_t> (start_y + linemod_template.region.height * linemod_detection.scale),
                                   static_cast<size_t> (cloud_xyz_->height));

    detection.region.x = linemod_detection.x;
    detection.region.y = linemod_detection.y;
    detection.region.width  = linemod_template.region.width * linemod_detection.scale;
    detection.region.height = linemod_template.region.height * linemod_detection.scale;

    //std::cerr << "detection region: " << linemod_detection.x << ", "
    //  << linemod_detection.y << ", "
    //  << linemod_template.region.width << ", "
    //  << linemod_template.region.height << std::endl;

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

  // refine detections along depth
  //refineDetectionsAlongDepth ();
  //applyProjectiveDepthICPOnDetections();

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
    const size_t detection_id, pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
  if (detection_id >= detections_.size ())
    PCL_ERROR ("ERROR pcl::LineRGBD::computeTransformedTemplatePoints - detection_id is out of bounds\n");

  const size_t template_id = detections_[detection_id].template_id;
  const pcl::PointCloud<pcl::PointXYZRGBA> & template_point_cloud = template_point_clouds_[template_id];

  const pcl::BoundingBoxXYZ & template_bounding_box = bounding_boxes_[template_id];
  const pcl::BoundingBoxXYZ & detection_bounding_box = detections_[detection_id].bounding_box;

  //std::cerr << "detection: " 
  //  << detection_bounding_box.x << ", "
  //  << detection_bounding_box.y << ", "
  //  << detection_bounding_box.z << std::endl;
  //std::cerr << "template: " 
  //  << template_bounding_box.x << ", "
  //  << template_bounding_box.y << ", "
  //  << template_bounding_box.z << std::endl;
  const float translation_x = detection_bounding_box.x - template_bounding_box.x;
  const float translation_y = detection_bounding_box.y - template_bounding_box.y;
  const float translation_z = detection_bounding_box.z - template_bounding_box.z;

  //std::cerr << "translation: " 
  //  << translation_x << ", "
  //  << translation_y << ", "
  //  << translation_z << std::endl;

  const size_t nr_points = template_point_cloud.size ();
  cloud.resize (nr_points);
  cloud.width = template_point_cloud.width;
  cloud.height = template_point_cloud.height;
  for (size_t point_index = 0; point_index < nr_points; ++point_index)
  {
    pcl::PointXYZRGBA point = template_point_cloud.points[point_index];

    point.x += translation_x;
    point.y += translation_y;
    point.z += translation_z;

    cloud.points[point_index] = point;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointXYZT, typename PointRGBT> void 
pcl::LineRGBD<PointXYZT, PointRGBT>::refineDetectionsAlongDepth ()
{
  const size_t nr_detections = detections_.size ();
  for (size_t detection_index = 0; detection_index < nr_detections; ++detection_index)
  {
    typename LineRGBD<PointXYZT, PointRGBT>::Detection & detection = detections_[detection_index];

    // find depth with most valid points
    const size_t start_x = std::max (detection.region.x, 0);
    const size_t start_y = std::max (detection.region.y, 0);
    const size_t end_x = std::min (static_cast<size_t> (detection.region.x + detection.region.width),
                                   static_cast<size_t> (cloud_xyz_->width));
    const size_t end_y = std::min (static_cast<size_t> (detection.region.y + detection.region.height),
                                   static_cast<size_t> (cloud_xyz_->height));


    float min_depth = std::numeric_limits<float>::max ();
    float max_depth = -std::numeric_limits<float>::max ();
    for (size_t row_index = start_y; row_index < end_y; ++row_index)
    {
      for (size_t col_index = start_x; col_index < end_x; ++col_index)
      {
        const PointXYZT & point = (*cloud_xyz_) (col_index, row_index);

        if (/*pcl_isfinite (point.x) && pcl_isfinite (point.y) && */pcl_isfinite (point.z))
        {
          min_depth = std::min (min_depth, point.z);
          max_depth = std::max (max_depth, point.z);
        }
      }
    }

    const size_t nr_bins = 1000;
    const float step_size = (max_depth - min_depth) / static_cast<float> (nr_bins-1);
    std::vector<size_t> depth_bins (nr_bins, 0);
    for (size_t row_index = start_y; row_index < end_y; ++row_index)
    {
      for (size_t col_index = start_x; col_index < end_x; ++col_index)
      {
        const PointXYZT & point = (*cloud_xyz_) (col_index, row_index);

        if (/*pcl_isfinite (point.x) && pcl_isfinite (point.y) && */pcl_isfinite (point.z))
        {
          const size_t bin_index = static_cast<size_t> ((point.z - min_depth) / step_size);
          ++depth_bins[bin_index];
        }
      }
    }

    std::vector<size_t> integral_depth_bins (nr_bins, 0);
    
    integral_depth_bins[0] = depth_bins[0];
    for (size_t bin_index = 1; bin_index < nr_bins; ++bin_index)
    {
      integral_depth_bins[bin_index] = depth_bins[bin_index] + integral_depth_bins[bin_index-1];
    }

    const size_t bb_depth_range = static_cast<size_t> (detection.bounding_box.depth / step_size);

    size_t max_nr_points = 0;
    size_t max_index = 0;
    for (size_t bin_index = 0; (bin_index+bb_depth_range) < nr_bins; ++bin_index)
    {
      const size_t nr_points_in_range = integral_depth_bins[bin_index+bb_depth_range] - integral_depth_bins[bin_index];

      if (nr_points_in_range > max_nr_points)
      {
        max_nr_points = nr_points_in_range;
        max_index = bin_index;
      }
    }

    const float z = static_cast<float> (max_index) * step_size + min_depth;

    detection.bounding_box.z = z;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointXYZT, typename PointRGBT> void 
pcl::LineRGBD<PointXYZT, PointRGBT>::applyProjectiveDepthICPOnDetections ()
{
  const size_t nr_detections = detections_.size ();
  for (size_t detection_index = 0; detection_index < nr_detections; ++detection_index)
  {
    typename pcl::LineRGBD<PointXYZT, PointRGBT>::Detection & detection = detections_[detection_index];

    const size_t template_id = detection.template_id;
    pcl::PointCloud<pcl::PointXYZRGBA> & point_cloud = template_point_clouds_[template_id];

    const size_t start_x = detection.region.x;
    const size_t start_y = detection.region.y;
    const size_t pc_width = point_cloud.width;
    const size_t pc_height = point_cloud.height;
    
    std::vector<std::pair<float, float> > depth_matches;
    for (size_t row_index = 0; row_index < pc_height; ++row_index)
    {
      for (size_t col_index = 0; col_index < pc_width; ++col_index)
      {
        const pcl::PointXYZRGBA & point_template = point_cloud (col_index, row_index);
        const PointXYZT & point_input = (*cloud_xyz_) (col_index + start_x, row_index + start_y);

        if (!pcl_isfinite (point_template.z) || !pcl_isfinite (point_input.z))
          continue;

        depth_matches.push_back (std::make_pair (point_template.z, point_input.z));
      }
    }

    // apply ransac on matches
    const size_t nr_matches = depth_matches.size ();
    const size_t nr_iterations = 100; // todo: should be a parameter...
    const float inlier_threshold = 0.01f; // 5cm // todo: should be a parameter...
    size_t best_nr_inliers = 0;
    float best_z_translation = 0.0f;
    for (size_t iteration_index = 0; iteration_index < nr_iterations; ++iteration_index)
    {
      const size_t rand_index = (rand () * nr_matches) / RAND_MAX;

      const float z_translation = depth_matches[rand_index].second - depth_matches[rand_index].first;

      size_t nr_inliers = 0;
      for (size_t match_index = 0; match_index < nr_matches; ++match_index)
      {
        const float error = fabsf (depth_matches[match_index].first + z_translation - depth_matches[match_index].second);

        if (error <= inlier_threshold)
        {
          ++nr_inliers;
        }
      }

      if (nr_inliers > best_nr_inliers)
      {
        best_nr_inliers = nr_inliers;
        best_z_translation = z_translation;
      }
    }

    float average_depth = 0.0f;
    size_t average_counter = 0;
    for (size_t match_index = 0; match_index < nr_matches; ++match_index)
    {
      const float error = fabsf (depth_matches[match_index].first + best_z_translation - depth_matches[match_index].second);

      if (error <= inlier_threshold)
      {
        //average_depth += depth_matches[match_index].second;
        average_depth += depth_matches[match_index].second - depth_matches[match_index].first;
        ++average_counter;
      }
    }
    average_depth /= static_cast<float> (average_counter);

    detection.bounding_box.z = bounding_boxes_[template_id].z + average_depth;// - detection.bounding_box.depth/2.0f;
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
    detection_to_cluster_mapping[detection_index] = static_cast<int> (cluster_id);

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
        detection_to_cluster_mapping[detection_index_2] = static_cast<int> (cluster_id);
      }
    }

    clusters.push_back (cluster);
  }

  // compute detection representatives for every cluster
  std::vector<typename LineRGBD<PointXYZT, PointRGBT>::Detection> clustered_detections;

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

    float average_region_x = 0.0f;
    float average_region_y = 0.0f;

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

      const Detection & d = detections_[detection_id];
      const float p_center_x = d.bounding_box.x + d.bounding_box.width / 2.0f;
      const float p_center_y = d.bounding_box.y + d.bounding_box.height / 2.0f;
      const float p_center_z = d.bounding_box.z + d.bounding_box.depth / 2.0f;

      average_center_x += p_center_x * weight;
      average_center_y += p_center_y * weight;
      average_center_z += p_center_z * weight;
      weight_sum += weight;

      average_region_x += float (detections_[detection_id].region.x) * weight;
      average_region_y += float (detections_[detection_id].region.y) * weight;
    }

    typename LineRGBD<PointXYZT, PointRGBT>::Detection detection;
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

    detection.region.x = int (average_region_x * inv_weight_sum);
    detection.region.y = int (average_region_y * inv_weight_sum);
    detection.region.width = detections_[best_detection_id].region.width;
    detection.region.height = detections_[best_detection_id].region.height;

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

