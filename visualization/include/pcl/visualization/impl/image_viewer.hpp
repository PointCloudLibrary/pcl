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
 * $Id$
 */

#ifndef PCL_VISUALIZATION_IMAGE_VISUALIZER_HPP_
#define	PCL_VISUALIZATION_IMAGE_VISUALIZER_HPP_

#include <pcl/search/organized.h>

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> void
pcl::visualization::ImageViewer::showRGBImage (const pcl::PointCloud<T> &cloud,
                                               const std::string &layer_id,
                                               double opacity)
{
  if (data_size_ < cloud.width * cloud.height)
  {
    data_size_ = cloud.width * cloud.height * 3;
    data_.reset (new unsigned char[data_size_]);
  }
  
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    memcpy (&data_[i * 3], reinterpret_cast<const unsigned char*> (&cloud.points[i].rgba), sizeof (unsigned char) * 3);
    /// Convert from BGR to RGB
    unsigned char aux = data_[i*3];
    data_[i*3] = data_[i*3+2];
    data_[i*3+2] = aux;
    for (int j = 0; j < 3; ++j)
      if (pcl_isnan (data_[i * 3 + j]))
          data_[i * 3 + j] = 0;
  }
  return (showRGBImage (data_.get (), cloud.width, cloud.height, layer_id, opacity));
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> bool
pcl::visualization::ImageViewer::addMask (
    const typename pcl::PointCloud<T>::ConstPtr &image,
    const pcl::PointCloud<T> &mask, 
    double r, double g, double b,
    const std::string &layer_id, double opacity)
{
  // We assume that the data passed into image is organized, otherwise this doesn't make sense
  if (!image->isOrganized ())
    return (false);

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addMask] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->SetDrawColor (r * 255.0, g * 255.0, b * 255.0, opacity * 255.0);

  // Construct a search object to get the camera parameters
  pcl::search::OrganizedNeighbor<T> search;
  search.setInputCloud (image);
  for (size_t i = 0; i < mask.points.size (); ++i)
  {
    pcl::PointXY p_projected;
    search.projectPoint (mask.points[i], p_projected);

    am_it->canvas->DrawPoint (int (p_projected.x), 
                              int (float (image->height) - p_projected.y));
  }

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename T> bool
pcl::visualization::ImageViewer::addMask (
    const typename pcl::PointCloud<T>::ConstPtr &image,
    const pcl::PointCloud<T> &mask, 
    const std::string &layer_id, double opacity)
{
  // We assume that the data passed into image is organized, otherwise this doesn't make sense
  if (!image->isOrganized ())
    return (false);

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addMask] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->SetDrawColor (255.0, 0.0, 0.0, opacity * 255.0);

  // Construct a search object to get the camera parameters
  pcl::search::OrganizedNeighbor<T> search;
  search.setInputCloud (image);
  for (size_t i = 0; i < mask.points.size (); ++i)
  {
    pcl::PointXY p_projected;
    search.projectPoint (mask.points[i], p_projected);

    am_it->canvas->DrawPoint (int (p_projected.x), 
                              int (float (image->height) - p_projected.y));
  }

  return (true);
}

#endif      // PCL_VISUALIZATION_IMAGE_VISUALIZER_HPP_
