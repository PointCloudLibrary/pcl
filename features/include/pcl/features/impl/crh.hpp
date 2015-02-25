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

#ifndef PCL_FEATURES_IMPL_CRH_H_
#define PCL_FEATURES_IMPL_CRH_H_

#include <pcl/features/crh.h>
#include <pcl/common/fft/kiss_fftr.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT>
void
pcl::CRHEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  // Check if input was set
  if (!normals_)
  {
    PCL_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  if (normals_->points.size () != surface_->points.size ())
  {
    PCL_ERROR ("[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  Eigen::Vector3f plane_normal;
  plane_normal[0] = -centroid_[0];
  plane_normal[1] = -centroid_[1];
  plane_normal[2] = -centroid_[2];
  Eigen::Vector3f z_vector = Eigen::Vector3f::UnitZ ();
  plane_normal.normalize ();
  Eigen::Vector3f axis = plane_normal.cross (z_vector);
  double rotation = -asin (axis.norm ());
  axis.normalize ();

  int nbins = nbins_;
  int bin_angle = 360 / nbins;

  Eigen::Affine3f transformPC (Eigen::AngleAxisf (static_cast<float> (rotation), axis));

  pcl::PointCloud<pcl::PointNormal> grid;
  grid.points.resize (indices_->size ());

  for (size_t i = 0; i < indices_->size (); i++)
  {
    grid.points[i].getVector4fMap () = surface_->points[(*indices_)[i]].getVector4fMap ();
    grid.points[i].getNormalVector4fMap () = normals_->points[(*indices_)[i]].getNormalVector4fMap ();
  }

  pcl::transformPointCloudWithNormals (grid, grid, transformPC);

  //fill spatial data vector and the zero-initialize or "value-initialize" an array on c++, 
  // the initialization is made with () after the [nbins]
  kiss_fft_scalar * spatial_data = new kiss_fft_scalar[nbins]();
  

  float sum_w = 0, w = 0;
  int bin = 0;
  for (size_t i = 0; i < grid.points.size (); ++i)
  {
    bin = static_cast<int> ((((atan2 (grid.points[i].normal_y, grid.points[i].normal_x) + M_PI) * 180 / M_PI) / bin_angle)) % nbins;
    w = sqrtf (grid.points[i].normal_y * grid.points[i].normal_y + grid.points[i].normal_x * grid.points[i].normal_x);
    sum_w += w;
    spatial_data[bin] += w;
  }

  for (int i = 0; i < nbins; ++i)
    spatial_data[i] /= sum_w;

  kiss_fft_cpx * freq_data = new kiss_fft_cpx[nbins / 2 + 1];
  kiss_fftr_cfg mycfg = kiss_fftr_alloc (nbins, 0, NULL, NULL);
  kiss_fftr (mycfg, spatial_data, freq_data);

  output.points.resize (1);
  output.width = output.height = 1;

  output.points[0].histogram[0] = freq_data[0].r / freq_data[0].r; //dc
  int k = 1;
  for (int i = 1; i < (nbins / 2); i++, k += 2)
  {
    output.points[0].histogram[k] = freq_data[i].r / freq_data[0].r;
    output.points[0].histogram[k + 1] = freq_data[i].i / freq_data[0].r;
  }

  output.points[0].histogram[nbins - 1] = freq_data[nbins / 2].r / freq_data[0].r; //nyquist

  delete[] spatial_data;
  delete[] freq_data;

}

#define PCL_INSTANTIATE_CRHEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::CRHEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_CRH_H_
