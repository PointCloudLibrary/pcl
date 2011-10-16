/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef PCL_FEATURES_IMPL_GSS3D_H_
#define PCL_FEATURES_IMPL_GSS3D_H_

#include "pcl/features/gss3d.h"


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  calculateGeometricScaleSpace ();
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::calculateGeometricScaleSpace ()
{
  normal_maps_.resize (scales_.size ());
  // The normal map at the largest scale is the given one
//  normal_maps_[0] = normals_->makeShared ();

  // Compute the following scale spaces by convolving with a geodesic Gaussian kernel

  float sigma, sigma_squared, dist, gauss_coeff, normalization_factor;
  PointNT result, aux;
  for (size_t scale_i = 0; scale_i < scales_.size (); ++scale_i)
  {
    printf ("Computing for scale %d\n", scales_[scale_i]);
    PointCloudNPtr normal_map (new PointCloudN ());
    normal_map->header = normals_->header;
    normal_map->width = normals_->width;
    normal_map->height = normals_->height;
    normal_map->resize (normals_->size ());

    sigma = scales_[scale_i];
    sigma_squared = sigma * sigma;

    // For all points on the depth image
    for (int x = 0; x < int (normal_map->width); ++x)
      for (int y = 0; y < int (normal_map->height); ++y)
      {
        result.normal_x = result.normal_y = result.normal_z = 0.0f;

        // For all points in the Gaussian window
        for (int win_x = -window_size_/2 * sigma; win_x <= window_size_/2 * sigma; ++win_x)
          for (int win_y = -window_size_/2 * sigma; win_y <= window_size_/2 * sigma; ++win_y)
            // This virtually borders the image with 0-normals
            if ( (x + win_x >= 0) && (x + win_x < int (normal_map->width)) && (y + win_y >= 0) && (y + win_y < int (normal_map->height)))
            {
//              printf ("%d %d    %d %d\n", x+win_x, y+win_y, int (normal_map->width), int (normal_map->height));
              dist = computeGeodesicDistance (x, y, x+win_x, y+win_y);
//              printf ("dist = %f\n", dist);
              gauss_coeff = 1 / (2 * M_PI * sigma_squared) * exp ((-1) * dist*dist / (2 * sigma_squared));
              aux = normals_->at (x + win_x, y + win_y);
              result.normal_x += aux.normal_x * gauss_coeff; result.normal_y += aux.normal_y * gauss_coeff; result.normal_z += aux.normal_z * gauss_coeff;
            }
        // Normalize the resulting normal
        normalization_factor = sqrt (result.normal_x*result.normal_x + result.normal_y*result.normal_y + result.normal_z*result.normal_z);
        result.normal_x /= normalization_factor; result.normal_y /= normalization_factor; result.normal_z /= normalization_factor;
        (*normal_map) (x, y) = result;
//        printf ("resulting normal: (%f, %f, %f)\n", result.normal_x, result.normal_y, result.normal_z);
      }

    normal_maps_[scale_i] = normal_map;
  }

//  for (int i = 0; i < normal_maps_.size (); ++ i)
//    for (int j = 0; j < normal_maps_[i]->size (); ++j) printf ("(%f, %f, %f)\n", normal_maps_[i]->points[j].normal_x, normal_maps_[i]->points[j].normal_y, normal_maps_[i]->points[j].normal_z);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> float
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::computeGeodesicDistance (size_t x0, size_t y0,
                                                                             size_t x1, size_t y1)
{
  // Based on the Bresenham's algorithm: http://en.wikipedia.org/wiki/Bresenham's_line_algorithm
  bool steep = abs ((long) (y1 - y0)) > abs ((long) (x1 - x0));
  size_t aux;
  if (steep)
  {
    aux = x0;
    x0 = y0;
    y0 = aux;

    aux = x1;
    x1 = y1;
    y1 = aux;
  }

  if (x0 > x1)
  {
    aux = x0;
    x0 = x1;
    x1 = aux;

    aux = y0;
    y0 = y1;
    y1 = aux;
  }

  size_t delta_x = x1 - x0,
      delta_y = abs ((long) (y1 - y0));
  float error = 0.0f;
  float delta_error = float (delta_y) / delta_x;
  int ystep;
  size_t y = y0;
  if (y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  PointInT previous_point, current_point;
  if (steep)
    current_point = surface_->at (y0, x0);
  else
    current_point = surface_->at (x0, y0);
  float distance,
  total_distance = 0.0f;

  for (size_t x = x0; x <= x1; ++x)
  {
    if (steep)
    {
      previous_point = current_point;
      current_point = surface_->at (y, x);
    }
    else
    {
      previous_point = current_point;
      current_point = surface_->at (x, y);
    }

    distance = euclideanDistance (previous_point, current_point);
    if (pcl_isnan (distance)) return (-1.0f);
    total_distance += distance;

    error += delta_error;
    if (error >= 0.5f)
    {
      y += ystep;
      error -= 1.0f;
    }
  }

  return total_distance;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::extractCorners ()
{
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::extractEdges ()
{
}

#define PCL_INSTANTIATE_GSS3DEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::GSS3DEstimation<T,NT,OutT>;


#endif /* PCL_FEATURES_IMPL_GSS3D_H_ */
