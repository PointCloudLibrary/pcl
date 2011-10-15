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

}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::calculateGeometricScaleSpace ()
{
  // The normal map at the largest scale is the given one
//  normal_maps[0] = normals_;

  // Compute the following scale spaces by convolving with a geodesic Gaussian kernel
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> float
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::computeGeodesicDistance (size_t x0, size_t y0,
                                                                             size_t x1, size_t y1)
{
  // based on the Bresenham's algorithm: http://en.wikipedia.org/wiki/Bresenham's_line_algorithm
  bool steep = abs (y1 - y0) > abs (x1 - x0);
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
         delta_y = abs (y1 - y0);
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
    current_point = surface_->points [y0 * surface_->width + x0];
  else
    current_point = surface_->points [x0 * surface_->width + x0];
  float distance,
        total_distance = 0.0f;

  for (size_t x = x0; x <= x1; ++x)
  {
    if (steep)
    {
      previous_point = current_point;
      current_point = surface_->points [y * surface_->width + x];
    }
    else
    {
      previous_point = current_point;
      current_point = surface_->points [x*surface_->width + y];
    }

    distance = euclideanDistance (previous_point, current_point);
    if (!isnumber (distance)) return (-1.0f);
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
