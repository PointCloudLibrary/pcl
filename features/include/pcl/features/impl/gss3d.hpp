/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef PCL_FEATURES_IMPL_GSS3D_H_
#define PCL_FEATURES_IMPL_GSS3D_H_

#include <pcl/features/gss3d.h>
#include <pcl/common/distances.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
  calculateGeometricScaleSpace ();
  computeDerivatives ();
  extractEdges ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::calculateGeometricScaleSpace ()
{
  normal_maps_.resize (scales_.size ());
  // TODO Do we need the original normal_map? It's noisy anyway ...
  // The normal map at the largest scale is the given one
//  normal_maps_[0] = PointCloudNPtr ( new PointCloudN (*normals_));


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
              dist = computeGeodesicDistance (x, y, x+win_x, y+win_y);
              if (dist == -1.0f) continue;
              gauss_coeff = 1 / (2 * M_PI * sigma_squared) * exp ((-1) * dist*dist / (2 * sigma_squared));
              aux = normals_->at (x + win_x, y + win_y);
              if (pcl_isnan (aux.normal_x)) continue;
              result.normal_x += aux.normal_x * gauss_coeff; result.normal_y += aux.normal_y * gauss_coeff; result.normal_z += aux.normal_z * gauss_coeff;
            }
        // Normalize the resulting normal
        normalization_factor = sqrt (result.normal_x*result.normal_x + result.normal_y*result.normal_y + result.normal_z*result.normal_z);
        result.normal_x /= normalization_factor; result.normal_y /= normalization_factor; result.normal_z /= normalization_factor;
        (*normal_map) (x, y) = result;
      }

    normal_maps_[scale_i] = normal_map;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::computeDerivatives ()
{
  printf ("Computing derivatives ...\n");
  PointNT zero_normal;
  zero_normal.normal_x = zero_normal.normal_y = zero_normal.normal_z = 0.0f;

  d_horiz_normal_maps_ = boost::shared_ptr<Array3D> (new Array3D (boost::extents [scales_.size ()][normals_->width][normals_->height]));
  d_vert_normal_maps_ = boost::shared_ptr<Array3D> (new Array3D (boost::extents [scales_.size ()][normals_->width][normals_->height]));
  dd_horiz_normal_maps_ = boost::shared_ptr<Array3D> (new Array3D (boost::extents [scales_.size ()][normals_->width][normals_->height]));
  dd_vert_normal_maps_ = boost::shared_ptr<Array3D> (new Array3D (boost::extents [scales_.size ()][normals_->width][normals_->height]));

  for (size_t scale_i = 0; scale_i < scales_.size (); ++scale_i)
  {
    /// TODO now have a hack to get rid of the border cases - fix this properly
    // Assign values to the left/right and upper/lower normals in the normal map and take care of bordering with 0-normals
      /*     if (x - 1 < 0) u_s_minus1_t = zero_normal;
           else u_s_minus1_t = normal_maps_[scale_i]->at (x-1, y);
           if (x + 1 > normals_->width) u_s_plus1_t = zero_normal;
           else u_s_plus1_t = normal_maps_[scale_i]->at (x+1, y);
           if (y - 1 < 0) u_s_t_minus1 = zero_normal;
           else u_s_t_minus1 = normal_maps_[scale_i]->at (x, y-1);
           if (y + 1 > normals_->height) u_s_t_plus1 = zero_normal;
           else u_s_t_plus1 = normal_maps_[scale_i]->at (x, y+1);

           if (x - 2 < 0) u_s_minus2_t = zero_normal;
           else u_s_minus2_t = normal_maps_[scale_i]->at (x-2, y);
           if (x + 2 > normals_->width) u_s_plus2_t = zero_normal;
           else u_s_plus2_t = normal_maps_[scale_i]->at (x+2, y);
           if (y - 2 < 0) u_s_t_minus2 = zero_normal;
           else u_s_t_minus2 = normal_maps_[scale_i]->at (x, y-2);
           if (y + 2 > normals_->height) u_s_t_plus2 = zero_normal;
           else u_s_t_plus2 = normal_maps_[scale_i]->at (x, y+2);*/

    for (int x = 2; x < int (normals_->width) - 2; ++x)
      for (int y = 2; y < int (normals_->height) - 2; ++y)
      {
        PointNT u_s_t = normal_maps_[scale_i]->at (x, y);
        PointNT u_s_minus1_t, u_s_plus1_t, u_s_t_minus1, u_s_t_plus1,
                u_s_minus2_t, u_s_plus2_t, u_s_t_minus2, u_s_t_plus2;

        u_s_minus1_t = normal_maps_[scale_i]->at (x-1, y);
        u_s_plus1_t = normal_maps_[scale_i]->at (x+1, y);
        u_s_t_minus1 = normal_maps_[scale_i]->at (x, y-1);
        u_s_t_plus1 = normal_maps_[scale_i]->at (x, y+1);

        u_s_minus2_t = normal_maps_[scale_i]->at (x-2, y);
        u_s_plus2_t = normal_maps_[scale_i]->at (x+2, y);
        u_s_t_minus2 = normal_maps_[scale_i]->at (x, y-2);
        u_s_t_plus2 = normal_maps_[scale_i]->at (x, y+2);

        float theta_horiz1 = acos (u_s_minus1_t.normal_x * u_s_plus1_t.normal_x +
                                   u_s_minus1_t.normal_y * u_s_plus1_t.normal_y +
                                   u_s_minus1_t.normal_z * u_s_plus1_t.normal_z),
              theta_vert1 = acos (u_s_t_minus1.normal_x * u_s_t_plus1.normal_x +
                                  u_s_t_minus1.normal_y * u_s_t_plus1.normal_y +
                                  u_s_t_minus1.normal_z * u_s_t_plus1.normal_z),

              theta_horiz_minus2 = acos (u_s_minus2_t.normal_x * u_s_t.normal_x +
                                         u_s_minus2_t.normal_y * u_s_t.normal_y +
                                         u_s_minus2_t.normal_z * u_s_t.normal_z),
              theta_horiz_plus2 = acos (u_s_plus2_t.normal_x * u_s_t.normal_x +
                                        u_s_plus2_t.normal_y * u_s_t.normal_y +
                                        u_s_plus2_t.normal_z * u_s_t.normal_z),
              theta_vert_minus2 = acos (u_s_t_minus2.normal_x * u_s_t.normal_x +
                                        u_s_t_minus2.normal_y * u_s_t.normal_y +
                                        u_s_t_minus2.normal_z * u_s_t.normal_z),
              theta_vert_plus2 = acos (u_s_t_plus2.normal_x * u_s_t.normal_x +
                                       u_s_t_plus2.normal_y * u_s_t.normal_y +
                                       u_s_t_plus2.normal_z * u_s_t.normal_z);

        float dist_horiz = computeGeodesicDistance (x-1, y, x+1, y),
              dist_vert = computeGeodesicDistance (x, y-1, x, y+1);

        (*d_horiz_normal_maps_)[scale_i][x][y] = sin (theta_horiz1 / 2.0f) / dist_horiz;
        (*d_vert_normal_maps_)[scale_i][x][y] = sin (theta_vert1 / 2.0f) / dist_vert;

        float dot_prod_horiz = u_s_minus1_t.normal_x * u_s_plus1_t.normal_x +
                               u_s_minus1_t.normal_y * u_s_plus1_t.normal_y +
                               u_s_minus1_t.normal_z * u_s_plus1_t.normal_z,
              dot_prod_vert = u_s_t_minus1.normal_x * u_s_t_plus1.normal_x +
                              u_s_t_minus1.normal_y * u_s_t_plus1.normal_y +
                              u_s_t_minus1.normal_z * u_s_t_plus1.normal_z;

//        printf ("%f %f %f %f\n", theta_horiz_minus2, theta_horiz_plus2, theta_vert_minus2, theta_vert_plus2);
        (*dd_horiz_normal_maps_)[scale_i][x][y] = (theta_horiz_minus2 - theta_horiz_plus2) * sqrt ( (1.0f + dot_prod_horiz) / 2.0f) / (dist_horiz * dist_horiz);
        (*dd_vert_normal_maps_)[scale_i][x][y] = (theta_vert_minus2 - theta_vert_plus2) * sqrt ( (1.0f + dot_prod_vert) / 2.0f) / (dist_vert * dist_vert);
      }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::extractEdges ()
{
  printf ("Extracting edges ...\n");
  laplacians_ = boost::shared_ptr<Array3D> (new Array3D (boost::extents [scales_.size ()][normals_->width][normals_->height]));

  for (size_t scale_i = 0; scale_i < scales_.size (); ++scale_i)
  {
    PointCloudInPtr edges (new PointCloudIn ());

    // Border the image
    for (int x = 0; x < int (normals_->width); ++x)
      (*laplacians_)[scale_i][x][0] = (*laplacians_)[scale_i][x][normals_->height - 1] = 0.0f;
    for (int y = 0; y < int (normals_->height); ++y)
      (*laplacians_)[scale_i][0][y] = (*laplacians_)[scale_i][normals_->width - 1][y] = 0.0f;

    // Look for zero crossings in the Laplacian of the normal map at this scale
    for (int x = 1; x < int (normals_->width); ++x)
      for (int y = 1; y < int (normals_->height); ++y)
      {
        (*laplacians_)[scale_i][x][y] = (*dd_horiz_normal_maps_)[scale_i][x][y] + (*dd_vert_normal_maps_)[scale_i][x][y];
        /// TODO also need to threshold on the first derivative magnitude
        if ( pcl_isnan ((*laplacians_)[scale_i][x][y]) || pcl_isnan ((*laplacians_)[scale_i][x-1][y]) || pcl_isnan ((*laplacians_)[scale_i][x][y-1]))
          continue;
        if ( ((*laplacians_)[scale_i][x-1][y] * (*laplacians_)[scale_i][x][y] < 0.0f ||
              (*laplacians_)[scale_i][x][y-1] * (*laplacians_)[scale_i][x][y] < 0.0f) &&
              (*d_horiz_normal_maps_)[scale_i][x][y] > 50 && (*d_vert_normal_maps_)[scale_i][x][y] > 50)
        {
          // This point is on an edge
          printf ("%f %f\n", (*laplacians_)[scale_i][x-1][y], (*d_horiz_normal_maps_)[scale_i][x][y]);
          edges->push_back (surface_->at(x, y));
        }
      }

    edges_.push_back (edges);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::extractCorners ()
{
  printf ("Extracting corners ...\n");
  grams_ = boost::shared_ptr<Array3D> (new Array3D (boost::extents [scales_.size ()][normals_->width][normals_->height]));

  for (size_t scale_i = 0; scale_i < scales_.size (); ++scale_i)
  {
    float sigma = scales_[scale_i],
          tau = sigma / 2.0f,
          tau_squared = tau * tau;
    float dist, gauss_coeff;
    PointCloudInPtr corners (new PointCloudIn ());

    for (int x = 0; x < int (normals_->width); ++x)
      for (int y = 0; y < int (normals_->height); ++y)
      {
        // Define the Gram matrix values
        // | a b |
        // | c d |
        float a = 0.0f, b = 0.0f, c = 0.0f, d = 0.0f;

        // For all points in the Gaussian window
        for (int win_x = -window_size_/2 * sigma; win_x <= window_size_/2 * sigma; ++win_x)
          for (int win_y = -window_size_/2 * sigma; win_y <= window_size_/2 * sigma; ++win_y)
            // This virtually borders the image with 0-normals
            if ( (x + win_x >= 0) && (x + win_x < int (normals_->width)) && (y + win_y >= 0) && (y + win_y < int (normals_->height)))
            {
              dist = computeGeodesicDistance (x, y, x+win_x, y+win_y);
              if (dist == -1.0f) continue;
              gauss_coeff = 1 / (2 * M_PI * tau_squared) * exp ((-1) * dist*dist / (2 * tau_squared));
              a += gauss_coeff * (*d_horiz_normal_maps_)[scale_i][x+win_x][y+win_y] * (*d_horiz_normal_maps_)[scale_i][x+win_x][y+win_y];
              b += gauss_coeff * (*d_horiz_normal_maps_)[scale_i][x+win_x][y+win_y] * (*d_vert_normal_maps_)[scale_i][x+win_x][y+win_y];
              c += gauss_coeff * (*d_horiz_normal_maps_)[scale_i][x+win_x][y+win_y] * (*d_vert_normal_maps_)[scale_i][x+win_x][y+win_y];
              d += gauss_coeff * (*d_vert_normal_maps_)[scale_i][x+win_x][y+win_y] * (*d_vert_normal_maps_)[scale_i][x+win_x][y+win_y];
            }

        // Compute the eigenvalue of the Gram matrix
         if ( (a+d)*(a+d) - 4 * (a*d - b*c) >= 0.0f) (*grams_)[scale_i][x][y] = 0.5f * ( (a+d)*(a+d) + sqrt ( (a+d)*(a+d) - 4 * (a*d - b*c)));
         else (*grams_)[scale_i][x][y] = 0.0f;
      }

   // corners_.push_back (corners);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> float
pcl::GSS3DEstimation<PointInT, PointNT, PointOutT>::computeGeodesicDistance (size_t x0, size_t y0,
                                                                             size_t x1, size_t y1)
{
  // Based on the Bresenham's algorithm: http://en.wikipedia.org/wiki/Bresenham's_line_algorithm
  bool steep = abs (static_cast<long> (y1 - y0)) > abs (static_cast<long> (x1 - x0));
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
      delta_y = abs (static_cast<long> (y1 - y0));
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

  return (total_distance);
}

#define PCL_INSTANTIATE_GSS3DEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::GSS3DEstimation<T,NT,OutT>;

#endif /* PCL_FEATURES_IMPL_GSS3D_H_ */
