/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_FEATURES_IMPL_SPIN_IMAGE_H_
#define PCL_FEATURES_IMPL_SPIN_IMAGE_H_

#include <limits>
#include <pcl/point_types.h>
#include <pcl/exceptions.h>
#include <pcl/features/spin_image.h>
#include <cmath>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT>
pcl::SpinImageEstimation<PointInT, PointNT, PointOutT>::SpinImageEstimation (
  unsigned int image_width, double support_angle_cos, unsigned int min_pts_neighb) :
  input_normals_ (), rotation_axes_cloud_ (), 
  is_angular_ (false), rotation_axis_ (), use_custom_axis_(false), use_custom_axes_cloud_ (false), 
  is_radial_ (false), image_width_ (image_width), support_angle_cos_ (support_angle_cos), 
  min_pts_neighb_ (min_pts_neighb)
{
  assert (support_angle_cos_ <= 1.0 && support_angle_cos_ >= 0.0); // may be permit negative cosine?

  feature_name_ = "SpinImageEstimation";
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> Eigen::ArrayXXd 
pcl::SpinImageEstimation<PointInT, PointNT, PointOutT>::computeSiForPoint (int index) const
{
  assert (image_width_ > 0);
  assert (support_angle_cos_ <= 1.0 && support_angle_cos_ >= 0.0); // may be permit negative cosine?

  const Eigen::Vector3f origin_point ((*input_)[index].getVector3fMap ());

  Eigen::Vector3f origin_normal;
  origin_normal = 
    input_normals_ ? 
      (*input_normals_)[index].getNormalVector3fMap () :
      Eigen::Vector3f (); // just a placeholder; should never be used!

  const Eigen::Vector3f rotation_axis = use_custom_axis_ ? 
    rotation_axis_.getNormalVector3fMap () : 
    use_custom_axes_cloud_ ?
      (*rotation_axes_cloud_)[index].getNormalVector3fMap () :
      origin_normal;  

  Eigen::ArrayXXd m_matrix (Eigen::ArrayXXd::Zero (image_width_+1, 2*image_width_+1));
  Eigen::ArrayXXd m_averAngles (Eigen::ArrayXXd::Zero (image_width_+1, 2*image_width_+1));

  // OK, we are interested in the points of the cylinder of height 2*r and
  // base radius r, where r = m_dBinSize * in_iImageWidth
  // it can be embedded to the sphere of radius sqrt(2) * m_dBinSize * in_iImageWidth
  // suppose that points are uniformly distributed, so we lose ~40%
  // according to the volumes ratio
  double bin_size = 0.0;
  if (is_radial_)
    bin_size = search_radius_ / image_width_;  
  else
    bin_size = search_radius_ / image_width_ / sqrt(2.0);

  pcl::Indices nn_indices;
  std::vector<float> nn_sqr_dists;
  const int neighb_cnt = this->searchForNeighbors (index, search_radius_, nn_indices, nn_sqr_dists);
  if (neighb_cnt < static_cast<int> (min_pts_neighb_))
  {
    throw PCLException (
      "Too few points for spin image, use setMinPointCountInNeighbourhood() to decrease the threshold or use larger feature radius",
      "spin_image.hpp", "computeSiForPoint");
  }

  // for all neighbor points
  for (int i_neigh = 0; i_neigh < neighb_cnt ; i_neigh++)
  {
    // first, skip the points with distant normals
    double cos_between_normals = -2.0; // should be initialized if used
    if (support_angle_cos_ > 0.0 || is_angular_) // not bogus
    {
      cos_between_normals = origin_normal.dot ((*input_normals_)[nn_indices[i_neigh]].getNormalVector3fMap ());
      if (std::abs (cos_between_normals) > (1.0 + 10*std::numeric_limits<float>::epsilon ())) // should be okay for numeric stability
      {      
        PCL_ERROR ("[pcl::%s::computeSiForPoint] Normal for the point %d and/or the point %d are not normalized, dot ptoduct is %f.\n", 
          getClassName ().c_str (), nn_indices[i_neigh], index, cos_between_normals);
        throw PCLException ("Some normals are not normalized",
          "spin_image.hpp", "computeSiForPoint");
      }
      cos_between_normals = std::max (-1.0, std::min (1.0, cos_between_normals));

      if (std::abs (cos_between_normals) < support_angle_cos_ )    // allow counter-directed normals
      {
        continue;
      }

      if (cos_between_normals < 0.0)
      {
        cos_between_normals = -cos_between_normals; // the normal is not used explicitly from now
      }
    }
    
    // now compute the coordinate in cylindric coordinate system associated with the origin point
    const Eigen::Vector3f direction (
      (*surface_)[nn_indices[i_neigh]].getVector3fMap () - origin_point);
    const double direction_norm = direction.norm ();
    if (std::abs(direction_norm) < 10*std::numeric_limits<double>::epsilon ())  
      continue;  // ignore the point itself; it does not contribute really
    assert (direction_norm > 0.0);

    // the angle between the normal vector and the direction to the point
    double cos_dir_axis = direction.dot(rotation_axis) / direction_norm;
    if (std::abs(cos_dir_axis) > (1.0 + 10*std::numeric_limits<float>::epsilon())) // should be okay for numeric stability
    {      
      PCL_ERROR ("[pcl::%s::computeSiForPoint] Rotation axis for the point %d are not normalized, dot ptoduct is %f.\n", 
        getClassName ().c_str (), index, cos_dir_axis);
      throw PCLException ("Some rotation axis is not normalized",
        "spin_image.hpp", "computeSiForPoint");
    }
    cos_dir_axis = std::max (-1.0, std::min (1.0, cos_dir_axis));

    // compute coordinates w.r.t. the reference frame
    double beta = std::numeric_limits<double>::signaling_NaN ();
    double alpha = std::numeric_limits<double>::signaling_NaN ();
    if (is_radial_) // radial spin image structure
    {
	    beta = asin (cos_dir_axis);  // yes, arc sine! to get the angle against tangent, not normal!
	    alpha = direction_norm;
    }
    else // rectangular spin-image structure
    {
      beta = direction_norm * cos_dir_axis;
      alpha = direction_norm * sqrt (1.0 - cos_dir_axis*cos_dir_axis);

      if (std::abs (beta) >= bin_size * image_width_ || alpha >= bin_size * image_width_)
      {
        continue;  // outside the cylinder
      }
    }

    assert (alpha >= 0.0);
    assert (alpha <= bin_size * image_width_ + 20 * std::numeric_limits<float>::epsilon () );


    // bilinear interpolation
    double beta_bin_size = is_radial_ ? (M_PI / 2 / image_width_) : bin_size;
    int beta_bin = int(std::floor (beta / beta_bin_size)) + int(image_width_);
    assert (0 <= beta_bin && beta_bin < m_matrix.cols ());
    int alpha_bin = int(std::floor (alpha / bin_size));
    assert (0 <= alpha_bin && alpha_bin < m_matrix.rows ());

    if (alpha_bin == static_cast<int> (image_width_))  // border points
    {
      alpha_bin--;
      // HACK: to prevent a > 1
      alpha = bin_size * (alpha_bin + 1) - std::numeric_limits<double>::epsilon ();
    }
    if (beta_bin == int(2*image_width_) )  // border points
    {
      beta_bin--;
      // HACK: to prevent b > 1
      beta = beta_bin_size * (beta_bin - int(image_width_) + 1) - std::numeric_limits<double>::epsilon ();
    }

    double a = alpha/bin_size - double(alpha_bin);
    double b = beta/beta_bin_size - double(beta_bin-int(image_width_)); 

    assert (0 <= a && a <= 1);
    assert (0 <= b && b <= 1);

    m_matrix (alpha_bin, beta_bin) += (1-a) * (1-b);
    m_matrix (alpha_bin+1, beta_bin) += a * (1-b);
    m_matrix (alpha_bin, beta_bin+1) += (1-a) * b;
    m_matrix (alpha_bin+1, beta_bin+1) += a * b;

    if (is_angular_)
    {
      m_averAngles (alpha_bin, beta_bin) += (1-a) * (1-b) * std::acos (cos_between_normals); 
      m_averAngles (alpha_bin+1, beta_bin) += a * (1-b) * std::acos (cos_between_normals);
      m_averAngles (alpha_bin, beta_bin+1) += (1-a) * b * std::acos (cos_between_normals);
      m_averAngles (alpha_bin+1, beta_bin+1) += a * b * std::acos (cos_between_normals);
    }
  }

  if (is_angular_)
  {
    // transform sum to average
    m_matrix = m_averAngles / (m_matrix + std::numeric_limits<double>::epsilon ()); // +eps to avoid division by zero
  }
  else if (neighb_cnt > 1) // to avoid division by zero, also no need to divide by 1
  {
    // normalization
    m_matrix /= m_matrix.sum();
  }

  return m_matrix;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool 
pcl::SpinImageEstimation<PointInT, PointNT, PointOutT>::initCompute ()
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // Check if input normals are set
  if (!input_normals_)
  {
    PCL_ERROR ("[pcl::%s::initCompute] No input dataset containing normals was given!\n", getClassName ().c_str ());
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  // Check if the size of normals is the same as the size of the surface
  if (input_normals_->size () != input_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] ", getClassName ().c_str ());
    PCL_ERROR ("The number of points in the input dataset differs from ");
    PCL_ERROR ("the number of points in the dataset containing the normals!\n");
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

   // We need a positive definite search radius to continue
  if (search_radius_ == 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] Need a search radius different than 0!\n", getClassName ().c_str ());
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }
  if (k_ != 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] K-nearest neighbor search for spin images not implemented. Used a search radius instead!\n", getClassName ().c_str ());
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }
  // If the surface won't be set, make fake surface and fake surface normals
  // if we wouldn't do it here, the following method would alarm that no surface normals is given
  if (!surface_)
  {
    surface_ = input_;
    fake_surface_ = true;
  }

  //if (fake_surface_ && !input_normals_)
  //  input_normals_ = normals_; // normals_ is set, as checked earlier
  
  assert(!(use_custom_axis_ && use_custom_axes_cloud_));

  if (!use_custom_axis_ && !use_custom_axes_cloud_ // use input normals as rotation axes
    && !input_normals_)
  {
    PCL_ERROR ("[pcl::%s::initCompute] No normals for input cloud were given!\n", getClassName ().c_str ());
    // Cleanup
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  if ((is_angular_ || support_angle_cos_ > 0.0) // support angle is not bogus NOTE this is for randomly-flipped normals
    && !input_normals_)
  {
    PCL_ERROR ("[pcl::%s::initCompute] No normals for input cloud were given!\n", getClassName ().c_str ());
    // Cleanup
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  if (use_custom_axes_cloud_ 
    && rotation_axes_cloud_->size () == input_->size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Rotation axis cloud have different size from input!\n", getClassName ().c_str ());
    // Cleanup
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  return (true);
}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> void 
pcl::SpinImageEstimation<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{ 
  for (std::size_t i_input = 0; i_input < indices_->size (); ++i_input)
  {
    Eigen::ArrayXXd res = computeSiForPoint (indices_->at (i_input));

    // Copy into the resultant cloud
    for (Eigen::Index iRow = 0; iRow < res.rows () ; iRow++)
    {
      for (Eigen::Index iCol = 0; iCol < res.cols () ; iCol++)
      {
        output[i_input].histogram[ iRow*res.cols () + iCol ] = static_cast<float> (res (iRow, iCol));
      }
    }   
  } 
}

#define PCL_INSTANTIATE_SpinImageEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::SpinImageEstimation<T,NT,OutT>;

#endif    // PCL_FEATURES_IMPL_SPIN_IMAGE_H_

