/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CONE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CONE_H_

#include <unsupported/Eigen/NonLinearOptimization> // for LevenbergMarquardt
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/common/common.h> // for getAngle3D
#include <pcl/common/concatenate.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelCone<PointT, PointNT>::isSampleGood(const Indices &samples) const
{
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCone::isSampleGood] Wrong number of samples (is %lu, should be %lu)!\n", samples.size (), sample_size_);
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelCone<PointT, PointNT>::computeModelCoefficients (
    const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
  // Make sure that the samples are valid
  if (!isSampleGood (samples))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCone::computeModelCoefficients] Invalid set of samples given\n");
    return (false);
  }

  if (!normals_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCone::computeModelCoefficients] No input dataset containing normals was given!\n");
    return (false);
  }

  Eigen::Vector4f p1 ((*input_)[samples[0]].x, (*input_)[samples[0]].y, (*input_)[samples[0]].z, 0.0f);
  Eigen::Vector4f p2 ((*input_)[samples[1]].x, (*input_)[samples[1]].y, (*input_)[samples[1]].z, 0.0f);
  Eigen::Vector4f p3 ((*input_)[samples[2]].x, (*input_)[samples[2]].y, (*input_)[samples[2]].z, 0.0f);

  Eigen::Vector4f n1 ((*normals_)[samples[0]].normal[0], (*normals_)[samples[0]].normal[1], (*normals_)[samples[0]].normal[2], 0.0f);
  Eigen::Vector4f n2 ((*normals_)[samples[1]].normal[0], (*normals_)[samples[1]].normal[1], (*normals_)[samples[1]].normal[2], 0.0f);
  Eigen::Vector4f n3 ((*normals_)[samples[2]].normal[0], (*normals_)[samples[2]].normal[1], (*normals_)[samples[2]].normal[2], 0.0f);

  //calculate apex (intersection of the three planes defined by points and belonging normals
  Eigen::Vector4f ortho12 = n1.cross3(n2);
  Eigen::Vector4f ortho23 = n2.cross3(n3);
  Eigen::Vector4f ortho31 = n3.cross3(n1);

  float denominator = n1.dot(ortho23);

  float d1 = p1.dot (n1);
  float d2 = p2.dot (n2);
  float d3 = p3.dot (n3);

  Eigen::Vector4f apex = (d1 * ortho23 + d2 * ortho31 + d3 * ortho12) / denominator;

  //compute axis (normal of plane defined by: { apex+(p1-apex)/(||p1-apex||), apex+(p2-apex)/(||p2-apex||), apex+(p3-apex)/(||p3-apex||)}
  Eigen::Vector4f ap1 = p1 - apex;
  Eigen::Vector4f ap2 = p2 - apex;
  Eigen::Vector4f ap3 = p3 - apex;

  Eigen::Vector4f np1 = apex + (ap1/ap1.norm ());
  Eigen::Vector4f np2 = apex + (ap2/ap2.norm ());
  Eigen::Vector4f np3 = apex + (ap3/ap3.norm ());

  Eigen::Vector4f np1np2 = np2 - np1;
  Eigen::Vector4f np1np3 = np3 - np1;

  Eigen::Vector4f axis_dir = np1np2.cross3 (np1np3);
  axis_dir.normalize ();

  // normalize the vector (apex->p) for opening angle calculation
  ap1.normalize ();
  ap2.normalize ();
  ap3.normalize ();

  //compute opening angle
  float opening_angle = ( std::acos (ap1.dot (axis_dir)) + std::acos (ap2.dot (axis_dir)) + std::acos (ap3.dot (axis_dir)) ) / 3.0f;

  model_coefficients.resize (model_size_);
  // model_coefficients.template head<3> ()    = line_pt.template head<3> ();
  model_coefficients[0] = apex[0];
  model_coefficients[1] = apex[1];
  model_coefficients[2] = apex[2];
  // model_coefficients.template segment<3> (3) = line_dir.template head<3> ();
  model_coefficients[3] = axis_dir[0];
  model_coefficients[4] = axis_dir[1];
  model_coefficients[5] = axis_dir[2];
  // cone radius
  model_coefficients[6] = opening_angle;

  if (model_coefficients[6] != -std::numeric_limits<double>::max() && model_coefficients[6] < min_angle_)
    return (false);
  if (model_coefficients[6] !=  std::numeric_limits<double>::max() && model_coefficients[6] > max_angle_)
    return (false);

  PCL_DEBUG ("[pcl::SampleConsensusModelCone::computeModelCoefficients] Model is (%g,%g,%g,%g,%g,%g,%g).\n",
             model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3],
             model_coefficients[4], model_coefficients[5], model_coefficients[6]);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCone<PointT, PointNT>::getDistancesToModel (
    const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }

  distances.resize (indices_->size ());

  Eigen::Vector4f apex (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);
  Eigen::Vector4f axis_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0.0f);
  const float sin_opening_angle = std::sin (model_coefficients[6]),
              cos_opening_angle = std::cos (model_coefficients[6]),
              tan_opening_angle = std::tan (model_coefficients[6]);

  float apexdotdir = apex.dot (axis_dir);
  float dirdotdir = 1.0f / axis_dir.dot (axis_dir);
  // Iterate through the 3d points and calculate the distances from them to the cone
  for (std::size_t i = 0; i  < indices_->size (); ++i)
  {
    Eigen::Vector4f pt ((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z, 0.0f);

    // Calculate the point's projection on the cone axis
    float k = (pt.dot (axis_dir) - apexdotdir) * dirdotdir;
    Eigen::Vector4f pt_proj = apex + k * axis_dir;

    // Calculate the actual radius of the cone at the level of the projected point
    Eigen::Vector4f height = apex - pt_proj;
    float actual_cone_radius = tan_opening_angle * height.norm ();

    // Approximate the distance from the point to the cone as the difference between
    // dist(point,cone_axis) and actual cone radius
    const double weighted_euclid_dist = (1.0 - normal_distance_weight_) * std::abs (pointToAxisDistance (pt, model_coefficients) - actual_cone_radius);

    // Calculate the direction of the point from center
    Eigen::Vector4f dir = pt - pt_proj;
    dir.normalize ();

    // Calculate the cones perfect normals
    height.normalize ();
    Eigen::Vector4f cone_normal = sin_opening_angle * height + cos_opening_angle * dir;

    // Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
    Eigen::Vector4f n  ((*normals_)[(*indices_)[i]].normal[0], (*normals_)[(*indices_)[i]].normal[1], (*normals_)[(*indices_)[i]].normal[2], 0.0f);
    double d_normal = std::abs (getAngle3D (n, cone_normal));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    distances[i] = std::abs (normal_distance_weight_ * d_normal + weighted_euclid_dist);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCone<PointT, PointNT>::selectWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold, Indices &inliers)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }

  inliers.clear ();
  error_sqr_dists_.clear ();
  inliers.reserve (indices_->size ());
  error_sqr_dists_.reserve (indices_->size ());

  Eigen::Vector4f apex (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);
  Eigen::Vector4f axis_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0.0f);
  const float sin_opening_angle = std::sin (model_coefficients[6]),
              cos_opening_angle = std::cos (model_coefficients[6]),
              tan_opening_angle = std::tan (model_coefficients[6]);

  float apexdotdir = apex.dot (axis_dir);
  float dirdotdir = 1.0f / axis_dir.dot (axis_dir);
  // Iterate through the 3d points and calculate the distances from them to the cone
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt ((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z, 0.0f);

    // Calculate the point's projection on the cone axis
    float k = (pt.dot (axis_dir) - apexdotdir) * dirdotdir;
    Eigen::Vector4f pt_proj = apex + k * axis_dir;

    // Calculate the actual radius of the cone at the level of the projected point
    Eigen::Vector4f height = apex - pt_proj;
    double actual_cone_radius = tan_opening_angle * height.norm ();

    // Approximate the distance from the point to the cone as the difference between
    // dist(point,cone_axis) and actual cone radius
    const double weighted_euclid_dist = (1.0 - normal_distance_weight_) * std::abs (pointToAxisDistance (pt, model_coefficients) - actual_cone_radius);
    if (weighted_euclid_dist > threshold) // Early termination: cannot be an inlier
      continue;

    // Calculate the direction of the point from center
    Eigen::Vector4f pp_pt_dir = pt - pt_proj;
    pp_pt_dir.normalize ();

    // Calculate the cones perfect normals
    height.normalize ();
    Eigen::Vector4f cone_normal = sin_opening_angle * height + cos_opening_angle * pp_pt_dir;

    // Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
    Eigen::Vector4f n  ((*normals_)[(*indices_)[i]].normal[0], (*normals_)[(*indices_)[i]].normal[1], (*normals_)[(*indices_)[i]].normal[2], 0.0f);
    double d_normal = std::abs (getAngle3D (n, cone_normal));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    double distance = std::abs (normal_distance_weight_ * d_normal + weighted_euclid_dist);
    
    if (distance < threshold)
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      inliers.push_back ((*indices_)[i]);
      error_sqr_dists_.push_back (distance);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> std::size_t
pcl::SampleConsensusModelCone<PointT, PointNT>::countWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold) const
{

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return (0);

  std::size_t nr_p = 0;

  Eigen::Vector4f apex (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);
  Eigen::Vector4f axis_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0.0f);
  const float sin_opening_angle = std::sin (model_coefficients[6]),
              cos_opening_angle = std::cos (model_coefficients[6]),
              tan_opening_angle = std::tan (model_coefficients[6]);

  float apexdotdir = apex.dot (axis_dir);
  float dirdotdir = 1.0f / axis_dir.dot (axis_dir);
  // Iterate through the 3d points and calculate the distances from them to the cone
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt ((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y, (*input_)[(*indices_)[i]].z, 0.0f);

    // Calculate the point's projection on the cone axis
    float k = (pt.dot (axis_dir) - apexdotdir) * dirdotdir;
    Eigen::Vector4f pt_proj = apex + k * axis_dir;

    // Calculate the actual radius of the cone at the level of the projected point
    Eigen::Vector4f height = apex - pt_proj;
    double actual_cone_radius = tan_opening_angle * height.norm ();

    // Approximate the distance from the point to the cone as the difference between
    // dist(point,cone_axis) and actual cone radius
    const double weighted_euclid_dist = (1.0 - normal_distance_weight_) * std::abs (pointToAxisDistance (pt, model_coefficients) - actual_cone_radius);
    if (weighted_euclid_dist > threshold) // Early termination: cannot be an inlier
      continue;

    // Calculate the direction of the point from center
    Eigen::Vector4f pp_pt_dir = pt - pt_proj;
    pp_pt_dir.normalize ();

    // Calculate the cones perfect normals
    height.normalize ();
    Eigen::Vector4f cone_normal = sin_opening_angle * height + cos_opening_angle * pp_pt_dir;

    // Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
    Eigen::Vector4f n  ((*normals_)[(*indices_)[i]].normal[0], (*normals_)[(*indices_)[i]].normal[1], (*normals_)[(*indices_)[i]].normal[2], 0.0f);
    double d_normal = std::abs (getAngle3D (n, cone_normal));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    if (std::abs (normal_distance_weight_ * d_normal + weighted_euclid_dist) < threshold)
      nr_p++;
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCone<PointT, PointNT>::optimizeModelCoefficients (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
{
  optimized_coefficients = model_coefficients;

  // Needs a set of valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCone::optimizeModelCoefficients] Given model is invalid!\n");
    return;
  }

  // Need more than the minimum sample size to make a difference
  if (inliers.size () <= sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCone:optimizeModelCoefficients] Not enough inliers found to optimize model coefficients (%lu)! Returning the same coefficients.\n", inliers.size ());
    return;
  }

  OptimizationFunctor functor (this, inliers);
  Eigen::NumericalDiff<OptimizationFunctor > num_diff (functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, float> lm (num_diff);
  int info = lm.minimize (optimized_coefficients);

  // Compute the L2 norm of the residuals
  PCL_DEBUG ("[pcl::SampleConsensusModelCone::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g %g\n",
             info, lm.fvec.norm (), model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3],
             model_coefficients[4], model_coefficients[5], model_coefficients[6], optimized_coefficients[0], optimized_coefficients[1], optimized_coefficients[2], optimized_coefficients[3], optimized_coefficients[4], optimized_coefficients[5], optimized_coefficients[6]);

  Eigen::Vector3f line_dir (optimized_coefficients[3], optimized_coefficients[4], optimized_coefficients[5]);
  line_dir.normalize ();
  optimized_coefficients[3] = line_dir[0];
  optimized_coefficients[4] = line_dir[1];
  optimized_coefficients[5] = line_dir[2];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCone<PointT, PointNT>::projectPoints (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCone::projectPoints] Given model is invalid!\n");
    return;
  }

  projected_points.header = input_->header;
  projected_points.is_dense = input_->is_dense;

  Eigen::Vector4f apex  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);
  Eigen::Vector4f axis_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0.0f);
  const float tan_opening_angle = std::tan (model_coefficients[6]);

  float apexdotdir = apex.dot (axis_dir);
  float dirdotdir = 1.0f / axis_dir.dot (axis_dir);

  // Copy all the data fields from the input cloud to the projected one?
  if (copy_data_fields)
  {
    // Allocate enough space and copy the basics
    projected_points.resize (input_->size ());
    projected_points.width    = input_->width;
    projected_points.height   = input_->height;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < projected_points.size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> ((*input_)[i], projected_points[i]));

    // Iterate through the 3d points and calculate the distances from them to the cone
    for (const auto &inlier : inliers)
    {
      Eigen::Vector4f pt ((*input_)[inlier].x, 
                          (*input_)[inlier].y, 
                          (*input_)[inlier].z, 
                          1);

      float k = (pt.dot (axis_dir) - apexdotdir) * dirdotdir;

      pcl::Vector4fMap pp = projected_points[inlier].getVector4fMap ();
      pp.matrix () = apex + k * axis_dir;

      Eigen::Vector4f dir = pt - pp;
      dir.normalize ();

      // Calculate the actual radius of the cone at the level of the projected point
      Eigen::Vector4f height = apex - pp;
      float actual_cone_radius = tan_opening_angle * height.norm ();

      // Calculate the projection of the point onto the cone
      pp += dir * actual_cone_radius;
    }
  }
  else
  {
    // Allocate enough space and copy the basics
    projected_points.resize (inliers.size ());
    projected_points.width    = inliers.size ();
    projected_points.height   = 1;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < inliers.size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> ((*input_)[inliers[i]], projected_points[i]));

    // Iterate through the 3d points and calculate the distances from them to the cone
    for (std::size_t i = 0; i < inliers.size (); ++i)
    {
      pcl::Vector4fMap pp = projected_points[i].getVector4fMap ();
      pcl::Vector4fMapConst pt = (*input_)[inliers[i]].getVector4fMap ();

      float k = (pt.dot (axis_dir) - apexdotdir) * dirdotdir;
      // Calculate the projection of the point on the line
      pp.matrix () = apex + k * axis_dir;

      Eigen::Vector4f dir = pt - pp;
      dir.normalize ();

      // Calculate the actual radius of the cone at the level of the projected point
      Eigen::Vector4f height = apex - pp;
      float actual_cone_radius = tan_opening_angle * height.norm ();

      // Calculate the projection of the point onto the cone
      pp += dir * actual_cone_radius;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelCone<PointT, PointNT>::doSamplesVerifyModel (
      const std::set<index_t> &indices, const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Needs a valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCone::doSamplesVerifyModel] Given model is invalid!\n");
    return (false);
  }

  Eigen::Vector4f apex (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);
  Eigen::Vector4f axis_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0.0f);
  const float tan_opening_angle = std::tan (model_coefficients[6]);

  float apexdotdir = apex.dot (axis_dir);
  float dirdotdir = 1.0f / axis_dir.dot (axis_dir);

  // Iterate through the 3d points and calculate the distances from them to the cone
  for (const auto &index : indices)
  {
    Eigen::Vector4f pt ((*input_)[index].x, (*input_)[index].y, (*input_)[index].z, 0.0f);

    // Calculate the point's projection on the cone axis
    float k = (pt.dot (axis_dir) - apexdotdir) * dirdotdir;
    Eigen::Vector4f pt_proj = apex + k * axis_dir;
    Eigen::Vector4f dir = pt - pt_proj;
    dir.normalize ();

    // Calculate the actual radius of the cone at the level of the projected point
    Eigen::Vector4f height = apex - pt_proj;
    double actual_cone_radius = tan_opening_angle * height.norm ();

    // Approximate the distance from the point to the cone as the difference between
    // dist(point,cone_axis) and actual cone radius
    if (std::abs (static_cast<double>(pointToAxisDistance (pt, model_coefficients) - actual_cone_radius)) > threshold)
      return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> double
pcl::SampleConsensusModelCone<PointT, PointNT>::pointToAxisDistance (
      const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients) const
{
  Eigen::Vector4f apex  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);
  Eigen::Vector4f axis_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0.0f);
  return sqrt(pcl::sqrPointToLineDistance (pt, apex, axis_dir));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool 
pcl::SampleConsensusModelCone<PointT, PointNT>::isModelValid (const Eigen::VectorXf &model_coefficients) const
{
  if (!SampleConsensusModel<PointT>::isModelValid (model_coefficients))
    return (false);

  // Check against template, if given
  if (eps_angle_ > 0.0)
  {
    // Obtain the cone direction
    const Eigen::Vector3f coeff(model_coefficients[3], model_coefficients[4], model_coefficients[5]);

    double angle_diff = std::abs (getAngle3D (axis_, coeff));
    angle_diff = (std::min) (angle_diff, M_PI - angle_diff);
    // Check whether the current cone model satisfies our angle threshold criterion with respect to the given axis
    if (angle_diff > eps_angle_)
    {
      PCL_DEBUG ("[pcl::SampleConsensusModelCone::isModelValid] Angle between cone direction and given axis is too large.\n");
      return (false);
    }
  }

  if (model_coefficients[6] != -std::numeric_limits<double>::max() && model_coefficients[6] < min_angle_)
  {
    PCL_DEBUG ("[pcl::SampleConsensusModelCone::isModelValid] The opening angle is too small: should be larger than %g, but is %g.\n",
               min_angle_, model_coefficients[6]);
    return (false);
  }
  if (model_coefficients[6] !=  std::numeric_limits<double>::max() && model_coefficients[6] > max_angle_)
  {
    PCL_DEBUG ("[pcl::SampleConsensusModelCone::isModelValid] The opening angle is too big: should be smaller than %g, but is %g.\n",
               max_angle_, model_coefficients[6]);
    return (false);
  }

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelCone(PointT, PointNT)	template class PCL_EXPORTS pcl::SampleConsensusModelCone<PointT, PointNT>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CONE_H_

