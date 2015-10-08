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

#include <pcl/sample_consensus/eigen.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/common/concatenate.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelCone<PointT, PointNT>::isSampleGood(const std::vector<int> &) const
{
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelCone<PointT, PointNT>::computeModelCoefficients (
    const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
  // Need 3 samples
  if (samples.size () != 3)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCone::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
    return (false);
  }

  if (!normals_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCone::computeModelCoefficients] No input dataset containing normals was given!\n");
    return (false);
  }

  Eigen::Vector4f p1 (input_->points[samples[0]].x, input_->points[samples[0]].y, input_->points[samples[0]].z, 0);
  Eigen::Vector4f p2 (input_->points[samples[1]].x, input_->points[samples[1]].y, input_->points[samples[1]].z, 0);
  Eigen::Vector4f p3 (input_->points[samples[2]].x, input_->points[samples[2]].y, input_->points[samples[2]].z, 0);

  Eigen::Vector4f n1 (normals_->points[samples[0]].normal[0], normals_->points[samples[0]].normal[1], normals_->points[samples[0]].normal[2], 0);
  Eigen::Vector4f n2 (normals_->points[samples[1]].normal[0], normals_->points[samples[1]].normal[1], normals_->points[samples[1]].normal[2], 0);
  Eigen::Vector4f n3 (normals_->points[samples[2]].normal[0], normals_->points[samples[2]].normal[1], normals_->points[samples[2]].normal[2], 0);

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
  float opening_angle = ( acosf (ap1.dot (axis_dir)) + acosf (ap2.dot (axis_dir)) + acosf (ap3.dot (axis_dir)) ) / 3.0f;

  model_coefficients.resize (7);
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

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCone<PointT, PointNT>::getDistancesToModel (
    const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }

  distances.resize (indices_->size ());

  Eigen::Vector4f apex (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f axis_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  float opening_angle = model_coefficients[6];

  float apexdotdir = apex.dot (axis_dir);
  float dirdotdir = 1.0f / axis_dir.dot (axis_dir);
  // Iterate through the 3d points and calculate the distances from them to the cone
  for (size_t i = 0; i  < indices_->size (); ++i)
  {
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 0);
    Eigen::Vector4f n  (normals_->points[(*indices_)[i]].normal[0], normals_->points[(*indices_)[i]].normal[1], normals_->points[(*indices_)[i]].normal[2], 0);

    // Calculate the point's projection on the cone axis
    float k = (pt.dot (axis_dir) - apexdotdir) * dirdotdir;
    Eigen::Vector4f pt_proj = apex + k * axis_dir;
    Eigen::Vector4f dir = pt - pt_proj;
    dir.normalize ();

    // Calculate the actual radius of the cone at the level of the projected point
    Eigen::Vector4f height = apex - pt_proj;
    float actual_cone_radius = tanf (opening_angle) * height.norm ();
    height.normalize ();

    // Calculate the cones perfect normals
    Eigen::Vector4f cone_normal = sinf (opening_angle) * height + cosf (opening_angle) * dir;

    // Aproximate the distance from the point to the cone as the difference between
    // dist(point,cone_axis) and actual cone radius
    double d_euclid = fabs (pointToAxisDistance (pt, model_coefficients) - actual_cone_radius);

    // Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
    double d_normal = fabs (getAngle3D (n, cone_normal));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    distances[i] = fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCone<PointT, PointNT>::selectWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }

  int nr_p = 0;
  inliers.resize (indices_->size ());
  error_sqr_dists_.resize (indices_->size ());

  Eigen::Vector4f apex (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f axis_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  float opening_angle = model_coefficients[6];

  float apexdotdir = apex.dot (axis_dir);
  float dirdotdir = 1.0f / axis_dir.dot (axis_dir);
  // Iterate through the 3d points and calculate the distances from them to the cone
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 0);
    Eigen::Vector4f n  (normals_->points[(*indices_)[i]].normal[0], normals_->points[(*indices_)[i]].normal[1], normals_->points[(*indices_)[i]].normal[2], 0);

    // Calculate the point's projection on the cone axis
    float k = (pt.dot (axis_dir) - apexdotdir) * dirdotdir;
    Eigen::Vector4f pt_proj = apex + k * axis_dir;

    // Calculate the direction of the point from center
    Eigen::Vector4f pp_pt_dir = pt - pt_proj;
    pp_pt_dir.normalize ();

    // Calculate the actual radius of the cone at the level of the projected point
    Eigen::Vector4f height = apex - pt_proj;
    double actual_cone_radius = tan(opening_angle) * height.norm ();
    height.normalize ();

    // Calculate the cones perfect normals
    Eigen::Vector4f cone_normal = sinf (opening_angle) * height + cosf (opening_angle) * pp_pt_dir;

    // Aproximate the distance from the point to the cone as the difference between
    // dist(point,cone_axis) and actual cone radius
    double d_euclid = fabs (pointToAxisDistance (pt, model_coefficients) - actual_cone_radius);

    // Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
    double d_normal = fabs (getAngle3D (n, cone_normal));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    double distance = fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid);
    
    if (distance < threshold)
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      inliers[nr_p] = (*indices_)[i];
      error_sqr_dists_[nr_p] = distance;
      ++nr_p;
    }
  }
  inliers.resize (nr_p);
  error_sqr_dists_.resize (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> int
pcl::SampleConsensusModelCone<PointT, PointNT>::countWithinDistance (
    const Eigen::VectorXf &model_coefficients, const double threshold)
{

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return (0);

  int nr_p = 0;

  Eigen::Vector4f apex (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f axis_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  float opening_angle = model_coefficients[6];

  float apexdotdir = apex.dot (axis_dir);
  float dirdotdir = 1.0f / axis_dir.dot (axis_dir);
  // Iterate through the 3d points and calculate the distances from them to the cone
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 0);
    Eigen::Vector4f n  (normals_->points[(*indices_)[i]].normal[0], normals_->points[(*indices_)[i]].normal[1], normals_->points[(*indices_)[i]].normal[2], 0);

    // Calculate the point's projection on the cone axis
    float k = (pt.dot (axis_dir) - apexdotdir) * dirdotdir;
    Eigen::Vector4f pt_proj = apex + k * axis_dir;

    // Calculate the direction of the point from center
    Eigen::Vector4f pp_pt_dir = pt - pt_proj;
    pp_pt_dir.normalize ();

    // Calculate the actual radius of the cone at the level of the projected point
    Eigen::Vector4f height = apex - pt_proj;
    double actual_cone_radius = tan(opening_angle) * height.norm ();
    height.normalize ();

    // Calculate the cones perfect normals
    Eigen::Vector4f cone_normal = sinf (opening_angle) * height + cosf (opening_angle) * pp_pt_dir;

    // Aproximate the distance from the point to the cone as the difference between
    // dist(point,cone_axis) and actual cone radius
    double d_euclid = fabs (pointToAxisDistance (pt, model_coefficients) - actual_cone_radius);

    // Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
    double d_normal = fabs (getAngle3D (n, cone_normal));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    if (fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid) < threshold)
      nr_p++;
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCone<PointT, PointNT>::optimizeModelCoefficients (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients)
{
  optimized_coefficients = model_coefficients;

  // Needs a set of valid model coefficients
  if (model_coefficients.size () != 7)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCone::optimizeModelCoefficients] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return;
  }

  if (inliers.empty ())
  {
    PCL_DEBUG ("[pcl::SampleConsensusModelCone:optimizeModelCoefficients] Inliers vector empty! Returning the same coefficients.\n");
    return;
  }

  tmp_inliers_ = &inliers;

  OptimizationFunctor functor (static_cast<int> (inliers.size ()), this);
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
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 7)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCone::projectPoints] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return;
  }

  projected_points.header = input_->header;
  projected_points.is_dense = input_->is_dense;

  Eigen::Vector4f apex  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f axis_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  float opening_angle = model_coefficients[6];

  float apexdotdir = apex.dot (axis_dir);
  float dirdotdir = 1.0f / axis_dir.dot (axis_dir);

  // Copy all the data fields from the input cloud to the projected one?
  if (copy_data_fields)
  {
    // Allocate enough space and copy the basics
    projected_points.points.resize (input_->points.size ());
    projected_points.width    = input_->width;
    projected_points.height   = input_->height;

    typedef typename pcl::traits::fieldList<PointT>::type FieldList;
    // Iterate over each point
    for (size_t i = 0; i < projected_points.points.size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> (input_->points[i], projected_points.points[i]));

    // Iterate through the 3d points and calculate the distances from them to the cone
    for (size_t i = 0; i < inliers.size (); ++i)
    {
      Eigen::Vector4f pt (input_->points[inliers[i]].x, 
                          input_->points[inliers[i]].y, 
                          input_->points[inliers[i]].z, 
                          1);

      float k = (pt.dot (axis_dir) - apexdotdir) * dirdotdir;

      pcl::Vector4fMap pp = projected_points.points[inliers[i]].getVector4fMap ();
      pp.matrix () = apex + k * axis_dir;

      Eigen::Vector4f dir = pt - pp;
      dir.normalize ();

      // Calculate the actual radius of the cone at the level of the projected point
      Eigen::Vector4f height = apex - pp;
      float actual_cone_radius = tanf (opening_angle) * height.norm ();

      // Calculate the projection of the point onto the cone
      pp += dir * actual_cone_radius;
    }
  }
  else
  {
    // Allocate enough space and copy the basics
    projected_points.points.resize (inliers.size ());
    projected_points.width    = static_cast<uint32_t> (inliers.size ());
    projected_points.height   = 1;

    typedef typename pcl::traits::fieldList<PointT>::type FieldList;
    // Iterate over each point
    for (size_t i = 0; i < inliers.size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> (input_->points[inliers[i]], projected_points.points[i]));

    // Iterate through the 3d points and calculate the distances from them to the cone
    for (size_t i = 0; i < inliers.size (); ++i)
    {
      pcl::Vector4fMap pp = projected_points.points[i].getVector4fMap ();
      pcl::Vector4fMapConst pt = input_->points[inliers[i]].getVector4fMap ();

      float k = (pt.dot (axis_dir) - apexdotdir) * dirdotdir;
      // Calculate the projection of the point on the line
      pp.matrix () = apex + k * axis_dir;

      Eigen::Vector4f dir = pt - pp;
      dir.normalize ();

      // Calculate the actual radius of the cone at the level of the projected point
      Eigen::Vector4f height = apex - pp;
      float actual_cone_radius = tanf (opening_angle) * height.norm ();

      // Calculate the projection of the point onto the cone
      pp += dir * actual_cone_radius;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelCone<PointT, PointNT>::doSamplesVerifyModel (
      const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, const double threshold)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 7)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCone::doSamplesVerifyModel] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return (false);
  }

  Eigen::Vector4f apex (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f axis_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  float openning_angle = model_coefficients[6];

  float apexdotdir = apex.dot (axis_dir);
  float dirdotdir = 1.0f / axis_dir.dot (axis_dir);

  // Iterate through the 3d points and calculate the distances from them to the cone
  for (std::set<int>::const_iterator it = indices.begin (); it != indices.end (); ++it)
  {
    Eigen::Vector4f pt (input_->points[*it].x, input_->points[*it].y, input_->points[*it].z, 0);

    // Calculate the point's projection on the cone axis
    float k = (pt.dot (axis_dir) - apexdotdir) * dirdotdir;
    Eigen::Vector4f pt_proj = apex + k * axis_dir;
    Eigen::Vector4f dir = pt - pt_proj;
    dir.normalize ();

    // Calculate the actual radius of the cone at the level of the projected point
    Eigen::Vector4f height = apex - pt_proj;
    double actual_cone_radius = tan (openning_angle) * height.norm ();

    // Aproximate the distance from the point to the cone as the difference between
    // dist(point,cone_axis) and actual cone radius
    if (fabs (static_cast<double>(pointToAxisDistance (pt, model_coefficients) - actual_cone_radius)) > threshold)
      return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> double
pcl::SampleConsensusModelCone<PointT, PointNT>::pointToAxisDistance (
      const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients)
{
  Eigen::Vector4f apex  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f axis_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  return sqrt(pcl::sqrPointToLineDistance (pt, apex, axis_dir));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool 
pcl::SampleConsensusModelCone<PointT, PointNT>::isModelValid (const Eigen::VectorXf &model_coefficients)
{
  if (!SampleConsensusModel<PointT>::isModelValid (model_coefficients))
    return (false);

  // Check against template, if given
  if (eps_angle_ > 0.0)
  {
    // Obtain the cone direction
    Eigen::Vector4f coeff;
    coeff[0] = model_coefficients[3];
    coeff[1] = model_coefficients[4];
    coeff[2] = model_coefficients[5];
    coeff[3] = 0;

    Eigen::Vector4f axis (axis_[0], axis_[1], axis_[2], 0);
    double angle_diff = fabs (getAngle3D (axis, coeff));
    angle_diff = (std::min) (angle_diff, M_PI - angle_diff);
    // Check whether the current cone model satisfies our angle threshold criterion with respect to the given axis
    if (angle_diff > eps_angle_)
      return (false);
  }

  if (model_coefficients[6] != -std::numeric_limits<double>::max() && model_coefficients[6] < min_angle_)
    return (false);
  if (model_coefficients[6] !=  std::numeric_limits<double>::max() && model_coefficients[6] > max_angle_)
    return (false);

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelCone(PointT, PointNT)	template class PCL_EXPORTS pcl::SampleConsensusModelCone<PointT, PointNT>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CONE_H_

