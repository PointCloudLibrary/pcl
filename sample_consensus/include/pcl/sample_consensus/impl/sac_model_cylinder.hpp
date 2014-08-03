/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2010, Willow Garage, Inc.
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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CYLINDER_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CYLINDER_H_

#include <pcl/sample_consensus/eigen.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/common/concatenate.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelCylinder<PointT, PointNT>::isSampleGood (const std::vector<int> &) const
{
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelCylinder<PointT, PointNT>::computeModelCoefficients (
      const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
  // Need 2 samples
  if (samples.size () != 2)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCylinder::computeModelCoefficients] Invalid set of samples given (%lu)!\n", samples.size ());
    return (false);
  }

  if (!normals_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCylinder::computeModelCoefficients] No input dataset containing normals was given!\n");
    return (false);
  }

  if (fabs (input_->points[samples[0]].x - input_->points[samples[1]].x) <= std::numeric_limits<float>::epsilon () && 
      fabs (input_->points[samples[0]].y - input_->points[samples[1]].y) <= std::numeric_limits<float>::epsilon () && 
      fabs (input_->points[samples[0]].z - input_->points[samples[1]].z) <= std::numeric_limits<float>::epsilon ()) 
  {
    return (false);
  }
  
  Eigen::Vector4f p1 (input_->points[samples[0]].x, input_->points[samples[0]].y, input_->points[samples[0]].z, 0);
  Eigen::Vector4f p2 (input_->points[samples[1]].x, input_->points[samples[1]].y, input_->points[samples[1]].z, 0);

  Eigen::Vector4f n1 (normals_->points[samples[0]].normal[0], normals_->points[samples[0]].normal[1], normals_->points[samples[0]].normal[2], 0);
  Eigen::Vector4f n2 (normals_->points[samples[1]].normal[0], normals_->points[samples[1]].normal[1], normals_->points[samples[1]].normal[2], 0);
  Eigen::Vector4f w = n1 + p1 - p2;

  float a = n1.dot (n1);
  float b = n1.dot (n2);
  float c = n2.dot (n2);
  float d = n1.dot (w);
  float e = n2.dot (w);
  float denominator = a*c - b*b;
  float sc, tc;
  // Compute the line parameters of the two closest points
  if (denominator < 1e-8)          // The lines are almost parallel
  {
    sc = 0.0f;
    tc = (b > c ? d / b : e / c);  // Use the largest denominator
  }
  else
  {
    sc = (b*e - c*d) / denominator;
    tc = (a*e - b*d) / denominator;
  }

  // point_on_axis, axis_direction
  Eigen::Vector4f line_pt  = p1 + n1 + sc * n1;
  Eigen::Vector4f line_dir = p2 + tc * n2 - line_pt;
  line_dir.normalize ();

  model_coefficients.resize (7);
  // model_coefficients.template head<3> ()    = line_pt.template head<3> ();
  model_coefficients[0] = line_pt[0];
  model_coefficients[1] = line_pt[1];
  model_coefficients[2] = line_pt[2];
  // model_coefficients.template segment<3> (3) = line_dir.template head<3> ();
  model_coefficients[3] = line_dir[0];
  model_coefficients[4] = line_dir[1];
  model_coefficients[5] = line_dir[2];
  // cylinder radius
  model_coefficients[6] = static_cast<float> (sqrt (pcl::sqrPointToLineDistance (p1, line_pt, line_dir)));

  if (model_coefficients[6] > radius_max_ || model_coefficients[6] < radius_min_)
    return (false);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCylinder<PointT, PointNT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }

  distances.resize (indices_->size ());

  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  float ptdotdir = line_pt.dot (line_dir);
  float dirdotdir = 1.0f / line_dir.dot (line_dir);
  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Aproximate the distance from the point to the cylinder as the difference between
    // dist(point,cylinder_axis) and cylinder radius
    // @note need to revise this.
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 0);
    Eigen::Vector4f n  (normals_->points[(*indices_)[i]].normal[0], normals_->points[(*indices_)[i]].normal[1], normals_->points[(*indices_)[i]].normal[2], 0);

    double d_euclid = fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]);

    // Calculate the point's projection on the cylinder axis
    float k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
    Eigen::Vector4f pt_proj = line_pt + k * line_dir;
    Eigen::Vector4f dir = pt - pt_proj;
    dir.normalize ();

    // Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
    double d_normal = fabs (getAngle3D (n, dir));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    distances[i] = fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCylinder<PointT, PointNT>::selectWithinDistance (
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

  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  float ptdotdir = line_pt.dot (line_dir);
  float dirdotdir = 1.0f / line_dir.dot (line_dir);
  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Aproximate the distance from the point to the cylinder as the difference between
    // dist(point,cylinder_axis) and cylinder radius
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 0);
    Eigen::Vector4f n  (normals_->points[(*indices_)[i]].normal[0], normals_->points[(*indices_)[i]].normal[1], normals_->points[(*indices_)[i]].normal[2], 0);
    double d_euclid = fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]);

    // Calculate the point's projection on the cylinder axis
    float k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
    Eigen::Vector4f pt_proj = line_pt + k * line_dir;
    Eigen::Vector4f dir = pt - pt_proj;
    dir.normalize ();

    // Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
    double d_normal = fabs (getAngle3D (n, dir));
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
pcl::SampleConsensusModelCylinder<PointT, PointNT>::countWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return (0);

  int nr_p = 0;

  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  float ptdotdir = line_pt.dot (line_dir);
  float dirdotdir = 1.0f / line_dir.dot (line_dir);
  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Aproximate the distance from the point to the cylinder as the difference between
    // dist(point,cylinder_axis) and cylinder radius
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 0);
    Eigen::Vector4f n  (normals_->points[(*indices_)[i]].normal[0], normals_->points[(*indices_)[i]].normal[1], normals_->points[(*indices_)[i]].normal[2], 0);
    double d_euclid = fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]);

    // Calculate the point's projection on the cylinder axis
    float k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
    Eigen::Vector4f pt_proj = line_pt + k * line_dir;
    Eigen::Vector4f dir = pt - pt_proj;
    dir.normalize ();

    // Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
    double d_normal = fabs (getAngle3D (n, dir));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    if (fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid) < threshold)
      nr_p++;
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCylinder<PointT, PointNT>::optimizeModelCoefficients (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients)
{
  optimized_coefficients = model_coefficients;

  // Needs a set of valid model coefficients
  if (model_coefficients.size () != 7)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCylinder::optimizeModelCoefficients] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return;
  }

  if (inliers.empty ())
  {
    PCL_DEBUG ("[pcl::SampleConsensusModelCylinder:optimizeModelCoefficients] Inliers vector empty! Returning the same coefficients.\n"); 
    return;
  }

  tmp_inliers_ = &inliers;

  OptimizationFunctor functor (static_cast<int> (inliers.size ()), this);
  Eigen::NumericalDiff<OptimizationFunctor > num_diff (functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, float> lm (num_diff);
  int info = lm.minimize (optimized_coefficients);
  
  // Compute the L2 norm of the residuals
  PCL_DEBUG ("[pcl::SampleConsensusModelCylinder::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g %g\n",
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
pcl::SampleConsensusModelCylinder<PointT, PointNT>::projectPoints (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 7)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCylinder::projectPoints] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return;
  }

  projected_points.header = input_->header;
  projected_points.is_dense = input_->is_dense;

  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  float ptdotdir = line_pt.dot (line_dir);
  float dirdotdir = 1.0f / line_dir.dot (line_dir);

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

    // Iterate through the 3d points and calculate the distances from them to the cylinder
    for (size_t i = 0; i < inliers.size (); ++i)
    {
      Eigen::Vector4f p (input_->points[inliers[i]].x,
                         input_->points[inliers[i]].y,
                         input_->points[inliers[i]].z,
                         1);

      float k = (p.dot (line_dir) - ptdotdir) * dirdotdir;

      pcl::Vector4fMap pp = projected_points.points[inliers[i]].getVector4fMap ();
      pp.matrix () = line_pt + k * line_dir;

      Eigen::Vector4f dir = p - pp;
      dir.normalize ();

      // Calculate the projection of the point onto the cylinder
      pp += dir * model_coefficients[6];
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

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < inliers.size (); ++i)
    {
      pcl::Vector4fMap pp = projected_points.points[i].getVector4fMap ();
      pcl::Vector4fMapConst p = input_->points[inliers[i]].getVector4fMap ();

      float k = (p.dot (line_dir) - ptdotdir) * dirdotdir;
      // Calculate the projection of the point on the line
      pp.matrix () = line_pt + k * line_dir;

      Eigen::Vector4f dir = p - pp;
      dir.normalize ();

      // Calculate the projection of the point onto the cylinder
      pp += dir * model_coefficients[6];
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelCylinder<PointT, PointNT>::doSamplesVerifyModel (
      const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, const double threshold)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 7)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCylinder::doSamplesVerifyModel] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return (false);
  }

  for (std::set<int>::const_iterator it = indices.begin (); it != indices.end (); ++it)
  {
    // Aproximate the distance from the point to the cylinder as the difference between
    // dist(point,cylinder_axis) and cylinder radius
    // @note need to revise this.
    Eigen::Vector4f pt (input_->points[*it].x, input_->points[*it].y, input_->points[*it].z, 0);
    if (fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]) > threshold)
      return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> double
pcl::SampleConsensusModelCylinder<PointT, PointNT>::pointToLineDistance (
      const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients)
{
  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  return sqrt(pcl::sqrPointToLineDistance (pt, line_pt, line_dir));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCylinder<PointT, PointNT>::projectPointToCylinder (
      const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients, Eigen::Vector4f &pt_proj)
{
  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

  float k = (pt.dot (line_dir) - line_pt.dot (line_dir)) * line_dir.dot (line_dir);
  pt_proj = line_pt + k * line_dir;

  Eigen::Vector4f dir = pt - pt_proj;
  dir.normalize ();

  // Calculate the projection of the point onto the cylinder
  pt_proj += dir * model_coefficients[6];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool 
pcl::SampleConsensusModelCylinder<PointT, PointNT>::isModelValid (const Eigen::VectorXf &model_coefficients)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 7)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelCylinder::isModelValid] Invalid number of model coefficients given (%lu)!\n", model_coefficients.size ());
    return (false);
  }
 
  // Check against template, if given
  if (eps_angle_ > 0.0)
  {
    // Obtain the cylinder direction
    Eigen::Vector4f coeff;
    coeff[0] = model_coefficients[3];
    coeff[1] = model_coefficients[4];
    coeff[2] = model_coefficients[5];
    coeff[3] = 0;

    Eigen::Vector4f axis (axis_[0], axis_[1], axis_[2], 0);
    double angle_diff = fabs (getAngle3D (axis, coeff));
    angle_diff = (std::min) (angle_diff, M_PI - angle_diff);
    // Check whether the current cylinder model satisfies our angle threshold criterion with respect to the given axis
    if (angle_diff > eps_angle_)
      return (false);
  }

  if (radius_min_ != -std::numeric_limits<double>::max() && model_coefficients[6] < radius_min_)
    return (false);
  if (radius_max_ != std::numeric_limits<double>::max() && model_coefficients[6] > radius_max_)
    return (false);

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelCylinder(PointT, PointNT)	template class PCL_EXPORTS pcl::SampleConsensusModelCylinder<PointT, PointNT>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CYLINDER_H_

