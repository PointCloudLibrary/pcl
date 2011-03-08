/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009-2010, Willow Garage, Inc.
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
 * $Id: sac_model_cylinder.hpp 34403 2010-12-01 01:48:52Z rusu $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CYLINDER_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CYLINDER_H_

#include "pcl/sample_consensus/sac_model_cylinder.h"
#include <pcl/common/distances.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Get 2 random points with their normals as data samples and return them as point indices.
  * \param iterations the internal number of iterations used by SAC methods
  * \param samples the resultant model samples
  * \note assumes unique points!
  */
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCylinder<PointT, PointNT>::getSamples (int &iterations, std::vector<int> &samples)
{
  // We're assuming that indices_ have already been set in the constructor
  if (indices_->empty ())
  {
    ROS_ERROR ("[pcl::SampleConsensusModelCylinder::getSamples] Empty set of indices given!");
    return;
  }

  samples.resize (2);
  double trand = indices_->size () / (RAND_MAX + 1.0);

  // Check if we have enough points
  if (samples.size () > indices_->size ())
  {
    ROS_ERROR ("[pcl::SampleConsensusModelCylinder::getSamples] Can not select %zu unique points out of %zu!", samples.size (), indices_->size ());
    // one of these will make it stop :) TODO static constant for each model that the method has to check
    samples.clear ();
    iterations = INT_MAX - 1;
    return;
  }

  // Get a random number between 1 and max_indices
  int idx = (int)(rand () * trand);
  // Get the index
  samples[0] = (*indices_)[idx];

  // Get a second point which is different than the first
  do
  {
    idx = (int)(rand () * trand);
    samples[1] = (*indices_)[idx];
    //iterations++;
  } while (samples[1] == samples[0]);
  //iterations--;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Check whether the given index samples can form a valid cylinder model, compute the model coefficients
  * from these samples and store them in model_coefficients. The cylinder coefficients are: point_on_axis,
  * axis_direction, cylinder_radius_R
  * \param samples the point indices found as possible good candidates for creating a valid model
  * \param model_coefficients the resultant model coefficients
  */
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelCylinder<PointT, PointNT>::computeModelCoefficients (
      const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
  // Need 2 samples
  if (samples.size () != 2)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelCylinder::computeModelCoefficients] Invalid set of samples given (%zu)!", samples.size ());
    return (false);
  }

  if (!normals_)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelCylinder::computeModelCoefficients] No input dataset containing normals was given!");
    return (false);
  }

  Eigen::Vector4f p1 (input_->points[samples[0]].x, input_->points[samples[0]].y, input_->points[samples[0]].z, 0);
  Eigen::Vector4f p2 (input_->points[samples[1]].x, input_->points[samples[1]].y, input_->points[samples[1]].z, 0);

  Eigen::Vector4f n1 (normals_->points[samples[0]].normal[0], normals_->points[samples[0]].normal[1], normals_->points[samples[0]].normal[2], 0);
  Eigen::Vector4f n2 (normals_->points[samples[1]].normal[0], normals_->points[samples[1]].normal[1], normals_->points[samples[1]].normal[2], 0);
  Eigen::Vector4f w = n1 + p1 - p2;

  double a = n1.dot (n1);
  double b = n1.dot (n2);
  double c = n2.dot (n2);
  double d = n1.dot (w);
  double e = n2.dot (w);
  double denominator = a*c - b*b;
  double sc, tc;
  // Compute the line parameters of the two closest points
  if (denominator < 1e-8)          // The lines are almost parallel
  {
    sc = 0.0;
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
  model_coefficients.template head<3> ()    = line_pt.template head<3> ();
  model_coefficients.template segment<3> (3) = line_dir.template head<3> ();
  // cylinder radius
  model_coefficients[6] = sqrt(pcl::sqrPointToLineDistance (p1, line_pt, line_dir));

  if (model_coefficients[6] > radius_max_ || model_coefficients[6] < radius_min_)
    return (false);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Compute all distances from the cloud data to a given cylinder model.
  * \param model_coefficients the coefficients of a cylinder model that we need to compute distances to
  * \param distances the resultant estimated distances
  */
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCylinder<PointT, PointNT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 7)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelCylinder::getDistancesToModel] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }

  distances.resize (indices_->size ());

  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  double ptdotdir = line_pt.dot (line_dir);
  double dirdotdir = 1.0 / line_dir.dot (line_dir);
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
    double k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
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
/** \brief Select all the points which respect the given model coefficients as inliers.
  * \param model_coefficients the coefficients of a cylinder model that we need to compute distances to
  * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
  * \param inliers the resultant model inliers
  */
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCylinder<PointT, PointNT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 7)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelCylinder::selectWithinDistance] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }

  int nr_p = 0;
  inliers.resize (indices_->size ());

  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  double ptdotdir = line_pt.dot (line_dir);
  double dirdotdir = 1.0 / line_dir.dot (line_dir);
  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Aproximate the distance from the point to the cylinder as the difference between
    // dist(point,cylinder_axis) and cylinder radius
    Eigen::Vector4f pt (input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z, 0);
    Eigen::Vector4f n  (normals_->points[(*indices_)[i]].normal[0], normals_->points[(*indices_)[i]].normal[1], normals_->points[(*indices_)[i]].normal[2], 0);
    double d_euclid = fabs (pointToLineDistance (pt, model_coefficients) - model_coefficients[6]);

    // Calculate the point's projection on the cylinder axis
    double k = (pt.dot (line_dir) - ptdotdir) * dirdotdir;
    Eigen::Vector4f pt_proj = line_pt + k * line_dir;
    Eigen::Vector4f dir = pt - pt_proj;
    dir.normalize ();

    // Calculate the angular distance between the point normal and the (dir=pt_proj->pt) vector
    double d_normal = fabs (getAngle3D (n, dir));
    d_normal = (std::min) (d_normal, M_PI - d_normal);

    if (fabs (normal_distance_weight_ * d_normal + (1 - normal_distance_weight_) * d_euclid) < threshold)
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      inliers[nr_p] = (*indices_)[i];
      nr_p++;
    }
  }
  inliers.resize (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Recompute the cylinder coefficients using the given inlier set and return them to the user.
  * @note: these are the coefficients of the cylinder model after refinement (eg. after SVD)
  * \param inliers the data inliers found as supporting the model
  * \param model_coefficients the initial guess for the optimization
  * \param optimized_coefficients the resultant recomputed coefficients after non-linear optimization
  */
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCylinder<PointT, PointNT>::optimizeModelCoefficients (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients)
{
  boost::mutex::scoped_lock lock (tmp_mutex_);

  const int n_unknowns = 7;      // 4 unknowns
  // Needs a set of valid model coefficients
  if (model_coefficients.size () != n_unknowns)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelCylinder::optimizeModelCoefficients] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  if (inliers.empty ())
  {
    ROS_DEBUG ("[pcl::SampleConsensusModelCylinder:optimizeModelCoefficients] Inliers vector empty! Returning the same coefficients."); 
    optimized_coefficients = model_coefficients;
    return;
  }

  tmp_inliers_ = &inliers;
  int m = inliers.size ();

  double *fvec = new double[m];

  int iwa[n_unknowns];

  int lwa = m * n_unknowns + 5 * n_unknowns + m;
  double *wa = new double[lwa];

  // Set the initial solution
  double x[n_unknowns];
  for (int d = 0; d < n_unknowns; ++d)
    x[d] = model_coefficients[d];   // initial guess

  // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
  double tol = sqrt (dpmpar (1));

  // Optimize using forward-difference approximation LM
  int info = lmdif1 (&pcl::SampleConsensusModelCylinder<PointT, PointNT>::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa);

  // Compute the L2 norm of the residuals
  ROS_DEBUG ("[pcl::SampleConsensusModelCylinder::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g %g %g %g \nFinal solution: %g %g %g %g %g %g %g",
             info, enorm (m, fvec), model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3],
             model_coefficients[4], model_coefficients[5], model_coefficients[6], x[0], x[1], x[2], x[3], x[4], x[5], x[6]);

  optimized_coefficients.resize (n_unknowns);
  for (int d = 0; d < n_unknowns; ++d)
    optimized_coefficients[d] = x[d];

  free (wa); free (fvec);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////(
/** \brief Cost function to be minimized
  * \param p a pointer to our data structure array
  * \param m the number of functions
  * \param n the number of variables
  * \param x a pointer to the variables array
  * \param fvec a pointer to the resultant functions evaluations
  * \param iflag set to -1 inside the function to terminate execution
  */
template <typename PointT, typename PointNT> int
pcl::SampleConsensusModelCylinder<PointT, PointNT>::functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag)
{
  SampleConsensusModelCylinder *model = (SampleConsensusModelCylinder*)p;

  Eigen::Vector4f line_pt  (x[0], x[1], x[2], 0);
  Eigen::Vector4f line_dir (x[3], x[4], x[5], 0);

  for (int i = 0; i < m; ++i)
  {
    // dist = f - r
    Eigen::Vector4f pt (model->input_->points[(*model->tmp_inliers_)[i]].x,
                        model->input_->points[(*model->tmp_inliers_)[i]].y,
                        model->input_->points[(*model->tmp_inliers_)[i]].z, 0);
    fvec[i] = pcl::sqrPointToLineDistance (pt, line_pt, line_dir) - x[6]*x[6];
  }
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Create a new point cloud with inliers projected onto the cylinder model.
  * \param inliers the data inliers that we want to project on the cylinder model
  * \param model_coefficients the coefficients of a cylinder model
  * \param projected_points the resultant projected points
  * \param copy_data_fields set to true if we need to copy the other data fields
  * \todo implement this.
  */
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCylinder<PointT, PointNT>::projectPoints (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields)
{
  // Needs a valid set of model coefficients
  if (model_coefficients.size () != 7)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelCylinder::projectPoints] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return;
  }

  projected_points.header = input_->header;
  projected_points.is_dense = input_->is_dense;

  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  double ptdotdir = line_pt.dot (line_dir);
  double dirdotdir = 1.0 / line_dir.dot (line_dir);

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

      double k = (p.dot (line_dir) - ptdotdir) * dirdotdir;

      pcl::Vector4fMap pp = projected_points.points[inliers[i]].getVector4fMap ();
      pp = line_pt + k * line_dir;

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
    projected_points.width    = inliers.size ();
    projected_points.height   = 1;

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < inliers.size (); ++i)
    {
      pcl::Vector4fMap pp = projected_points.points[i].getVector4fMap ();
      pcl::Vector4fMapConst p = input_->points[inliers[i]].getVector4fMap ();

      double k = (p.dot (line_dir) - ptdotdir) * dirdotdir;
      // Calculate the projection of the point on the line
      pp = line_pt + k * line_dir;

      Eigen::Vector4f dir = p - pp;
      dir.normalize ();

      // Calculate the projection of the point onto the cylinder
      pp += dir * model_coefficients[6];
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Verify whether a subset of indices verifies the given cylinder model coefficients.
  * \param indices the data indices that need to be tested against the cylinder model
  * \param model_coefficients the cylinder model coefficients
  * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
  */
template <typename PointT, typename PointNT> bool
pcl::SampleConsensusModelCylinder<PointT, PointNT>::doSamplesVerifyModel (
      const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, double threshold)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 7)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelCylinder::doSamplesVerifyModel] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
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
/** \brief Get the distance from a point to a line (represented by a point and a direction)
  * \param pt a point
  * \param model_coefficients the line coefficients (a point on the line, line direction)
  */
template <typename PointT, typename PointNT> double
pcl::SampleConsensusModelCylinder<PointT, PointNT>::pointToLineDistance (
      const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients)
{
  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
  return sqrt(pcl::sqrPointToLineDistance (pt, line_pt, line_dir));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Project a point onto a cylinder given by its model coefficients (point_on_axis, axis_direction,
  * cylinder_radius_R)
  * \param pt the input point to project
  * \param model_coefficients the coefficients of the cylinder (point_on_axis, axis_direction, cylinder_radius_R)
  * \param pt_proj the resultant projected point
  */
template <typename PointT, typename PointNT> void
pcl::SampleConsensusModelCylinder<PointT, PointNT>::projectPointToCylinder (
      const Eigen::Vector4f &pt, const Eigen::VectorXf &model_coefficients, Eigen::Vector4f &pt_proj)
{
  Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
  Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

  double k = (pt.dot (line_dir) - line_pt.dot (line_dir)) * line_dir.dot (line_dir);
  pt_proj = line_pt + k * line_dir;

  Eigen::Vector4f dir = pt - pt_proj;
  dir.normalize ();

  // Calculate the projection of the point onto the cylinder
  pt_proj += dir * model_coefficients[6];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Check whether a model is valid given the user constraints.
  * \param model_coefficients the set of model coefficients
  */
template <typename PointT, typename PointNT> bool 
pcl::SampleConsensusModelCylinder<PointT, PointNT>::isModelValid (const Eigen::VectorXf &model_coefficients)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 7)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelCylinder::isModelValid] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
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

  if (radius_min_ != -DBL_MAX && model_coefficients[3] < radius_min_)
    return (false);
  if (radius_max_ != DBL_MAX && model_coefficients[3] > radius_max_)
    return (false);

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelCylinder(PointT, PointNT)	template class pcl::SampleConsensusModelCylinder<PointT, PointNT>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_CYLINDER_H_

