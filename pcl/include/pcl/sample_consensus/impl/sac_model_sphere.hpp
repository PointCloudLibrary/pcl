/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: sac_model_sphere.hpp 34403 2010-12-01 01:48:52Z rusu $
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_SPHERE_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_SPHERE_H_

#include "pcl/sample_consensus/sac_model_sphere.h"

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelSphere<PointT>::getSamples (int &iterations, std::vector<int> &samples)
{
  // We're assuming that indices_ have already been set in the constructor
  if (indices_->empty ())
  {
    ROS_ERROR ("[pcl::SampleConsensusModelSphere::getSamples] Empty set of indices given!");
    return;
  }

  samples.resize (4);
  double trand = indices_->size () / (RAND_MAX + 1.0);

  // Check if we have enough points
  if (samples.size () > indices_->size ())
  {
    ROS_ERROR ("[pcl::SampleConsensusModelSphere::getSamples] Can not select %zu unique points out of %zu!", samples.size (), indices_->size ());
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

  // Get the values at the two points
  pcl::Array4fMapConst p0 = input_->points[samples[0]].getArray4fMap ();
  pcl::Array4fMapConst p1 = input_->points[samples[1]].getArray4fMap ();

  // Compute the segment values (in 3d) between p1 and p0
  Eigen::Array4f p1p0 = p1 - p0;

  Eigen::Array4f dy1dy2;
  int iter = 0;
  do
  {
    // Get the third point, different from the first two
    do
    {
      idx = (int)(rand () * trand);
      samples[2] = (*indices_)[idx];
      //iterations++;
    } while ( (samples[2] == samples[1]) || (samples[2] == samples[0]) );
    //iterations--;

    pcl::Array4fMapConst p2 = input_->points[samples[2]].getArray4fMap ();

    // Compute the segment values (in 3d) between p2 and p0
    Eigen::Array4f p2p0 = p2 - p0;

    dy1dy2 = p1p0 / p2p0;
    ++iter;
    if (iter > MAX_ITERATIONS_COLLINEAR )
    {
      ROS_DEBUG ("[pcl::SampleConsensusModelSphere::getSamples] WARNING: Could not select 3 non collinear points in %d iterations!", MAX_ITERATIONS_COLLINEAR);
      break;
    }
    //iterations++;
  }
  while ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) );
  //iterations--;

  // Need to improve this: we need 4 points, 3 non-collinear always, and the 4th should not be in the same plane as the other 3
  // otherwise we can encounter degenerate cases
  do
  {
    samples[3] = (int)(rand () * trand);
    //iterations++;
  } while ( (samples[3] == samples[2]) || (samples[3] == samples[1]) || (samples[3] == samples[0]) );
  //iterations--;

}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelSphere<PointT>::computeModelCoefficients (
      const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
  // Need 4 samples
  if (samples.size () != 4)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelSphere::computeModelCoefficients] Invalid set of samples given (%zu)!", samples.size ());
    return (false);
  }

  Eigen::Matrix4f temp;
  for (int i = 0; i < 4; i++)
  {
    temp (i, 0) = input_->points[samples[i]].x;
    temp (i, 1) = input_->points[samples[i]].y;
    temp (i, 2) = input_->points[samples[i]].z;
    temp (i, 3) = 1;
  }
  float m11 = temp.determinant ();
  if (m11 == 0)
    return (false);             // the points don't define a sphere!

  for (int i = 0; i < 4; ++i)
    temp (i, 0) = (input_->points[samples[i]].x) * (input_->points[samples[i]].x) +
                  (input_->points[samples[i]].y) * (input_->points[samples[i]].y) +
                  (input_->points[samples[i]].z) * (input_->points[samples[i]].z);
  float m12 = temp.determinant ();

  for (int i = 0; i < 4; ++i)
  {
    temp (i, 1) = temp (i, 0);
    temp (i, 0) = input_->points[samples[i]].x;
  }
  float m13 = temp.determinant ();

  for (int i = 0; i < 4; ++i)
  {
    temp (i, 2) = temp (i, 1);
    temp (i, 1) = input_->points[samples[i]].y;
  }
  float m14 = temp.determinant ();

  for (int i = 0; i < 4; ++i)
  {
    temp (i, 0) = temp (i, 2);
    temp (i, 1) = input_->points[samples[i]].x;
    temp (i, 2) = input_->points[samples[i]].y;
    temp (i, 3) = input_->points[samples[i]].z;
  }
  float m15 = temp.determinant ();

  // Center (x , y, z)
  model_coefficients.resize (4);
  model_coefficients[0] = 0.5 * m12 / m11;
  model_coefficients[1] = 0.5 * m13 / m11;
  model_coefficients[2] = 0.5 * m14 / m11;
  // Radius
  model_coefficients[3] = sqrt (
                                model_coefficients[0] * model_coefficients[0] +
                                model_coefficients[1] * model_coefficients[1] +
                                model_coefficients[2] * model_coefficients[2] - m15 / m11);

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelSphere<PointT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }
  distances.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (size_t i = 0; i < indices_->size (); ++i)
    // Calculate the distance from the point to the sphere as the difference between
    //dist(point,sphere_origin) and sphere_radius
    distances[i] = fabs (sqrt (
                               ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) *
                               ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) +

                               ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) *
                               ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) +

                               ( input_->points[(*indices_)[i]].z - model_coefficients[2] ) *
                               ( input_->points[(*indices_)[i]].z - model_coefficients[2] )
                              ) - model_coefficients[3]);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelSphere<PointT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers)
{
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }

  int nr_p = 0;
  inliers.resize (indices_->size ());

  // Iterate through the 3d points and calculate the distances from them to the sphere
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    // Calculate the distance from the point to the sphere as the difference between
    // dist(point,sphere_origin) and sphere_radius
    if (fabs (sqrt (
                    ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) *
                    ( input_->points[(*indices_)[i]].x - model_coefficients[0] ) +

                    ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) *
                    ( input_->points[(*indices_)[i]].y - model_coefficients[1] ) +

                    ( input_->points[(*indices_)[i]].z - model_coefficients[2] ) *
                    ( input_->points[(*indices_)[i]].z - model_coefficients[2] )
                   ) - model_coefficients[3]) < threshold)
    {
      // Returns the indices of the points whose distances are smaller than the threshold
      inliers[nr_p] = (*indices_)[i];
      nr_p++;
    }
  }
  inliers.resize (nr_p);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelSphere<PointT>::optimizeModelCoefficients (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients)
{
  boost::mutex::scoped_lock lock (tmp_mutex_);

  const int n_unknowns = 4;      // 4 unknowns
  // Needs a set of valid model coefficients
  if (model_coefficients.size () != n_unknowns)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelSphere::optimizeModelCoefficients] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  // Need at least 4 samples
  if (inliers.size () <= 4)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelSphere::optimizeModelCoefficients] Not enough inliers found to support a model (%zu)! Returning the same coefficients.", inliers.size ());
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
  int info = lmdif1 (&pcl::SampleConsensusModelSphere<PointT>::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa);

  // Compute the L2 norm of the residuals
  ROS_DEBUG ("[pcl::SampleConsensusModelSphere::optimizeModelCoefficients] LM solver finished with exit code %i, having a residual norm of %g. \nInitial solution: %g %g %g %g \nFinal solution: %g %g %g %g",
             info, enorm (m, fvec), model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3], x[0], x[1], x[2], x[3]);

  optimized_coefficients = Eigen::Vector4f (x[0], x[1], x[2], x[3]);

  free (wa); free (fvec);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::SampleConsensusModelSphere<PointT>::functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag)
{
  SampleConsensusModelSphere *model = (SampleConsensusModelSphere*)p;

  Eigen::Vector4f cen_t;
  cen_t[3] = 0;
  for (int i = 0; i < m; ++i)
  {
    // Compute the difference between the center of the sphere and the datapoint X_i
    cen_t[0] = model->input_->points[(*model->tmp_inliers_)[i]].x - x[0];
    cen_t[1] = model->input_->points[(*model->tmp_inliers_)[i]].y - x[1];
    cen_t[2] = model->input_->points[(*model->tmp_inliers_)[i]].z - x[2];

    // g = sqrt ((x-a)^2 + (y-b)^2 + (z-c)^2) - R
    fvec[i] = sqrt (cen_t.dot (cen_t)) - x[3];
  }
  return (0);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelSphere<PointT>::projectPoints (
      const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 4)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelSphere::projectPoints] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return;
  }

  // Allocate enough space and copy the basics
  projected_points.points.resize (input_->points.size ());
  projected_points.header   = input_->header;
  projected_points.width    = input_->width;
  projected_points.height   = input_->height;
  projected_points.is_dense = input_->is_dense;

  ROS_WARN ("[pcl::SampleConsensusModelSphere::projectPoints] Not implemented yet.");
  projected_points.points = input_->points;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelSphere<PointT>::doSamplesVerifyModel (
      const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, double threshold)
{
  // Needs a valid model coefficients
  if (model_coefficients.size () != 4)
  {
    ROS_ERROR ("[pcl::SampleConsensusModelSphere::doSamplesVerifyModel] Invalid number of model coefficients given (%zu)!", model_coefficients.size ());
    return (false);
  }

  for (std::set<int>::const_iterator it = indices.begin (); it != indices.end (); ++it)
    // Calculate the distance from the point to the sphere as the difference between
    //dist(point,sphere_origin) and sphere_radius
    if (fabs (sqrt (
                    ( input_->points[*it].x - model_coefficients[0] ) *
                    ( input_->points[*it].x - model_coefficients[0] ) +
                    ( input_->points[*it].y - model_coefficients[1] ) *
                    ( input_->points[*it].y - model_coefficients[1] ) +
                    ( input_->points[*it].z - model_coefficients[2] ) *
                    ( input_->points[*it].z - model_coefficients[2] )
                   ) - model_coefficients[3]) > threshold)
      return (false);

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelSphere(T) template class pcl::SampleConsensusModelSphere<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_SPHERE_H_

