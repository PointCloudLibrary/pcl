/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: icp_nl.hpp 35881 2011-02-09 02:02:52Z rusu $
 *
 */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using an iterative
  * non-linear Levenberg-Marquardt approach.
  * \param cloud_src the source point cloud dataset
  * \param cloud_tgt the target point cloud dataset
  * \param transformation_matrix the resultant transformation matrix
  */
template <typename PointSource, typename PointTarget> void
pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::estimateRigidTransformationLM (
      const PointCloudSource &cloud_src, const PointCloudTarget &cloud_tgt, Eigen::Matrix4f &transformation_matrix)
{
  boost::mutex::scoped_lock lock (tmp_mutex_);

  int n_unknowns = 6;      // 6 unknowns: 3 translation + 3 rotation (quaternion)

  if (cloud_src.points.size () != cloud_tgt.points.size ())
  {
    ROS_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] Number or points in source (%zu) differs than target (%zu)!", cloud_src.points.size (), cloud_tgt.points.size ());
    return;
  }
  if (cloud_src.points.size () < 4)     // need at least 4 samples
  {
    ROS_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] Need at least 4 points to estimate a transform! Source and target have %zu points!", cloud_src.points.size ());
    return;
  }

  int m = cloud_src.points.size ();
  double *fvec = new double[m];
  int *iwa = new int[n_unknowns];
  int lwa = m * n_unknowns + 5 * n_unknowns + m;
  double *wa = new double[lwa];

  // Set the initial solution
  double *x = new double[n_unknowns];
  // Translation estimates - initial guess
  x[0] = 0; x[1] = 0; x[2] = 0;
  // Rotation estimates - initial guess quaternion: x-y-z-w
  x[3] = 0; x[4] = 0; x[5] = 0;

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;

  // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
  double tol = sqrt (dpmpar (1));

  // Optimize using forward-difference approximation LM
  //int info = lmdif1 (&pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa);
  int info = lmdif1 (&pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::functionToOptimize, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa);

  // Compute the norm of the residuals
  ROS_DEBUG ("[pcl::%s::estimateRigidTransformationLM] LM solver finished with exit code %i, having a residual norm of %g. ",
             //"\nFinal solution: [%f %f %f %f] [%f %f %f]", 
             getClassName ().c_str (), info, enorm (m, fvec));
             //x[0], x[1], x[2], x[3], x[4], x[5], x[6]);

  delete [] wa; delete [] fvec;
  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setZero ();

  // Return the correct transformation
  // Compute w from the unit quaternion
  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));
  transformation_matrix.topLeftCorner<3, 3> () = q.toRotationMatrix ();

  Eigen::Vector4f t (x[4], x[5], x[6], 1.0);
  transformation_matrix.block <4, 1> (0, 3) = t;

  tmp_src_ = tmp_tgt_ = NULL;

  delete[] iwa;
  delete[] x;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using an iterative
  * non-linear Levenberg-Marquardt approach.
  * \param cloud_src the source point cloud dataset
  * \param indices_src the vector of indices describing the points of interest in \a cloud_src
  * \param cloud_tgt the target point cloud dataset
  * \param indices_tgt the vector of indices describing the correspondences of the interst points from \a indices_src
  * \param transformation_matrix the resultant transformation matrix
  */
template <typename PointSource, typename PointTarget> void
  pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::estimateRigidTransformationLM (
      const PointCloudSource &cloud_src, const std::vector<int> &indices_src, const PointCloudTarget &cloud_tgt, const std::vector<int> &indices_tgt, Eigen::Matrix4f &transformation_matrix)
{
  boost::mutex::scoped_lock lock (tmp_mutex_);

  int n_unknowns = 6;      // 6 unknowns:  3 translation + 3 rotation (quaternion)

  if (indices_src.size () != indices_tgt.size ())
  {
    ROS_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] Number or points in source (%zu) differs than target (%zu)!", indices_src.size (), indices_tgt.size ());
    return;
  }
  if (indices_src.size () < 4)     // need at least 4 samples
  {
    ROS_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] Need at least 4 points to estimate a transform! Source and target have %zu points!", indices_src.size ());
    return;
  }

  int m = indices_src.size ();
  double *fvec = new double[m];
  int *iwa = new int[n_unknowns];
  int lwa = m * n_unknowns + 5 * n_unknowns + m;
  double *wa = new double[lwa];

  // Set the initial solution
  double *x = new double[n_unknowns];
  // Translation estimates - initial guess
  x[0] = 0; x[1] = 0; x[2] = 0;
  // Rotation estimates - initial guess quaternion: x-y-z-w
  x[3] = 0; x[4] = 0; x[5] = 0;

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;
  tmp_idx_src_ = &indices_src;
  tmp_idx_tgt_ = &indices_tgt;

  // Set tol to the square root of the machine. Unless high solutions are required, these are the recommended settings.
  double tol = sqrt (dpmpar (1));

  // Optimize using forward-difference approximation LM
  int info = lmdif1 (&pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::functionToOptimizeIndices, this, m, n_unknowns, x, fvec, tol, iwa, wa, lwa);

  // Compute the norm of the residuals
  ROS_DEBUG ("[pcl::%s::estimateRigidTransformationLM] LM solver finished with exit code %i, having a residual norm of %g. ",
             //"\nFinal solution: [%f %f %f %f] [%f %f %f]", 
             getClassName ().c_str (), info, enorm (m, fvec));
             //x[0], x[1], x[2], x[3], x[4], x[5], x[6]);

  delete [] wa; delete [] fvec;
  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setZero ();

  // Return the correct transformation
  // Compute w from the unit quaternion
  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));
  transformation_matrix.topLeftCorner<3, 3> () = q.toRotationMatrix ();

  Eigen::Vector4f t (x[0], x[1], x[2], 1.0);
  transformation_matrix.block <4, 1> (0, 3) = t;

  tmp_src_ = tmp_tgt_ = NULL;
  tmp_idx_src_ = tmp_idx_tgt_ = NULL;

  delete[] iwa;
  delete[] x;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Rigid transformation computation method.
  * \param output the transformed input point cloud dataset using the rigid transformation found
  */
template <typename PointSource, typename PointTarget> void
  pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::computeTransformation (PointCloudSource &output)
{
  // Allocate enough space to hold the results
  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // Point cloud containing the correspondences of each point in <input, indices>
  PointCloudTarget input_corresp;
  input_corresp.points.resize (indices_->size ());

  nr_iterations_ = 0;
  converged_ = false;
  double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;

  while (!converged_)           // repeat until convergence
  {
    // Save the previously estimated transformation
    previous_transformation_ = transformation_;

    int cnt = 0;
    std::vector<int> source_indices (indices_->size ());
    std::vector<int> target_indices (indices_->size ());

    // Iterating over the entire index vector and  find all correspondences
    for (size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if (!searchForNeighbors (output, idx, nn_indices, nn_dists))
      {
        ROS_ERROR ("[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the source!", getClassName ().c_str (), (*indices_)[idx]);
        return;
      }

      // Check if the distance to the nearest neighbor is smaller than the user imposed threshold
      if (nn_dists[0] < dist_threshold)   // nn_dists should be squared distances
      {
        source_indices[cnt] = idx;
        target_indices[cnt] = nn_indices[0];
        cnt++;
      }

      //input_corresp.points[idx] = target_->points[nn_indices[0]];
    }
    if (cnt < min_number_correspondences_)
    {
      ROS_ERROR ("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.", getClassName ().c_str ());
      converged_ = false;
      return;
    }

    // Resize to the actual number of valid correspondences
    source_indices.resize (cnt); target_indices.resize (cnt);

    std::vector<int> source_indices_good;
    std::vector<int> target_indices_good;
    {
      // From the set of correspondences found, attempt to remove outliers
      // Create the registration model
      typedef typename SampleConsensusModelRegistration<PointSource>::Ptr SampleConsensusModelRegistrationPtr;
      SampleConsensusModelRegistrationPtr model;
      model.reset (new SampleConsensusModelRegistration<PointSource> (output.makeShared (), source_indices));
      // Pass the target_indices
      model->setInputTarget (target_, target_indices);
      // Create a RANSAC model
      RandomSampleConsensus<PointSource> sac (model, inlier_threshold_);
      sac.setMaxIterations (1000);

      // Compute the set of inliers
      if (!sac.computeModel ())
      {
        source_indices_good = source_indices;
        target_indices_good = target_indices;
      }
      else
      {
        std::vector<int> inliers;
        // Get the inliers
        sac.getInliers (inliers);
        source_indices_good.resize (inliers.size ());
        target_indices_good.resize (inliers.size ());
        // Copy just the inliers
        for (size_t i = 0; i < inliers.size (); ++i)
        {
          source_indices_good[i] = source_indices[inliers[i]];
          target_indices_good[i] = target_indices[inliers[i]];
        }
      }
    }

    // Check whether we have enough correspondences
    cnt = (int)source_indices_good.size ();
    if (cnt < min_number_correspondences_)
    {
      ROS_ERROR ("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.", getClassName ().c_str ());
      converged_ = false;
      return;
    }

    ROS_DEBUG ("[pcl::%s::computeTransformation] Number of correspondences %d [%f%%] out of %zu points [100.0%%], RANSAC rejected: %zu [%f%%].", getClassName ().c_str (), cnt, (cnt * 100.0) / indices_->size (), indices_->size (), source_indices.size () - cnt, (source_indices.size () - cnt) * 100.0 / source_indices.size ());
  
    // Estimate the transform
    estimateRigidTransformationLM (output, source_indices_good, *target_, target_indices_good, transformation_);
    //estimateRigidTransformationLM (output, source_indices, *target_, target_indices, transformation_);

    // Tranform the data
    transformPointCloud (output, output, transformation_);

    // Obtain the final transformation
    final_transformation_ = transformation_ * final_transformation_;

    nr_iterations_++;
    // Check for convergence
    if (nr_iterations_ >= max_iterations_ ||
        fabs ((transformation_ - previous_transformation_).sum ()) < transformation_epsilon_)
    {
      converged_ = true;
      ROS_DEBUG ("[pcl::%s::computeTransformation] Convergence reached. Number of iterations: %d out of %d. Transformation difference: %f",
                 getClassName ().c_str (), nr_iterations_, max_iterations_, fabs ((transformation_ - previous_transformation_).sum ()));
    }
  }
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
template <typename PointSource, typename PointTarget> inline int
  pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag)
{
  IterativeClosestPointNonLinear *model = (IterativeClosestPointNonLinear*)p;

  // Copy the rotation and translation components
  Eigen::Vector4f t (x[0], x[1], x[2], 1.0);
  // Compute w from the unit quaternion
  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));
  // If the quaternion is used to rotate several points (>1) then it is much more efficient to first convert it
  // to a 3x3 Matrix. Comparison of the operation cost for n transformations:
  // * Quaternion: 30n
  // * Via a Matrix3: 24 + 15n
  Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Zero ();
  transformation_matrix.topLeftCorner<3, 3> () = q.toRotationMatrix ();
  transformation_matrix.block <4, 1> (0, 3) = t;

  double sigma = model->getMaxCorrespondenceDistance ();
  for (int i = 0; i < m; ++i)
  {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_src = model->tmp_src_->points[i].getVector4fMap ();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt = model->tmp_tgt_->points[i].getVector4fMap ();

    Eigen::Vector4f pp = transformation_matrix * p_src;

    // Estimate the distance (cost function)
    //fvec[i] = model->distL2Sqr (p_tgt, pp);
    //fvec[i] = model->distL1 (pp, p_tgt);
    fvec[i] = model->distHuber (pp, p_tgt, sigma);
  }
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Compute the median value from a set of doubles
  * \param fvec the set of doubles
  * \param m the number of doubles in the set
  */
template <typename PointSource, typename PointTarget> inline double
  pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::computeMedian (double *fvec, int m)
{
  double median;
  // Copy the values to vectors for faster sorting
  std::vector<double> data (m);
  memcpy (&data[0], fvec, sizeof (double) * m);

  std::sort (data.begin (), data.end ());

  int mid = data.size () / 2;
  if (data.size () % 2 == 0)
    median = (data[mid-1] + data[mid]) / 2.0;
  else
    median = data[mid];
  return (median);
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
template <typename PointSource, typename PointTarget> inline int
  pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::functionToOptimizeIndices (void *p, int m, int n, const double *x, double *fvec, int iflag)
{
  IterativeClosestPointNonLinear *model = (IterativeClosestPointNonLinear*)p;

  // Copy the rotation and translation components
  Eigen::Vector4f t (x[0], x[1], x[2], 1.0);
  // Compute w from the unit quaternion
  Eigen::Quaternionf q (0, x[3], x[4], x[5]);
  q.w () = sqrt (1 - q.dot (q));

  // If the quaternion is used to rotate several points (>1) then it is much more efficient to first convert it
  // to a 3x3 Matrix. Comparison of the operation cost for n transformations:
  // * Quaternion: 30n
  // * Via a Matrix3: 24 + 15n
  Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Zero ();
  transformation_matrix.topLeftCorner<3, 3> () = q.toRotationMatrix ();
  transformation_matrix.block <4, 1> (0, 3) = t;

  //double sigma = model->getMaxCorrespondenceDistance ();
  for (int i = 0; i < m; ++i)
  {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_src = model->tmp_src_->points[(*model->tmp_idx_src_)[i]].getVector4fMap ();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt = model->tmp_tgt_->points[(*model->tmp_idx_tgt_)[i]].getVector4fMap ();

    Eigen::Vector4f pp = transformation_matrix * p_src;

    // Estimate the distance (cost function)
    //fvec[i] = model->distL2Sqr (p_tgt, pp);
    fvec[i] = model->distL1 (pp, p_tgt);
    //fvec[i] = model->distHuber (pp, p_tgt, sigma);
  }
/*  // Compute the median
  double median = model->computeMedian (fvec, m);

  double stddev = 1.4826 * median;

  std::vector<double> weights (m);
  // For all points, compute weights
  for (int i = 0; i < m; ++i)
  {
    weights[i] = model->distHuber (fvec[i], stddev);
    fvec[i] *= weights[i];
  }*/
  return (0);
}

