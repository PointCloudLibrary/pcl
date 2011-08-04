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
 * $Id$
 *
 */

#include <boost/unordered_map.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::estimateRigidTransformationLM (
      const PointCloudSource &cloud_src, const PointCloudTarget &cloud_tgt, Eigen::Matrix4f &transformation_matrix)
{
  boost::mutex::scoped_lock lock (tmp_mutex_);

  int n_unknowns = 6;      // 6 unknowns: 3 translation + 3 rotation (quaternion)

  if (cloud_src.points.size () != cloud_tgt.points.size ())
  {
    PCL_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)cloud_src.points.size (), (unsigned long)cloud_tgt.points.size ());
    return;
  }
  if (cloud_src.points.size () < 4)     // need at least 4 samples
  {
    PCL_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] Need at least 4 points to estimate a transform! Source and target have %lu points!\n", (unsigned long)cloud_src.points.size ());
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
  PCL_DEBUG ("[pcl::%s::estimateRigidTransformationLM] LM solver finished with exit code %i, having a residual norm of %g. \n",
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

  delete[] iwa;
  delete[] x;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
  pcl::IterativeClosestPointNonLinear<PointSource, PointTarget>::estimateRigidTransformationLM (
      const PointCloudSource &cloud_src, const std::vector<int> &indices_src, const PointCloudTarget &cloud_tgt, const std::vector<int> &indices_tgt, Eigen::Matrix4f &transformation_matrix)
{
  boost::mutex::scoped_lock lock (tmp_mutex_);

  int n_unknowns = 6;      // 6 unknowns:  3 translation + 3 rotation (quaternion)

  if (indices_src.size () != indices_tgt.size ())
  {
    PCL_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)indices_src.size (), (unsigned long)indices_tgt.size ());
    return;
  }
  if (indices_src.size () < 4)     // need at least 4 samples
  {
    PCL_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateRigidTransformationLM] Need at least 4 points to estimate a transform! Source and target have %lu points!", (unsigned long)indices_src.size ());
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
  PCL_DEBUG ("[pcl::%s::estimateRigidTransformationLM] LM solver finished with exit code %i, having a residual norm of %g. \n",
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////(
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
    fvec[i] = model->distL1 (pp, p_tgt);
    //fvec[i] = model->distHuber (pp, p_tgt, sigma);
  }
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    fvec[i] = model->distL2Sqr (p_tgt, pp);
    //fvec[i] = model->distL1 (pp, p_tgt);
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

