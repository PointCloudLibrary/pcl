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
template <typename PointSource, typename PointTarget> 
template<typename PointT> void
pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeCovariances(typename pcl::PointCloud<PointT>::ConstPtr cloud, 
                   const typename pcl::KdTree<PointT>::Ptr kdtree,
                   std::vector<Eigen::Matrix3d>& cloud_covariances)
{
  if((size_t)k_correspondences_ > cloud->size ())
  {
    PCL_ERROR ("[pcl::GeneralizedIterativeClosestPoint::computeCovariances] Number or points in cloud (%lu) is less than k_correspondences_ (%lu)!\n", (unsigned long)cloud->size (), (unsigned long)k_correspondences_);
    return;
  }

  Eigen::Vector3d mean;
  std::vector<int> nn_indecies; nn_indecies.reserve (k_correspondences_);
  std::vector<float> nn_dist_sq; nn_dist_sq.reserve (k_correspondences_);

  if(cloud_covariances.size () < cloud->size ())
    cloud_covariances.reserve (cloud->size ());

  size_t points_counter =0;
  typename pcl::PointCloud<PointT>::const_iterator points_iterator = cloud->begin ();
  std::vector<Eigen::Matrix3d>::iterator matrices_iterator = cloud_covariances.begin ();
  for(;
      points_iterator != cloud->end ();
      ++points_iterator, ++matrices_iterator, points_counter++)
  {
    const PointT &query_point = *points_iterator;
    Eigen::Matrix3d &cov = *matrices_iterator;
    // Zero out the cov and mean
    cov.setZero ();
    mean.setZero ();

    // Search for the K nearest neighbours
    kdtree->nearestKSearch(query_point, k_correspondences_, nn_indecies, nn_dist_sq);
    
    // Find the covariance matrix
    for(int j = 0; j < k_correspondences_; j++) {
      const PointT &pt = (*cloud)[nn_indecies[j]];
      
      mean[0] += pt.x;
      mean[1] += pt.y;
      mean[2] += pt.z;
      
      cov(0,0) += pt.x*pt.x;
      
      cov(1,0) += pt.y*pt.x;
      cov(1,1) += pt.y*pt.y;
      
      cov(2,0) += pt.z*pt.x;
      cov(2,1) += pt.z*pt.y;
      cov(2,2) += pt.z*pt.z;	  
    }
	
    mean/= (double)k_correspondences_;
    // Get the actual covariance
    for(int k = 0; k < 3; k++) {
      for(int l = 0; l <= k; l++) {
        cov(k,l) /= (double)k_correspondences_;
        cov(k,l) -= mean[k]*mean[l];
        cov(l,k) = cov(k,l);
      }
    }
    
    // Compute the SVD (covariance matrix is symmetric so U = V')
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU);
    cov.setZero ();
    Eigen::Matrix3d U = svd.matrixU ();
    // Reconstitute the covariance matrix with modified singular values using the column     // vectors in V.
    for(int k = 0; k < 3; k++) {
      Eigen::Vector3d col = U.col(k);
      double v = 1.; // biggest 2 singular values replaced by 1
      if(k == 2)   // smallest singular value replaced by gicp_epsilon
        v = gicp_epsilon_;
      cov+= v * col * col.transpose(); 
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeRDerivative(const double x[], const Eigen::Matrix3d &R, double g[]) 
{
  Eigen::Matrix3d dR_dPhi;
  Eigen::Matrix3d dR_dTheta;
  Eigen::Matrix3d dR_dPsi;

  // Take phi, theta and psi from x vector according to the algorithm in
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  double q0 = 0;
  // double q1 = x[3];
  // double q2 = x[4];
  // double q3 = x[5];
  // double phi = atan2 (2*(q0*q1 + q2*q3),1-(q1*q1 + q2*q2));
  // double theta = asin (2*(q0*q2 - q3*q1));
  // double psi = atan2 (2*(q0*q3 + q1*q2),1-(q2*q2 + q3*q3));

  double phi = atan2 (2*(q0*x[3] + x[4]*x[5]),1-(x[3]*x[3] + x[4]*x[4]));
  double theta = asin (2*(q0*x[4] - x[5]*x[3]));
  double psi = atan2 (2*(q0*x[5] + x[3]*x[4]),1-(x[4]*x[4] + x[5]*x[5]));
  
  double cphi = cos(phi), sphi = sin(phi);
  double ctheta = cos(theta), stheta = sin(theta);
  double cpsi = cos(psi), spsi = sin(psi);
      
  dR_dPhi(0,0) = 0.;
  dR_dPhi(1,0) = 0.;
  dR_dPhi(2,0) = 0.;

  dR_dPhi(0,1) = sphi*spsi + cphi*cpsi*stheta;
  dR_dPhi(1,1) = -cpsi*sphi + cphi*spsi*stheta;
  dR_dPhi(2,1) = cphi*ctheta;

  dR_dPhi(0,2) = cphi*spsi - cpsi*sphi*stheta;
  dR_dPhi(1,2) = -cphi*cpsi - sphi*spsi*stheta;
  dR_dPhi(2,2) = -ctheta*sphi;

  dR_dTheta(0,0) = -cpsi*stheta;
  dR_dTheta(1,0) = -spsi*stheta;
  dR_dTheta(2,0) = -ctheta;

  dR_dTheta(0,1) = cpsi*ctheta*sphi;
  dR_dTheta(1,1) = ctheta*sphi*spsi;
  dR_dTheta(2,1) = -sphi*stheta;

  dR_dTheta(0,2) = cphi*cpsi*ctheta;
  dR_dTheta(1,2) = cphi*ctheta*spsi;
  dR_dTheta(2,2) = -cphi*stheta;

  dR_dPsi(0,0) = -ctheta*spsi;
  dR_dPsi(1,0) = cpsi*ctheta;
  dR_dPsi(2,0) = 0.;

  dR_dPsi(0,1) = -cphi*cpsi - sphi*spsi*stheta;
  dR_dPsi(1,1) = -cphi*spsi + cpsi*sphi*stheta;
  dR_dPsi(2,1) = 0.;

  dR_dPsi(0,2) = cpsi*sphi - cphi*spsi*stheta;
  dR_dPsi(1,2) = sphi*spsi + cphi*cpsi*stheta;
  dR_dPsi(2,2) = 0.;
      
  // set d/d_rx = tr(dR_dPhi'*gsl_temp_mat_r) [= <dR_dPhi, gsl_temp_mat_r>]
  g[3] = matricesInnerProd(dR_dPhi, R);
  // set d/d_ry = tr(dR_dTheta'*gsl_temp_mat_r) = [<dR_dTheta, gsl_temp_mat_r>]
  g[4] = matricesInnerProd(dR_dTheta, R);
  // set d/d_rz = tr(dR_dPsi'*gsl_temp_mat_r) = [<dR_dPsi, gsl_temp_mat_r>]
  g[5] = matricesInnerProd(dR_dPsi, R);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::estimateRigidTransformationLM (
      const PointCloudSource &cloud_src, const PointCloudTarget &cloud_tgt, Eigen::Matrix4f &transformation_matrix)
{
  boost::mutex::scoped_lock lock (tmp_mutex_);

  static const int n_unknowns = 6;      // 6 unknowns: 3 translation + 3 rotation (quaternion)

  if (cloud_src.size () != cloud_tgt.size ())
  {
    PCL_ERROR ("[pcl::GeneralizedIterativeClosestPoint::estimateRigidTransformationLM] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)cloud_src.size (), (unsigned long)cloud_tgt.size ());
    return;
  }
  if (cloud_src.size () < 4)     // need at least 4 samples
  {
    PCL_ERROR ("[pcl::GeneralizedIterativeClosestPoint::estimateRigidTransformationLM] Need at least 4 points to estimate a transform! Source and target have %lu points!\n", (unsigned long)cloud_src.size ());
    return;
  }

  int m = cloud_src.size ();
  double *fvec = new double[m];
  double *fjac = new double[m*n_unknowns];
  int *iwa = new int[n_unknowns];
  int lwa = m * n_unknowns + 5 * n_unknowns + m;
  double *wa = new double[lwa];
  int ldfjac = m;

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
  int info = lmder1 (&pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::functionToOptimize, this, m, n_unknowns, x, fvec, fjac, ldfjac, tol, iwa, wa, lwa);

  // Compute the norm of the residuals
  PCL_DEBUG ("[pcl::%s::estimateRigidTransformationLM] LM solver finished with exit code %i, having a residual norm of %g. \n",
             //"\nFinal solution: [%f %f %f %f] [%f %f %f]", 
             getClassName ().c_str (), info, enorm (m, fvec));
             //x[0], x[1], x[2], x[3], x[4], x[5], x[6]);

  delete [] wa; delete [] fvec; delete [] fjac;
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
  pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::estimateRigidTransformationLM (
      const PointCloudSource &cloud_src, const std::vector<int> &indices_src, const PointCloudTarget &cloud_tgt, const std::vector<int> &indices_tgt, Eigen::Matrix4f &transformation_matrix)
{
  boost::mutex::scoped_lock lock (tmp_mutex_);

  static const int n_unknowns = 6;      // 6 unknowns:  3 translation + 3 rotation (quaternion)

  if (indices_src.size () != indices_tgt.size ())
  {
    PCL_ERROR ("[pcl::GeneralizedIterativeClosestPoint::estimateRigidTransformationLM] Number or points in source (%lu) differs than target (%lu)!\n", (unsigned long)indices_src.size (), (unsigned long)indices_tgt.size ());
    return;
  }
  if (indices_src.size () < 4)     // need at least 4 samples
  {
    PCL_ERROR ("[pcl::GeneralizedIterativeClosestPoint::estimateRigidTransformationLM] Need at least 4 points to estimate a transform! Source and target have %lu points!", (unsigned long)indices_src.size ());
    return;
  }

  int m = indices_src.size ();
  double* fvec = new double[m];
  double* fjac = new double[m * n_unknowns];
  int *iwa = new int[n_unknowns];
  int lwa = m * n_unknowns + 5 * n_unknowns + m;
  double *wa = new double[lwa];
  int ldfjac = m;
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
  int info = lmder1 (&pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::functionToOptimizeIndices, this, m, n_unknowns, x, fvec, fjac, ldfjac, tol, iwa, wa, lwa);

  // Compute the norm of the residuals
  PCL_DEBUG ("[pcl::%s::estimateRigidTransformationLM] LM solver finished with exit code %i, having a residual norm of %g. \n",
             //"\nFinal solution: [%f %f %f %f] [%f %f %f]", 
             getClassName ().c_str (), info, enorm (m, fvec));
             //x[0], x[1], x[2], x[3], x[4], x[5], x[6]);

  delete [] wa; delete [] fvec; delete [] fjac;
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
pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::functionToOptimize (void *p, int m, int n, const double *x, double *fvec, double* fjac, int ldfjac, int iflag)
{
  GeneralizedIterativeClosestPoint *model = (GeneralizedIterativeClosestPoint*)p;

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
  transformation_matrix = transformation_matrix * model->base_transformation_;
  // If iflag == 1 compute fvec at x
  if(iflag == 1)
  {
    for (int i = 0; i < m; i++)
    {
      // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
      Vector4fMapConst p_src = model->tmp_src_->points[i].getVector4fMap ();
      // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
      Vector4fMapConst p_tgt = model->tmp_tgt_->points[i].getVector4fMap ();
      
      Eigen::Vector4f pp = transformation_matrix * p_src;
      // The last coordiante is still guaranteed to be set to 1.0
      Eigen::Vector3d res(pp[0] - p_tgt[0], pp[1] - p_tgt[1], pp[2] - p_tgt[2]);
      Eigen::Vector3d temp = model->mahalanobis(i) * res;
      // Cost function
      // Originally this is divided by m but as we are using LM it is discarded
      fvec[i] = double(res.transpose() * temp);
    }
  }
  else {
    // If iflag == 2 compute fjac at x
    if(iflag == 2)
    {
      for (int i = 0; i < m; i++)
      {
        // Map the jacobian translation component
        Eigen::Map<Eigen::Vector3d> g_t(&fjac[i*n]);
        // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
        Vector4fMapConst p_src = model->tmp_src_->points[i].getVector4fMap ();
        // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
        Vector4fMapConst p_tgt = model->tmp_tgt_->points[i].getVector4fMap ();
        
        Eigen::Vector4f pp = transformation_matrix * p_src;
        // The last coordiante is still guaranteed to be set to 1.0
        Eigen::Vector3d res(pp[0] - p_tgt[0], pp[1] - p_tgt[1], pp[2] - p_tgt[2]);
        // temp = M*res
        Eigen::Vector3d temp = model->mahalanobis(i) * res;
        // temp_double = res'*temp = temp'*M*res
        //double temp_double = res.transpose() * temp;
        // Increment translation gradient
        // orignally g_t+= 2*M*res/m
        g_t= 2 * model->mahalanobis(i) * res;
        pp = model->base_transformation_ * p_src;
        Eigen::Vector3d p_src3(pp[0], pp[1], pp[2]);
        // R = 2/m*pt1*temp^T + R with R set to zeros originally i.e.
        // R = 2/m*pt1*temp^T, we discard the /m
        Eigen::Matrix3d Ri = 2.0 * (p_src3 * temp.transpose());
        model->computeRDerivative(x, Ri, &fjac[i*n]);
      }
    }
  }
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////(
template <typename PointSource, typename PointTarget> inline int
pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::functionToOptimizeIndices (void *p, int m, int n, const double *x, double *fvec, double *fjac, int ldfjac, int iflag)
{
  GeneralizedIterativeClosestPoint *model = (GeneralizedIterativeClosestPoint*)p;

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
  transformation_matrix = transformation_matrix * model->base_transformation_;

  if (iflag == 1)
  {
    for (int i = 0; i < m; ++i)
    {
      // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
      Vector4fMapConst p_src = model->tmp_src_->points[(*model->tmp_idx_src_)[i]].getVector4fMap ();
      // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
      Vector4fMapConst p_tgt = model->tmp_tgt_->points[(*model->tmp_idx_tgt_)[i]].getVector4fMap ();
      
      Eigen::Vector4f pp = transformation_matrix * p_src;
      // Estimate the distance (cost function)
      // The last coordiante is still guaranteed to be set to 1.0
      Eigen::Vector3d res(pp[0] - p_tgt[0], pp[1] - p_tgt[1], pp[2] - p_tgt[2]);
      Eigen::Vector3d temp = model->mahalanobis((*model->tmp_idx_src_)[i]) * res;
      fvec[i] = double(res.transpose() * temp) / m;
    }
  }
  else
  {
    if (iflag == 2)
    {
      for (int i = 0; i < m; ++i)
      {
        // Map the jacobian translation component
        Eigen::Map<Eigen::Vector3d> g_t(&fjac[i*n]);
        // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
        Vector4fMapConst p_src = model->tmp_src_->points[(*model->tmp_idx_src_)[i]].getVector4fMap ();
        // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
        Vector4fMapConst p_tgt = model->tmp_tgt_->points[(*model->tmp_idx_tgt_)[i]].getVector4fMap ();
      
        Eigen::Vector4f pp = transformation_matrix * p_src;
        // The last coordiante is still guaranteed to be set to 1.0
        Eigen::Vector3d res(pp[0] - p_tgt[0], pp[1] - p_tgt[1], pp[2] - p_tgt[2]);
        // temp = M*res
        Eigen::Vector3d temp = model->mahalanobis((*model->tmp_idx_src_)[i]) * res;
        // Increment translation gradient
        // g_t+= 2*M*res/num_matches
        g_t= 2 * model->mahalanobis((*model->tmp_idx_src_)[i]) * res / m;
        pp = model->base_transformation_ * p_src;
        Eigen::Vector3d p_src3 (pp[0], pp[1], pp[2]);
        Eigen::Matrix3d Ri = 2.0 * p_src3 * temp.transpose();
        model->computeRDerivative(x, Ri, &fjac[i*n]);
      }
    }
  }
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess)
{
  // Difference between consecutive transforms
  double delta = 0;
  // Get the size of the target
  const size_t N = indices_->size ();
  // Set the mahalanobis matrices to identity
  mahalanobis_.resize (N, Eigen::Matrix3d::Identity ());
  // Compute target cloud covariance matrices
  computeCovariances<PointTarget> (target_, tree_, target_covariances_);
  // Compute input cloud covariance matrices
  computeCovariances<PointSource> (input_, input_tree_, input_covariances_);
  base_transformation_ = guess;
  nr_iterations_ = 0;
  converged_ = false;
  double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;
  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);
  while(!converged_)
  {
    size_t cnt = 0;
    std::vector<int> source_indices (indices_->size ());
    std::vector<int> target_indices (indices_->size ());

    // guess corresponds to base_t and transformation_ to t
    // dgc_transform_t transform_R;
    // dgc_transform_copy(transform_R, base_t);
    // dgc_transform_left_multiply(transform_R, t);
    Eigen::Matrix4d transform_R;
    heteregenious_product(transformation_, guess, transform_R);
    Eigen::Matrix3d R = transform_R.topLeftCorner<3,3> ();
    for(size_t i = 0; i < N; i++)
    {
      PointSource query = output[i];
      query.getVector4fMap () = guess * query.getVector4fMap ();
      query.getVector4fMap () = transformation_ * query.getVector4fMap ();

      if (!searchForNeighbors (query, nn_indices, nn_dists))
      {
        PCL_ERROR ("[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the source!\n", getClassName ().c_str (), (*indices_)[i]);
        return;
      }

      // Check if the distance to the nearest neighbor is smaller than the user imposed threshold
      if (nn_dists[0] < dist_threshold)
      {
        Eigen::Matrix3d &C1 = input_covariances_[i];
        Eigen::Matrix3d &C2 = target_covariances_[i];
        Eigen::Matrix3d &M = mahalanobis_[i];
        // M = R*C1
        M = R * C1;
        // tmp = M*R' + C2 = R*C1*R' + C2
        Eigen::Matrix3d tmp = M * R.transpose() + C2;
        // M = temp^-1
        M = tmp.inverse();
        source_indices[cnt] = i;
        target_indices[cnt] = nn_indices[0];
        cnt++;
      }
    }
    // Resize to the actual number of valid correspondences
    source_indices.resize(cnt); target_indices.resize(cnt);
    /* optimize transformation using the current assignment and Mahalanobis metrics*/
    previous_transformation_ = transformation_;
    //optimization right here
    rigid_transformation_estimation_(output, source_indices, *target_, target_indices, transformation_);
    /* compute the delta from this iteration */
    delta = 0.;
    for(int k = 0; k < 4; k++) {
      for(int l = 0; l < 4; l++) {
        double ratio = 1;
        if(k < 3 && l < 3) // rotation part of the transform
          ratio = 1./rotation_epsilon_;
        else
          ratio = 1./transformation_epsilon_;
        double c_delta = ratio*fabs(previous_transformation_(k,l) - transformation_(k,l));
        if(c_delta > delta)
          delta = c_delta;
      }
    }

    nr_iterations_++;
    // Check for convergence
    if (nr_iterations_ >= max_iterations_ || delta < 1)
    {
      converged_ = true;
      PCL_DEBUG ("[pcl::%s::computeTransformation] Convergence reached. Number of iterations: %d out of %d. Transformation difference: %f\n",
                 getClassName ().c_str (), nr_iterations_, max_iterations_, fabs ((transformation_ - previous_transformation_).sum ()));
    }
  }
}
