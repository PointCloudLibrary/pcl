#include <Eigen/Eigenvalues>
#include "pcl/point_types.h"
#include "pcl/common/centroid.h"
#include <pcl/console/print.h>

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> inline void 
pcl::PCA<PointT>::compute (const pcl::PointCloud<PointT>& cloud) 
{
  // Compute mean and covariance
  mean_ = Eigen::Vector4f::Zero ();
  compute3DCentroid (cloud, mean_);
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized (cloud, mean_, covariance);
  
  // Compute demeanished cloud
  Eigen::MatrixXf cloud_demean;
  demeanPointCloud (cloud, mean_, cloud_demean);

  // Compute eigen vectors and values
  Eigen::EigenSolver<Eigen::Matrix3f> evd (covariance, true);
  eigenvalues_ = evd.eigenvalues ().real ();
  eigenvectors_ = evd.eigenvectors ().real ();
  if (!basis_only_)
    coefficients_ = eigenvectors_.transpose() * cloud_demean.topRows<3>();
  compute_done_ = true;
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> inline void 
pcl::PCA<PointT>::update (const PointT& input_point, FLAG flag) 
{
  if (!compute_done_)
    PCL_ERROR ("[pcl::PCA::update] PCA computing still not done.\n");

  Eigen::Vector3f input (input_point.x, input_point.y, input_point.z);
  const size_t n = eigenvectors_.cols ();// number of eigen vectors
  Eigen::VectorXf meanp = (float(n) * (mean_.head<3>() + input)) / float(n + 1);
  Eigen::VectorXf a = eigenvectors_.transpose() * (input - mean_.head<3>());
  Eigen::VectorXf y = (eigenvectors_ * a) + mean_.head<3>();
  Eigen::VectorXf h = y - input;
  if (h.norm() > 0) 
    h.normalize ();
  else
    h.setZero ();
  float gamma = h.dot(input - mean_.head<3>());
  Eigen::MatrixXf D = Eigen::MatrixXf::Zero (a.size() + 1, a.size() + 1);
  D.block(0,0,n,n) = a * a.transpose();
  D /=  float(n)/float((n+1) * (n+1));
  for(std::size_t i=0; i < a.size(); i++) {
    D(i,i)+= float(n)/float(n+1)*eigenvalues_(i);
    D(D.rows()-1,i) = float(n) / float((n+1) * (n+1)) * gamma * a(i);
    D(i,D.cols()-1) = D(D.rows()-1,i);
    D(D.rows()-1,D.cols()-1) = float(n)/float((n+1) * (n+1)) * gamma * gamma;
  }

  Eigen::MatrixXf R(D.rows(), D.cols());
  Eigen::EigenSolver<Eigen::MatrixXf> D_evd (D, false);
  Eigen::VectorXf alphap = D_evd.eigenvalues().real();
  eigenvalues_.resize(eigenvalues_.size() +1);
  for(std::size_t i=0;i<eigenvalues_.size();i++) {
    eigenvalues_(i) = alphap(eigenvalues_.size()-i-1);
    R.col(i) = D.col(D.cols()-i-1);
  }
  Eigen::MatrixXf Up = Eigen::MatrixXf::Zero(eigenvectors_.rows(), eigenvectors_.cols()+1);
  Up.topLeftCorner(eigenvectors_.rows(),eigenvectors_.cols()) = eigenvectors_;
  Up.rightCols<1>() = h;
  eigenvectors_ = Up*R;
  if (!basis_only_) {
    Eigen::Vector3f etha = Up.transpose() * (mean_.head<3>() - meanp);
    coefficients_.resize(coefficients_.rows()+1,coefficients_.cols()+1);
    for(std::size_t i=0; i < (coefficients_.cols() - 1); i++) {
      coefficients_(coefficients_.rows()-1,i) = 0;
      coefficients_.col(i) = (R.transpose() * coefficients_.col(i)) + etha;
    }
    a.resize(a.size()+1);
    a(a.size()-1) = 0;
    coefficients_.col(coefficients_.cols()-1) = (R.transpose() * a) + etha;
  }
  mean_.head<3>() = meanp;
  switch (flag) 
  {
    case increase:
      if (eigenvectors_.rows() >= eigenvectors_.cols())
        break;
    case preserve:
      if (!basis_only_)
        coefficients_ = coefficients_.topRows(coefficients_.rows() - 1);
      eigenvectors_ = eigenvectors_.leftCols(eigenvectors_.cols() - 1);
      eigenvalues_.resize(eigenvalues_.size()-1);
      break;
    default:
      PCL_ERROR("[pcl::PCA] unknown flag\n");
  }
}

template<typename PointT>
inline void pcl::PCA<PointT>::project(const PointT& input, PointT& projection) const 
{
  if(!compute_done_)
    PCL_ERROR("[pcl::PCA::project] PCA computing still not done\n");
  Eigen::Vector3f demean_input = input.getVector3fMap () - mean_.head<3>();
  projection.getVector3fMap () = eigenvectors_.transpose() * demean_input;
}

template<typename PointT>
inline void pcl::PCA<PointT>::reconstruct(const PointT& projection, PointT& input) const 
{
  if(!compute_done_)
    PCL_ERROR("[pcl::PCA::reconstruct] PCA computing still not done\n");
  input.getVector3fMap () = (eigenvectors_ * projection.getVector3fMap ()) + mean_.head<3>();
}

