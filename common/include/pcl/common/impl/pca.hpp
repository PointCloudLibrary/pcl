#include <Eigen/Eigenvalues>
#include "pcl/point_types.h"
#include "pcl/common/centroid.h"

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
  if (!compute_done ())
    PCL_ERROR ("[pcl::PCA::update] PCA computing still not done.");

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
    D(i,i)+= float(n)/float(n+1)*eigenvalues(i);
    D(D.rows()-1,i) = float(n) / float((n+1) * (n+1)) * gamma * a(i);
    D(i,D.cols()-1) = D(D.rows()-1,i);
    D(D.rows()-1,D.cols()-1) = float(n)/float((n+1) * (n+1)) * gamma * gamma;
  }

  Eigen::MatrixXf R(D.rows(), D.cols());
  Eigen::EigenSolver<Eigen::MatrixXf> D_evd (D, false);
  Eigen::VectorXf alphap = D_evd.eigenvalues().real();
  eigenvalues.resize(eigenvalues.size() +1);
  for(std::size_t i=0;i<eigenvalues.size();i++) {
    eigenvalues(i) = alphap(eigenvalues.size()-i-1);
    R.col(i) = D.col(D.cols()-i-1);
  }
  Eigen::MatrixXf Up = Eigen::MatrixXf::Zero(eigenvectors.rows(), eigenvectors.cols()+1);
  Up.topLeftCorner(eigenvectors.rows(),eigenvectors.cols()) = eigenvectors;
  Up.rightCols<1>() = h;
  eigenvectors = Up*R;
  if (!basis_only) {
    Eigen::Vector3f etha = Up.transpose() * (mean.head<3>() - meanp);
    coefficients.resize(coefficients.rows()+1,coefficients.cols()+1);
    for(std::size_t i=0; i < (coefficients.cols() - 1); i++) {
      coefficients(coefficients.rows()-1,i) = 0;
      coefficients.col(i) = (R.transpose() * coefficients.col(i)) + etha;
    }
    a.resize(a.size()+1);
    a(a.size()-1) = 0;
    coefficients.col(coefficients.cols()-1) = (R.transpose() * a) + etha;
  }
  mean.head<3>() = meanp;
  switch (flag) 
  {
    case increase:
      if (eigenvectors.rows() >= eigenvectors.cols())
        break;
    case preserve:
      if (!basis_only)
        coefficients = coefficients.topRows(coefficients.rows() - 1);
      eigenvectors = eigenvectors.leftCols(eigenvectors.cols() - 1);
      eigenvalues.resize(eigenvalues.size()-1);
      break;
    default:
      PCL_ERROR("[pcl::PCA] unknown flag");
  }
}

template<typename PointT>
inline void pcl::PCA<PointT>::project(const PointT& input, PointT& projection) const 
{
  if(!compute_done)
    PCL_ERROR("[pcl::PCA::project] PCA computing still not done");
  Eigen::Vector3f demean_input = input.getVector3fMap () - mean.head<3>();
  projection.getVector3fMap () = eigenvectors.transpose() * demean_input;
}

template<typename PointT>
inline void pcl::PCA<PointT>::reconstruct(const PointT& projection, PointT& input) const 
{
  if(!compute_done)
    PCL_ERROR("[pcl::PCA::reconstruct] PCA computing still not done");
  input.getVector3fMap () = (eigenvectors * projection.getVector3fMap ()) + mean.head<3>();
}

