#include "proctor/si_wrapper.h"

#include <pcl/features/spin_image.h>

namespace pcl
{

  namespace proctor
  {
    
    void
    SIWrapper::compute(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints, pcl::PointCloud<Eigen::MatrixXf> &output)
    {
      SpinImageEstimation<PointNormal, PointNormal, Eigen::MatrixXf> si (8, 0.5, 0);
      si.setRadiusSearch(0.02);
      si.setInputCloud(keypoints);
      search::KdTree<PointNormal>::Ptr kdt (new search::KdTree<PointNormal>());
      si.setSearchMethod(kdt);
      si.setSearchSurface(cloud);
      si.setInputNormals(cloud);
      si.computeEigen(output);
    }

  }

}

