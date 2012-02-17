#include "proctor/si_wrapper.h"

#include <pcl/features/spin_image.h>

namespace pcl
{

  namespace proctor
  {

    void
    SIWrapper::compute(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints, pcl::PointCloud<Eigen::MatrixXf> &output)
    {
      SpinImageEstimation<PointNormal, PointNormal, Eigen::MatrixXf> si;
      search::KdTree<PointNormal>::Ptr kdt (new search::KdTree<PointNormal>());
      si.setSearchMethod(kdt);
      si.setRadiusSearch(0.08);

      si.setInputCloud(keypoints);
      si.setInputNormals(keypoints);
      si.setSearchSurface(cloud);
      si.computeEigen(output);
    }

  }

}

