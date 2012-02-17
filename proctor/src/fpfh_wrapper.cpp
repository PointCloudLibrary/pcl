#include "proctor/fpfh_wrapper.h"

#include <pcl/features/fpfh.h>

namespace pcl
{

  namespace proctor
  {

    void
    FPFHWrapper::compute(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints, pcl::PointCloud<Eigen::MatrixXf> &output)
    {
      FPFHEstimation<PointNormal, PointNormal, Eigen::MatrixXf> fpfh;
      fpfh.setRadiusSearch(0.05);
      //fpfh.setKSearch(10);
      fpfh.setInputCloud(keypoints);
      search::KdTree<PointNormal>::Ptr kdt (new search::KdTree<PointNormal>());
      fpfh.setSearchMethod(kdt);

      fpfh.setSearchSurface(cloud);
      fpfh.setInputNormals(cloud);

      //fpfh.setInputNormals(keypoints);

      fpfh.computeEigen(output);
    }

  }

}

