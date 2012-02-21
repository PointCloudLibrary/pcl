#include "proctor/pfh_wrapper.h"

#include <pcl/features/pfh.h>

namespace pcl
{

  namespace proctor
  {

    void
    PFHWrapper::compute(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints, pcl::PointCloud<Eigen::MatrixXf> &output)
    {
      PFHEstimation<PointNormal, PointNormal, Eigen::MatrixXf> pfh;
      pfh.setRadiusSearch(0.05);
      //pfh.setKSearch(10);
      pfh.setInputCloud(keypoints);
      search::KdTree<PointNormal>::Ptr kdt (new search::KdTree<PointNormal>());
      pfh.setSearchMethod(kdt);

      pfh.setSearchSurface(cloud);
      pfh.setInputNormals(cloud);

      //pfh.setInputNormals(keypoints);

      pfh.computeEigen(output);
    }

  }

}

