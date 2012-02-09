#include "proctor/si_wrapper.h"

#include <pcl/features/spin_image.h>

namespace pcl
{

  namespace proctor
  {
    
    void
    SIWrapper::compute(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints, pcl::PointCloud<Eigen::MatrixXf> &output)
    {
      SpinImageEstimation<PointNormal, PointNormal, Eigen::MatrixXf> si (8, 0.5, 10);
      si.setRadiusSearch(0.08);
      search::KdTree<PointNormal>::Ptr kdt (new search::KdTree<PointNormal>());
      si.setSearchMethod(kdt);

      si.setInputCloud(keypoints);
      //si.setSearchSurface(cloud);
      //si.setSearchSurfaceNormals(cloud);
      //si.setInputNormals(keypoints);
      //si.setSearchSurfaceWithNormals(cloud, cloud);
      //si.setInputWithNormals(keypoints, keypoints);
      si.computeEigen(output);
    }

  }

}

