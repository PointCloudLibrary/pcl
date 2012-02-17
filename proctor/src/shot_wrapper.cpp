#include "proctor/shot_wrapper.h"

#include <pcl/features/shot.h>
#include <pcl/common/common.h>

namespace pcl
{

  namespace proctor
  {
    
    void
    SHOTWrapper::compute(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints, pcl::PointCloud<Eigen::MatrixXf> &output)
    {
      // TODO Investigate by it used to be 12
      //Eigen::Vector4f min, max;
      //getMinMax3D(*cloud, min, max);
      //std::cout << "Min: " << min << "\tMax: " << max << std::endl;
      SHOTEstimation<PointNormal, PointNormal, Eigen::MatrixXf> shot;
      shot.setRadiusSearch(0.05);
      shot.setInputCloud(keypoints);
      search::KdTree<PointNormal>::Ptr kdt (new search::KdTree<PointNormal>());
      shot.setSearchMethod(kdt);
      shot.setSearchSurface(cloud);
      shot.setInputNormals(cloud);
      shot.computeEigen(output);
    }

  }

}

