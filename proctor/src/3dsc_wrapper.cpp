#include "proctor/3dsc_wrapper.h"

#include <pcl/features/3dsc.h>

namespace pcl
{

  namespace proctor
  {
    
    void
    ShapeContextWrapper::compute(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints, pcl::PointCloud<Eigen::MatrixXf> &output)
    {
      ShapeContext3DEstimation<PointNormal, PointNormal, Eigen::MatrixXf> est;
      est.setAzimuthBins (5);
      est.setElevationBins (5);
      est.setRadiusBins (5);
      est.setMinimalRadius (0.004);
      est.setPointDensityRadius (0.008);
      est.setRadiusSearch(0.03);
      est.setInputCloud(keypoints);
      search::KdTree<PointNormal>::Ptr kdt (new search::KdTree<PointNormal>());
      est.setSearchMethod(kdt);

      est.setSearchSurface(cloud);
      est.setInputNormals(cloud);

      est.computeEigen(output);
    }

  }

}

