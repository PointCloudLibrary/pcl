#ifndef DATABASE_ENTRY_
#define DATABASE_ENTRY_

namespace pcl
{

  namespace proctor
  {

    typedef Eigen::MatrixXf Signature;

    struct Entry {
      std::string id;

      PointCloud<PointNormal>::Ptr cloud;
      PointCloud<PointNormal>::Ptr keypoints;
      PointCloud<Signature>::Ptr features;
      KdTreeFLANN<Signature>::Ptr tree;
    };

  }

}

#endif
