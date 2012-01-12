#ifndef DATABASE_ENTRY_
#define DATABASE_ENTRY_

namespace pcl
{

  namespace proctor
  {

    typedef FPFHSignature33 Signature;

    struct Entry {
      PointCloud<PointNormal>::Ptr cloud;
      IndicesPtr indices;
      PointCloud<Signature>::Ptr features;
      KdTree<Signature>::Ptr tree;
    };

  }

}

#endif
