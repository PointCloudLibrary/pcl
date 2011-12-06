#ifndef SCANNER_H_
#define SCANNER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace pcl {
  namespace proctor {
    struct Model;

    /** this class provides convenience methods for scanning meshes to point clouds */
    class Scanner {
      public:
        static const float distance_multiplier;
        static const double fov_x;
        static const double fov_y;
        static const unsigned int res_x;
        static const unsigned int res_y;

        /** information about a simulated scan */
        typedef struct {
          int mi;
          float theta;
          float phi;
        } Scan;

        /** does one scan. testing should use this */
        static PointCloud<PointNormal>::Ptr getCloud(Scan scan, Model &model);

        /** try to load scan from disk, or do it from scratch. for training only */
        static PointCloud<PointNormal>::Ptr getCloudCached(int mi, int ti, int pi, Model &model);
    };
  }
}

#endif  //#ifndef SCANNER_H_
