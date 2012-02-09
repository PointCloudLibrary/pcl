#ifndef SCANNER_H_
#define SCANNER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace pcl
{

  namespace proctor
  {

    struct Model {
      std::string id;
      vtkAlgorithmOutput* mesh;
      float cx, cy, cz;
      float scale;
    };

    /** this class provides convenience methods for scanning meshes to point clouds */
    class Scanner {
      public:
        static const float distance_multiplier;
        static const double fov_x;
        static const double fov_y;
        static const unsigned int res_x;
        static const unsigned int res_y;

        // parameters for test scans
        static const float theta_start;
        static const float theta_step;
        static const float theta_min;
        static const float theta_max;
        static const int theta_count;

        static const float phi_start;
        static const float phi_step;
        static const float phi_min;
        static const float phi_max;
        static const int phi_count;

        /** information about a simulated scan */
        struct Scan {
          float theta;
          float phi;
        };

        /** does one scan. testing should use this */
        static PointCloud<PointNormal>::Ptr
        getCloud(Scan scan, Model &model);

        /** try to load scan from disk, or do it from scratch. for training only */
        // theta and phi are in radians
        static PointCloud<PointNormal>::Ptr
        getCloudCached(float theta, float phi, Model &model);
    };
  }
}

#endif  //#ifndef SCANNER_H_
