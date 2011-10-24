#ifndef SCANNER_H_
#define SCANNER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace pcl;

/** this class provides convenience methods for scanning meshes to point clouds */
class Scanner {
public:

  static const float distance_multiplier = 3;
  static const double fov_x = M_PI / 3;
  static const double fov_y = M_PI / 4;
  static const unsigned int res_x = 320;
  static const unsigned int res_y = 240;

  /** information about a simulated scan */
  typedef struct {
    int mi;
    float theta;
    float phi;
  } Scan;

  /** does one scan. testing should use this */
  static PointCloud<PointNormal>::Ptr getCloud(Scan scan);

  /** try to load scan from disk, or do it from scratch. for training only */
  static PointCloud<PointNormal>::Ptr getCloudCached(int mi, int ti, int pi);

};

#endif  //#ifndef SCANNER_H_
