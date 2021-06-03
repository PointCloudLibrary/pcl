#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/gpu/octree/impl/internal.hpp>
// Defined for all CORE_POINT_TYPES
template class pcl::device::OctreeImpl<pcl::PointXYZRGB>;
template class pcl::device::OctreeImpl<pcl::PointXYZ>;
template class pcl::device::OctreeImpl<pcl::PointXYZI>;
template class pcl::device::OctreeImpl<pcl::PointXYZRGBL>;
template class pcl::device::OctreeImpl<pcl::PointNormal>;
