#include <pcl/gpu/octree/impl/internal.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

#include "bfrs.cu"
// Defined for all CORE_POINT_TYPES
template class pcl::device::OctreeImpl<pcl::PointXYZRGB>;
template class pcl::device::OctreeImpl<pcl::PointXYZ>;
template class pcl::device::OctreeImpl<pcl::PointXYZI>;
template class pcl::device::OctreeImpl<pcl::PointXYZRGBL>;
template class pcl::device::OctreeImpl<pcl::PointNormal>;
template void
pcl::device::bruteForceRadiusSearch<pcl::PointXYZRGB>(
    const typename OctreeImpl<pcl::PointXYZRGB>::PointCloud& cloud,
    const typename OctreeImpl<pcl::PointXYZRGB>::PointType& query,
    float radius,
    DeviceArray<int>& result,
    DeviceArray<int>& buffer);
template void
pcl::device::bruteForceRadiusSearch<pcl::PointXYZ>(
    const typename OctreeImpl<pcl::PointXYZ>::PointCloud& cloud,
    const typename OctreeImpl<pcl::PointXYZ>::PointType& query,
    float radius,
    DeviceArray<int>& result,
    DeviceArray<int>& buffer);
template void
pcl::device::bruteForceRadiusSearch<pcl::PointXYZI>(
    const typename OctreeImpl<pcl::PointXYZI>::PointCloud& cloud,
    const typename OctreeImpl<pcl::PointXYZI>::PointType& query,
    float radius,
    DeviceArray<int>& result,
    DeviceArray<int>& buffer);
template void
pcl::device::bruteForceRadiusSearch<pcl::PointXYZRGBL>(
    const typename OctreeImpl<pcl::PointXYZRGBL>::PointCloud& cloud,
    const typename OctreeImpl<pcl::PointXYZRGBL>::PointType& query,
    float radius,
    DeviceArray<int>& result,
    DeviceArray<int>& buffer);
template void
pcl::device::bruteForceRadiusSearch<pcl::PointNormal>(
    const typename OctreeImpl<pcl::PointNormal>::PointCloud& cloud,
    const typename OctreeImpl<pcl::PointNormal>::PointType& query,
    float radius,
    DeviceArray<int>& result,
    DeviceArray<int>& buffer);
