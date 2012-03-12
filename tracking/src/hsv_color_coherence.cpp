#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/impl/hsv_color_coherence.hpp>

PCL_INSTANTIATE_PRODUCT(HSVColorCoherence, ((pcl::PointXYZRGB)(pcl::PointXYZRGBNormal)(pcl::PointXYZRGBA)))
