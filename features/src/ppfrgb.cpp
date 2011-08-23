#include "pcl/features/ppfrgb.h"
#include "pcl/features/impl/ppfrgb.hpp"
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>

// Instantiations of specific point types
PCL_INSTANTIATE_PRODUCT(PPFRGBEstimation, ((pcl::PointXYZRGB) (pcl::PointXYZRGBNormal))
                        ((pcl::Normal) (pcl::PointNormal)  (pcl::PointXYZRGBNormal))
                        ((pcl::PPFRGBSignature)));

PCL_INSTANTIATE_PRODUCT(PPFRGBRegionEstimation, ((pcl::PointXYZRGB) (pcl::PointXYZRGBNormal))
                        ((pcl::Normal) (pcl::PointNormal)  (pcl::PointXYZRGBNormal))
                        ((pcl::PPFRGBSignature)));

