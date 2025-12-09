/*
 * global_nn_classifier.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#include <pcl/apps/3d_rec_framework/pipeline/impl/global_nn_classifier.hpp>
#include <pcl/apps/3d_rec_framework/utils/metrics.h>

// Instantiation
// GlobalClassifier is the parent class of GlobalNNPipeline. They must be instantiated
// in this order, otherwise visibility attributes of the former are not applied
// correctly.
template class PCL_EXPORTS pcl::rec_3d_framework::GlobalClassifier<pcl::PointXYZ>;

template class PCL_EXPORTS pcl::rec_3d_framework::
    GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::VFHSignature308>;
template class PCL_EXPORTS
    pcl::rec_3d_framework::GlobalNNPipeline<Metrics::HistIntersectionUnionDistance,
                                            pcl::PointXYZ,
                                            pcl::VFHSignature308>;
template class PCL_EXPORTS pcl::rec_3d_framework::
    GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::ESFSignature640>;
