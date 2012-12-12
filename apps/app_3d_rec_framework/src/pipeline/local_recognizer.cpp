#include <pcl/apps/3d_rec_framework/pipeline/impl/local_recognizer.hpp>

//This stuff is needed to be able to make the SHOT histograms persistent
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<352>,
    (float[352], histogram, histogram352)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<1344>,
    (float[1344], histogram, histogram1344)
)

template class PCL_EXPORTS pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZ, pcl::Histogram<352> >;
template class PCL_EXPORTS pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZRGB, pcl::Histogram<352> >;
template class PCL_EXPORTS pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZRGB, pcl::Histogram<1344> >;
template class PCL_EXPORTS pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZ, pcl::FPFHSignature33>;
template class PCL_EXPORTS pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZRGB, pcl::FPFHSignature33>;

template class PCL_EXPORTS pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L2, pcl::PointXYZ, pcl::Histogram<352> >;
template class PCL_EXPORTS pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L2, pcl::PointXYZRGB, pcl::Histogram<352> >;
template class PCL_EXPORTS pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L2, pcl::PointXYZRGB, pcl::Histogram<1344> >;
template class PCL_EXPORTS pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L2, pcl::PointXYZ, pcl::FPFHSignature33>;
template class PCL_EXPORTS pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L2, pcl::PointXYZRGB, pcl::FPFHSignature33>;
