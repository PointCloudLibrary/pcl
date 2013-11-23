#ifndef _COMMON_TYPEDEFS_H_
#define _COMMON_TYPEDEFS_H_

// pcl headers
#include "pcl_includes.h"

typedef pcl::PointXYZRGB			PointT;
typedef pcl::PointCloud<PointT>			pc;
typedef pcl::PointCloud<PointT>::Ptr		pcPtr;
typedef pcl::PointCloud<PointT>::ConstPtr	pcConstPtr;

typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> SurfaceNormals;
typedef pcl::PointCloud<NormalT>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalT>::ConstPtr SurfaceNormalsConstPtr;

typedef pcl::SHOT352		         shotDescriptorT;
typedef pcl::PointCloud<shotDescriptorT> shotDescriptors;
typedef pcl::PointCloud<shotDescriptorT>::Ptr shotDescriptorsPtr;
typedef pcl::PointCloud<shotDescriptorT>::ConstPtr shotDescriptorsConstPtr;

typedef pcl::SHOT1344		          cshotDescriptorT;
typedef pcl::PointCloud<cshotDescriptorT> cshotDescriptors;
typedef pcl::PointCloud<cshotDescriptorT>::Ptr cshotDescriptorsPtr;
typedef pcl::PointCloud<cshotDescriptorT>::ConstPtr cshotDescriptorsConstPtr;

typedef pcl::FPFHSignature33		 fpfhDescriptorT;
typedef pcl::PointCloud<fpfhDescriptorT> fpfhDescriptors;
typedef pcl::PointCloud<fpfhDescriptorT>::Ptr fpfhDescriptorsPtr;
typedef pcl::PointCloud<fpfhDescriptorT>::ConstPtr fpfhDescriptorsConstPtr;

#endif
