#include <jni.h>
#include <sys/types.h>
#include <android/log.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>


void loadPCLPointCloud();
void simplePclRegistration();
void ICP_PCL();
void ICP();
