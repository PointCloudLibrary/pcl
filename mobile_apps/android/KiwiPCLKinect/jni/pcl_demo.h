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
void CreatePCLFileFromKinect(short * buf_int, int frame_width, int frame_height, int maxZ);
void CreatePointCloud(short * buf_int, int frame_width, int frame_height, int maxZ); 
