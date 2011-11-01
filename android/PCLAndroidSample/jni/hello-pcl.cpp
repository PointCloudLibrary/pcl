#include "hello-pcl.h"

#include <cstdio>
#include <android/log.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <jni.h>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "hello-ndk", __VA_ARGS__))


using namespace boost::filesystem;

/* This is a trivial JNI example where we use a native method
 * to return a new VM String. See the corresponding Java source
 * file located at:
 *
 *   apps/samples/hello-jni/project/src/com/example/HelloJni/HelloJni.java
 */
JNIEXPORT jstring JNICALL Java_org_pointclouds_PCLAndroidSample_HelloPCL_boostMkDir
(JNIEnv * env, jobject)
{
	std::string hello( "hello world!" );
	
	sregex rex = sregex::compile( "(\\w+) (\\w+)!" );
	smatch what;
	
	if( regex_match( hello, what, rex ) )
	{
		std::cout << what[0] << '\n'; // whole match
		std::cout << what[1] << '\n'; // first capture
		std::cout << what[2] << '\n'; // second capture
	}
	
	
	printf("Hello from NDK using PCL\n");
	LOGI("Hello from NDK using PCL");
	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data
  cloud->width  = 50;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
	
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);		
  }
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	for (int i = 0; i < 100; ++i)
	{
		std::vector<int> indices;
		std::vector<float> distances;
		kdtree.nearestKSearch (0, 3, indices, distances);
		printf ("indices size: %d\n", indices.size ());
		LOGI ("indices size %d\n", indices.size ());
	}
	
	LOGI ("Doing stuff with PCL on Android");
	
	
  pcl::io::savePCDFileASCII ("/mnt/sdcard/KiwiViewer/output.pcd", *cloud);
	
	
	std::ofstream file ("/mnt/sdcard/KiwiViewer/caca.caca");
	file << "caca caca caca\n";
	file.close ();
	
	printf ("Doing stuff with PCL on Android; cloud size: %d %d\n", cloud->width, cloud->height);
	
	
	
	return env->NewStringUTF("caca masii de pe noul PCL Android care merge bine");
}


JNIEXPORT jstring JNICALL Java_org_pointclouds_PCLAndroidSample_HelloPCL_smoothPointCloud
(JNIEnv * env, jobject)
{
	
}