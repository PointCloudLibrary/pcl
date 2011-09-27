#include <cstdio>
#include <android/log.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "hello-ndk", __VA_ARGS__))

int main(void)
{
	printf("Hello from NDK using PCL\n");
	LOGI("Hello from NDK using PCL");
	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
	
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);		
  }
	
	LOGI ("Doing stuff with PCL on Android");
	
	
	printf ("Doing stuff with PCL on Android; cloud size: %d %d\n", cloud->width, cloud->height);

	pcl::io::savePCDFileASCII ("cloud.pcd", *cloud);
	return 0;
}