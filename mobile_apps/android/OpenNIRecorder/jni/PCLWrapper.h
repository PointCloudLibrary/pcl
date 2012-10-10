#ifndef __INCLUDED_PCL_WRAPPER_H
#define __INCLUDED_PCL_WRAPPER_H


#include <math.h>
//android logging only
#include <android/log.h>
#include <iostream>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include "common.h"

//NEON hardware accelerated libraries
#include "NE10_neon.h"
#include "NE10_c.h"
#include "NE10_types.h"
#include "NE10.h"
#include "NE10_asm.h"
#include "NE10_init.h"



//png library
extern "C" {
	#include "test_png.h"

}
//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
using namespace std;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};
class PCLWrapper {
public:
	PCLWrapper();
	~PCLWrapper();
	void init(int x, int y);
	int filter(unsigned short *depth_data, float *point_cloud_data);
	int voxel_filter(unsigned short *depth_data, float *point_cloud_data);
	void setData(unsigned short *dept_map);
	void benchmark(int iterations, int average);
	void benchmark_png();
	void benchmark_raw();
	int test_registration();
private:
//	//memory for the PCL pointcloud data
	pcl::PointCloud<pcl::PointXYZ>::Ptr input;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered;
	pcl::PointCloud<pcl::PointXYZ>::Ptr bigCloud;
};
#endif

