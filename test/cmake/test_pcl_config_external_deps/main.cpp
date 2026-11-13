#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>

int main()
{
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
    cloud.push_back(pcl::PointXYZ(1.0f, 0.0f, 0.0f));
    cloud.push_back(pcl::PointXYZ(0.0f, 1.0f, 0.0f));

    pcl::PointCloud<pcl::PointXYZ> output;
    mls.setInputCloud(cloud.makeShared());
    mls.process(output);
    return 0;
}
