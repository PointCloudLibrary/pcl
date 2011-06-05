#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

int
 main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
  sensor_msgs::PointCloud2 cloud_blob;
  loadPCDFile ("bun0.pcd", cloud_blob);
  fromROSMsg (cloud_blob, *cloud);
  //* the data should be available in cloud

  // Normal estimation*
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
  KdTree<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  KdTree<PointNormal>::Ptr tree2 (new KdTreeFLANN<PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  GreedyProjectionTriangulation<PointNormal> gp3;
  PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAgle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  // Finish
  return (0);
}
