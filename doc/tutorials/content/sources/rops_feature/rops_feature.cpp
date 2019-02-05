#include <pcl/features/rops_estimation.h>
#include <pcl/io/pcd_io.h>

int main (int argc, char** argv)
{
  if (argc != 4)
    return (-1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  if (pcl::io::loadPCDFile (argv[1], *cloud) == -1)
    return (-1);

  pcl::PointIndicesPtr indices (new pcl::PointIndices);
  std::ifstream indices_file;
  indices_file.open (argv[2], std::ifstream::in);
  for (std::string line; std::getline (indices_file, line);)
  {
    std::istringstream in (line);
    unsigned int index = 0;
    in >> index;
    indices->indices.push_back (index - 1);
  }
  indices_file.close ();

  std::vector <pcl::Vertices> triangles;
  std::ifstream triangles_file;
  triangles_file.open (argv[3], std::ifstream::in);
  for (std::string line; std::getline (triangles_file, line);)
  {
    pcl::Vertices triangle;
    std::istringstream in (line);
    unsigned int vertex = 0;
    in >> vertex;
    triangle.vertices.push_back (vertex - 1);
    in >> vertex;
    triangle.vertices.push_back (vertex - 1);
    in >> vertex;
    triangle.vertices.push_back (vertex - 1);
    triangles.push_back (triangle);
  }

  float support_radius = 0.0285f;
  unsigned int number_of_partition_bins = 5;
  unsigned int number_of_rotations = 3;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method (new pcl::search::KdTree<pcl::PointXYZ>);
  search_method->setInputCloud (cloud);

  pcl::ROPSEstimation <pcl::PointXYZ, pcl::Histogram <135> > feature_estimator;
  feature_estimator.setSearchMethod (search_method);
  feature_estimator.setSearchSurface (cloud);
  feature_estimator.setInputCloud (cloud);
  feature_estimator.setIndices (indices);
  feature_estimator.setTriangles (triangles);
  feature_estimator.setRadiusSearch (support_radius);
  feature_estimator.setNumberOfPartitionBins (number_of_partition_bins);
  feature_estimator.setNumberOfRotations (number_of_rotations);
  feature_estimator.setSupportRadius (support_radius);

  pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms (new pcl::PointCloud <pcl::Histogram <135> > ());
  feature_estimator.compute (*histograms);

  return (0);
}
