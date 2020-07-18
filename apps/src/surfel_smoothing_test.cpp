#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/surfel_smoothing.h>

using namespace pcl;

int
main(int argc, char** argv)
{
  if (argc != 5) {
    PCL_ERROR("./surfel_smoothing_test normal_search_radius surfel_scale source_cloud "
              "destination_cloud\n");
    return -1;
  }
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());

  PCDReader reader;
  reader.read(argv[3], *cloud);
  PCL_INFO("Cloud read: %s\n", argv[3]);

  float normal_search_radius = static_cast<float>(atof(argv[1]));
  float surfel_scale = static_cast<float>(atof(argv[2]));

  NormalEstimation<PointXYZ, Normal> normal_estimation;
  normal_estimation.setInputCloud(cloud);
  search::KdTree<PointXYZ>::Ptr search_tree(new search::KdTree<PointXYZ>);
  normal_estimation.setSearchMethod(search_tree);
  normal_estimation.setRadiusSearch(normal_search_radius);
  normal_estimation.compute(*normals);

  SurfelSmoothing<PointXYZ, Normal> surfel_smoothing(surfel_scale);
  surfel_smoothing.setInputCloud(cloud);
  surfel_smoothing.setInputNormals(normals);
  surfel_smoothing.setSearchMethod(search_tree);
  PointCloud<PointXYZ>::Ptr output_positions;
  PointCloud<Normal>::Ptr output_normals;
  surfel_smoothing.computeSmoothedCloud(output_positions, output_normals);

  PointCloud<PointNormal>::Ptr output_with_normals(new PointCloud<PointNormal>());
  pcl::concatenateFields(*output_positions, *normals, *output_with_normals);

  io::savePCDFileASCII(argv[4], *output_with_normals);

  return 0;
}
