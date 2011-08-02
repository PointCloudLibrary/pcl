#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;

int
main (int argc, char **argv)
{
  double dist = 0.05;
  pcl::console::parse_argument (argc, argv, "-d", dist);

  double rans = 0.05;
  pcl::console::parse_argument (argc, argv, "-r", rans);

  int iter = 50;
  pcl::console::parse_argument (argc, argv, "-i", iter);

  std::vector<int> pcd_indices;
  pcd_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  CloudPtr model (new Cloud);
  if (pcl::io::loadPCDFile (argv[pcd_indices[0]], *model) == -1)
  {
    std::cout << "Could not read file" << std::endl;
    return -1;
  }
  std::cout << argv[pcd_indices[0]] << " width: " << model->width << " height: " << model->height << std::endl;

  std::string result_filename (argv[pcd_indices[0]]);
  result_filename = result_filename.substr (result_filename.rfind ("/") + 1);
  pcl::io::savePCDFile (result_filename.c_str (), *model);
  std::cout << "saving first model to " << result_filename << std::endl;

  Eigen::Matrix4f t (Eigen::Matrix4f::Identity ());

  for (size_t i = 1; i < pcd_indices.size (); i++)
  {
    CloudPtr data (new Cloud);
    if (pcl::io::loadPCDFile (argv[pcd_indices[i]], *data) == -1)
    {
      std::cout << "Could not read file" << std::endl;
      return -1;
    }
    std::cout << argv[pcd_indices[i]] << " width: " << data->width << " height: " << data->height << std::endl;

    pcl::IterativeClosestPoint<PointType, PointType> icp;

    icp.setMaximumIterations (iter);
    icp.setMaxCorrespondenceDistance (dist);
    icp.setRANSACOutlierRejectionThreshold (rans);

    icp.setInputTarget (model);

    icp.setInputCloud (data);

    CloudPtr tmp (new Cloud);
    icp.align (*tmp);

    t = icp.getFinalTransformation () * t;

    pcl::transformPointCloud (*data, *tmp, t);

    std::cout << icp.getFinalTransformation () << std::endl;

    *model = *data;

    std::string result_filename (argv[pcd_indices[i]]);
    result_filename = result_filename.substr (result_filename.rfind ("/") + 1);
    pcl::io::savePCDFileBinary (result_filename.c_str (), *tmp);
    std::cout << "saving result to " << result_filename << std::endl;
  }

  return 0;
}
