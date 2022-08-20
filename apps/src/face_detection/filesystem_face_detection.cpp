/*
 * openni_face_detection.cpp
 *
 *  Created on: 22 Sep 2012
 *      Author: Aitor Aldoma
 */

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/recognition/face_detection/rf_face_detector_trainer.h>
#include <pcl/visualization/pcl_visualizer.h>
// clang-format off
#include <pcl/apps/face_detection/face_detection_apps_utils.h>
// clang-format on

#include <boost/filesystem.hpp>

namespace bf = boost::filesystem;

bool SHOW_GT = false;
bool VIDEO = false;

template <class PointInT>
void
run(pcl::RFFaceDetectorTrainer& fdrf,
    typename pcl::PointCloud<PointInT>::Ptr& scene_vis,
    pcl::visualization::PCLVisualizer& vis,
    bool heat_map,
    bool show_votes,
    const std::string& filename)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*scene_vis, *scene);

  fdrf.setInputCloud(scene);

  if (heat_map) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    fdrf.setFaceHeatMapCloud(intensity_cloud);
  }

  fdrf.detectFaces();

  using FieldListM = typename pcl::traits::fieldList<PointInT>::type;

  float rgb_m;
  bool exists_m;
  pcl::for_each_type<FieldListM>(
      pcl::CopyIfFieldExists<PointInT, float>((*scene_vis)[0], "rgb", exists_m, rgb_m));

  std::cout << "Color exists:" << static_cast<int>(exists_m) << std::endl;
  if (exists_m) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr to_visualize(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*scene_vis, *to_visualize);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
        handler_keypoints(to_visualize);
    vis.addPointCloud<pcl::PointXYZRGB>(to_visualize, handler_keypoints, "scene_cloud");
  }
  else {
    vis.addPointCloud(scene_vis, "scene_cloud");
  }

  if (heat_map) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    fdrf.getFaceHeatMap(intensity_cloud);

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
        handler_keypoints(intensity_cloud, "intensity");
    vis.addPointCloud<pcl::PointXYZI>(intensity_cloud, handler_keypoints, "heat_map");
  }

  if (show_votes) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr votes_cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    fdrf.getVotes2(votes_cloud);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
        handler_votes(votes_cloud, "intensity");
    vis.addPointCloud<pcl::PointXYZI>(votes_cloud, handler_votes, "votes_cloud");
    vis.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 14, "votes_cloud");
  }

  vis.addCoordinateSystem(0.1, "global");

  std::vector<Eigen::VectorXf> heads;
  fdrf.getDetectedFaces(heads);
  face_detection_apps_utils::displayHeads(heads, vis);

  if (SHOW_GT) {
    // check if there is ground truth data
    std::string pose_file(filename);
    boost::replace_all(pose_file, ".pcd", "_pose.txt");

    Eigen::Matrix4f pose_mat;
    pose_mat.setIdentity(4, 4);
    bool result = face_detection_apps_utils::readMatrixFromFile(pose_file, pose_mat);

    if (result) {
      Eigen::Vector3f ea = pose_mat.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
      Eigen::Vector3f trans_vector =
          Eigen::Vector3f(pose_mat(0, 3), pose_mat(1, 3), pose_mat(2, 3));
      std::cout << ea << std::endl;
      std::cout << trans_vector << std::endl;

      pcl::PointXYZ center_point;
      center_point.x = trans_vector[0];
      center_point.y = trans_vector[1];
      center_point.z = trans_vector[2];
      vis.addSphere(center_point, 0.05, 255, 0, 0, "sphere");

      pcl::ModelCoefficients cylinder_coeff;
      cylinder_coeff.values.resize(7); // We need 7 values
      cylinder_coeff.values[0] = center_point.x;
      cylinder_coeff.values[1] = center_point.y;
      cylinder_coeff.values[2] = center_point.z;

      cylinder_coeff.values[3] = ea[0];
      cylinder_coeff.values[4] = ea[1];
      cylinder_coeff.values[5] = ea[2];

      Eigen::Vector3f vec = Eigen::Vector3f::UnitZ() * -1.f;
      Eigen::Matrix3f matrixxx;

      matrixxx = Eigen::AngleAxisf(ea[0], Eigen::Vector3f::UnitX()) *
                 Eigen::AngleAxisf(ea[1], Eigen::Vector3f::UnitY()) *
                 Eigen::AngleAxisf(ea[2], Eigen::Vector3f::UnitZ());

      // matrixxx = pose_mat.block<3,3>(0,0);
      vec = matrixxx * vec;

      cylinder_coeff.values[3] = vec[0];
      cylinder_coeff.values[4] = vec[1];
      cylinder_coeff.values[5] = vec[2];

      cylinder_coeff.values[6] = 0.01f;
      vis.addCylinder(cylinder_coeff, "cylinder");
    }
  }

  vis.setRepresentationToSurfaceForAllActors();

  if (VIDEO) {
    vis.spinOnce(50, true);
  }
  else {
    vis.spin();
  }

  vis.removeAllPointClouds();
  vis.removeAllShapes();
}

int
main(int argc, char** argv)
{
  int STRIDE_SW = 5;
  std::string forest_fn = "forest.txt";
  int use_normals = 0;
  float trans_max_variance = 800.f;
  int min_votes_size = 400;
  float face_threshold = 0.95f;
  int heat_map = 1;
  int show_votes = 0;
  std::string test_directory;
  std::string filename;
  int rgb_exists = 0;
  int pose_refinement_ = 0;
  int icp_iterations = 5;
  std::string model_path_;

  pcl::console::parse_argument(argc, argv, "-forest_fn", forest_fn);
  pcl::console::parse_argument(argc, argv, "-max_variance", trans_max_variance);
  pcl::console::parse_argument(argc, argv, "-min_votes_size", min_votes_size);
  pcl::console::parse_argument(argc, argv, "-use_normals", use_normals);
  pcl::console::parse_argument(argc, argv, "-face_threshold", face_threshold);
  pcl::console::parse_argument(argc, argv, "-stride_sw", STRIDE_SW);
  pcl::console::parse_argument(argc, argv, "-heat_map", heat_map);
  pcl::console::parse_argument(argc, argv, "-show_votes", show_votes);
  pcl::console::parse_argument(argc, argv, "-test_directory", test_directory);
  pcl::console::parse_argument(argc, argv, "-filename", filename);
  pcl::console::parse_argument(argc, argv, "-rgb_exists", rgb_exists);
  pcl::console::parse_argument(argc, argv, "-show_gt", SHOW_GT);
  pcl::console::parse_argument(argc, argv, "-pose_refinement", pose_refinement_);
  pcl::console::parse_argument(argc, argv, "-model_path", model_path_);
  pcl::console::parse_argument(argc, argv, "-icp_iterations", icp_iterations);
  pcl::console::parse_argument(argc, argv, "-video", VIDEO);

  pcl::RFFaceDetectorTrainer fdrf;
  fdrf.setForestFilename(forest_fn);
  fdrf.setWSize(80);
  fdrf.setUseNormals(static_cast<bool>(use_normals));
  fdrf.setWStride(STRIDE_SW);
  fdrf.setLeavesFaceMaxVariance(trans_max_variance);
  fdrf.setLeavesFaceThreshold(face_threshold);
  fdrf.setFaceMinVotes(min_votes_size);

  if (pose_refinement_) {
    fdrf.setPoseRefinement(true, icp_iterations);
    fdrf.setModelPath(model_path_);
  }

  // load forest from file and pass it to the detector
  std::filebuf fb;
  fb.open(forest_fn.c_str(), std::ios::in);
  std::istream os(&fb);

  using NodeType = pcl::face_detection::RFTreeNode<pcl::face_detection::FeatureType>;
  pcl::DecisionForest<NodeType> forest;
  forest.deserialize(os);
  fb.close();

  fdrf.setForest(forest);

  pcl::visualization::PCLVisualizer vis("PCL Face detection");

  if (!test_directory.empty()) {
    // recognize all files in directory...
    std::string start;
    std::string ext = std::string("pcd");
    bf::path dir = test_directory;

    std::vector<std::string> files;
    face_detection_apps_utils::getFilesInDirectory(dir, start, files, ext);

    std::sort(files.begin(), files.end(), face_detection_apps_utils::sortFiles);

    for (const auto& filename : files) {
      std::string file = test_directory + '/' + filename;
      std::cout << file << std::endl;

      if (rgb_exists) {
        std::cout << "Loading a PointXYZRGBA cloud..." << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile(file, *cloud);
        run<pcl::PointXYZRGB>(fdrf, cloud, vis, heat_map, show_votes, file);
      }
      else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(file, *cloud);
        run<pcl::PointXYZ>(fdrf, cloud, vis, heat_map, show_votes, file);
      }
    }
  }
  else {
    if (rgb_exists) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::io::loadPCDFile(filename, *cloud);
      run<pcl::PointXYZRGB>(fdrf, cloud, vis, heat_map, show_votes, filename);
    }
    else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::io::loadPCDFile(filename, *cloud);
      run<pcl::PointXYZ>(fdrf, cloud, vis, heat_map, show_votes, filename);
    }
  }
}
