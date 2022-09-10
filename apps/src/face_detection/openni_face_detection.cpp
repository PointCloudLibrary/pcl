/*
 * openni_face_detection.cpp
 *
 *  Created on: 22 Sep 2012
 *      Author: Aitor Aldoma
 */

#include <pcl/apps/face_detection/openni_frame_source.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/recognition/face_detection/rf_face_detector_trainer.h>
// clang-format off
#include <pcl/apps/face_detection/face_detection_apps_utils.h>
// clang-format on

void
run(pcl::RFFaceDetectorTrainer& fdrf, bool heat_map = false, bool show_votes = false)
{
  OpenNIFrameSource::OpenNIFrameSource camera;
  OpenNIFrameSource::PointCloudPtr scene_vis;

  pcl::visualization::PCLVisualizer vis("Face dection");
  vis.addCoordinateSystem(0.1, "global");

  // keyboard callback to stop getting frames and finalize application
  std::function<void(const pcl::visualization::KeyboardEvent&)> keyboard_cb =
      [&](const pcl::visualization::KeyboardEvent& event) {
        camera.onKeyboardEvent(event);
      };
  vis.registerKeyboardCallback(keyboard_cb);

  while (camera.isActive()) {
    scene_vis = camera.snap();

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*scene_vis, *scene);

    fdrf.setInputCloud(scene);

    if (heat_map) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud(
          new pcl::PointCloud<pcl::PointXYZI>);
      fdrf.setFaceHeatMapCloud(intensity_cloud);
    }

    {
      pcl::ScopeTime t("Detect faces...");
      fdrf.detectFaces();
    }
    pcl::visualization::PointCloudColorHandlerRGBField<OpenNIFrameSource::PointT>
        handler_keypoints(scene_vis);
    vis.addPointCloud<OpenNIFrameSource::PointT>(
        scene_vis, handler_keypoints, "scene_cloud");

    if (heat_map) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud(
          new pcl::PointCloud<pcl::PointXYZI>);
      fdrf.getFaceHeatMap(intensity_cloud);
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
          handler_keypoints(intensity_cloud, "intensity");
      vis.addPointCloud<pcl::PointXYZI>(intensity_cloud, handler_keypoints, "heat_map");
    }

    if (show_votes) {
      // display votes_
      pcl::PointCloud<pcl::PointXYZ>::Ptr votes_cloud(
          new pcl::PointCloud<pcl::PointXYZ>());
      fdrf.getVotes(votes_cloud);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_votes(
          votes_cloud, 255, 0, 0);
      vis.addPointCloud<pcl::PointXYZ>(votes_cloud, handler_votes, "votes_cloud");
      vis.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 14, "votes_cloud");
      vis.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "votes_cloud");
      vis.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_OPACITY, 0.75, "votes_cloud");
    }

    std::vector<Eigen::VectorXf> heads;
    fdrf.getDetectedFaces(heads);

    face_detection_apps_utils::displayHeads(heads, vis);

    vis.setRepresentationToSurfaceForAllActors();
    vis.spinOnce();

    vis.removeAllPointClouds();
    vis.removeAllShapes();
  }
}

int
main(int argc, char** argv)
{
  int STRIDE_SW = 4;
  int use_normals = 0;
  float trans_max_variance = 1600.f;
  int min_votes_size = 300;
  float face_threshold = 0.99f;
  int heat_map = 1;
  int show_votes = 0;
  int pose_refinement_ = 0;
  int icp_iterations = 5;

  std::string forest_fn = "../data/forests/forest.txt";
  std::string model_path_ = "../data/ply_models/face.pcd";

  pcl::console::parse_argument(argc, argv, "-forest_fn", forest_fn);
  pcl::console::parse_argument(argc, argv, "-max_variance", trans_max_variance);
  pcl::console::parse_argument(argc, argv, "-min_votes_size", min_votes_size);
  pcl::console::parse_argument(argc, argv, "-use_normals", use_normals);
  pcl::console::parse_argument(argc, argv, "-face_threshold", face_threshold);
  pcl::console::parse_argument(argc, argv, "-stride_sw", STRIDE_SW);
  pcl::console::parse_argument(argc, argv, "-heat_map", heat_map);
  pcl::console::parse_argument(argc, argv, "-show_votes", show_votes);
  pcl::console::parse_argument(argc, argv, "-pose_refinement", pose_refinement_);
  pcl::console::parse_argument(argc, argv, "-model_path", model_path_);
  pcl::console::parse_argument(argc, argv, "-icp_iterations", icp_iterations);

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

  run(fdrf, heat_map, show_votes);
}
