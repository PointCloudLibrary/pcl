/**
 * Demo program for simulation library
 * A virtual camera generates simulated point clouds
 * No visual output, point clouds saved to file
 *
 * three different demo modes:
 * 0 - static camera, 100 poses
 * 1 - circular camera flying around the scene, 16 poses
 * 2 - camera translates between 2 poses using slerp, 20 poses
 * pcl_sim_terminal_demo 2 ../../../../kmcl/models/table_models/meta_model.ply
 */

#include <pcl/common/time.h> // for getTime
#include <pcl/io/pcd_io.h>   // for PCDWriter
#include <pcl/memory.h>

#include "simulation_io.hpp"

#include <cmath>
#include <iostream>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

using namespace Eigen;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;
using namespace pcl::simulation;

SimExample::Ptr simexample;

void
printHelp(int, char** argv)
{
  print_error("Syntax is: %s <mode 1,2 or 3> <filename>\n", argv[0]);
  print_info("acceptable filenames include vtk, obj and ply. ply can support colour\n");
}

// Output the simulated output to file:
void
write_sim_output(const std::string& fname_root)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Read Color Buffer from the GPU before creating PointCloud:
  // By default the buffers are not read back from the GPU
  simexample->rl_->getColorBuffer();
  simexample->rl_->getDepthBuffer();
  // Add noise directly to the CPU depth buffer
  simexample->rl_->addNoise();

  // Optional argument to save point cloud in global frame:
  // Save camera relative:
  // simexample->rl_->getPointCloud(pc_out);
  // Save in global frame - applying the camera frame:
  // simexample->rl_->getPointCloud(pc_out,true,simexample->camera_->getPose());
  // Save in local frame
  simexample->rl_->getPointCloud(pc_out, false, simexample->camera_->getPose());
  // TODO: what to do when there are more than one simulated view?

  if (!pc_out->points.empty()) {
    std::cout << pc_out->size() << " points written to file\n";

    pcl::PCDWriter writer;
    // writer.write ( string (fname_root + ".pcd"), *pc_out,	false);  /// ASCII
    writer.writeBinary(std::string(fname_root + ".pcd"), *pc_out);
    // std::cout << "finished writing file\n";
  }
  else {
    std::cout << pc_out->size() << " points in cloud, not written\n";
  }

  // simexample->write_score_image (simexample->rl_->getScoreBuffer (),
  //                               string (fname_root + "_score.png") );
  simexample->write_rgb_image(simexample->rl_->getColorBuffer(),
                              std::string(fname_root + "_rgb.png"));
  simexample->write_depth_image(simexample->rl_->getDepthBuffer(),
                                std::string(fname_root + "_depth.png"));
  // simexample->write_depth_image_uint (simexample->rl_->getDepthBuffer (),
  //                                    string (fname_root + "_depth_uint.png") );

  // Demo interacton with RangeImage:
  pcl::RangeImagePlanar rangeImage;
  simexample->rl_->getRangeImagePlanar(rangeImage);
}

// A 'halo' camera - a circular ring of poses all pointing at a center point
// @param: focus_center: the center points
// @param: halo_r: radius of the ring
// @param: halo_dz: elevation of the camera above/below focus_center's z value
// @param: n_poses: number of generated poses
void
generate_halo(
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& poses,
    Eigen::Vector3d focus_center,
    double halo_r,
    double halo_dz,
    int n_poses)
{

  for (double t = 0; t < (2 * M_PI); t = t + (2 * M_PI) / ((double)n_poses)) {
    double x = halo_r * std::cos(t);
    double y = halo_r * sin(t);
    double z = halo_dz;
    double pitch = std::atan2(halo_dz, halo_r);
    double yaw = std::atan2(-y, -x);

    Eigen::Isometry3d pose;
    pose.setIdentity();
    Eigen::Matrix3d m;
    m = AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        AngleAxisd(0, Eigen::Vector3d::UnitZ());

    pose *= m;
    Vector3d v(x, y, z);
    v += focus_center;
    pose.translation() = v;
    poses.push_back(pose);
  }
  return;
}

int
main(int argc, char** argv)
{
  // 1. Parse arguments:
  print_info("Manually generate a simulated RGB-D point cloud using pcl::simulation. "
             "For more information, use: %s -h\n",
             argv[0]);
  if (argc < 3) {
    printHelp(argc, argv);
    return (-1);
  }
  int mode = atoi(argv[1]);

  // 2 Construct the simulation method:
  int width = 640;
  int height = 480;
  simexample = SimExample::Ptr(new SimExample(argc, argv, height, width));

  // 3 Generate a series of simulation poses:
  // -0 100 fixed poses
  // -1 a 'halo' camera
  // -2 slerp between two different poses
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
  if (mode == 0) {
    // Create a pose:
    Eigen::Isometry3d pose;
    pose.setIdentity();
    Matrix3d m;
    // ypr:
    m = AngleAxisd(-9.14989, Vector3d::UnitZ()) *
        AngleAxisd(0.20944, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitX());
    pose *= m;
    Vector3d v;
    v << 1.31762, 0.382931, 1.89533;
    pose.translation() = v;
    for (int i = 0; i < 100; i++) { // duplicate the pose 100 times
      poses.push_back(pose);
    }
  }
  else if (mode == 1) {
    Eigen::Vector3d focus_center(0, 0, 1.3);
    double halo_r = 4;
    double halo_dz = 2;
    int n_poses = 16;
    generate_halo(poses, focus_center, halo_r, halo_dz, n_poses);
  }
  else if (mode == 2) {
    Eigen::Isometry3d pose1;
    pose1.setIdentity();
    pose1.translation() << 1, 0.75, 2;
    Eigen::Matrix3d rot1;
    rot1 = AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
           AngleAxisd(M_PI / 10, Eigen::Vector3d::UnitY()) *
           AngleAxisd(0.0, Eigen::Vector3d::UnitZ()); // ypr
    pose1.rotate(rot1);

    Eigen::Isometry3d pose2;
    pose2.setIdentity();
    pose2.translation() << 1, -1, 3;
    Eigen::Matrix3d rot2;
    rot2 = AngleAxisd(3 * M_PI / 4, Eigen::Vector3d::UnitZ()) *
           AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()) *
           AngleAxisd(0.0, Eigen::Vector3d::UnitZ()); // ypr
    pose2.rotate(rot2);

    int n_poses = 20;
    for (double i = 0; i <= 1; i += 1 / ((double)n_poses - 1)) {
      Eigen::Quaterniond rot3;
      Eigen::Quaterniond r1(pose1.rotation());
      Eigen::Quaterniond r2(pose2.rotation());
      rot3 = r1.slerp(i, r2);
      Eigen::Isometry3d pose;
      pose.setIdentity();
      Eigen::Vector3d trans3 = (1 - i) * pose1.translation() + i * pose2.translation();
      pose.translation() << trans3[0], trans3[1], trans3[2];
      pose.rotate(rot3);
      poses.push_back(pose);
    }
  }

  // 4 Do the simulation and write the output:
  double tic_main = getTime();
  for (std::size_t i = 0; i < poses.size(); i++) {
    std::stringstream ss;
    ss.precision(20);
    ss << "simcloud_" << i; // << ".pcd";
    double tic = getTime();
    simexample->doSim(poses[i]);

    write_sim_output(ss.str());
    std::cout << (getTime() - tic) << " sec\n";
  }
  std::cout << poses.size() << " poses simulated in " << (getTime() - tic_main)
            << "seconds\n";
  std::cout << (poses.size() / (getTime() - tic_main)) << "Hz on average\n";

  return 0;
}
