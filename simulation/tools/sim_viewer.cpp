/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: pcd_viewer.cpp 5094 2012-03-15 01:03:51Z rusu $
 *
 */

#include <pcl/common/common.h>
#include <pcl/common/time.h> // for getTime
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/range_image/range_image_planar.h> // RangeImage
#include <pcl/simulation/camera.h>
#include <pcl/simulation/model.h>
#include <pcl/simulation/range_likelihood.h>
#include <pcl/simulation/scene.h>
#include <pcl/visualization/cloud_viewer.h> // Pop-up viewer
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/memory.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <vtkPolyDataReader.h>

#include <GL/glew.h>

#ifdef OPENGL_IS_A_FRAMEWORK
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <cmath>
#include <iostream>
#include <limits>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

using namespace Eigen;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;
using namespace pcl::simulation;

using ColorHandler = pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>;
using ColorHandlerPtr = ColorHandler::Ptr;
using ColorHandlerConstPtr = ColorHandler::ConstPtr;

using GeometryHandler =
    pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>;
using GeometryHandlerPtr = GeometryHandler::Ptr;
using GeometryHandlerConstPtr = GeometryHandler::ConstPtr;

constexpr double NORMALS_SCALE = 0.01;
constexpr double PC_SCALE = 0.001;

std::uint16_t t_gamma[2048];
Scene::Ptr scene_;
Camera::Ptr camera_;
RangeLikelihood::Ptr range_likelihood_;
int window_width_;
int window_height_;
bool paused_;
bool write_file_;

bool
isValidFieldName(const std::string& field)
{
  if (field == "_")
    return (false);

  if ((field == "vp_x") || (field == "vx") || (field == "vp_y") || (field == "vy") ||
      (field == "vp_z") || (field == "vz"))
    return (false);
  return (true);
}

bool
isMultiDimensionalFeatureField(const pcl::PCLPointField& field)
{
  if (field.count > 1)
    return (true);
  return (false);
}

void
printHelp(int, char** argv)
{
  print_error("Syntax is: %s <file_name 1..N>.<pcd or vtk> <options>\n", argv[0]);
  print_info("pcl::simulation viewer\n");
  print_info("  where options are:\n");
  print_info("                     -bc r,g,b                = background color\n");
  print_info("                     -fc r,g,b                = foreground color\n");
  print_info("                     -ps X                    = point size (");
  print_value("1..64");
  print_info(") \n");
  print_info(
      "                     -opaque X                = rendered point cloud opacity (");
  print_value("0..1");
  print_info(")\n");

  print_info("                     -ax ");
  print_value("n");
  print_info("                    = enable on-screen display of ");
  print_color(stdout, TT_BRIGHT, TT_RED, "X");
  print_color(stdout, TT_BRIGHT, TT_GREEN, "Y");
  print_color(stdout, TT_BRIGHT, TT_BLUE, "Z");
  print_info(" axes and scale them to ");
  print_value("n\n");
  print_info("                     -ax_pos X,Y,Z            = if axes are enabled, set "
             "their X,Y,Z position in space (default ");
  print_value("0,0,0");
  print_info(")\n");

  print_info("\n");
  print_info("                     -cam (*)                 = use given camera "
             "settings as initial view\n");
  print_info(stderr,
             " (*) [Clipping Range / Focal Point / Position / ViewUp / Distance / "
             "Field of View Y / Window Size / Window Pos] or use a <filename.cam> that "
             "contains the same information.\n");

  print_info("\n");
  print_info("                     -multiview 0/1           = enable/disable "
             "auto-multi viewport rendering (default ");
  print_value("disabled");
  print_info(")\n");
  print_info("\n");

  print_info("\n");
  print_info("                     -normals 0/X             = disable/enable the "
             "display of every Xth point's surface normal as lines (default ");
  print_value("disabled");
  print_info(")\n");
  print_info("                     -normals_scale X         = resize the normal unit "
             "vector size to X (default ");
  print_value("0.02");
  print_info(")\n");
  print_info("\n");
  print_info("                     -pc 0/X                  = disable/enable the "
             "display of every Xth point's principal curvatures as lines (default ");
  print_value("disabled");
  print_info(")\n");
  print_info("                     -pc_scale X              = resize the principal "
             "curvatures vectors size to X (default ");
  print_value("0.02");
  print_info(")\n");
  print_info("\n");

  print_info("\n(Note: for multiple .pcd files, provide multiple -{fc,ps,opaque} "
             "parameters; they will be automatically assigned to the right file)\n");
}

// Global visualizer object
pcl::visualization::PCLHistogramVisualizer ph_global;
pcl::visualization::PCLVisualizer::Ptr p;

void
pp_callback(const pcl::visualization::PointPickingEvent& event, void* cookie)
{
  if (event.getPointIndex() == -1)
    return;
  pcl::PCLPointCloud2::Ptr cloud = *static_cast<pcl::PCLPointCloud2::Ptr*>(cookie);
  if (!cloud)
    return;

  // If two points were selected, draw an arrow between them
  pcl::PointXYZ p1, p2;
  if (event.getPoints(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z) && p) {
    std::stringstream ss;
    ss << p1 << p2;
    p->addArrow<pcl::PointXYZ, pcl::PointXYZ>(p1, p2, 1.0, 1.0, 1.0, ss.str());
    return;
  }

  // Else, if a single point has been selected
  std::string pointIndexStr = std::to_string(event.getPointIndex());
  // Get the cloud's fields
  for (std::size_t i = 0; i < cloud->fields.size(); ++i) {
    if (!isMultiDimensionalFeatureField(cloud->fields[i]))
      continue;
    ph_global.addFeatureHistogram(
        *cloud, cloud->fields[i].name, event.getPointIndex(), pointIndexStr);
  }
  if (p) {
    pcl::PointXYZ pos;
    event.getPoint(pos.x, pos.y, pos.z);
    p->addText3D<pcl::PointXYZ>(
        pointIndexStr, pos, 0.0005, 1.0, 1.0, 1.0, pointIndexStr);
  }
  ph_global.spinOnce();
}

void
capture(Eigen::Isometry3d pose_in)
{
  // No reference image - but this is kept for compatibility with range_test_v2:
  float* reference =
      new float[range_likelihood_->getRowHeight() * range_likelihood_->getColWidth()];
  const float* depth_buffer = range_likelihood_->getDepthBuffer();
  // Copy one image from our last as a reference.
  for (int i = 0, n = 0; i < range_likelihood_->getRowHeight(); ++i) {
    for (int j = 0; j < range_likelihood_->getColWidth(); ++j) {
      reference[n++] = depth_buffer[i * range_likelihood_->getWidth() + j];
    }
  }

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
  std::vector<float> scores;
  poses.push_back(pose_in);

  range_likelihood_->computeLikelihoods(reference, poses, scores);
  std::cout << "score: ";
  for (const float& score : scores) {
    std::cout << " " << score;
  }
  std::cout << std::endl;

  std::cout << "camera: " << camera_->getX() << " " << camera_->getY() << " "
            << camera_->getZ() << " " << camera_->getRoll() << " "
            << camera_->getPitch() << " " << camera_->getYaw() << std::endl;

  delete[] reference;

  // Benchmark Values for
  // 27840 triangle faces
  // 13670 vertices

  // 45.00Hz: simuation only
  //  1.28Hz: simuation, addNoise?    , getPointCloud, writeASCII
  // 33.33Hz: simuation, getPointCloud
  // 23.81Hz: simuation, getPointCloud, writeBinary
  // 14.28Hz: simuation, addNoise, getPointCloud, writeBinary
  // MODULE        TIME      FRACTION
  // simuation     0.02222   31%
  // addNoise      0.03	     41%
  // getPointCloud 0.008     11%
  // writeBinary   0.012     16%
  // total	   0.07222

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void
print_Quaterniond(Eigen::Quaterniond r, std::stringstream& ss)
{
  ss << r.w() << ", " << r.x() << ", " << r.y() << ", " << r.z();
  //  std::cout << r.str() << "q\n";
}

// Normalize angle to be within the interval [-pi,pi].
double
standardRad(double t)
{
  if (t >= 0.) {
    t = std::fmod(t + M_PI, 2 * M_PI) - M_PI;
  }
  else {
    t = std::fmod(t - M_PI, -2 * M_PI) + M_PI;
  }
  return t;
}

void
wRo_to_euler(const Eigen::Matrix3f& wRo, double& yaw, double& pitch, double& roll)
{
  yaw = standardRad(std::atan2(wRo(1, 0), wRo(0, 0)));
  double c = std::cos(yaw);
  double s = sin(yaw);
  pitch = standardRad(
      std::atan2(static_cast<double>(-wRo(2, 0)), wRo(0, 0) * c + wRo(1, 0) * s));
  roll = standardRad(
      std::atan2(wRo(0, 2) * s - wRo(1, 2) * c, -wRo(0, 1) * s + wRo(1, 1) * c));
}

void
print_Isometry3d(Eigen::Isometry3d pose, std::stringstream& ss)
{
  Eigen::Vector3d t(pose.translation());
  Eigen::Quaterniond r(pose.rotation());
  ss << t[0] << ", " << t[1] << ", " << t[2] << " | " << r.w() << ", " << r.x() << ", "
     << r.y() << ", " << r.z();
  //  std::cout << ss.str() << "q\n";
}

void
simulate_callback(const pcl::visualization::KeyboardEvent& event, void* viewer_void)
{
  pcl::visualization::PCLVisualizer::Ptr viewer =
      *static_cast<pcl::visualization::PCLVisualizer::Ptr*>(viewer_void);
  // I choose v for virtual as s for simulate is takwen
  if (event.getKeySym() == "v" && event.keyDown()) {
    std::cout << "v was pressed => simulate cloud" << std::endl;

    std::vector<pcl::visualization::Camera> cams;
    viewer->getCameras(cams);

    if (cams.size() != 1) {
      std::cout << "n cams not 1 exiting\n"; // for now in case ...
      return;
    }
    // std::cout << "n cams: " << cams.size() << "\n";
    pcl::visualization::Camera cam = cams[0];

    Eigen::Affine3f pose;
    pose = viewer->getViewerPose();

    std::cout << cam.pos[0] << " " << cam.pos[1] << " " << cam.pos[2] << " p\n";

    Eigen::Matrix3f m;
    m = pose.rotation();
    // All axies use right hand rule. x=red axis, y=green axis, z=blue axis z direction
    // is point into the screen. z \ \ \ -----------> x | | | | | | y

    std::cout << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " "
              << pose(0, 3) << " x0\n";
    std::cout << pose(1, 0) << " " << pose(1, 1) << " " << pose(1, 2) << " "
              << pose(1, 3) << " x1\n";
    std::cout << pose(2, 0) << " " << pose(2, 1) << " " << pose(2, 2) << " "
              << pose(2, 3) << "x2\n";

    double yaw, pitch, roll;
    wRo_to_euler(m, yaw, pitch, roll);

    printf("RPY: %f %f %f\n", roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);

    // matrix->GetElement(1,0);

    std::cout << m(0, 0) << " " << m(0, 1) << " " << m(0, 2) << " "
              << " x0\n";
    std::cout << m(1, 0) << " " << m(1, 1) << " " << m(1, 2) << " "
              << " x1\n";
    std::cout << m(2, 0) << " " << m(2, 1) << " " << m(2, 2) << " "
              << "x2\n\n";

    Eigen::Quaternionf rf;
    rf = Eigen::Quaternionf(m);

    Eigen::Quaterniond r(rf.w(), rf.x(), rf.y(), rf.z());

    Eigen::Isometry3d init_pose;
    init_pose.setIdentity();
    init_pose.translation() << cam.pos[0], cam.pos[1], cam.pos[2];
    // Eigen::Quaterniond m = euler_to_quat(-1.54, 0, 0);
    init_pose.rotate(r);
    //

    std::stringstream ss;
    print_Isometry3d(init_pose, ss);
    std::cout << "init_pose: " << ss.str() << "\n";

    viewer->addCoordinateSystem(1.0, pose, "reference");

    double tic = getTime();
    std::stringstream ss2;
    ss2.precision(20);
    ss2 << "simulated_pcl_" << tic << ".pcd";
    capture(init_pose);
    std::cout << (getTime() - tic) << " sec\n";
  }
}

// Read in a 3D model
void
loadPolygonMeshModel(char* polygon_file)
{
  pcl::PolygonMesh mesh; // (new pcl::PolygonMesh);
  // pcl::io::loadPolygonFile("/home/mfallon/data/models/dalet/Darlek_modified_works.obj",mesh);
  pcl::io::loadPolygonFile(polygon_file, mesh);
  pcl::PolygonMesh::Ptr cloud(new pcl::PolygonMesh(mesh));

  // Not sure if PolygonMesh assumes triangles if to
  // TODO: Ask a developer
  // PolygonMeshModel::Ptr model = PolygonMeshModel::Ptr (new PolygonMeshModel
  // (GL_POLYGON, cloud));
  TriangleMeshModel::Ptr model = TriangleMeshModel::Ptr(new TriangleMeshModel(cloud));
  scene_->add(model);

  std::cout << "Just read " << polygon_file << std::endl;
  std::cout << mesh.polygons.size() << " polygons and " << mesh.cloud.data.size()
            << " triangles\n";
}

void
initialize(int, char** argv)
{
  const GLubyte* version = glGetString(GL_VERSION);
  std::cout << "OpenGL Version: " << version << std::endl;

  // works for small files:
  camera_->set(-5.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  pcl::console::print_info("About to read: %s", argv[2]);
  loadPolygonMeshModel(argv[2]);
}

int
main(int argc, char** argv)
{
  srand(time(nullptr));

  print_info("The viewer window provides interactive commands; for help, press 'h' or "
             "'H' from within the window.\n");

  if (argc < 2) {
    printHelp(argc, argv);
    return (-1);
  }

  // Command line parsing
  double bcolor[3] = {0, 0, 0};
  pcl::console::parse_3x_arguments(argc, argv, "-bc", bcolor[0], bcolor[1], bcolor[2]);

  std::vector<double> fcolor_r, fcolor_b, fcolor_g;
  bool fcolorparam = pcl::console::parse_multiple_3x_arguments(
      argc, argv, "-fc", fcolor_r, fcolor_g, fcolor_b);

  std::vector<int> psize;
  pcl::console::parse_multiple_arguments(argc, argv, "-ps", psize);

  std::vector<double> opaque;
  pcl::console::parse_multiple_arguments(argc, argv, "-opaque", opaque);

  int mview = 0;
  pcl::console::parse_argument(argc, argv, "-multiview", mview);

  int normals = 0;
  pcl::console::parse_argument(argc, argv, "-normals", normals);
  double normals_scale = NORMALS_SCALE;
  pcl::console::parse_argument(argc, argv, "-normals_scale", normals_scale);

  int pc = 0;
  pcl::console::parse_argument(argc, argv, "-pc", pc);
  double pc_scale = PC_SCALE;
  pcl::console::parse_argument(argc, argv, "-pc_scale", pc_scale);

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices =
      pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  std::vector<int> vtk_file_indices =
      pcl::console::parse_file_extension_argument(argc, argv, ".vtk");

  if (p_file_indices.empty() && vtk_file_indices.empty()) {
    print_error("No .PCD or .VTK file given. Nothing to visualize.\n");
    return (-1);
  }

  // Multiview enabled?
  int x_s = 0;
  double x_step = 0, y_step = 0;
  if (mview) {
    print_highlight("Multi-viewport rendering enabled.\n");

    int y_s = 0;
    if (!p_file_indices.empty()) {
      y_s = static_cast<int>(
          std::floor(std::sqrt(static_cast<float>(p_file_indices.size()))));
      x_s = y_s + static_cast<int>(std::ceil(
                      (p_file_indices.size() / static_cast<double>(y_s)) - y_s));
      print_highlight("Preparing to load ");
      print_value("%d", p_file_indices.size());
    }
    else if (!vtk_file_indices.empty()) {
      y_s = static_cast<int>(
          std::floor(std::sqrt(static_cast<float>(vtk_file_indices.size()))));
      x_s = y_s + static_cast<int>(std::ceil(
                      (vtk_file_indices.size() / static_cast<double>(y_s)) - y_s));
      print_highlight("Preparing to load ");
      print_value("%d", vtk_file_indices.size());
    }

    x_step = static_cast<double>(1.0 / static_cast<double>(x_s));
    y_step = static_cast<double>(1.0 / static_cast<double>(y_s));
    print_info(" files (");
    print_value("%d", x_s);
    print_info("x");
    print_value("%d", y_s);
    print_info(" / ");
    print_value("%f", x_step);
    print_info("x");
    print_value("%f", y_step);
    print_info(")\n");
  }

  // Fix invalid multiple arguments
  if (psize.size() != p_file_indices.size() && !psize.empty())
    for (std::size_t i = psize.size(); i < p_file_indices.size(); ++i)
      psize.push_back(1);
  if (opaque.size() != p_file_indices.size() && !opaque.empty())
    for (std::size_t i = opaque.size(); i < p_file_indices.size(); ++i)
      opaque.push_back(1.0);

  // Create the PCLHistogramVisualizer object
  pcl::visualization::PCLHistogramVisualizer::Ptr ph;

  // Using min_p, max_p to set the global Y min/max range for the histogram
  float min_p = std::numeric_limits<float>::max();
  float max_p = std::numeric_limits<float>::lowest();

  int k = 0, l = 0, viewport = 0;
  // Load the data files
  pcl::PCDReader pcd;
  pcl::console::TicToc tt;
  ColorHandlerPtr color_handler;
  GeometryHandlerPtr geometry_handler;

  // Go through VTK files
  for (std::size_t i = 0; i < vtk_file_indices.size(); ++i) {
    const char* vtk_file = argv[vtk_file_indices[i]];
    // Load file
    tt.tic();
    print_highlight(stderr, "Loading ");
    print_value(stderr, "%s ", vtk_file);
    vtkPolyDataReader* reader = vtkPolyDataReader::New();
    reader->SetFileName(vtk_file);
    reader->Update();
    vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
    if (!polydata)
      return (-1);
    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms : ");
    print_value("%d", polydata->GetNumberOfPoints());
    print_info(" points]\n");

    // Create the PCLVisualizer object here on the first encountered XYZ file
    if (!p)
      p.reset(new pcl::visualization::PCLVisualizer(argc, argv, "PCD viewer"));

    // Multiview enabled?
    if (mview) {
      p->createViewPort(
          k * x_step, l * y_step, (k + 1) * x_step, (l + 1) * y_step, viewport);
      k++;
      if (k >= x_s) {
        k = 0;
        l++;
      }
    }

    // Add as actor
    const std::string cloud_name =
        "vtk-" + std::string(vtk_file) + "-" + std::to_string(i);
    p->addModelFromPolyData(polydata, cloud_name, viewport);

    // Change the shape rendered color
    if (fcolorparam && fcolor_r.size() > i && fcolor_g.size() > i &&
        fcolor_b.size() > i)
      p->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                     fcolor_r[i],
                                     fcolor_g[i],
                                     fcolor_b[i],
                                     cloud_name);

    // Change the shape rendered point size
    if (!psize.empty())
      p->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize.at(i), cloud_name);

    // Change the shape rendered opacity
    if (!opaque.empty())
      p->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_OPACITY, opaque.at(i), cloud_name);
  }

  pcl::PCLPointCloud2::Ptr cloud;
  // Go through PCD files
  for (std::size_t i = 0; i < p_file_indices.size(); ++i) {
    const std::string p_file = argv[p_file_indices[i]];
    cloud.reset(new pcl::PCLPointCloud2);
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int version;

    print_highlight(stderr, "Loading ");
    print_value(stderr, "%s ", p_file.c_str());

    tt.tic();
    if (pcd.read(argv[p_file_indices.at(i)], *cloud, origin, orientation, version) < 0)
      return (-1);

    // ---[ Special check for 1-point multi-dimension histograms
    if (cloud->fields.size() == 1 && isMultiDimensionalFeatureField(cloud->fields[0])) {
      if (!ph)
        ph.reset(new pcl::visualization::PCLHistogramVisualizer);
      print_info("[done, ");
      print_value("%g", tt.toc());
      print_info(" ms : ");
      print_value("%d", cloud->fields[0].count);
      print_info(" points]\n");

      pcl::getMinMax(*cloud, 0, cloud->fields[0].name, min_p, max_p);
      ph->addFeatureHistogram(*cloud, cloud->fields[0].name, p_file);
      continue;
    }

    // Create the PCLVisualizer object here on the first encountered XYZ file
    if (!p) {
      p.reset(new pcl::visualization::PCLVisualizer(argc, argv, "PCD viewer"));
      p->registerPointPickingCallback(&pp_callback, (void*)&cloud);
      Eigen::Matrix3f rotation;
      rotation = orientation;
      p->setCameraPosition(origin[0],
                           origin[1],
                           origin[2],
                           origin[0] + rotation(0, 2),
                           origin[1] + rotation(1, 2),
                           origin[2] + rotation(2, 2),
                           rotation(0, 1),
                           rotation(1, 1),
                           rotation(2, 1));
    }

    // Multiview enabled?
    if (mview) {
      p->createViewPort(
          k * x_step, l * y_step, (k + 1) * x_step, (l + 1) * y_step, viewport);
      k++;
      if (k >= x_s) {
        k = 0;
        l++;
      }
    }

    if (cloud->width * cloud->height == 0) {
      print_error("[error: no points found!]\n");
      return (-1);
    }
    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms : ");
    print_value("%d", (int)cloud->width * cloud->height);
    print_info(" points]\n");
    print_info("Available dimensions: ");
    print_value("%s\n", pcl::getFieldsList(*cloud).c_str());

    // If no color was given, get random colors
    if (fcolorparam) {
      if (fcolor_r.size() > i && fcolor_g.size() > i && fcolor_b.size() > i)
        color_handler.reset(
            new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2>(
                cloud, fcolor_r[i], fcolor_g[i], fcolor_b[i]));
      else
        color_handler.reset(
            new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(
                cloud));
    }
    else
      color_handler.reset(
          new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(
              cloud));

    // Add the dataset with a XYZ and a random handler
    geometry_handler.reset(
        new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>(
            cloud));
    // Add the cloud to the renderer
    const std::string cloud_name = p_file + "-" + std::to_string(i);
    p->addPointCloud(cloud,
                     geometry_handler,
                     color_handler,
                     origin,
                     orientation,
                     cloud_name,
                     viewport);

    const std::string cloud_name_normals = cloud_name + "-normals";
    // If normal lines are enabled
    if (normals != 0) {
      int normal_idx = pcl::getFieldIndex(*cloud, "normal_x");
      if (normal_idx == -1) {
        print_error("Normal information requested but not available.\n");
        continue;
        // return (-1);
      }
      //
      // Convert from blob to pcl::PointCloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*cloud, *cloud_xyz);
      cloud_xyz->sensor_origin_ = origin;
      cloud_xyz->sensor_orientation_ = orientation;

      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
      pcl::fromPCLPointCloud2(*cloud, *cloud_normals);
      p->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_xyz,
                                                          cloud_normals,
                                                          normals,
                                                          normals_scale,
                                                          cloud_name_normals,
                                                          viewport);
    }

    // If principal curvature lines are enabled
    if (pc != 0) {
      if (normals == 0)
        normals = pc;

      int normal_idx = pcl::getFieldIndex(*cloud, "normal_x");
      if (normal_idx == -1) {
        print_error("Normal information requested but not available.\n");
        continue;
        // return (-1);
      }
      int pc_idx = pcl::getFieldIndex(*cloud, "principal_curvature_x");
      if (pc_idx == -1) {
        print_error("Principal Curvature information requested but not available.\n");
        continue;
        // return (-1);
      }
      //
      // Convert from blob to pcl::PointCloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*cloud, *cloud_xyz);
      cloud_xyz->sensor_origin_ = origin;
      cloud_xyz->sensor_orientation_ = orientation;
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
      pcl::fromPCLPointCloud2(*cloud, *cloud_normals);
      pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_pc(
          new pcl::PointCloud<pcl::PrincipalCurvatures>);
      pcl::fromPCLPointCloud2(*cloud, *cloud_pc);
      int factor = (std::min)(normals, pc);
      p->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_xyz,
                                                          cloud_normals,
                                                          factor,
                                                          normals_scale,
                                                          cloud_name_normals,
                                                          viewport);
      p->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, cloud_name_normals);
      p->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, cloud_name_normals);
      const auto cloud_name_normals_pc = cloud_name_normals + "-pc";
      p->addPointCloudPrincipalCurvatures<pcl::PointXYZ, pcl::Normal>(
          cloud_xyz,
          cloud_normals,
          cloud_pc,
          factor,
          pc_scale,
          cloud_name_normals_pc,
          viewport);
      p->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, cloud_name_normals_pc);
    }

    // Add every dimension as a possible color
    if (!fcolorparam) {
      for (std::size_t f = 0; f < cloud->fields.size(); ++f) {
        if (cloud->fields[f].name == "rgb" || cloud->fields[f].name == "rgba")
          color_handler.reset(new pcl::visualization::PointCloudColorHandlerRGBField<
                              pcl::PCLPointCloud2>(cloud));
        else {
          if (!isValidFieldName(cloud->fields[f].name))
            continue;
          color_handler.reset(
              new pcl::visualization::PointCloudColorHandlerGenericField<
                  pcl::PCLPointCloud2>(cloud, cloud->fields[f].name));
        }
        // Add the cloud to the renderer
        p->addPointCloud(
            cloud, color_handler, origin, orientation, cloud_name, viewport);
      }
    }
    // Additionally, add normals as a handler
    geometry_handler.reset(
        new pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<
            pcl::PCLPointCloud2>(cloud));
    if (geometry_handler->isCapable())
      p->addPointCloud(
          cloud, geometry_handler, origin, orientation, cloud_name, viewport);

    // Set immediate mode rendering ON
    p->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0, cloud_name);

    // Change the cloud rendered point size
    if (!psize.empty())
      p->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize.at(i), cloud_name);

    // Change the cloud rendered opacity
    if (!opaque.empty())
      p->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_OPACITY, opaque.at(i), cloud_name);
  }

  ////////////////////////////////////////////////////////////////
  // Key binding for saving simulated point cloud:
  if (p)
    p->registerKeyboardCallback(simulate_callback, (void*)&p);

  int width = 640;
  int height = 480;
  window_width_ = width * 2;
  window_height_ = height * 2;

  print_info("Manually generate a simulated RGB-D point cloud using pcl::simulation. "
             "For more information, use: %s -h\n",
             argv[0]);

  for (int i = 0; i < 2048; i++) {
    float v = i / 2048.0;
    v = powf(v, 3) * 6;
    t_gamma[i] = v * 6 * 256;
  }

  GLenum err = glewInit();
  if (GLEW_OK != err) {
    std::cerr << "Error: " << glewGetErrorString(err) << std::endl;
    exit(-1);
  }

  std::cout << "Status: Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;

  if (glewIsSupported("GL_VERSION_2_0"))
    std::cout << "OpenGL 2.0 supported" << std::endl;
  else {
    std::cerr << "Error: OpenGL 2.0 not supported" << std::endl;
    exit(1);
  }

  camera_ = Camera::Ptr(new Camera());
  scene_ = Scene::Ptr(new Scene());

  range_likelihood_ =
      RangeLikelihood::Ptr(new RangeLikelihood(1, 1, height, width, scene_));

  // range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(10, 10, 96, 96,
  // scene_)); range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(1, 1, 480,
  // 640, scene_));

  // Actually corresponds to default parameters:
  range_likelihood_->setCameraIntrinsicsParameters(
      640, 480, 576.09757860, 576.09757860, 321.06398107, 242.97676897);
  range_likelihood_->setComputeOnCPU(false);
  range_likelihood_->setSumOnCPU(true);
  range_likelihood_->setUseColor(true);
  initialize(argc, argv);

  if (p)
    p->setBackgroundColor(bcolor[0], bcolor[1], bcolor[2]);
  // Read axes settings
  double axes = 0.0;
  pcl::console::parse_argument(argc, argv, "-ax", axes);
  if (axes != 0.0 && p) {
    double ax_x = 0.0, ax_y = 0.0, ax_z = 0.0;
    pcl::console::parse_3x_arguments(argc, argv, "-ax_pos", ax_x, ax_y, ax_z, false);
    // Draw XYZ axes if command-line enabled
    p->addCoordinateSystem(axes, ax_x, ax_y, ax_z, "reference");
  }

  // Clean up the memory used by the binary blob
  // Note: avoid resetting the cloud, otherwise the PointPicking callback will fail
  // cloud.reset ();

  if (ph) {
    print_highlight("Setting the global Y range for all histograms to: ");
    print_value("%f -> %f\n", min_p, max_p);
    ph->setGlobalYRange(min_p, max_p);
    ph->updateWindowPositions();
    if (p)
      p->spin();
    else
      ph->spin();
  }
  else if (p)
    p->spin();
}
