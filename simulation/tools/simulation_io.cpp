#include "simulation_io.hpp"

#include <pcl/io/png_io.h>

pcl::simulation::SimExample::SimExample(int argc, char** argv, int height, int width)
: height_(height), width_(width)
{

  initializeGL(argc, argv);

  // 1. construct member elements:
  camera_ = Camera::Ptr(new Camera());
  scene_ = Scene::Ptr(new Scene());

  // rl_ = RangeLikelihoodGLSL::Ptr(new RangeLikelihoodGLSL(1, 1, height, width, scene_,
  // 0));
  rl_ = RangeLikelihood::Ptr(new RangeLikelihood(1, 1, height, width, scene_));
  // rl_ = RangeLikelihood::Ptr(new RangeLikelihood(10, 10, 96, 96, scene_));
  // rl_ = RangeLikelihood::Ptr(new RangeLikelihood(1, 1, height_, width_, scene_));

  // Actually corresponds to default parameters:
  rl_->setCameraIntrinsicsParameters(
      width_, height_, 576.09757860, 576.09757860, 321.06398107, 242.97676897);
  rl_->setComputeOnCPU(false);
  rl_->setSumOnCPU(true);
  rl_->setUseColor(true);

  // 2. read mesh and setup model:
  std::cout << "About to read: " << argv[2] << std::endl;
  pcl::PolygonMesh mesh; // (new pcl::PolygonMesh);
  pcl::io::loadPolygonFile(argv[2], mesh);
  pcl::PolygonMesh::Ptr cloud(new pcl::PolygonMesh(mesh));

  // Not sure if PolygonMesh assumes triangles if to, TODO: Ask a developer
  PolygonMeshModel::Ptr model =
      PolygonMeshModel::Ptr(new PolygonMeshModel(GL_POLYGON, cloud));
  scene_->add(model);

  std::cout << "Just read " << argv[2] << std::endl;
  std::cout << mesh.polygons.size() << " polygons and " << mesh.cloud.data.size()
            << " triangles\n";

  // works well for MIT CSAIL model 3rd floor:
  // camera_->set(4.04454, 44.9377, 1.1, 0.0, 0.0, -2.00352);

  // works well for MIT CSAIL model 2nd floor:
  //  camera_->set (27.4503, 37.383, 4.30908, 0.0, 0.0654498, -2.25802);

  // works for small files:
  // camera_->set(-5.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  camera_->set(0.471703, 1.59862, 3.10937, 0, 0.418879, -12.2129);
  camera_->setPitch(0.418879); // not sure why this is here:

  for (int i = 0; i < 2048; i++) {
    float v = i / 2048.0;
    v = powf(v, 3) * 6;
    t_gamma[i] = v * 6 * 256;
  }
}

void
pcl::simulation::SimExample::initializeGL(int argc, char** argv)
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB); // was GLUT_RGBA
  glutInitWindowPosition(10, 10);
  glutInitWindowSize(10, 10);
  // glutInitWindowSize (window_width_, window_height_);
  glutCreateWindow("OpenGL range likelihood");

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

  std::cout << "GL_MAX_VIEWPORTS: " << GL_MAX_VIEWPORTS << std::endl;
  const GLubyte* version = glGetString(GL_VERSION);
  std::cout << "OpenGL Version: " << version << std::endl;
}

void
pcl::simulation::SimExample::doSim(Eigen::Isometry3d pose_in)
{
  // No reference image - but this is kept for compatibility with range_test_v2:
  float* reference = new float[rl_->getRowHeight() * rl_->getColWidth()];
  const float* depth_buffer = rl_->getDepthBuffer();
  // Copy one image from our last as a reference.
  for (int i = 0, n = 0; i < rl_->getRowHeight(); ++i) {
    for (int j = 0; j < rl_->getColWidth(); ++j) {
      reference[n++] = depth_buffer[i * rl_->getWidth() + j];
    }
  }

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
  std::vector<float> scores;
  poses.push_back(pose_in);
  rl_->computeLikelihoods(reference, poses, scores);
  std::cout << "camera: " << camera_->getX() << " " << camera_->getY() << " "
            << camera_->getZ() << " " << camera_->getRoll() << " "
            << camera_->getPitch() << " " << camera_->getYaw() << std::endl;

  delete[] reference;
}

void
pcl::simulation::SimExample::write_score_image(const float* score_buffer,
                                               std::string fname)
{
  int npixels = rl_->getWidth() * rl_->getHeight();
  auto* score_img = new std::uint8_t[npixels * 3];

  float min_score = score_buffer[0];
  float max_score = score_buffer[0];
  for (int i = 1; i < npixels; i++) {
    if (score_buffer[i] < min_score)
      min_score = score_buffer[i];
    if (score_buffer[i] > max_score)
      max_score = score_buffer[i];
  }

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      int i = y * width_ + x;
      int i_in = (height_ - 1 - y) * width_ + x; // flip up

      float d = (score_buffer[i_in] - min_score) / (max_score - min_score);
      score_img[3 * i + 0] = 0;
      score_img[3 * i + 1] = d * 255;
      score_img[3 * i + 2] = 0;
    }
  }

  // Write to file:
  pcl::io::saveRgbPNGFile(fname, score_img, width_, height_);

  delete[] score_img;
}

void
pcl::simulation::SimExample::write_depth_image(const float* depth_buffer,
                                               std::string fname)
{
  int npixels = rl_->getWidth() * rl_->getHeight();
  auto* depth_img = new std::uint8_t[npixels * 3];

  float min_depth = depth_buffer[0];
  float max_depth = depth_buffer[0];
  for (int i = 1; i < npixels; i++) {
    if (depth_buffer[i] < min_depth)
      min_depth = depth_buffer[i];
    if (depth_buffer[i] > max_depth)
      max_depth = depth_buffer[i];
  }

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      int i = y * width_ + x;
      int i_in = (height_ - 1 - y) * width_ + x; // flip up down

      float zn = 0.7;
      float zf = 20.0;
      float d = depth_buffer[i_in];
      float z = -zf * zn / ((zf - zn) * (d - zf / (zf - zn)));
      float b = 0.075;
      float f = 580.0;
      auto kd = static_cast<std::uint16_t>(1090 - b * f / z * 8);
      if (kd > 2047)
        kd = 2047;

      int pval = t_gamma[kd];
      int lb = pval & 0xff;
      switch (pval >> 8) {
      case 0:
        depth_img[3 * i + 0] = 255;
        depth_img[3 * i + 1] = 255 - lb;
        depth_img[3 * i + 2] = 255 - lb;
        break;
      case 1:
        depth_img[3 * i + 0] = 255;
        depth_img[3 * i + 1] = lb;
        depth_img[3 * i + 2] = 0;
        break;
      case 2:
        depth_img[3 * i + 0] = 255 - lb;
        depth_img[3 * i + 1] = 255;
        depth_img[3 * i + 2] = 0;
        break;
      case 3:
        depth_img[3 * i + 0] = 0;
        depth_img[3 * i + 1] = 255;
        depth_img[3 * i + 2] = lb;
        break;
      case 4:
        depth_img[3 * i + 0] = 0;
        depth_img[3 * i + 1] = 255 - lb;
        depth_img[3 * i + 2] = 255;
        break;
      case 5:
        depth_img[3 * i + 0] = 0;
        depth_img[3 * i + 1] = 0;
        depth_img[3 * i + 2] = 255 - lb;
        break;
      default:
        depth_img[3 * i + 0] = 0;
        depth_img[3 * i + 1] = 0;
        depth_img[3 * i + 2] = 0;
        break;
      }
    }
  }

  // Write to file:
  pcl::io::saveRgbPNGFile(fname, depth_img, width_, height_);

  delete[] depth_img;
}

void
pcl::simulation::SimExample::write_depth_image_uint(const float* depth_buffer,
                                                    std::string fname)
{
  int npixels = rl_->getWidth() * rl_->getHeight();
  auto* depth_img = new unsigned short[npixels];

  float min_depth = depth_buffer[0];
  float max_depth = depth_buffer[0];
  for (int i = 1; i < npixels; i++) {
    if (depth_buffer[i] < min_depth)
      min_depth = depth_buffer[i];
    if (depth_buffer[i] > max_depth)
      max_depth = depth_buffer[i];
  }

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      int i = y * width_ + x;
      int i_in = (height_ - 1 - y) * width_ + x; // flip up down

      float zn = 0.7;
      float zf = 20.0;
      float d = depth_buffer[i_in];

      unsigned short z_new = static_cast<unsigned short>(std::min(
          std::floor(1000 * (-zf * zn / ((zf - zn) * (d - zf / (zf - zn))))), 65535.f));

      if (z_new < 18000) {
        std::cout << z_new << " " << d << " " << x << "\n";
      }

      depth_img[i] = z_new;
    }
  }

  // Write to file:
  pcl::io::saveShortPNGFile(fname, depth_img, width_, height_, 1);

  delete[] depth_img;
}

void
pcl::simulation::SimExample::write_rgb_image(const std::uint8_t* rgb_buffer,
                                             std::string fname)
{
  int npixels = rl_->getWidth() * rl_->getHeight();
  auto* rgb_img = new std::uint8_t[npixels * 3];

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      int px = y * width_ + x;
      int px_in = (height_ - 1 - y) * width_ + x; // flip up down
      rgb_img[3 * (px) + 0] = rgb_buffer[3 * px_in + 0];
      rgb_img[3 * (px) + 1] = rgb_buffer[3 * px_in + 1];
      rgb_img[3 * (px) + 2] = rgb_buffer[3 * px_in + 2];
    }
  }

  // Write to file:
  pcl::io::saveRgbPNGFile(fname, rgb_img, width_, height_);

  delete[] rgb_img;
}
