#pragma once

#include <boost/shared_ptr.hpp>

#include <GL/glew.h>

#include <pcl/pcl_config.h>
#ifdef OPENGL_IS_A_FRAMEWORK
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#ifdef GLUT_IS_A_FRAMEWORK
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>

#include <pcl/simulation/camera.h>
#include <pcl/simulation/range_likelihood.h>
#include <pcl/simulation/scene.h>

namespace pcl {
namespace simulation {

class PCL_EXPORTS SimExample {
public:
  using Ptr = shared_ptr<SimExample>;
  using ConstPtr = shared_ptr<const SimExample>;

  SimExample(int argc, char** argv, int height, int width);
  void
  initializeGL(int argc, char** argv);

  Scene::Ptr scene_;
  Camera::Ptr camera_;
  RangeLikelihood::Ptr rl_;

  void
  doSim(Eigen::Isometry3d pose_in);

  void
  write_score_image(const float* score_buffer, std::string fname);

  void
  write_depth_image(const float* depth_buffer, std::string fname);

  void
  write_depth_image_uint(const float* depth_buffer, std::string fname);

  void
  write_rgb_image(const std::uint8_t* rgb_buffer, std::string fname);

private:
  std::uint16_t t_gamma[2048];

  // of platter, usually 640x480
  int height_;
  int width_;
};

} // namespace simulation
} // namespace pcl
