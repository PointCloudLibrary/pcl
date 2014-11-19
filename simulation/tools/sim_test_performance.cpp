/**
 * This program performance tests for the range image likelihood library.
 *
 *  Esc - quit
 *
 */

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <boost/shared_ptr.hpp>
#ifdef _WIN32
# define WIN32_LEAN_AND_MEAN
# include <windows.h>
#endif
#include <GL/glew.h>

#include <pcl/pcl_config.h>
#ifdef OPENGL_IS_A_FRAMEWORK
# include <OpenGL/gl.h>
# include <OpenGL/glu.h>
#else
# include <GL/gl.h>
# include <GL/glu.h>
#endif
#ifdef GLUT_IS_A_FRAMEWORK
# include <GLUT/glut.h>
#else
# include <GL/glut.h>
#endif

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/vtk_lib_io.h>

#include <pcl/simulation/camera.h>
#include <pcl/simulation/model.h>
#include <pcl/simulation/scene.h>
#include <pcl/simulation/range_likelihood.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace Eigen;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;
using namespace pcl::simulation;
using namespace std;

uint16_t t_gamma[2048];

Scene::Ptr scene_;
Camera::Ptr camera_;
RangeLikelihood::Ptr range_likelihood_;
// This is only used for displaying
RangeLikelihood::Ptr range_likelihood_visualization_;

int cols_;
int rows_;
int col_width_;
int row_height_;
int window_width_;
int window_height_;
TexturedQuad::Ptr textured_quad_;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s <filename>\n", argv[0]);
  print_info ("acceptable filenames include vtk, obj and ply. ply can support color\n");
}

void
display_score_image (const float* score_buffer)
{
  int npixels = range_likelihood_->getWidth () * range_likelihood_->getHeight ();
  uint8_t* score_img = new uint8_t[npixels * 3];

  float min_score = score_buffer[0];
  float max_score = score_buffer[0];
  for (int i=1; i<npixels; i++)
  {
    if (score_buffer[i] < min_score) min_score = score_buffer[i];
    if (score_buffer[i] > max_score) max_score = score_buffer[i];
  }
  for (int i=0; i < npixels; i++)
  {
    float d = (score_buffer[i]-min_score)/(max_score-min_score);
    score_img[3*i+0] = 0;
    score_img[3*i+1] = static_cast<unsigned char> (d*255);
    score_img[3*i+2] = 0;
  }
  textured_quad_->setTexture (score_img);
  textured_quad_->render ();

  delete [] score_img;
}

void display_depth_image (const float* depth_buffer, int width, int height)
{
  int npixels = width * height;
  uint8_t* depth_img = new uint8_t[npixels * 3];

  float min_depth = depth_buffer[0];
  float max_depth = depth_buffer[0];
  for (int i = 1; i < npixels; ++i)
  {
    if (depth_buffer[i] < min_depth) min_depth = depth_buffer[i];
    if (depth_buffer[i] > max_depth) max_depth = depth_buffer[i];
  }

  for (int i = 0; i < npixels; ++i)
  {
    float zn = 0.7f;
    float zf = 20.0f;
    float d = depth_buffer[i];
    float z = -zf*zn/((zf-zn)*(d - zf/(zf-zn)));
    float b = 0.075f;
    float f = 580.0f;
    int kd = static_cast<int>(1090 - b*f/z*8);
    if (kd < 0) kd = 0;
    else if (kd > 2047) kd = 2047;

    int pval = t_gamma[kd];
    uint8_t lb = static_cast<uint8_t> (pval & 0xff);
    switch (pval >> 8)
    {
      case 0:
	  depth_img[3*i+0] = 255;
	  depth_img[3*i+1] = static_cast<uint8_t> (255-lb);
	  depth_img[3*i+2] = static_cast<uint8_t> (255-lb);
	  break;
      case 1:
	  depth_img[3*i+0] = 255;
	  depth_img[3*i+1] = lb;
	  depth_img[3*i+2] = 0;
	  break;
      case 2:
	  depth_img[3*i+0] = static_cast<uint8_t> (255-lb);
	  depth_img[3*i+1] = 255;
	  depth_img[3*i+2] = 0;
	  break;
      case 3:
	  depth_img[3*i+0] = 0;
	  depth_img[3*i+1] = 255;
	  depth_img[3*i+2] = lb;
	  break;
      case 4:
	  depth_img[3*i+0] = 0;
	  depth_img[3*i+1] = static_cast<uint8_t> (255-lb);
	  depth_img[3*i+2] = 255;
	  break;
      case 5:
	  depth_img[3*i+0] = 0;
	  depth_img[3*i+1] = 0;
	  depth_img[3*i+2] = static_cast<uint8_t> (255-lb);
	  break;
      default:
	  depth_img[3*i+0] = 0;
	  depth_img[3*i+1] = 0;
	  depth_img[3*i+2] = 0;
	  break;
    }
  }

  glRasterPos2i (-1,-1);
  glDrawPixels (width, height,
                GL_RGB, GL_UNSIGNED_BYTE, depth_img);

  delete [] depth_img;
}

void
display ()
{
  float* reference = new float[range_likelihood_->getRowHeight () * range_likelihood_->getColWidth ()];
  const float* depth_buffer = range_likelihood_->getDepthBuffer ();
  // Copy one image from our last as a reference.
  for (int i = 0, n = 0; i < range_likelihood_->getRowHeight (); ++i)
  {
    for (int j = 0; j < range_likelihood_->getColWidth (); ++j)
    {
      reference[n++] = depth_buffer[ (i + range_likelihood_->getRowHeight () * range_likelihood_->getRows () / 2) * range_likelihood_->getWidth ()
                                    + j + range_likelihood_->getColWidth () * range_likelihood_->getCols () / 2];
    }
  }

  float* reference_vis = new float[range_likelihood_visualization_->getRowHeight () * range_likelihood_visualization_->getColWidth ()];
  const float* depth_buffer_vis = range_likelihood_visualization_->getDepthBuffer ();
  // Copy one image from our last as a reference.
  for (int i = 0, n = 0; i < range_likelihood_visualization_->getRowHeight (); ++i)
  {
    for (int j = 0; j < range_likelihood_visualization_->getColWidth (); ++j)
    {
      reference_vis[n++] = depth_buffer_vis[i*range_likelihood_visualization_->getWidth () + j];
    }
  }

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;
  std::vector<float> scores;

  // Render a single pose for visualization
  poses.clear ();
  poses.push_back (camera_->getPose ());
  range_likelihood_visualization_->computeLikelihoods (reference_vis, poses, scores);

  glDrawBuffer (GL_BACK);
  glReadBuffer (GL_BACK);

  // Draw the resulting images from the range_likelihood
  glViewport (range_likelihood_visualization_->getWidth (), 0,
              range_likelihood_visualization_->getWidth (), range_likelihood_visualization_->getHeight ());
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();

  // Draw the color image
  glColorMask (true, true, true, true);
  glClearColor (0, 0, 0, 0);
  glClearDepth (1);
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glDisable (GL_DEPTH_TEST);

  glRasterPos2i (-1,-1);
  glDrawPixels (range_likelihood_visualization_->getWidth (), range_likelihood_visualization_->getHeight (),
                GL_RGB, GL_UNSIGNED_BYTE, range_likelihood_visualization_->getColorBuffer ());

  // Draw the depth image
  glViewport (0, 0, range_likelihood_visualization_->getWidth (), range_likelihood_visualization_->getHeight ());

  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();
  display_depth_image (range_likelihood_visualization_->getDepthBuffer (),
                       range_likelihood_visualization_->getWidth (), range_likelihood_visualization_->getHeight ());


  poses.clear ();
  for (int i = 0; i < range_likelihood_->getRows (); ++i)
  {
    for (int j = 0; j < range_likelihood_->getCols (); ++j)
    {
      Camera camera (*camera_);
      camera.move ((j - range_likelihood_->getCols () / 2.0) * 0.1,
                   (i - range_likelihood_->getRows () / 2.0) * 0.1,
                   0.0);
      poses.push_back (camera.getPose ());
    }
  }
  std::cout << std::endl;

  TicToc tt;
  tt.tic();
  range_likelihood_->computeLikelihoods (reference, poses, scores);
  tt.toc();
  tt.toc_print();

  if (gllib::getGLError () != GL_NO_ERROR)
  {
    std::cerr << "GL Error: RangeLikelihood::computeLikelihoods: finished" << std::endl;
  }

#if 0
  std::cout << "score: ";
  for (size_t i = 0; i < scores.size (); ++i)
  {
    std::cout << " " << scores[i];
  }
  std::cout << std::endl;
#endif

  std::cout << "camera: " << camera_->getX ()
       << " " << camera_->getY ()
       << " " << camera_->getZ ()
       << " " << camera_->getRoll ()
       << " " << camera_->getPitch ()
       << " " << camera_->getYaw ()
       << std::endl;

  delete [] reference_vis;
  delete [] reference;

  if (gllib::getGLError () != GL_NO_ERROR)
  {
    std::cerr << "GL Error: before buffers" << std::endl;
  }

  glBindFramebuffer (GL_FRAMEBUFFER, 0);
  glDrawBuffer (GL_BACK);
  glReadBuffer (GL_BACK);

  if (gllib::getGLError () != GL_NO_ERROR)
  {
    std::cerr << "GL Error: after buffers" << std::endl;
  }


  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();

  if (gllib::getGLError () != GL_NO_ERROR)
  {
    std::cerr << "GL Error: before viewport" << std::endl;
  }

  // Draw the score image for the particles
  glViewport (0, range_likelihood_visualization_->getHeight (),
              range_likelihood_visualization_->getWidth (), range_likelihood_visualization_->getHeight ());


  if (gllib::getGLError () != GL_NO_ERROR)
  {
    std::cerr << "GL Error: after viewport" << std::endl;
  }

  display_score_image (range_likelihood_->getScoreBuffer ());

  // Draw the depth image for the particles
  glViewport (range_likelihood_visualization_->getWidth (), range_likelihood_visualization_->getHeight (),
              range_likelihood_visualization_->getWidth (), range_likelihood_visualization_->getHeight ());

  display_score_image (range_likelihood_->getDepthBuffer ());

  glutSwapBuffers ();
}

// Handle normal keys
void
on_keyboard (unsigned char key, int, int)
{
  if (key == 27)
    exit (0);
}

// Read in a 3D model
void
loadPolygonMeshModel (char* polygon_file)
{
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile (polygon_file, mesh);
  pcl::PolygonMesh::Ptr cloud (new pcl::PolygonMesh (mesh));
  
  TriangleMeshModel::Ptr model = TriangleMeshModel::Ptr (new TriangleMeshModel (cloud));
  scene_->add (model);
  
  std::cout << "Just read " << polygon_file << std::endl;
  std::cout << mesh.polygons.size () << " polygons and "
	    << mesh.cloud.data.size () << " triangles\n";
}

void
initialize (int argc, char** argv)
{
  const GLubyte* version = glGetString (GL_VERSION);
  print_info ("OpenGL Version: %s\n", version);

  // works well for MIT CSAIL model 2nd floor:
  camera_->set (27.4503, 37.383, 4.30908, 0.0, 0.0654498, -2.25802);

  if (argc > 1) loadPolygonMeshModel (argv[1]);
}

int
main (int argc, char** argv)
{
  int width = 640;
  int height = 480;

  window_width_ = width * 2;
  window_height_ = height * 2;

  int cols = 30;
  int rows = 30;
  int col_width = 20;
  int row_height = 15;
  
  print_info ("Range likelihood performance tests using pcl::simulation. For more information, use: %s -h\n", argv[0]);

  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }  
  
  for (int i = 0; i < 2048; ++i)
  {
    float v = static_cast<float> (i / 2048.0);
    v = powf(v, 3)* 6;
    t_gamma[i] = static_cast<uint16_t> (v*6*256);
  }  

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
  glutInitWindowPosition (10, 10);
  glutInitWindowSize (window_width_, window_height_);
  glutCreateWindow ("OpenGL range likelihood");

  GLenum err = glewInit ();
  if (GLEW_OK != err)
  {
    std::cerr << "Error: " << glewGetErrorString (err) << std::endl;
    exit (-1);
  }

  std::cout << "Status: Using GLEW " << glewGetString (GLEW_VERSION) << std::endl;

  if (glewIsSupported ("GL_VERSION_2_0"))
    std::cout << "OpenGL 2.0 supported" << std::endl;
  else
  {
    std::cerr << "Error: OpenGL 2.0 not supported" << std::endl;
    exit (1);
  }

  std::cout << "GL_MAX_VIEWPORTS: " << GL_MAX_VIEWPORTS << std::endl;

  camera_ = Camera::Ptr (new Camera ());
  scene_ = Scene::Ptr (new Scene ());

  range_likelihood_visualization_ = RangeLikelihood::Ptr (new RangeLikelihood (1, 1, height, width, scene_));
  range_likelihood_ = RangeLikelihood::Ptr (new RangeLikelihood (rows, cols, row_height, col_width, scene_));

  // Actually corresponds to default parameters:
  range_likelihood_visualization_->setCameraIntrinsicsParameters (
      640, 480, 576.09757860f, 576.09757860f, 321.06398107f, 242.97676897f);
  range_likelihood_visualization_->setComputeOnCPU (false);
  range_likelihood_visualization_->setSumOnCPU (false);
  range_likelihood_visualization_->setUseColor (true);

  range_likelihood_->setCameraIntrinsicsParameters (
      640, 480, 576.09757860f, 576.09757860f, 321.06398107f, 242.97676897f);
  range_likelihood_->setComputeOnCPU (false);
  range_likelihood_->setSumOnCPU (false);
  range_likelihood_->setUseColor (false);

  textured_quad_ = TexturedQuad::Ptr (new TexturedQuad (range_likelihood_->getWidth (),
                                                        range_likelihood_->getHeight ()));

  initialize (argc, argv);

  glutDisplayFunc (display);
  glutIdleFunc (display);
  glutKeyboardFunc (on_keyboard);
  glutMainLoop ();
}
