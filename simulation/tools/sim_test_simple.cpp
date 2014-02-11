/**
 * This program tests the range image likelihood library.
 *
 * You can move around using
 *  w - forward
 *  a - backward
 *  s - left
 *  d - right
 *  q - up
 *  z - down
 *  f - write point cloud file
 * 
 * The mouse controls the direction of movement
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

#include "pcl/common/common.h"
#include "pcl/common/transforms.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/vtk_lib_io.h>

#include "pcl/simulation/camera.h"
#include "pcl/simulation/model.h"
#include "pcl/simulation/scene.h"
#include "pcl/simulation/range_likelihood.h"

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

// RangeImage:
#include <pcl/range_image/range_image_planar.h>

// Pop-up viewer
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

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

int window_width_;
int window_height_;
bool paused_;
bool write_file_;

void printHelp (int argc, char **argv)
{
  print_error ("Syntax is: %s <filename>\n", argv[0]);
  print_info ("acceptable filenames include vtk, obj and ply. ply can support colour\n");
}

void wait ()
{
      std::cout << "Press enter to continue";
      getchar();
      std::cout << "\n\n";
}

void display_score_image(const float* score_buffer)
{
  int npixels = range_likelihood_->getWidth() * range_likelihood_->getHeight();
  uint8_t* score_img = new uint8_t[npixels * 3];

  float min_score = score_buffer[0];
  float max_score = score_buffer[0];
  for (int i=1; i<npixels; i++)
  {
    if (score_buffer[i] < min_score) min_score = score_buffer[i];
    if (score_buffer[i] > max_score) max_score = score_buffer[i];
  }

  for (int i=0; i<npixels; i++)
  {
    float d = (score_buffer[i]-min_score)/(max_score-min_score);
    score_img[3*i+0] = 0;
    score_img[3*i+1] = d*255;
    score_img[3*i+2] = 0;
  }

  glRasterPos2i(-1,-1);
  glDrawPixels(range_likelihood_->getWidth(), range_likelihood_->getHeight(), GL_RGB, GL_UNSIGNED_BYTE, score_img);

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
    uint16_t kd = static_cast<uint16_t>(1090 - b*f/z*8);
    if (kd < 0) kd = 0;
    else if (kd > 2047) kd = 2047;

    int pval = t_gamma[kd];
    int lb = pval & 0xff;
    switch (pval >> 8)
    {
      case 0:
	  depth_img[3*i+0] = 255;
	  depth_img[3*i+1] = 255-lb;
	  depth_img[3*i+2] = 255-lb;
	  break;
      case 1:
	  depth_img[3*i+0] = 255;
	  depth_img[3*i+1] = lb;
	  depth_img[3*i+2] = 0;
	  break;
      case 2:
	  depth_img[3*i+0] = 255-lb;
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
	  depth_img[3*i+1] = 255-lb;
	  depth_img[3*i+2] = 255;
	  break;
      case 5:
	  depth_img[3*i+0] = 0;
	  depth_img[3*i+1] = 0;
	  depth_img[3*i+2] = 255-lb;
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

/*
void display_depth_image(const float* depth_buffer)
{
  int npixels = range_likelihood_->getWidth() * range_likelihood_->getHeight();
  uint8_t* depth_img = new uint8_t[npixels * 3];

  float min_depth = depth_buffer[0];
  float max_depth = depth_buffer[0];
  for (int i=1; i<npixels; i++)
  {
    if (depth_buffer[i] < min_depth) min_depth = depth_buffer[i];
    if (depth_buffer[i] > max_depth) max_depth = depth_buffer[i];
  }

  for (int i=0; i<npixels; i++)
  {
    float zn = 0.7;
    float zf = 20.0;
    float d = depth_buffer[i];
    float z = -zf*zn/((zf-zn)*(d - zf/(zf-zn)));
    float b = 0.075;
    float f = 580.0;
    uint16_t kd = static_cast<uint16_t>(1090 - b*f/z*8);
    if (kd < 0) kd = 0;
    else if (kd>2047) kd = 2047;

    int pval = t_gamma[kd];
    int lb = pval & 0xff;
    switch (pval>>8) {
      case 0:
	  depth_img[3*i+0] = 255;
	  depth_img[3*i+1] = 255-lb;
	  depth_img[3*i+2] = 255-lb;
	  break;
      case 1:
	  depth_img[3*i+0] = 255;
	  depth_img[3*i+1] = lb;
	  depth_img[3*i+2] = 0;
	  break;
      case 2:
	  depth_img[3*i+0] = 255-lb;
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
	  depth_img[3*i+1] = 255-lb;
	  depth_img[3*i+2] = 255;
	  break;
      case 5:
	  depth_img[3*i+0] = 0;
	  depth_img[3*i+1] = 0;
	  depth_img[3*i+2] = 255-lb;
	  break;
      default:
	  depth_img[3*i+0] = 0;
	  depth_img[3*i+1] = 0;
	  depth_img[3*i+2] = 0;
	  break;
    }
  }

  glRasterPos2i(-1,-1);
  glDrawPixels(range_likelihood_->getWidth(), range_likelihood_->getHeight(), GL_RGB, GL_UNSIGNED_BYTE, depth_img);

  delete [] depth_img;
}
*/

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0, "reference");
  viewer->initCameraParameters ();
  return (viewer);
}


void display ()
{
  float* reference = new float[range_likelihood_->getRowHeight() * range_likelihood_->getColWidth()];
  const float* depth_buffer = range_likelihood_->getDepthBuffer();
  // Copy one image from our last as a reference.
  for (int i=0, n=0; i<range_likelihood_->getRowHeight(); ++i)
  {
    for (int j=0; j<range_likelihood_->getColWidth(); ++j)
    {
      reference[n++] = depth_buffer[i*range_likelihood_->getWidth() + j];
    }
  }

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;
  std::vector<float> scores;
  int n = range_likelihood_->getRows ()*range_likelihood_->getCols ();
  for (int i = 0; i < n; ++i)
  {
    Camera camera(*camera_);
    camera.move(0.0,i*0.02,0.0);
    //camera.move(0.0,i*0.02,0.0);
    poses.push_back (camera.getPose ());
  }
  
  range_likelihood_->computeLikelihoods (reference, poses, scores);
  
  range_likelihood_->computeLikelihoods (reference, poses, scores);
  std::cout << "score: ";
  for (size_t i = 0; i<scores.size (); ++i)
  {
    std::cout << " " << scores[i];
  }
  std::cout << std::endl;

  std::cout << "camera: " << camera_->getX ()
       << " " << camera_->getY ()
       << " " << camera_->getZ ()
       << " " << camera_->getRoll ()
       << " " << camera_->getPitch ()
       << " " << camera_->getYaw ()
       << std::endl;

  delete [] reference;

  glDrawBuffer (GL_BACK);
  glReadBuffer (GL_BACK);

  // Draw the resulting images from the range_likelihood
  glViewport (range_likelihood_->getWidth (), 0, range_likelihood_->getWidth (), range_likelihood_->getHeight ());
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();

  // Draw the color image
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColorMask (true, true, true, true);
  glDisable (GL_DEPTH_TEST);

  glRasterPos2i (-1,-1);
  glDrawPixels (range_likelihood_->getWidth (), range_likelihood_->getHeight (),
                GL_RGB, GL_UNSIGNED_BYTE, range_likelihood_->getColorBuffer ());

  // Draw the depth image
  glViewport (0, 0, range_likelihood_->getWidth (), range_likelihood_->getHeight ());

  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();
//  display_depth_image (range_likelihood_->getDepthBuffer ());
  display_depth_image (range_likelihood_->getDepthBuffer (),
                       range_likelihood_->getWidth (), range_likelihood_->getHeight ());
  
  
  // Draw the score image
  glViewport (0, range_likelihood_->getHeight (),
              range_likelihood_->getWidth (), range_likelihood_->getHeight ());
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();
  display_score_image (range_likelihood_->getScoreBuffer ());

  glutSwapBuffers ();
  
  if (write_file_)
  {
    range_likelihood_->addNoise ();
    pcl::RangeImagePlanar rangeImage;
    range_likelihood_->getRangeImagePlanar (rangeImage);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_out (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Optional argument to save point cloud in global frame:
    // Save camera relative:
    //range_likelihood_->getPointCloud(pc_out);
    // Save in global frame - applying the camera frame:
    //range_likelihood_->getPointCloud(pc_out,true,camera_->pose());
    // Save in local frame
    range_likelihood_->getPointCloud (pc_out,false,camera_->getPose ());
    // TODO: what to do when there are more than one simulated view?
    
    pcl::PCDWriter writer;
    writer.write ("simulated_range_image.pcd", *pc_out,	false);  
    cout << "finished writing file\n";
    
//     pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//     viewer.showCloud (pc_out);


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = simpleVis(pc_out);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }    
  
  // doesnt work:
//    viewer->~PCLVisualizer();
//    viewer.reset();
    
    
    cout << "done\n";
    // Problem: vtk and opengl dont seem to play very well together
    // vtk seems to misbehave after a little while and wont keep the window on the screen

    // method1: kill with [x] - but eventually it crashes:
    //while (!viewer.wasStopped ()){
    //}    
    
    // method2: eventually starts ignoring cin and pops up on screen and closes almost 
    // immediately
    //  cout << "enter 1 to cont\n";
    //  cin >> pause;
    //  viewer.wasStopped ();
    
    // method 3: if you interact with the window with keys, the window is not closed properly
    // TODO: use pcl methods as this time stuff is probably not cross playform
//     struct timespec t;
//     t.tv_sec = 100;
//     //t.tv_nsec = (time_t)(20000000); // short sleep
//     t.tv_nsec = (time_t)(0);  // long sleep - normal speed
//     nanosleep (&t, NULL);
    write_file_ = 0;
  }
}

// Handle normal keys
void
on_keyboard (unsigned char key, int x, int y)
{
  double speed = 0.1;

  if (key == 27)
    exit(0);
  else if (key == 'w' || key == 'W')
    camera_->move(speed,0,0);
  else if (key == 's' || key == 'S')
    camera_->move(-speed,0,0);
  else if (key == 'a' || key == 'A')
    camera_->move(0,speed,0);
  else if (key == 'd' || key == 'D')
    camera_->move(0,-speed,0);
  else if (key == 'q' || key == 'Q')
    camera_->move(0,0,speed);
  else if (key == 'z' || key == 'Z')
    camera_->move(0,0,-speed);
  else if (key == 'p' || key == 'P')
    paused_ = !paused_;
  else if (key == 'v' || key == 'V')
    write_file_ = 1;
  
  // Use glutGetModifiers for modifiers
  // GLUT_ACTIVE_SHIFT, GLUT_ACTIVE_CTRL, GLUT_ACTIVE_ALT
}

// Handle special keys, e.g. F1, F2, ...
void
on_special(int key, int x, int y)
{
  switch (key) {
  case GLUT_KEY_F1:
    break;
  case GLUT_KEY_HOME:
    break;
  }
}

void
on_reshape(int w, int h)
{
  // Window size changed
  window_width_ = w;
  window_height_ = h;
}

void
on_mouse(int button, int state, int x, int y)
{
  // button:
  // GLUT_LEFT_BUTTON
  // GLUT_MIDDLE_BUTTON
  // GLUT_RIGHT_BUTTON
  //
  // state:
  // GLUT_UP
  // GLUT_DOWN
}

void
on_motion(int x, int y)
{
}

void
on_passive_motion(int x, int y)
{
  if (paused_) return;

  double pitch = -(0.5-(double)y/window_height_)*M_PI * 4; // in window coordinates positive y-axis is down
  double yaw =    (0.5-(double)x/window_width_)*M_PI*2 * 4;

  camera_->setPitch (pitch);
  camera_->setYaw (yaw);
}

void on_entry (int state)
{
  // state:
  // GLUT_LEFT
  // GLUT_ENTERED
}


// Read in a 3D model
void load_PolygonMesh_model (char* polygon_file)
{
  pcl::PolygonMesh mesh;	// (new pcl::PolygonMesh);
  //pcl::io::loadPolygonFile("/home/mfallon/data/models/dalet/Darlek_modified_works.obj",mesh);
  pcl::io::loadPolygonFile (polygon_file, mesh);
  pcl::PolygonMesh::Ptr cloud (new pcl::PolygonMesh (mesh));
  
  // Not sure if PolygonMesh assumes triangles if to
  // TODO: Ask a developer
  PolygonMeshModel::Ptr model = PolygonMeshModel::Ptr (new PolygonMeshModel (GL_POLYGON, cloud));
  scene_->add (model);
  
  std::cout << "Just read " << polygon_file << std::endl;
  std::cout << mesh.polygons.size () << " polygons and "
	    << mesh.cloud.data.size () << " triangles\n";
  
}

void
initialize (int argc, char** argv)
{
  const GLubyte* version = glGetString (GL_VERSION);
  std::cout << "OpenGL Version: " << version << std::endl;

  // works well for MIT CSAIL model 3rd floor:
  //camera_->set(4.04454, 44.9377, 1.1, 0.0, 0.0, -2.00352);

  // works well for MIT CSAIL model 2nd floor:
//  camera_->set (27.4503, 37.383, 4.30908, 0.0, 0.0654498, -2.25802);

  // works for small files:
  //camera_->set(-5.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  camera_->set( 1.31762, 0.382931, 1.89533, 0, 0.20944, -9.14989);
  camera_->setPitch(0.20944); // not sure why this is here: 
  //camera_->set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  
  cout << "About to read: " << argv[1] << endl;
  load_PolygonMesh_model (argv[1]);
    
  paused_ = false;
}

int
main (int argc, char** argv)
{
  int width = 640;
  int height = 480;

  window_width_ = width * 2;
  window_height_ = height * 2;

  print_info ("Manually generate a simulated RGB-D point cloud using pcl::simulation. For more information, use: %s -h\n", argv[0]);

  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }  
  
  int i;
  for (i=0; i<2048; i++)
  {
    float v = i/2048.0;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }  

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);// was GLUT_RGBA
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
    exit(1);
  }
  std::cout << "GL_MAX_VIEWPORTS: " << GL_MAX_VIEWPORTS << std::endl;

  camera_ = Camera::Ptr (new Camera ());
  scene_ = Scene::Ptr (new Scene ());

  //range_likelihood_ = RangeLikelihoodGLSL::Ptr(new RangeLikelihoodGLSL(1, 1, height, width, scene_, 0));

  range_likelihood_ = RangeLikelihood::Ptr (new RangeLikelihood (2, 2, height/2, width/2, scene_));
  // range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(10, 10, 96, 96, scene_));
  // range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(1, 1, 480, 640, scene_));

  // Actually corresponds to default parameters:
  range_likelihood_->setCameraIntrinsicsParameters (640,480, 576.09757860,
            576.09757860, 321.06398107, 242.97676897);
  range_likelihood_->setComputeOnCPU (false);
  range_likelihood_->setSumOnCPU (true);
  range_likelihood_->setUseColor (true);
  
  initialize (argc, argv);

  glutReshapeFunc (on_reshape);
  glutDisplayFunc (display);
  glutIdleFunc (display);
  glutKeyboardFunc (on_keyboard);
  glutMouseFunc (on_mouse);
  glutMotionFunc (on_motion);
  glutPassiveMotionFunc (on_passive_motion);
  glutEntryFunc (on_entry);
  glutMainLoop ();

  return 0;
}
