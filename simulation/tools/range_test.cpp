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
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/simulation/camera.h>
#include <pcl/simulation/model.h>
#include <pcl/simulation/scene.h>
#include <pcl/simulation/range_likelihood.h>

using namespace Eigen;
using namespace pcl;
using namespace pcl::simulation;

uint16_t t_gamma[2048];

Scene::Ptr scene_;
Camera::Ptr camera_;
RangeLikelihood::Ptr range_likelihood_;
int window_width_;
int window_height_;
bool paused_;

void apply_camera_transform (const Camera & camera)
{
  glRotatef (rad_to_deg (-camera.roll ()),1.0,0.0,0.0);
  glRotatef (rad_to_deg (-camera.pitch ()),0.0,1.0,0.0);
  glRotatef (rad_to_deg (-camera.yaw ()),0.0,0.0,1.0);
  glTranslatef (-camera.x (), -camera.y (), -camera.z ());
}

void draw_grid(int w, int h, double size)
{
  glBegin(GL_LINES);
  for (int y=0; y<h; ++y) {
      glVertex3f(-size, ((float)y/h-0.5)*2.0 * size, 0.0);
      glVertex3f( size, ((float)y/h-0.5)*2.0 * size, 0.0);
  }

  for (int x=0; x<w; ++x) {
      glVertex3f(((float)x/w-0.5)*2.0 * size, -size, 0.0);
      glVertex3f(((float)x/w-0.5)*2.0 * size,  size, 0.0);
  }
  glEnd();
}

void draw_axis(double x, double y, double z, double yaw, double pitch, double roll, double size)
{
  glPushMatrix();
  glPushAttrib(GL_CURRENT_BIT);

  glTranslatef(x, y, z);

  glRotatef(rad_to_deg(yaw),  0., 0., 1.);
  glRotatef(rad_to_deg(pitch),0., 1., 0.);
  glRotatef(rad_to_deg(roll), 1., 0., 0.);

  glBegin(GL_LINES);
    glColor3f(1.0,0.0,0.0); glVertex3f(0.0,0.0,0.0); glVertex3f(size*1.0,0.0,0.0);
    glColor3f(0.0,1.0,0.0); glVertex3f(0.0,0.0,0.0); glVertex3f(0.0,size*1.0,0.0);
    glColor3f(0.0,0.0,1.0); glVertex3f(0.0,0.0,0.0); glVertex3f(0.0,0.0,size*1.0);
  glEnd();

  glPopAttrib();
  glPopMatrix();
}

void draw_cube()
{
  const float light_position[] = {1.0f, 1.0f, 1.0f};
  const float light_color[] = {1.0f, 1.0f, 1.0f};

  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_color);
  glColor3f(1.0f,0.0f,0.0f);
  glEnable(GL_LIGHT0);
  glutSolidCube(1.0);
  glPopAttrib();
}

void display_depth_image(const float* depth_buffer)
{
  int npixels = range_likelihood_->width() * range_likelihood_->height();
  uint8_t* depth_img = new uint8_t[npixels * 3];

  for (int i=0; i<npixels; i++) {
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

  //  glRasterPos2i(-1,-1);
  //  glDrawPixels(range_likelihood_->width(), range_likelihood_->height(), GL_LUMINANCE, GL_FLOAT, range_likelihood_->depth_buffer());

  glRasterPos2i(-1,-1);
  glDrawPixels(range_likelihood_->width(), range_likelihood_->height(), GL_RGB, GL_UNSIGNED_BYTE, depth_img);

  delete [] depth_img;
}

void display() {
  glViewport(range_likelihood_->width(), 0, range_likelihood_->width(), range_likelihood_->height());


  float* reference = new float[range_likelihood_->row_height() * range_likelihood_->col_width()];

  const float* depth_buffer = range_likelihood_->depth_buffer();
  // Copy one image from our last as a reference.
  for (int i=0, n=0; i<range_likelihood_->row_height(); ++i) {
    for (int j=0; j<range_likelihood_->col_width(); ++j) {
      reference[n++] = depth_buffer[i*range_likelihood_->width() + j];
    }
  }

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;
  std::vector<float> scores;
  int n = range_likelihood_->rows()*range_likelihood_->cols();
  for (int i=0; i<n; ++i) {
    Camera camera(*camera_);
    camera.move(0.0,i*0.02,0.0);
    poses.push_back(camera.pose());
  }
  float* depth_field =NULL;
  bool do_depth_field =false;
  range_likelihood_->compute_likelihoods(reference, poses, scores,depth_field,do_depth_field);
//  range_likelihood_->compute_likelihoods(reference, poses, scores);
  delete [] reference;
  delete [] depth_field;


  std::cout << "score: ";
  for (size_t i=0; i<scores.size(); ++i) {
    std::cout << " " << scores[i];
  }
  std::cout << std::endl;



  // Draw the depth image
//  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//  glColorMask(true, true, true, true);
  glDisable(GL_DEPTH_TEST);
  glViewport(0, 0, range_likelihood_->width(), range_likelihood_->height());
  //glViewport(0, 0, range_likelihood_->width(), range_likelihood_->height());

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  //glRasterPos2i(-1,-1);
  //glDrawPixels(range_likelihood_->width(), range_likelihood_->height(), GL_LUMINANCE, GL_FLOAT, range_likelihood_->depth_buffer());
  display_depth_image(range_likelihood_->depth_buffer());

  glutSwapBuffers();
}

// Handle normal keys
void on_keyboard(unsigned char key, int x, int y)
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

  // Use glutGetModifiers for modifiers
  // GLUT_ACTIVE_SHIFT, GLUT_ACTIVE_CTRL, GLUT_ACTIVE_ALT
}

// Handle special keys, e.g. F1, F2, ...
void on_special(int key, int x, int y)
{
  switch (key) {
  case GLUT_KEY_F1:
    break;
  case GLUT_KEY_HOME:
    break;
  }
}

void on_reshape(int w, int h)
{
  // Window size changed
  window_width_ = w;
  window_height_ = h;
}

void on_mouse(int button, int state, int x, int y)
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

void on_motion(int x, int y)
{
}

void on_passive_motion(int x, int y)
{
  if (paused_) return;

  double pitch = -(0.5-(double)y/window_height_)*M_PI; // in window coordinates positive y-axis is down
  double yaw =    (0.5-(double)x/window_width_)*M_PI*2;

  camera_->set_pitch(pitch);
  camera_->set_yaw(yaw);
}

void on_entry(int state)
{
  // state:
  // GLUT_LEFT
  // GLUT_ENTERED
}

void load_model(const std::vector<std::string> & files)
{
  for (std::vector<std::string>::const_iterator file = files.begin(); file != files.end(); ++file)
  {
    std::cout << "Load model: " << *file << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (*file, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s \n", file->c_str()) ;
      exit (-1);
    }

    PointCloudModel::Ptr model = PointCloudModel::Ptr(new PointCloudModel(GL_POLYGON, cloud));
    scene_->add(model);
  }
}

void initialize(int argc, char** argv)
{
  const GLubyte* version = glGetString(GL_VERSION);
  std::cout << "OpenGL Version: " << version << std::endl;

//  camera_->set(-5.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  camera_->set(4.04454, 44.9377, 1.1, 0.0, 0.0, -2.00352);

  std::vector<std::string> files;
  for (int i=1; i<argc; ++i) files.push_back(argv[i]);

  //load_rwx_models(files);
  load_model(files);

  int i;
  for (i=0; i<2048; i++) {
    float v = i/2048.0;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }

  paused_ = false;
}

int main(int argc, char** argv)
{
  camera_ = Camera::Ptr(new Camera());
  scene_ = Scene::Ptr(new Scene());

  range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(1, 1, 480, 640, scene_, 640));
//  range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(10, 10, 96, 96, scene_));
  //range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(1, 1, 480, 640, scene_));

  window_width_ = range_likelihood_->width() * 2;
  window_height_ = range_likelihood_->height();

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
  glutInitWindowPosition(10,10);
  glutInitWindowSize(window_width_, window_height_);
  glutCreateWindow("OpenGL range likelihood");

  initialize(argc, argv);

  glutReshapeFunc(on_reshape);
  glutDisplayFunc(display);
  glutIdleFunc(display);
  glutKeyboardFunc(on_keyboard);
  glutMouseFunc(on_mouse);
  glutMotionFunc(on_motion);
  glutPassiveMotionFunc(on_passive_motion);
  glutEntryFunc(on_entry);
  glutMainLoop();
}
