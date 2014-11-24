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
#include <Eigen/Geometry>
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
#else
# include <GL/gl.h>
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


#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <cfloat>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <vtkPolyDataReader.h>
#include <pcl/visualization/keyboard_event.h>

using namespace Eigen;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;
using namespace pcl::simulation;

using namespace std;

typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

typedef pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2> GeometryHandler;
typedef GeometryHandler::Ptr GeometryHandlerPtr;
typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;

#define NORMALS_SCALE 0.01
#define PC_SCALE 0.001

uint16_t t_gamma[2048];
Scene::Ptr scene_;
Camera::Ptr camera_;
RangeLikelihood::Ptr range_likelihood_;
int window_width_;
int window_height_;
bool paused_;
bool write_file_;

bool
isValidFieldName (const std::string &field)
{
  if (field == "_")
    return (false);

  if ((field == "vp_x") || (field == "vx") || (field == "vp_y") || (field == "vy") || (field == "vp_z") || (field == "vz"))
    return (false);
  return (true);
}

bool
isMultiDimensionalFeatureField (const pcl::PCLPointField &field)
{
  if (field.count > 1)
    return (true);
  return (false);
}

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s <file_name 1..N>.<pcd or vtk> <options>\n", argv[0]);
  print_info ("pcl::simulation viewer\n");  
  print_info ("  where options are:\n");
  print_info ("                     -bc r,g,b                = background color\n");
  print_info ("                     -fc r,g,b                = foreground color\n");
  print_info ("                     -ps X                    = point size ("); print_value ("1..64"); print_info (") \n");
  print_info ("                     -opaque X                = rendered point cloud opacity ("); print_value ("0..1"); print_info (")\n");

  print_info ("                     -ax "); print_value ("n"); print_info ("                    = enable on-screen display of ");
  print_color (stdout, TT_BRIGHT, TT_RED, "X"); print_color (stdout, TT_BRIGHT, TT_GREEN, "Y"); print_color (stdout, TT_BRIGHT, TT_BLUE, "Z");
  print_info (" axes and scale them to "); print_value ("n\n");
  print_info ("                     -ax_pos X,Y,Z            = if axes are enabled, set their X,Y,Z position in space (default "); print_value ("0,0,0"); print_info (")\n");

  print_info ("\n");
  print_info ("                     -cam (*)                 = use given camera settings as initial view\n");
  print_info (stderr, " (*) [Clipping Range / Focal Point / Position / ViewUp / Distance / Field of View Y / Window Size / Window Pos] or use a <filename.cam> that contains the same information.\n");

  print_info ("\n");
  print_info ("                     -multiview 0/1           = enable/disable auto-multi viewport rendering (default "); print_value ("disabled"); print_info (")\n");
  print_info ("\n");

  print_info ("\n");
  print_info ("                     -normals 0/X             = disable/enable the display of every Xth point's surface normal as lines (default "); print_value ("disabled"); print_info (")\n");
  print_info ("                     -normals_scale X         = resize the normal unit vector size to X (default "); print_value ("0.02"); print_info (")\n");
  print_info ("\n");
  print_info ("                     -pc 0/X                  = disable/enable the display of every Xth point's principal curvatures as lines (default "); print_value ("disabled"); print_info (")\n");
  print_info ("                     -pc_scale X              = resize the principal curvatures vectors size to X (default "); print_value ("0.02"); print_info (")\n");
  print_info ("\n");

  print_info ("\n(Note: for multiple .pcd files, provide multiple -{fc,ps,opaque} parameters; they will be automatically assigned to the right file)\n");
}

// Global visualizer object
pcl::visualization::PCLHistogramVisualizer ph_global;
boost::shared_ptr<pcl::visualization::PCLVisualizer> p;

void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* cookie)
{
  if (event.getPointIndex () == -1)
    return;
  pcl::PCLPointCloud2::Ptr cloud = *static_cast<pcl::PCLPointCloud2::Ptr*>(cookie);
  if (!cloud)
    return;

  // If two points were selected, draw an arrow between them
  pcl::PointXYZ p1, p2;
  if (event.getPoints (p1.x, p1.y, p1.z, p2.x, p2.y, p2.z) && p)
  {
    std::stringstream ss;
    ss << p1 << p2;
    p->addArrow<pcl::PointXYZ, pcl::PointXYZ> (p1, p2, 1.0, 1.0, 1.0, ss.str ());
    return;
  }

  // Else, if a single point has been selected
  std::stringstream ss;
  ss << event.getPointIndex ();
  // Get the cloud's fields
  for (size_t i = 0; i < cloud->fields.size (); ++i)
  {
    if (!isMultiDimensionalFeatureField (cloud->fields[i]))
      continue;
    ph_global.addFeatureHistogram (*cloud, cloud->fields[i].name, event.getPointIndex (), ss.str ());
  }
  if (p)
  {
    pcl::PointXYZ pos;
    event.getPoint (pos.x, pos.y, pos.z);
    p->addText3D<pcl::PointXYZ> (ss.str (), pos, 0.0005, 1.0, 1.0, 1.0, ss.str ());
  }
  ph_global.spinOnce ();
}


/*
void write_score_image(const float* score_buffer)
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

  for (int y = 0; y <  480; ++y)
  {
    for (int x = 0; x < 640; ++x)
    {
      int i = y*640 + x ;
      int i_in= (480-1 -y) *640 + x ; // flip up

      float d = (score_buffer[i_in]-min_score)/(max_score-min_score);
      score_img[3*i+0] = 0;
      score_img[3*i+1] = d*255;
      score_img[3*i+2] = 0;
    }
  }

  // Write to file:
  IplImage *cv_ipl = cvCreateImage( cvSize(640 ,480), 8, 3);
  cv::Mat cv_mat(cv_ipl);
  cv_mat.data = score_img;
  cv::imwrite("score_image.png", cv_mat);     
  
  delete [] score_img;
}

void write_depth_image(const float* depth_buffer)
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

  for (int y = 0; y <  480; ++y)
  {
    for (int x = 0; x < 640; ++x)
    {
      int i= y*640 + x ;
      int i_in= (480-1 -y) *640 + x ; // flip up down
    
    
      float zn = 0.7;
      float zf = 20.0;
      float d = depth_buffer[i_in];
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
	    depth_img[3*i+2] = 255;
	    depth_img[3*i+1] = 255-lb;
	    depth_img[3*i+0] = 255-lb;
	    break;
	case 1:
	    depth_img[3*i+2] = 255;
	    depth_img[3*i+1] = lb;
	    depth_img[3*i+0] = 0;
	    break;
	case 2:
	    depth_img[3*i+2] = 255-lb;
	    depth_img[3*i+1] = 255;
	    depth_img[3*i+0] = 0;
	    break;
	case 3:
	    depth_img[3*i+2] = 0;
	    depth_img[3*i+1] = 255;
	    depth_img[3*i+0] = lb;
	    break;
	case 4:
	    depth_img[3*i+2] = 0;
	    depth_img[3*i+1] = 255-lb;
	    depth_img[3*i+0] = 255;
	    break;
	case 5:
	    depth_img[3*i+2] = 0;
	    depth_img[3*i+1] = 0;
	    depth_img[3*i+0] = 255-lb;
	    break;
	default:
	    depth_img[3*i+2] = 0;
	    depth_img[3*i+1] = 0;
	    depth_img[3*i+0] = 0;
	    break;
      }
    }
  }

  // Write to file:
  IplImage *cv_ipl = cvCreateImage( cvSize(640 ,480), 8, 3);
  cv::Mat cv_mat(cv_ipl);
  cv_mat.data = depth_img;
  cv::imwrite("depth_image.png", cv_mat);     
  
  delete [] depth_img;
}


void write_rgb_image(const uint8_t* rgb_buffer)
{
  int npixels = range_likelihood_->getWidth() * range_likelihood_->getHeight();
  uint8_t* rgb_img = new uint8_t[npixels * 3];

  for (int y = 0; y <  480; ++y)
  {
    for (int x = 0; x < 640; ++x)
    {
      int px = y*640 + x ;
      rgb_img [3* (px) +0] = rgb_buffer[3*px+0];
      rgb_img [3* (px) +1] = rgb_buffer[3*px+1];
      rgb_img [3* (px) +2] = rgb_buffer[3*px+2];      
    }
  }  
  
  // Write to file:
  IplImage *cv_ipl = cvCreateImage( cvSize(640 ,480), 8, 3);
  cv::Mat cv_mat(cv_ipl);
  cv_mat.data = rgb_img ;
  cv::imwrite("rgb_image.png", cv_mat);     
  
  delete [] rgb_img;
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // Snippet taken from PCLVisualizer tutorial:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  
  //viewer->addModelFromPLYFile("/home/mfallon/projects/kmcl/kmcl/models/table_scene/meta_model.ply");
  
  
  return (viewer);
}
*/

void capture (Eigen::Isometry3d pose_in, string point_cloud_fname)
{
  // No reference image - but this is kept for compatability with range_test_v2:
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
  poses.push_back (pose_in);

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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_out (new pcl::PointCloud<pcl::PointXYZRGB>);
  bool write_cloud = true;
  
  if (write_cloud)
  {
    // Read Color Buffer from the GPU before creating PointCloud:
    // By default the buffers are not read back from the GPU
    range_likelihood_->getColorBuffer ();
    range_likelihood_->getDepthBuffer ();  
    
    // Add noise directly to the CPU depth buffer 
    range_likelihood_->addNoise ();

    // Optional argument to save point cloud in global frame:
    // Save camera relative:
    //range_likelihood_->getPointCloud(pc_out);
    // Save in global frame - applying the camera frame:
    //range_likelihood_->getPointCloud(pc_out,true,camera_->pose());
    // Save in local frame
    range_likelihood_->getPointCloud (pc_out,false,camera_->getPose ());
    // TODO: what to do when there are more than one simulated view?
    std::cout << pc_out->points.size() << " points written to file\n";
   
    pcl::PCDWriter writer;
    //writer.write (point_cloud_fname, *pc_out, false);  /// ASCII
    writer.writeBinary (point_cloud_fname, *pc_out);
    //cout << "finished writing file\n";
  }
  // Disabled all OpenCV stuff for now: dont want the dependency
  /*
  bool demo_other_stuff = false;
  if (demo_other_stuff && write_cloud)
  {
    write_score_image (range_likelihood_->getScoreBuffer ());  
    write_rgb_image (range_likelihood_->getColorBuffer ());  
    write_depth_image (range_likelihood_->getDepthBuffer ());  
    
    // Demo interacton with RangeImage:
    pcl::RangeImagePlanar rangeImage;
    range_likelihood_->getRangeImagePlanar (rangeImage);
 
    // display viewer: (currently seqfaults on exit of viewer)
    if (1==0){
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
      viewer = simpleVis(pc_out);
    
      while (!viewer->wasStopped ()){
	viewer->spinOnce (100);
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
    }
  }
  */
}






void print_Quaterniond(Eigen::Quaterniond r, std::stringstream &ss){
  ss <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() ;
  //  std::cout << r.str() << "q\n";
}


// Normalize angle to be within the interval [-pi,pi].
double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+M_PI, 2*M_PI) - M_PI;
  } else {
    t = fmod(t-M_PI, -2*M_PI) + M_PI;
  }
  return t;
}

void wRo_to_euler(const Eigen::Matrix3f& wRo, double& yaw, double& pitch, double& roll) {
  yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
  double c = cos(yaw);
  double s = sin(yaw);
  pitch = standardRad(atan2(static_cast<double> (-wRo(2,0)), wRo(0,0)*c + wRo(1,0)*s));
  roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

void print_Isometry3d(Eigen::Isometry3d pose, std::stringstream &ss){
  Eigen::Vector3d t(pose.translation());
  Eigen::Quaterniond r(pose.rotation());
  ss <<t[0]<<", "<<t[1]<<", "<<t[2]<<" | " 
       <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() ;
  //  std::cout << ss.str() << "q\n";
}



void simulate_callback (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  // I choose v for virtual as s for simulate is takwen
  if (event.getKeySym () == "v" && event.keyDown ())
  {
    std::cout << "v was pressed => simulate cloud" << std::endl;

    std::vector<pcl::visualization::Camera> cams;
    viewer->getCameras(cams);
    
    if (cams.size() !=1){
      std::cout << "n cams not 1 exiting\n"; // for now in case ...
     return; 
    }
   // cout << "n cams: " << cams.size() << "\n";
    pcl::visualization::Camera cam = cams[0];
    


	      
	      Eigen::Affine3f pose;
	      pose = viewer->getViewerPose() ;
	      
	      
     std::cout << cam.pos[0] << " " 
               << cam.pos[1] << " " 
               << cam.pos[2] << " p\n";	      
	      
	     Eigen::Matrix3f m;
	     m =pose.rotation();
	     //All axies use right hand rule. x=red axis, y=green axis, z=blue axis z direction is point into the screen. z \ \ \ -----------> x | | | | | | y 
	      
 cout << pose(0,0)  << " " << pose(0,1) << " " << pose(0,2)  << " " << pose(0,3) << " x0\n";
 cout << pose(1,0) << " " << pose(1,1) << " " << pose(1,2) << " " << pose(1,3) << " x1\n";
  cout << pose(2,0) << " " << pose(2,1)  << " " << pose(2,2) << " " << pose(2,3)<< "x2\n";

  double yaw,pitch, roll;
  wRo_to_euler(m,yaw,pitch,roll);
  
 printf("RPY: %f %f %f\n", roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);    

// matrix->GetElement(1,0);
  
 cout << m(0,0)  << " " << m(0,1) << " " << m(0,2)  << " "  << " x0\n";
 cout << m(1,0) << " " << m(1,1) << " " << m(1,2) << " "  << " x1\n";
  cout << m(2,0) << " " << m(2,1)  << " " << m(2,2) << " "<< "x2\n\n";
  
  Eigen::Quaternionf rf;
  rf = Eigen::Quaternionf(m);
  
  
   Eigen::Quaterniond r(rf.w(),rf.x(),rf.y(),rf.z());
  
   Eigen::Isometry3d init_pose;
   init_pose.setIdentity();
   init_pose.translation() << cam.pos[0], cam.pos[1], cam.pos[2];  
   //Eigen::Quaterniond m = euler_to_quat(-1.54, 0, 0);
   init_pose.rotate(r);  
//   
  
    std::stringstream ss;
  print_Isometry3d(init_pose,ss);
  std::cout << "init_pose: " << ss.str() << "\n";
  
  
  
  
  viewer->addCoordinateSystem (1.0,pose,"reference");
  
  
  
    double tic = getTime();
    std::stringstream ss2;
    ss2.precision(20);
    ss2 << "simulated_pcl_" << tic << ".pcd";
    capture(init_pose,ss2.str());
    cout << (getTime() -tic) << " sec\n";  
  
  
  
  // these three variables determine the position and orientation of
    // the camera.
	      
	      
//     double lookat[3]; - focal location
//     double eye[3]; - my location
//     double up[3]; - updirection
     
	      
	      
//    std::cout << view[0]  << "," << view[1]  << "," << view[2] 
    
//    cameras.back ().view[2] = renderer->GetActiveCamera ()->GetOrientationWXYZ()[2];    
    
//ViewTransform->GetOrientationWXYZ();    
    
  //  Superclass::OnKeyUp ();
    
//       vtkSmartPointer<vtkCamera> cam = event.Interactor->GetRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->GetActiveCamera ();
//       double clip[2], focal[3], pos[3], view[3];
//       cam->GetClippingRange (clip);
/*      cam->GetFocalPoint (focal);
      cam->GetPosition (pos);
      cam->GetViewUp (view);
      int *win_pos = Interactor->GetRenderWindow ()->GetPosition ();
      int *win_size = Interactor->GetRenderWindow ()->GetSize ();
      std::cerr << clip[0]  << "," << clip[1]  << "/" << focal[0] << "," << focal[1] << "," << focal[2] << "/" <<
                   pos[0]   << "," << pos[1]   << "," << pos[2]   << "/" << view[0]  << "," << view[1]  << "," << view[2] << "/" <<
                   cam->GetViewAngle () / 180.0 * M_PI  << "/" << win_size[0] << "," << win_size[1] << "/" << win_pos[0] << "," << win_pos[1]
                << endl;    
  */  
  }
}

/* ---[ */

// Read in a 3D model
void
loadPolygonMeshModel (char* polygon_file)
{
  pcl::PolygonMesh mesh;	// (new pcl::PolygonMesh);
  //pcl::io::loadPolygonFile("/home/mfallon/data/models/dalet/Darlek_modified_works.obj",mesh);
  pcl::io::loadPolygonFile (polygon_file, mesh);
  pcl::PolygonMesh::Ptr cloud (new pcl::PolygonMesh (mesh));
  
  // Not sure if PolygonMesh assumes triangles if to
  // TODO: Ask a developer
  //PolygonMeshModel::Ptr model = PolygonMeshModel::Ptr (new PolygonMeshModel (GL_POLYGON, cloud));
  TriangleMeshModel::Ptr model = TriangleMeshModel::Ptr (new TriangleMeshModel (cloud));
  scene_->add (model);
  
  std::cout << "Just read " << polygon_file << std::endl;
  std::cout << mesh.polygons.size () << " polygons and "
	    << mesh.cloud.data.size () << " triangles\n";
}

void
initialize (int, char** argv)
{
  const GLubyte* version = glGetString (GL_VERSION);
  std::cout << "OpenGL Version: " << version << std::endl;

  // works for small files:
  camera_->set(-5.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  pcl::console::print_info("About to read: %s", argv[2]);
  loadPolygonMeshModel (argv[2]);
}

int
main (int argc, char** argv)
{
  srand (time (0));

  print_info ("The viewer window provides interactive commands; for help, press 'h' or 'H' from within the window.\n");

  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Command line parsing
  double bcolor[3] = {0, 0, 0};
  pcl::console::parse_3x_arguments (argc, argv, "-bc", bcolor[0], bcolor[1], bcolor[2]);

  std::vector<double> fcolor_r, fcolor_b, fcolor_g;
  bool fcolorparam = pcl::console::parse_multiple_3x_arguments (argc, argv, "-fc", fcolor_r, fcolor_g, fcolor_b);

  std::vector<int> psize;
  pcl::console::parse_multiple_arguments (argc, argv, "-ps", psize);

  std::vector<double> opaque;
  pcl::console::parse_multiple_arguments (argc, argv, "-opaque", opaque);

  int mview = 0;
  pcl::console::parse_argument (argc, argv, "-multiview", mview);

  int normals = 0;
  pcl::console::parse_argument (argc, argv, "-normals", normals);
  double normals_scale = NORMALS_SCALE;
  pcl::console::parse_argument (argc, argv, "-normals_scale", normals_scale);

  int pc = 0;
  pcl::console::parse_argument (argc, argv, "-pc", pc);
  double pc_scale = PC_SCALE;
  pcl::console::parse_argument (argc, argv, "-pc_scale", pc_scale);

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices   = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> vtk_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".vtk");

  if (p_file_indices.size () == 0 && vtk_file_indices.size () == 0)
  {
    print_error ("No .PCD or .VTK file given. Nothing to visualize.\n");
    return (-1);
  }

  // Multiview enabled?
  int y_s = 0, x_s = 0;
  double x_step = 0, y_step = 0;
  if (mview)
  {
    print_highlight ("Multi-viewport rendering enabled.\n");

    if (p_file_indices.size () != 0)
    {
      y_s = static_cast<int>(floor (sqrt (static_cast<float>(p_file_indices.size ()))));
      x_s = y_s + static_cast<int>(ceil ((p_file_indices.size () / static_cast<double>(y_s)) - y_s));
      print_highlight ("Preparing to load "); print_value ("%d", p_file_indices.size ());
    }
    else if (vtk_file_indices.size () != 0)
    {
      y_s = static_cast<int>(floor (sqrt (static_cast<float>(vtk_file_indices.size ()))));
      x_s = y_s + static_cast<int>(ceil ((vtk_file_indices.size () / static_cast<double>(y_s)) - y_s));
      print_highlight ("Preparing to load "); print_value ("%d", vtk_file_indices.size ());
    }

    x_step = static_cast<double>(1.0 / static_cast<double>(x_s));
    y_step = static_cast<double>(1.0 / static_cast<double>(y_s));
    print_info (" files ("); print_value ("%d", x_s);    print_info ("x"); print_value ("%d", y_s);
    print_info (" / ");      print_value ("%f", x_step); print_info ("x"); print_value ("%f", y_step);
    print_info (")\n");
  }

  // Fix invalid multiple arguments
  if (psize.size () != p_file_indices.size () && psize.size () > 0)
    for (size_t i = psize.size (); i < p_file_indices.size (); ++i)
      psize.push_back (1);
  if (opaque.size () != p_file_indices.size () && opaque.size () > 0)
    for (size_t i = opaque.size (); i < p_file_indices.size (); ++i)
      opaque.push_back (1.0);

  // Create the PCLVisualizer object
  boost::shared_ptr<pcl::visualization::PCLHistogramVisualizer> ph;

  // Using min_p, max_p to set the global Y min/max range for the histogram
  float min_p = FLT_MAX; float max_p = -FLT_MAX;

  int k = 0, l = 0, viewport = 0;
  // Load the data files
  pcl::PCDReader pcd;
  pcl::console::TicToc tt;
  ColorHandlerPtr color_handler;
  GeometryHandlerPtr geometry_handler;

  // Go through VTK files
  for (size_t i = 0; i < vtk_file_indices.size (); ++i)
  {
    // Load file
    tt.tic ();
    print_highlight (stderr, "Loading "); print_value (stderr, "%s ", argv[vtk_file_indices.at (i)]);
    vtkPolyDataReader* reader = vtkPolyDataReader::New ();
    reader->SetFileName (argv[vtk_file_indices.at (i)]);
    reader->Update ();
    vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput ();
    if (!polydata)
      return (-1);
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", polydata->GetNumberOfPoints ()); print_info (" points]\n");

    // Create the PCLVisualizer object here on the first encountered XYZ file
    if (!p)
      p.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PCD viewer"));

    // Multiview enabled?
    if (mview)
    {
      p->createViewPort (k * x_step, l * y_step, (k + 1) * x_step, (l + 1) * y_step, viewport);
      k++;
      if (k >= x_s)
      {
        k = 0;
        l++;
      }
    }

    // Add as actor
    std::stringstream cloud_name ("vtk-");
    cloud_name << argv[vtk_file_indices.at (i)] << "-" << i;
    p->addModelFromPolyData (polydata, cloud_name.str (), viewport);

    // Change the shape rendered color
    if (fcolorparam && fcolor_r.size () > i && fcolor_g.size () > i && fcolor_b.size () > i)
      p->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, fcolor_r[i], fcolor_g[i], fcolor_b[i], cloud_name.str ());

    // Change the shape rendered point size
    if (psize.size () > 0)
      p->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize.at (i), cloud_name.str ());

    // Change the shape rendered opacity
    if (opaque.size () > 0)
      p->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opaque.at (i), cloud_name.str ());
  }

  pcl::PCLPointCloud2::Ptr cloud;
  // Go through PCD files
  for (size_t i = 0; i < p_file_indices.size (); ++i)
  {
    cloud.reset (new pcl::PCLPointCloud2);
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int version;

    print_highlight (stderr, "Loading "); print_value (stderr, "%s ", argv[p_file_indices.at (i)]);

    tt.tic ();
    if (pcd.read (argv[p_file_indices.at (i)], *cloud, origin, orientation, version) < 0)
      return (-1);

    std::stringstream cloud_name;

    // ---[ Special check for 1-point multi-dimension histograms
    if (cloud->fields.size () == 1 && isMultiDimensionalFeatureField (cloud->fields[0]))
    {
      cloud_name << argv[p_file_indices.at (i)];

      if (!ph)
        ph.reset (new pcl::visualization::PCLHistogramVisualizer);
      print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud->fields[0].count); print_info (" points]\n");

      pcl::getMinMax (*cloud, 0, cloud->fields[0].name, min_p, max_p);
      ph->addFeatureHistogram (*cloud, cloud->fields[0].name, cloud_name.str ());
      continue;
    }

    cloud_name << argv[p_file_indices.at (i)] << "-" << i;

    // Create the PCLVisualizer object here on the first encountered XYZ file
    if (!p)
    {
      p.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PCD viewer"));
      p->registerPointPickingCallback (&pp_callback, (void*)&cloud);
      Eigen::Matrix3f rotation;
      rotation = orientation;
      p->setCameraPosition (origin [0]                  , origin [1]                  , origin [2],
                        origin [0] + rotation (0, 2), origin [1] + rotation (1, 2), origin [2] + rotation (2, 2),
                                     rotation (0, 1),              rotation (1, 1),              rotation (2, 1));
    }

    // Multiview enabled?
    if (mview)
    {
      p->createViewPort (k * x_step, l * y_step, (k + 1) * x_step, (l + 1) * y_step, viewport);
      k++;
      if (k >= x_s)
      {
        k = 0;
        l++;
      }
    }

    if (cloud->width * cloud->height == 0)
    {
      print_error ("[error: no points found!]\n");
      return (-1);
    }
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", (int)cloud->width * cloud->height); print_info (" points]\n");
    print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (*cloud).c_str ());

    // If no color was given, get random colors
    if (fcolorparam)
    {
      if (fcolor_r.size () > i && fcolor_g.size () > i && fcolor_b.size () > i)
        color_handler.reset (new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2> (cloud, fcolor_r[i], fcolor_g[i], fcolor_b[i]));
      else
        color_handler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2> (cloud));
    }
    else
      color_handler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2> (cloud));

    // Add the dataset with a XYZ and a random handler
    geometry_handler.reset (new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2> (cloud));
    // Add the cloud to the renderer
    //p->addPointCloud<pcl::PointXYZ> (cloud_xyz, geometry_handler, color_handler, cloud_name.str (), viewport);
    p->addPointCloud (cloud, geometry_handler, color_handler, origin, orientation, cloud_name.str (), viewport);

    // If normal lines are enabled
    if (normals != 0)
    {
      int normal_idx = pcl::getFieldIndex (*cloud, "normal_x");
      if (normal_idx == -1)
      {
        print_error ("Normal information requested but not available.\n");
        continue;
        //return (-1);
      }
      //
      // Convert from blob to pcl::PointCloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2 (*cloud, *cloud_xyz);
      cloud_xyz->sensor_origin_ = origin;
      cloud_xyz->sensor_orientation_ = orientation;

      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      pcl::fromPCLPointCloud2 (*cloud, *cloud_normals);
      std::stringstream cloud_name_normals;
      cloud_name_normals << argv[p_file_indices.at (i)] << "-" << i << "-normals";
      p->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_xyz, cloud_normals, normals, normals_scale, cloud_name_normals.str (), viewport);
    }

    // If principal curvature lines are enabled
    if (pc != 0)
    {
      if (normals == 0)
        normals = pc;

      int normal_idx = pcl::getFieldIndex (*cloud, "normal_x");
      if (normal_idx == -1)
      {
        print_error ("Normal information requested but not available.\n");
        continue;
        //return (-1);
      }
      int pc_idx = pcl::getFieldIndex (*cloud, "principal_curvature_x");
      if (pc_idx == -1)
      {
        print_error ("Principal Curvature information requested but not available.\n");
        continue;
        //return (-1);
      }
      //
      // Convert from blob to pcl::PointCloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2 (*cloud, *cloud_xyz);
      cloud_xyz->sensor_origin_ = origin;
      cloud_xyz->sensor_orientation_ = orientation;
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      pcl::fromPCLPointCloud2 (*cloud, *cloud_normals);
      pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_pc (new pcl::PointCloud<pcl::PrincipalCurvatures>);
      pcl::fromPCLPointCloud2 (*cloud, *cloud_pc);
      std::stringstream cloud_name_normals_pc;
      cloud_name_normals_pc << argv[p_file_indices.at (i)] << "-" << i << "-normals";
      int factor = (std::min)(normals, pc);
      p->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_xyz, cloud_normals, factor, normals_scale, cloud_name_normals_pc.str (), viewport);
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, cloud_name_normals_pc.str ());
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, cloud_name_normals_pc.str ());
      cloud_name_normals_pc << "-pc";
      p->addPointCloudPrincipalCurvatures<pcl::PointXYZ, pcl::Normal> (cloud_xyz, cloud_normals, cloud_pc, factor, pc_scale, cloud_name_normals_pc.str (), viewport);
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, cloud_name_normals_pc.str ());
    }

    // Add every dimension as a possible color
    if (!fcolorparam)
    {
      for (size_t f = 0; f < cloud->fields.size (); ++f)
      {
        if (cloud->fields[f].name == "rgb" || cloud->fields[f].name == "rgba")
          color_handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2> (cloud));
        else
        {
          if (!isValidFieldName (cloud->fields[f].name))
            continue;
          color_handler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (cloud, cloud->fields[f].name));
        }
        // Add the cloud to the renderer
        //p->addPointCloud<pcl::PointXYZ> (cloud_xyz, color_handler, cloud_name.str (), viewport);
        p->addPointCloud (cloud, color_handler, origin, orientation, cloud_name.str (), viewport);
      }
    }
    // Additionally, add normals as a handler
    geometry_handler.reset (new pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<pcl::PCLPointCloud2> (cloud));
    if (geometry_handler->isCapable ())
      //p->addPointCloud<pcl::PointXYZ> (cloud_xyz, geometry_handler, cloud_name.str (), viewport);
      p->addPointCloud (cloud, geometry_handler, origin, orientation, cloud_name.str (), viewport);

    // Set immediate mode rendering ON
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0, cloud_name.str ());

    // Change the cloud rendered point size
    if (psize.size () > 0)
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize.at (i), cloud_name.str ());

    // Change the cloud rendered opacity
    if (opaque.size () > 0)
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opaque.at (i), cloud_name.str ());

    // Reset camera viewpoint to center of cloud if camera parameters were not passed manually and this is the first loaded cloud
    //if (i == 0 && !p->cameraParamsSet ())
     // p->resetCameraViewpoint (cloud_name.str ());
  }
  
  
  ////////////////////////////////////////////////////////////////
  // Key binding for saving simulated point cloud:
  if (p)
    p->registerKeyboardCallback(simulate_callback, (void*)&p);
  
  
  
  
  int width = 640;
  int height = 480;
  window_width_ = width * 2;
  window_height_ = height * 2;

  print_info ("Manually generate a simulated RGB-D point cloud using pcl::simulation. For more information, use: %s -h\n", argv[0]);

  for (int i=0; i<2048; i++)
  {
    float v = i/2048.0;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }    

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


  camera_ = Camera::Ptr (new Camera ());
  scene_ = Scene::Ptr (new Scene ());

  range_likelihood_ = RangeLikelihood::Ptr (new RangeLikelihood(1, 1, height, width, scene_));

  // range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(10, 10, 96, 96, scene_));
  // range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(1, 1, 480, 640, scene_));

  // Actually corresponds to default parameters:
  range_likelihood_->setCameraIntrinsicsParameters (640,480, 576.09757860,
            576.09757860, 321.06398107, 242.97676897);
  range_likelihood_->setComputeOnCPU (false);
  range_likelihood_->setSumOnCPU (true);
  range_likelihood_->setUseColor (true);
  initialize (argc, argv); 

  if (p)
    p->setBackgroundColor (bcolor[0], bcolor[1], bcolor[2]);
  // Read axes settings
  double axes  = 0.0;
  pcl::console::parse_argument (argc, argv, "-ax", axes);
  if (axes != 0.0 && p)
  {
    double ax_x = 0.0, ax_y = 0.0, ax_z = 0.0;
    pcl::console::parse_3x_arguments (argc, argv, "-ax_pos", ax_x, ax_y, ax_z, false);
    // Draw XYZ axes if command-line enabled
    p->addCoordinateSystem (axes, ax_x, ax_y, ax_z, "reference");
  }

  // Clean up the memory used by the binary blob
  // Note: avoid resetting the cloud, otherwise the PointPicking callback will fail
  //cloud.reset ();

  if (ph)
  {
    print_highlight ("Setting the global Y range for all histograms to: "); print_value ("%f -> %f\n", min_p, max_p);
    ph->setGlobalYRange (min_p, max_p);
    ph->updateWindowPositions ();
    if (p)
      p->spin ();
    else
      ph->spin ();
  }
  else if (p)
    p->spin ();
}
/* ]--- */
