// Hacked version of kinfu_app.cpp
// Simulates depth images
// using pcl::simulation. these views are of a camera flying 
// around a short range object e.g. a tabletop from >3m aways
// these are then fed into kinfu and a mesh reconstruction is made
//
// to run:
// kinfu_app_sim -plyfile ~/projects/kmcl/kmcl/models/table_models/meta_model.ply 
// Maurice Fallon - mfallon <AT> mit.edu april 2012

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#define _CRT_SECURE_NO_DEPRECATE
#include <iostream>

// need to include GLEW net the top to avoid linking errors FOR PCL::SIMULATION:
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

#include <pcl/console/parse.h>
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>

#include "openni_capture.h"
#include "color_handler.h"
#include "evaluation.h"

#include <pcl/common/angles.h>
#include <pcl/memory.h>

//#include "tsdf_volume.h"
//#include "tsdf_volume.hpp"

#ifdef HAVE_OPENCV  
  #include <opencv2/highgui/highgui.hpp>
  #include <opencv2/imgproc/imgproc.hpp>
//#include "video_recorder.h"
#endif
using ScopeTimeT = pcl::ScopeTime;

#include <cmath>
#include <iostream>
#ifdef _WIN32
# define WIN32_LEAN_AND_MEAN
# include <windows.h>
#endif
  

//SIMSTARTSIMSTARTSIMSTARTSIMSTARTSIMSTART
#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include <pcl/console/print.h>
#include <pcl/io/vtk_lib_io.h>
//
#include <pcl/simulation/camera.h>
#include "pcl/simulation/model.h"
#include "pcl/simulation/scene.h"
#include "pcl/simulation/range_likelihood.h"
// Writing PNG files:
#include <opencv/cv.h>
#include <opencv/highgui.h>
//SIMENDSIMENDSIMENDSIMENDSIMENDSIMENDSIMEND

using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;
namespace pc = pcl::console;

//SIMSTARTSIMSTARTSIMSTARTSIMSTARTSIMSTART
using namespace pcl::console;
using namespace pcl::io;
using namespace pcl::simulation;
std::uint16_t t_gamma[2048];
Scene::Ptr scene_;
Camera::Ptr camera_;
RangeLikelihood::Ptr range_likelihood_;
//SIMENDSIMENDSIMENDSIMENDSIMENDSIMENDSIMEND


namespace pcl
{
  namespace gpu
  {
    void paint3DView (const KinfuTracker::View& rgb24, KinfuTracker::View& view, float colors_weight = 0.5f);
    void mergePointNormal (const DeviceArray<PointXYZ>& cloud, const DeviceArray<Normal>& normals, DeviceArray<PointNormal>& output);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SampledScopeTime : public StopWatch
{          
  enum { EACH = 33 };
  SampledScopeTime(int& time_ms, int i) : time_ms_(time_ms), i_(i) {}
  ~SampledScopeTime()
  {
    time_ms_ += stopWatch_.getTime ();        
    if (i_ % EACH == 0 && i_)
    {
      std::cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )" << std::endl;
      time_ms_ = 0;        
    }
  }
private:
    StopWatch stopWatch_;
    int& time_ms_;
    int i_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
setViewerPose (visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Affine3f 
getViewerPose (visualization::PCLVisualizer& viewer)
{
  Eigen::Affine3f pose = viewer.getViewerPose();
  Eigen::Matrix3f rotation = pose.linear();

  Matrix3f axis_reorder;  
  axis_reorder << 0,  0,  1,
                 -1,  0,  0,
                  0, -1,  0;

  rotation *= axis_reorder;
  pose.linear() = rotation;
  return pose;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SIMSTARTSIMSTARTSIMSTARTSIMSTARTSIMSTARTSIMSTARTSIMSTARTSIMSTARTSIMSTARTSIMSTARTSIMSTART
void
write_depth_image(const float* depth_buffer)
{
  int npixels = range_likelihood_->getWidth() * range_likelihood_->getHeight();
  std::uint8_t* depth_img = new std::uint8_t[npixels * 3];

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
      std::uint16_t kd = static_cast<std::uint16_t>(1090 - b*f/z*8);
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
  
  std::stringstream ss;
  ss <<"depth_image.png" ;   
  cv::imwrite(ss.str()  , cv_mat);     
  
  delete [] depth_img;
}


void
write_rgb_image(const std::uint8_t* rgb_buffer)
{
  int npixels = range_likelihood_->getWidth() * range_likelihood_->getHeight();
  std::uint8_t* rgb_img = new std::uint8_t[npixels * 3];

  for (int y = 0; y <  480; ++y)
  {
    for (int x = 0; x < 640; ++x)
    {
      int px= y*640 + x ;
      int px_in= (480-1 -y) *640 + x ; // flip up down
      rgb_img [3* (px) +0] = rgb_buffer[3*px_in+0];
      rgb_img [3* (px) +1] = rgb_buffer[3*px_in+1];
      rgb_img [3* (px) +2] = rgb_buffer[3*px_in+2];      
    }
  }  
  
  // Write to file:
  IplImage *cv_ipl = cvCreateImage( cvSize(640 ,480), 8, 3);
  cv::Mat cv_mat(cv_ipl);
  cv_mat.data = rgb_img ;
//  cv::imwrite("rgb_image.png", cv_mat);     
  
  std::stringstream ss;
  ss <<"rgb_image.png" ;   
  cv::imwrite(ss.str()  , cv_mat);   
  
  delete [] rgb_img;
}


void
depthBufferToMM(const float* depth_buffer,unsigned short* depth_img)
{
  //int npixels = range_likelihood_->getWidth() * range_likelihood_->getHeight();
  //unsigned short * depth_img = new unsigned short[npixels ];
  for (int y = 0; y <  480; ++y)
  {
    for (int x = 0; x < 640; ++x)
    {
      int i= y*640 + x ;
      int i_in= (480-1 -y) *640 + x ; // flip up down
      float zn = 0.7;
      float zf = 20.0;
      float d = depth_buffer[i_in];
      unsigned short z_new = (unsigned short)  std::floor( 1000*( -zf*zn/((zf-zn)*(d - zf/(zf-zn)))));

      if (z_new < 0) z_new = 0;
//      else if (z_new>65535) z_new = 65535;
      else if (z_new>5000) z_new = 0;

      //      if ( z_new < 18000){
//	  std::cout << z_new << " " << d << " " << x << "\n";  
//      }
      depth_img[i] = z_new;
    }
  }
}


void 
write_depth_image_uint(unsigned short* depth_img)
{
  // Write to file:
  IplImage *cv_ipl = cvCreateImage( cvSize(640 ,480), IPL_DEPTH_16U, 1);
  cv::Mat cv_mat(cv_ipl);
  cv_mat.data =(uchar *) depth_img;
  cv::imwrite("depth_image_uint.png", cv_mat);     
}


// display_tic_toc: a helper function which accepts a set of
// timestamps and displays the elapsed time between them as
// a fraction and time used [for profiling]
void
display_tic_toc (vector<double> &tic_toc,const std::string &fun_name)
{
  std::size_t tic_toc_size = tic_toc.size ();

  double percent_tic_toc_last = 0;
  double dtime = tic_toc[tic_toc_size-1] - tic_toc[0];
  std::cout << "fraction_" << fun_name << ",";
  for (std::size_t i = 0; i < tic_toc_size; i++)
  {
    double percent_tic_toc =  (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    std::cout <<  percent_tic_toc - percent_tic_toc_last << ", ";
    percent_tic_toc_last = percent_tic_toc;
  }
  std::cout << "\ntime_" << fun_name << ",";
  double time_tic_toc_last = 0;
  for (std::size_t i = 0; i < tic_toc_size; i++)
  {
    double percent_tic_toc = (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    std::cout <<  percent_tic_toc*dtime - time_tic_toc_last << ", ";
    time_tic_toc_last = percent_tic_toc*dtime;
  }
  std::cout << "\ntotal_time_" << fun_name << " " << dtime << "\n";
}

void
capture (Eigen::Isometry3d pose_in,unsigned short* depth_buffer_mm,const std::uint8_t* color_buffer)//, string point_cloud_fname)
{
  // No reference image - but this is kept for compatibility with range_test_v2:
  float* reference = new float[range_likelihood_->getRowHeight() * range_likelihood_->getColWidth()];
  //const float* depth_buffer = range_likelihood_->getDepthBuffer();
  // Copy one image from our last as a reference.
  /*
  for (int i=0, n=0; i<range_likelihood_->getRowHeight(); ++i)
  {
    for (int j=0; j<range_likelihood_->getColWidth(); ++j)
    {
      reference[n++] = 0;//depth_buffer[i*range_likelihood_->getWidth() + j];
    }
  }
  */

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;
  std::vector<float> scores;
  poses.push_back (pose_in);
  // HACK: mfallon modified computeLikelihoods to only call render()  (which is currently private)
  // need to make render public and use it.
  // For simulation it is used alone from the reset of range_likelihood_
  range_likelihood_->computeLikelihoods (reference, poses, scores);

  color_buffer =range_likelihood_->getColorBuffer();
  const float*  db_ptr= range_likelihood_->getDepthBuffer ();
  range_likelihood_->addNoise ();
  depthBufferToMM (db_ptr,depth_buffer_mm);

  // Writing these images is a smaller computation draw relative to KinFu:
  write_depth_image (db_ptr);
  //write_depth_image_uint(depth_buffer_mm);
  write_rgb_image (color_buffer); 
  
/*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_out (new pcl::PointCloud<pcl::PointXYZRGB>);
  bool write_cloud=true;
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
    range_likelihood_->getPointCloud (pc_out,false,camera_->pose ());
    // TODO: what to do when there are more than one simulated view?
    std::cout << pc_out->size() << " points written to file\n";
   
    pcl::PCDWriter writer;
    //writer.write (point_cloud_fname, *pc_out,	false);  /// ASCII
    writer.writeBinary (point_cloud_fname, *pc_out);
    //std::cout << "finished writing file\n";
  }
  */

  delete [] reference;
}

// Read in a 3D model
void
load_PolygonMesh_model (std::string polygon_file)
{
  pcl::PolygonMesh mesh;	// (new pcl::PolygonMesh);
  //pcl::io::loadPolygonFile("/home/mfallon/data/models/dalet/Darlek_modified_works.obj",mesh);
  if (!pcl::io::loadPolygonFile (polygon_file, mesh)){
    std::cout << "No ply file found, exiting" << std::endl;
   exit(-1); 
  }
  pcl::PolygonMesh::Ptr cloud (new pcl::PolygonMesh (mesh));
  
  TriangleMeshModel::Ptr model = TriangleMeshModel::Ptr (new TriangleMeshModel (cloud));
  scene_->add (model);
  
  std::cout << "Just read " << polygon_file << std::endl;
  std::cout << mesh.polygons.size () << " polygons and "
	    << mesh.cloud.data.size () << " triangles\n";
}

// A 'halo' camera - a circular ring of poses all pointing at a center point
// @param: focus_center: the center points
// @param: halo_r: radius of the ring
// @param: halo_dz: elevation of the camera above/below focus_center's z value
// @param: n_poses: number of generated poses
void
generate_halo(
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > &poses,
  Eigen::Vector3d focus_center,double halo_r,double halo_dz,int n_poses)
{
  for (double t=0;t < (2*M_PI);t =t + (2*M_PI)/ ((double) n_poses) ){
    double x = halo_r*std::cos(t);
    double y = halo_r*sin(t);
    double z = halo_dz;
    double pitch =std::atan2( halo_dz,halo_r); 
    double yaw = std::atan2(-y,-x);
   
    Eigen::Isometry3d pose;
    pose.setIdentity();
    Eigen::Matrix3d m;
    m = AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
	* AngleAxisd(pitch, Eigen::Vector3d::UnitY())
	* AngleAxisd(0, Eigen::Vector3d::UnitZ());    

    pose *=m;
    Vector3d v(x,y,z);
    v += focus_center;
    pose.translation() = v;
    poses.push_back(pose);  
  }
  return ;
}
//SIMENDSIMENDSIMENDSIMENDSIMENDSIMENDSIMENDSIMENDSIMENDSIMENDSIMEND


template<typename CloudT> void
writeCloudFile (int format, const CloudT& cloud);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
writePolygonMeshFile (int format, const pcl::PolygonMesh& mesh);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename MergedT, typename PointT>
typename PointCloud<MergedT>::Ptr merge(const PointCloud<PointT>& points, const PointCloud<RGB>& colors)
{    
  typename PointCloud<MergedT>::Ptr merged_ptr(new PointCloud<MergedT>());
    
  pcl::copyPointCloud (points, *merged_ptr);
  //pcl::copyPointCloud (colors, *merged_ptr); why error?
  //pcl::concatenateFields (points, colors, *merged_ptr); why error? 
    
  for (std::size_t i = 0; i < colors.size (); ++i)
    (*merged_ptr)[i].rgba = colors[i].rgba;
      
  return merged_ptr;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::PolygonMesh::Ptr convertToMesh(const DeviceArray<PointXYZ>& triangles)
{ 
  if (triangles.empty())
      return PolygonMesh::Ptr ();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width  = triangles.size();
  cloud.height = 1;
  triangles.download(cloud.points);

  PolygonMesh::Ptr mesh_ptr = make_shared<PolygonMesh> ();
  pcl::toPCLPointCloud2(cloud, mesh_ptr->cloud);
      
  mesh_ptr->polygons.resize (triangles.size() / 3);
  for (std::size_t i = 0; i < mesh_ptr->polygons.size (); ++i)
  {
    pcl::Vertices v;
    v.vertices.push_back(i*3+0);
    v.vertices.push_back(i*3+2);
    v.vertices.push_back(i*3+1);              
    mesh_ptr->polygons[i] = v;
  }    
  return mesh_ptr;
  
  std::cout << mesh_ptr->polygons.size () << " plys\n";
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct CurrentFrameCloudView
{
  CurrentFrameCloudView() : cloud_device_ (480, 640), cloud_viewer_ ("Frame Cloud Viewer")
  {
    cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);

    cloud_viewer_.setBackgroundColor (0, 0, 0.15);
    cloud_viewer_.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    cloud_viewer_.addCoordinateSystem (1.0);
    cloud_viewer_.initCameraParameters ();
    cloud_viewer_.setPosition (0, 500);
    cloud_viewer_.setSize (640, 480);
    cloud_viewer_.setCameraClipDistances (0.01, 10.01);
  }

  void
  show (const KinfuTracker& kinfu)
  {
    kinfu.getLastFrameCloud (cloud_device_);

    int c;
    cloud_device_.download (cloud_ptr_->points, c);
    cloud_ptr_->width = cloud_device_.cols ();
    cloud_ptr_->height = cloud_device_.rows ();
    cloud_ptr_->is_dense = false;

    cloud_viewer_.removeAllPointClouds ();
    cloud_viewer_.addPointCloud<PointXYZ>(cloud_ptr_);
    cloud_viewer_.spinOnce ();
  }

  void
  setViewerPose (const Eigen::Affine3f& viewer_pose) {
    ::setViewerPose (cloud_viewer_, viewer_pose);
  }

  PointCloud<PointXYZ>::Ptr cloud_ptr_;
  DeviceArray2D<PointXYZ> cloud_device_;
  visualization::PCLVisualizer cloud_viewer_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct ImageView
{
  ImageView() : paint_image_ (false), accumulate_views_ (false)
  {
    viewerScene_.setWindowTitle ("View3D from ray tracing");
    viewerScene_.setPosition (0, 0);
    viewerDepth_.setWindowTitle ("Kinect Depth stream");
    viewerDepth_.setPosition (640, 0);
    //viewerColor_.setWindowTitle ("Kinect RGB stream");
  }

  void
  showScene (KinfuTracker& kinfu, const PtrStepSz<const KinfuTracker::PixelRGB>& rgb24, bool registration, Eigen::Affine3f* pose_ptr = 0)
  {
    if (pose_ptr)
    {
        raycaster_ptr_->run(kinfu.volume(), *pose_ptr);
        raycaster_ptr_->generateSceneView(view_device_);
    }
    else
      kinfu.getImage (view_device_);

    if (paint_image_ && registration && !pose_ptr)
    {
      colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
      paint3DView (colors_device_, view_device_);
    }

    int cols;
    view_device_.download (view_host_, cols);
    viewerScene_.showRGBImage (reinterpret_cast<unsigned char*> (&view_host_[0]), view_device_.cols (), view_device_.rows ());

    //viewerColor_.showRGBImage ((unsigned char*)&rgb24.data, rgb24.cols, rgb24.rows);

#ifdef HAVE_OPENCV
    if (accumulate_views_)
    {
      views_.push_back (cv::Mat ());
      cv::cvtColor (cv::Mat (480, 640, CV_8UC3, (void*)&view_host_[0]), views_.back (), CV_RGB2GRAY);
      //cv::copy(cv::Mat(480, 640, CV_8UC3, (void*)&view_host_[0]), views_.back());
    }
#endif
  }

  void
  showDepth (const PtrStepSz<const unsigned short>& depth) 
  { 
    viewerDepth_.showShortImage (depth.data, depth.cols, depth.rows, 0, 5000, true); 
  }
  
  void
  showGeneratedDepth (KinfuTracker& kinfu, const Eigen::Affine3f& pose)
  {            
    raycaster_ptr_->run(kinfu.volume(), pose);
    raycaster_ptr_->generateDepthImage(generated_depth_);    

    int c;
    std::vector<unsigned short> data;
    generated_depth_.download(data, c);

    viewerDepth_.showShortImage (&data[0], generated_depth_.cols(), generated_depth_.rows(), 0, 5000, true);
  }

  void
  toggleImagePaint()
  {
    paint_image_ = !paint_image_;
    std::cout << "Paint image: " << (paint_image_ ? "On   (requires registration mode)" : "Off") << std::endl;
  }

  bool paint_image_;
  bool accumulate_views_;

  visualization::ImageViewer viewerScene_;
  visualization::ImageViewer viewerDepth_;
  //visualization::ImageViewer viewerColor_;

  KinfuTracker::View view_device_;
  KinfuTracker::View colors_device_;
  std::vector<KinfuTracker::PixelRGB> view_host_;

  RayCaster::Ptr raycaster_ptr_;

  KinfuTracker::DepthMap generated_depth_;
  
#ifdef HAVE_OPENCV
  std::vector<cv::Mat> views_;
#endif
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SceneCloudView
{
  enum { GPU_Connected6 = 0, CPU_Connected6 = 1, CPU_Connected26 = 2 };

  SceneCloudView() : extraction_mode_ (GPU_Connected6), compute_normals_ (false), valid_combined_ (false), cube_added_(false), cloud_viewer_ ("Scene Cloud Viewer")
  {
    cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
    normals_ptr_ = PointCloud<Normal>::Ptr (new PointCloud<Normal>);
    combined_ptr_ = PointCloud<PointNormal>::Ptr (new PointCloud<PointNormal>);
    point_colors_ptr_ = PointCloud<RGB>::Ptr (new PointCloud<RGB>);

    cloud_viewer_.setBackgroundColor (0, 0, 0);
    cloud_viewer_.addCoordinateSystem (1.0);
    cloud_viewer_.initCameraParameters ();
    cloud_viewer_.setPosition (0, 500);
    cloud_viewer_.setSize (640, 480);
    cloud_viewer_.setCameraClipDistances (0.01, 10.01);

    cloud_viewer_.addText ("H: print help", 2, 15, 20, 34, 135, 246);         
  }

  void
  show (KinfuTracker& kinfu, bool integrate_colors)
  {
    viewer_pose_ = kinfu.getCameraPose();

    ScopeTimeT time ("PointCloud Extraction");
    std::cout << "\nGetting cloud... " << std::flush;

    valid_combined_ = false;

    if (extraction_mode_ != GPU_Connected6)     // So use CPU
    {
      kinfu.volume().fetchCloudHost (*cloud_ptr_, extraction_mode_ == CPU_Connected26);
    }
    else
    {
      DeviceArray<PointXYZ> extracted = kinfu.volume().fetchCloud (cloud_buffer_device_);             

      if (compute_normals_)
      {
        kinfu.volume().fetchNormals (extracted, normals_device_);
        pcl::gpu::mergePointNormal (extracted, normals_device_, combined_device_);
        combined_device_.download (combined_ptr_->points);
        combined_ptr_->width = combined_ptr_->size ();
        combined_ptr_->height = 1;

        valid_combined_ = true;
      }
      else
      {
        extracted.download (cloud_ptr_->points);
        cloud_ptr_->width = cloud_ptr_->size ();
        cloud_ptr_->height = 1;
      }

      if (integrate_colors)
      {
        kinfu.colorVolume().fetchColors(extracted, point_colors_device_);
        point_colors_device_.download(point_colors_ptr_->points);
        point_colors_ptr_->width = point_colors_ptr_->size ();
        point_colors_ptr_->height = 1;
      }
      else
        point_colors_ptr_->points.clear();
    }
    const auto size = valid_combined_ ? combined_ptr_->size () : cloud_ptr_->size ();
    std::cout << "Done.  Cloud size: " << size / 1000 << "K" << std::endl;

    cloud_viewer_.removeAllPointClouds ();    
    if (valid_combined_)
    {
      visualization::PointCloudColorHandlerRGBHack<PointNormal> rgb(combined_ptr_, point_colors_ptr_);
      cloud_viewer_.addPointCloud<PointNormal> (combined_ptr_, rgb, "Cloud");
      cloud_viewer_.addPointCloudNormals<PointNormal>(combined_ptr_, 50);
    }
    else
    {
      visualization::PointCloudColorHandlerRGBHack<PointXYZ> rgb(cloud_ptr_, point_colors_ptr_);
      cloud_viewer_.addPointCloud<PointXYZ> (cloud_ptr_, rgb);
    }    
  }

  void
  toggleCube(const Eigen::Vector3f& size)
  {
      if (cube_added_)
          cloud_viewer_.removeShape("cube");
      else
        cloud_viewer_.addCube(size*0.5, Eigen::Quaternionf::Identity(), size(0), size(1), size(2));

      cube_added_ = !cube_added_;
  }

  void
  toggleExtractionMode ()
  {
    extraction_mode_ = (extraction_mode_ + 1) % 3;

    switch (extraction_mode_)
    {
    case 0: std::cout << "Cloud exctraction mode: GPU, Connected-6" << std::endl; break;
    case 1: std::cout << "Cloud exctraction mode: CPU, Connected-6    (requires a lot of memory)" << std::endl; break;
    case 2: std::cout << "Cloud exctraction mode: CPU, Connected-26   (requires a lot of memory)" << std::endl; break;
    }
    ;
  }

  void
  toggleNormals ()
  {
    compute_normals_ = !compute_normals_;
    std::cout << "Compute normals: " << (compute_normals_ ? "On" : "Off") << std::endl;
  }

  void
  clearClouds (bool print_message = false)
  {
    cloud_viewer_.removeAllPointClouds ();
    cloud_ptr_->points.clear ();
    normals_ptr_->points.clear ();    
    if (print_message)
      std::cout << "Clouds/Meshes were cleared" << std::endl;
  }

  void
  showMesh(KinfuTracker& kinfu, bool /*integrate_colors*/)
  {
    ScopeTimeT time ("Mesh Extraction");
    std::cout << "\nGetting mesh... " << std::flush;

    if (!marching_cubes_)
      marching_cubes_ = MarchingCubes::Ptr( new MarchingCubes() );

    DeviceArray<PointXYZ> triangles_device = marching_cubes_->run(kinfu.volume(), triangles_buffer_device_);    
    mesh_ptr_ = convertToMesh(triangles_device);
    
    cloud_viewer_.removeAllPointClouds ();
    if (mesh_ptr_){
      cloud_viewer_.addPolygonMesh(*mesh_ptr_);	
      std::cout << "mesh ptr exist\n";
    }else{
      std::cout << "mesh ptr no exist\n";
    }
    
    std::cout << "Done.  Triangles number: " << triangles_device.size() / MarchingCubes::POINTS_PER_TRIANGLE / 1000 << "K" << std::endl;
  }
    
  int extraction_mode_;
  bool compute_normals_;
  bool valid_combined_;
  bool cube_added_;

  Eigen::Affine3f viewer_pose_;

  visualization::PCLVisualizer cloud_viewer_;

  PointCloud<PointXYZ>::Ptr cloud_ptr_;
  PointCloud<Normal>::Ptr normals_ptr_;

  DeviceArray<PointXYZ> cloud_buffer_device_;
  DeviceArray<Normal> normals_device_;

  PointCloud<PointNormal>::Ptr combined_ptr_;
  DeviceArray<PointNormal> combined_device_;  

  DeviceArray<RGB> point_colors_device_; 
  PointCloud<RGB>::Ptr point_colors_ptr_;

  MarchingCubes::Ptr marching_cubes_;
  DeviceArray<PointXYZ> triangles_buffer_device_;

  pcl::PolygonMesh::Ptr mesh_ptr_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct KinFuApp
{
  enum { PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8 };
  
  KinFuApp(CaptureOpenNI& source, float vsz) : exit_ (false), scan_ (false), scan_mesh_(false), scan_volume_ (false), independent_camera_ (false),
    registration_ (false), integrate_colors_ (false), capture_ (source)
  {    
    //Init Kinfu Tracker
    Eigen::Vector3f volume_size = Vector3f::Constant (vsz/*meters*/);

    float f = capture_.depth_focal_length_VGA;
    kinfu_.setDepthIntrinsics (f, f);
    kinfu_.volume().setSize (volume_size);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
    Eigen::Vector3f t = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

    Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);

    kinfu_.setInitalCameraPose (pose);
    kinfu_.volume().setTsdfTruncDist (0.030f/*meters*/);    
    kinfu_.setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(20.f) ));
    //kinfu_.setDepthTruncationForICP(5.f/*meters*/);
    kinfu_.setCameraMovementThreshold(0.001f);
    
    //Init KinfuApp            
    tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
    image_view_.raycaster_ptr_ = RayCaster::Ptr( new RayCaster(kinfu_.rows (), kinfu_.cols (), f, f) );

    scene_cloud_view_.cloud_viewer_.registerKeyboardCallback (keyboard_callback, (void*)this);
    image_view_.viewerScene_.registerKeyboardCallback (keyboard_callback, (void*)this);
    image_view_.viewerDepth_.registerKeyboardCallback (keyboard_callback, (void*)this);

    float diag = sqrt ((float)kinfu_.cols () * kinfu_.cols () + kinfu_.rows () * kinfu_.rows ());
    scene_cloud_view_.cloud_viewer_.setCameraFieldOfView (2 * std::atan (diag / (2 * f)) * 1.5);
    
    scene_cloud_view_.toggleCube(volume_size);    
  }

  ~KinFuApp()
  {
    if (evaluation_ptr_)
      evaluation_ptr_->saveAllPoses(kinfu_);
  }

  void
  initCurrentFrameView ()
  {
    current_frame_cloud_view_ = make_shared<CurrentFrameCloudView> ();
    current_frame_cloud_view_->cloud_viewer_.registerKeyboardCallback (keyboard_callback, (void*)this);
    current_frame_cloud_view_->setViewerPose (kinfu_.getCameraPose ());
  }

  void
  tryRegistrationInit ()
  {
    registration_ = capture_.setRegistration (true);
    std::cout << "Registration mode: " << (registration_ ?  "On" : "Off (not supported by source)") << std::endl;
  }

  void 
  toggleColorIntegration(bool force = false)
  {
    if (registration_ || force)
    {
      const int max_color_integration_weight = 2;
      kinfu_.initColorIntegration(max_color_integration_weight);
      integrate_colors_ = true;      
    }
    std::cout << "Color integration: " << (integrate_colors_ ? "On" : "Off (not supported by source)") << std::endl;
  }

  void
  toggleIndependentCamera()
  {
    independent_camera_ = !independent_camera_;
    std::cout << "Camera mode: " << (independent_camera_ ?  "Independent" : "Bound to Kinect pose") << std::endl;
  }
  
  void
  toggleEvaluationMode(const std::string& eval_folder, const std::string& match_file = std::string())
  {
    evaluation_ptr_ = Evaluation::Ptr( new Evaluation(eval_folder) );
    if (!match_file.empty())
        evaluation_ptr_->setMatchFile(match_file);

    kinfu_.setDepthIntrinsics (evaluation_ptr_->fx, evaluation_ptr_->fy, evaluation_ptr_->cx, evaluation_ptr_->cy);
    image_view_.raycaster_ptr_ = RayCaster::Ptr( new RayCaster(kinfu_.rows (), kinfu_.cols (), 
        evaluation_ptr_->fx, evaluation_ptr_->fy, evaluation_ptr_->cx, evaluation_ptr_->cy) );
  }

  void
  execute (int argc, char** argv, const std::string &plyfile)
  {
    PtrStepSz<const unsigned short> depth;
    PtrStepSz<const KinfuTracker::PixelRGB> rgb24;
    int time_ms = 0;
    bool has_image = false;

    // Create simulation environment:
    int width = 640;
    int height = 480;
    for (int i=0; i<2048; i++)
    {
      float v = i/2048.0;
      v = powf(v, 3)* 6;
      t_gamma[i] = v*6*256;
    }  

    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);// was GLUT_RGBA
    glutInitWindowPosition (10, 10);
    glutInitWindowSize (10, 10);
    //glutInitWindowSize (window_width_, window_height_);
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
    range_likelihood_ = RangeLikelihood::Ptr (new RangeLikelihood (1, 1, height, width, scene_));

    // Actually corresponds to default parameters:
    range_likelihood_->setCameraIntrinsicsParameters (640,480, 576.09757860,
	      576.09757860, 321.06398107, 242.97676897);
    range_likelihood_->setComputeOnCPU (false);
    range_likelihood_->setSumOnCPU (true);
    range_likelihood_->setUseColor (true);  

    camera_->set(0.471703, 1.59862, 3.10937, 0, 0.418879, -12.2129);
    camera_->set_pitch(0.418879); // not sure why this is here:

    std::cout << "About to read: "<< plyfile << std::endl;   
    load_PolygonMesh_model (plyfile);  
    
    // Generate a series of poses:
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;
    Eigen::Vector3d focus_center(0,0,1.3);
    //  double halo_r = 4.0;
    double halo_r = 1.5;  
    double halo_dz = 1.5; // was 2;
    // 20 is too quick when adding noise:
    // 50 is ok though
    int n_poses=50;
    int n_pose_stop = 10;
    // above means make a circle of 50 poses, stop after the 10th i.e. 1/5 of a halo ring:
    generate_halo(poses,focus_center,halo_r,halo_dz,n_poses);    
    
    unsigned short * disparity_buf_ = new unsigned short[width*height ];
    const std::uint8_t* color_buf_uint;
    
    // loop though and create the mesh:
    for (int i = 0; !exit_; ++i)
    { 
      std::vector<double> tic_toc;
      tic_toc.push_back(getTime());
      double tic_main = getTime();

      Eigen::Vector3d t(poses[i].translation());
      Eigen::Quaterniond r(poses[i].rotation());
      std::stringstream ss;
      ss << t[0]<<", "<<t[1]<<", "<<t[2]<<" | " 
          <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() ;       
      std::cout << i << ": " << ss.str() << " pose_simulatedposition\n";      
      
      capture (poses[i],disparity_buf_, color_buf_uint);//,ss.str());
      const KinfuTracker::PixelRGB* color_buf_ = (const KinfuTracker::PixelRGB*) color_buf_uint;
      PtrStepSz<const unsigned short> depth_sim = PtrStepSz<const unsigned short>(height, width, disparity_buf_, 2*width);
      //std::cout << depth_sim.rows << " by " << depth_sim.cols << " | s: " << depth_sim.step << "\n";
      // RGB-KinFu currently disabled for now - problems with color in KinFu apparently
      // but this constructor might not  be right either: not sure about step size
      integrate_colors_=false;
      PtrStepSz<const KinfuTracker::PixelRGB> rgb24_sim = PtrStepSz<const KinfuTracker::PixelRGB>(height, width, color_buf_, width);
      tic_toc.push_back (getTime ());
      
      if (1==0){ // live capture - probably doesn't work anymore, left in here for comparison:
	bool has_frame = evaluation_ptr_ ? evaluation_ptr_->grab(i, depth) : capture_.grab (depth, rgb24);      
	if (!has_frame)
	{
	  std::cout << "Can't grab" << std::endl;
	  break;
	}

	depth_device_.upload (depth.data, depth.step, depth.rows, depth.cols);
	if (integrate_colors_)
	    image_view_.colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
	
	{
	  SampledScopeTime fps(time_ms, i);
	
	  //run kinfu algorithm
	  if (integrate_colors_)
	    has_image = kinfu_ (depth_device_, image_view_.colors_device_);
	  else
	    has_image = kinfu_ (depth_device_);                  
	}
      }else{ //simulate:

	std::cout << " color: " << integrate_colors_ << "\n"; // integrate_colors_ seems to be zero
	depth_device_.upload (depth_sim.data, depth_sim.step, depth_sim.rows, depth_sim.cols);
	if (integrate_colors_){
	    image_view_.colors_device_.upload (rgb24_sim.data, rgb24_sim.step, rgb24_sim.rows, rgb24_sim.cols);
	}
	
	tic_toc.push_back (getTime ());
	
	{
	  SampledScopeTime fps(time_ms, i);
	  //run kinfu algorithm
	  if (integrate_colors_)
	    has_image = kinfu_ (depth_device_, image_view_.colors_device_);
	  else
	    has_image = kinfu_ (depth_device_);                  
	}
	
      }
      
      tic_toc.push_back (getTime ());
      
      Eigen::Affine3f k_aff = kinfu_.getCameraPose();
      Eigen::Matrix3f k_m;
      k_m =k_aff.rotation();
      Eigen::Quaternionf k_r;
      k_r = Eigen::Quaternionf(k_m);
      std::stringstream ss_k;      
      ss_k << k_aff(0,3) <<", "<< k_aff(1,3)<<", "<< k_aff(2,3)<<" | " 
          <<k_r.w()<<", "<<k_r.x()<<", "<<k_r.y()<<", "<<k_r.z() ;       
      std::cout << i << ": " << ss_k.str() << " pose_kinect\n";          
      
      // Everything below this is Visualization or I/O:
      if (i >n_pose_stop){
	int pause;
	std::cout << "Enter a key to write Mesh file\n";
	cin >> pause;

	scene_cloud_view_.showMesh(kinfu_, integrate_colors_);
	writeMesh(KinFuApp::MESH_VTK);       
	// writeMesh(KinFuApp::MESH_PLY);
      
	if (scan_)
	{
	  scan_ = false;
	  scene_cloud_view_.show (kinfu_, integrate_colors_);
			
	  if (scan_volume_)
	  {
	    // download tsdf volume
	    {
	      ScopeTimeT time ("tsdf volume download");
	      std::cout << "Downloading TSDF volume from device ... " << std::flush;
	      // kinfu_.volume().downloadTsdfAndWeighs (tsdf_volume_.volumeWriteable (), tsdf_volume_.weightsWriteable ());
              kinfu_.volume ().downloadTsdfAndWeighsLocal ();
	      // tsdf_volume_.setHeader (Eigen::Vector3i (pcl::device::VOLUME_X, pcl::device::VOLUME_Y, pcl::device::VOLUME_Z), kinfu_.volume().getSize ());
              kinfu_.volume ().setHeader (Eigen::Vector3i (pcl::device::VOLUME_X, pcl::device::VOLUME_Y, pcl::device::VOLUME_Z), kinfu_.volume().getSize ());
	      // std::cout << "done [" << tsdf_volume_.size () << " voxels]" << std::endl << std::endl;
              std::cout << "done [" << kinfu_.volume ().size () << " voxels]" << std::endl << std::endl;
	    }
	    {
	      ScopeTimeT time ("converting");
	      std::cout << "Converting volume to TSDF cloud ... " << std::flush;
	      // tsdf_volume_.convertToTsdfCloud (tsdf_cloud_ptr_);
              kinfu_.volume ().convertToTsdfCloud (tsdf_cloud_ptr_);
	      std::cout << "done [" << tsdf_cloud_ptr_->size () << " points]" << std::endl << std::endl;
	    }
	  }
	  else
	    std::cout << "[!] tsdf volume download is disabled" << std::endl << std::endl;
	}

	if (scan_mesh_)
	{
	    scan_mesh_ = false;
	    scene_cloud_view_.showMesh(kinfu_, integrate_colors_);
	}
	
	if (has_image)
	{
	  Eigen::Affine3f viewer_pose = getViewerPose(scene_cloud_view_.cloud_viewer_);
//	  image_view_.showScene (kinfu_, rgb24, registration_, independent_camera_ ? &viewer_pose : 0);
	  image_view_.showScene (kinfu_, rgb24_sim, registration_, independent_camera_ ? &viewer_pose : 0);
	}

	if (current_frame_cloud_view_)
	  current_frame_cloud_view_->show (kinfu_);
	
	image_view_.showDepth (depth_sim);
	//image_view_.showDepth (depth);
	// image_view_.showGeneratedDepth(kinfu_, kinfu_.getCameraPose());
    
	if (!independent_camera_)
	  setViewerPose (scene_cloud_view_.cloud_viewer_, kinfu_.getCameraPose());
	
	scene_cloud_view_.cloud_viewer_.spinOnce (3);    
	
	// As of April 2012, entering a key will end this program...
	std::cout << "Paused after view\n";
	cin >> pause;      
      }
      double elapsed = (getTime() -tic_main);
      std::cout << elapsed << " sec elapsed [" << (1/elapsed) << "]\n";          
      tic_toc.push_back (getTime ());
      display_tic_toc (tic_toc, "kinfu_app_sim");
    }
  }

  void
  writeCloud (int format) const
  {      
    const SceneCloudView& view = scene_cloud_view_;

    if (!view.cloud_ptr_->points.empty ())
    {    
      if(view.point_colors_ptr_->points.empty()) // no colors
      {
        if (view.valid_combined_)
          writeCloudFile (format, view.combined_ptr_);
        else
          writeCloudFile (format, view.cloud_ptr_);
      }
      else
      {        
        if (view.valid_combined_)
          writeCloudFile (format, merge<PointXYZRGBNormal>(*view.combined_ptr_, *view.point_colors_ptr_));
        else
          writeCloudFile (format, merge<PointXYZRGB>(*view.cloud_ptr_, *view.point_colors_ptr_));
      }
    }
  }

  void
  writeMesh(int format) const
  {
    if (scene_cloud_view_.mesh_ptr_) {
      writePolygonMeshFile(format, *scene_cloud_view_.mesh_ptr_);
    }
  }

  void
  printHelp ()
  {
    std::cout << std::endl;
    std::cout << "KinFu app hotkeys" << std::endl;
    std::cout << "=================" << std::endl;
    std::cout << "    H    : print this help" << std::endl;
    std::cout << "   Esc   : exit" << std::endl;
    std::cout << "    T    : take cloud" << std::endl;
    std::cout << "    A    : take mesh" << std::endl;
    std::cout << "    M    : toggle cloud exctraction mode" << std::endl;
    std::cout << "    N    : toggle normals exctraction" << std::endl;
    std::cout << "    I    : toggle independent camera mode" << std::endl;
    std::cout << "    B    : toggle volume bounds" << std::endl;
    std::cout << "    *    : toggle scene view painting ( requires registration mode )" << std::endl;
    std::cout << "    C    : clear clouds" << std::endl;    
    std::cout << "   1,2,3 : save cloud to PCD(binary), PCD(ASCII), PLY(ASCII)" << std::endl;
    std::cout << "    7,8  : save mesh to PLY, VTK" << std::endl;
    std::cout << "   X, V  : TSDF volume utility" << std::endl;
    std::cout << std::endl;
  }  

  bool exit_;
  bool scan_;
  bool scan_mesh_;
  bool scan_volume_;

  bool independent_camera_;

  bool registration_;
  bool integrate_colors_;
  
  CaptureOpenNI& capture_;
  KinfuTracker kinfu_;

  SceneCloudView scene_cloud_view_;
  ImageView image_view_;
  CurrentFrameCloudView::Ptr current_frame_cloud_view_;

  KinfuTracker::DepthMap depth_device_;

  // pcl::TSDFVolume<float, short> tsdf_volume_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;

  Evaluation::Ptr evaluation_ptr_;

  static void
  keyboard_callback (const visualization::KeyboardEvent &e, void *cookie)
  {
    KinFuApp* app = reinterpret_cast<KinFuApp*> (cookie);

    int key = e.getKeyCode ();

    if (e.keyUp ())    
      switch (key)
      {
      case 27: app->exit_ = true; break;
      case (int)'t': case (int)'T': app->scan_ = true; break;
      case (int)'a': case (int)'A': app->scan_mesh_ = true; break;
      case (int)'h': case (int)'H': app->printHelp (); break;
      case (int)'m': case (int)'M': app->scene_cloud_view_.toggleExtractionMode (); break;
      case (int)'n': case (int)'N': app->scene_cloud_view_.toggleNormals (); break;      
      case (int)'c': case (int)'C': app->scene_cloud_view_.clearClouds (true); break;
      case (int)'i': case (int)'I': app->toggleIndependentCamera (); break;
      case (int)'b': case (int)'B': app->scene_cloud_view_.toggleCube(app->kinfu_.volume().getSize()); break;
      case (int)'7': case (int)'8': app->writeMesh (key - (int)'0'); break;
      case (int)'1': case (int)'2': case (int)'3': app->writeCloud (key - (int)'0'); break;      
      case '*': app->image_view_.toggleImagePaint (); break;

      case (int)'x': case (int)'X':
        app->scan_volume_ = !app->scan_volume_;
        std::cout << std::endl << "Volume scan: " << (app->scan_volume_ ? "enabled" : "disabled") << std::endl << std::endl;
        break;
      case (int)'v': case (int)'V':
        std::cout << "Saving TSDF volume to tsdf_volume.dat ... " << std::flush;
        // app->tsdf_volume_.save ("tsdf_volume.dat", true);
        app->kinfu_.volume ().save ("tsdf_volume.dat", true);
        //std::cout << "done [" << app->tsdf_volume_.size () << " voxels]" << std::endl;
        std::cout << "done [" << app->app->kinfu_.volume ().size () << " voxels]" << std::endl;
        std::cout << "Saving TSDF volume cloud to tsdf_cloud.pcd ... " << std::flush;
        pcl::io::savePCDFile<pcl::PointXYZI> ("tsdf_cloud.pcd", *app->tsdf_cloud_ptr_, true);
        std::cout << "done [" << app->tsdf_cloud_ptr_->size () << " points]" << std::endl;
        break;

      default:
        break;
      }    
  }
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename CloudPtr> void
writeCloudFile (int format, const CloudPtr& cloud_prt)
{
  if (format == KinFuApp::PCD_BIN)
  {
    std::cout << "Saving point cloud to 'cloud_bin.pcd' (binary)... " << std::flush;
    pcl::io::savePCDFile ("cloud_bin.pcd", *cloud_prt, true);
  }
  else
  if (format == KinFuApp::PCD_ASCII)
  {
    std::cout << "Saving point cloud to 'cloud.pcd' (ASCII)... " << std::flush;
    pcl::io::savePCDFile ("cloud.pcd", *cloud_prt, false);
  }
  else   /* if (format == KinFuApp::PLY) */
  {
    std::cout << "Saving point cloud to 'cloud.ply' (ASCII)... " << std::flush;
    pcl::io::savePLYFileASCII ("cloud.ply", *cloud_prt);
  
  }
  std::cout << "Done" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
writePolygonMeshFile (int format, const pcl::PolygonMesh& mesh)
{
    std::cout << "writePolygonMeshFile mf" << std::endl;

  if (format == KinFuApp::MESH_PLY)
  {
    std::cout << "Saving mesh to to 'mesh.ply'... " << std::flush;
    pcl::io::savePLYFile("mesh.ply", mesh);		
  }
  else /* if (format == KinFuApp::MESH_VTK) */
  {
    std::cout << "Saving mesh to to 'mesh.vtk'... " << std::flush;
    pcl::io::saveVTKFile("mesh.vtk", mesh);    
  }  
  std::cout << "Done" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int
print_cli_help ()
{
  std::cout << "\nKinfu app concole parameters help:" << std::endl;
  std::cout << "    --help, -h                      : print this message" << std::endl;  
  std::cout << "    --registration, -r              : try to enable registration ( requires source to support this )" << std::endl;
  std::cout << "    --current-cloud, -cc            : show current frame cloud" << std::endl;
  std::cout << "    --save-views, -sv               : accumulate scene view and save in the end ( Requires OpenCV. Will cause 'bad_alloc' after some time )" << std::endl;  
  std::cout << "    --registration, -r              : enable registration mode" << std::endl; 
  std::cout << "    --integrate-colors, -ic         : enable color integration mode ( allows to get cloud with colors )" << std::endl;   
  std::cout << "    -volume_suze <size_in_meters>   : define integration volume size" << std::endl;   
  std::cout << "    -dev <device>, -oni <oni_file>  : select depth source. Default will be selected if not specified" << std::endl;
  std::cout << "";
  std::cout << " For RGBD benchmark (Requires OpenCV):" << std::endl; 
  std::cout << "    -eval <eval_folder> [-match_file <associations_file_in_the_folder>]" << std::endl;
  std::cout << " For Simuation (Requires pcl::simulation):" << std::endl; 
  std::cout << "    -plyfile                        : path to ply file for simulation testing " << std::endl;
    
  return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char* argv[])
{  
  if (pc::find_switch (argc, argv, "--help") || pc::find_switch (argc, argv, "-h"))
    return print_cli_help ();

  int device = 0;
  pc::parse_argument (argc, argv, "-gpu", device);
  pcl::gpu::setDevice (device);
  pcl::gpu::printShortCudaDeviceInfo (device);

  if(checkIfPreFermiGPU(device))
    return std::cout << std::endl << "Kinfu is not supported for pre-Fermi GPU architectures, and not built for them by default. Exiting..." << std::endl, 1;

  CaptureOpenNI capture;
  
  int openni_device = 0;
  std::string oni_file, eval_folder, match_file;
  if (pc::parse_argument (argc, argv, "-dev", openni_device) > 0)
  {
    capture.open (openni_device);
  }
  else
  if (pc::parse_argument (argc, argv, "-oni", oni_file) > 0)
  {
    capture.open (oni_file);
  }
  else
  if (pc::parse_argument (argc, argv, "-eval", eval_folder) > 0)
  {
    //init data source latter
    pc::parse_argument (argc, argv, "-match_file", match_file);
  }
  else
  {
    //capture.open (openni_device);
//    capture.open ("/home/mfallon/gcc-4.4/kinfu/skeletonrec.oni");
//    capture.open ("/home/mfallon/gcc-4.4/kinfu/MultipleHands.oni");
    //capture.open("d:/onis/20111013-224932.oni");
    //capture.open("d:/onis/reg20111229-180846.oni");
    //capture.open("d:/onis/white1.oni");
    //capture.open("/media/Main/onis/20111013-224932.oni");
    //capture.open("20111013-225218.oni");
    //capture.open("d:/onis/20111013-224551.oni");
    //capture.open("d:/onis/20111013-224719.oni");
  }
  
  //SIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIM
  // read model for simulation mode:
  std::string plyfile;
  pc::parse_argument (argc, argv, "-plyfile", plyfile);
  //SIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIMSIM
  
  float volume_size = 3.f;
  pc::parse_argument (argc, argv, "-volume_size", volume_size);
          
  KinFuApp app (capture, volume_size);

  if (pc::parse_argument (argc, argv, "-eval", eval_folder) > 0)
    app.toggleEvaluationMode(eval_folder, match_file);

  if (pc::find_switch (argc, argv, "--current-cloud") || pc::find_switch (argc, argv, "-cc"))
    app.initCurrentFrameView ();

  if (pc::find_switch (argc, argv, "--save-views") || pc::find_switch (argc, argv, "-sv"))
    app.image_view_.accumulate_views_ = true;  //will cause bad alloc after some time  
    
  if (pc::find_switch (argc, argv, "--registration") || pc::find_switch (argc, argv, "-r"))  
      app.tryRegistrationInit();
      
  bool force = pc::find_switch (argc, argv, "-icf");
  if (force || pc::find_switch (argc, argv, "--integrate-colors") || pc::find_switch (argc, argv, "-ic"))      
    app.toggleColorIntegration(force);

  
  // executing
  try { app.execute (argc, argv,plyfile); }
  catch (const std::bad_alloc& /*e*/) { std::cout << "Bad alloc" << std::endl; }
  catch (const std::exception& /*e*/) { std::cout << "Exception" << std::endl; }

#ifdef HAVE_OPENCV
  for (std::size_t t = 0; t < app.image_view_.views_.size (); ++t)
  {
    if (t == 0)
    {
      std::cout << "Saving depth map of first view." << std::endl;
      cv::imwrite ("./depthmap_1stview.png", app.image_view_.views_[0]);
      std::cout << "Saving sequence of (" << app.image_view_.views_.size () << ") views." << std::endl;
    }
    char buf[4096];
    sprintf (buf, "./%06d.png", (int)t);
    cv::imwrite (buf, app.image_view_.views_[t]);
    printf ("writing: %s\n", buf);
  }
#endif
  return 0;
}
