/*
Work in progress: patch by Marco (AUG,19th 2012)
> oni fixed
> pcl added: mostly to include rgb treatment while grabbing from PCD files obtained by pcl_openni_grab_frame -noend 
> sync issue fixed
> volume_size issue fixed
> world.pcd write exception on windows fixed on new trunk version

+ minor changes
*/

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

#include <functional>
#include <iostream>
#include <thread>

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/console/parse.h>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp> // for microsec_clock::local_time

#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>

#include <pcl/common/time.h>
#include <pcl/common/transforms.h> // for transformPoint
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#ifdef HAVE_OPENNI
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#endif
#include <pcl/io/pcd_grabber.h>
#ifdef HAVE_OPENNI2
#include <pcl/io/openni2_grabber.h>
#endif

#include "openni_capture.h"
#include "color_handler.h"
#include "evaluation.h"

#include <pcl/common/angles.h>
#include <pcl/memory.h>

#ifdef HAVE_OPENCV  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif
using ScopeTimeT = pcl::ScopeTime;

#include <pcl/gpu/kinfu_large_scale/screenshot_manager.h>

using namespace std::chrono_literals;
using namespace pcl;
using namespace Eigen;

using namespace pcl::gpu::kinfuLS;

using pcl::gpu::DeviceArray;
using pcl::gpu::DeviceArray2D;
using pcl::gpu::PtrStepSz;

namespace pc = pcl::console;

namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      void paint3DView (const KinfuTracker::View& rgb24, KinfuTracker::View& view, float colors_weight = 0.5f);
      void mergePointNormal (const DeviceArray<PointXYZ>& cloud, const DeviceArray<Normal>& normals, DeviceArray<PointNormal>& output);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<std::string> getPcdFilesInDir(const std::string& directory)
{
  namespace fs = boost::filesystem;
  fs::path dir(directory);

  std::cout << "path: " << directory << std::endl;
  if (directory.empty() || !fs::exists(dir) || !fs::is_directory(dir))
          PCL_THROW_EXCEPTION (pcl::IOException, "No valid PCD directory given!\n");

  std::vector<std::string> result;
  fs::directory_iterator pos(dir);
  fs::directory_iterator end;           

  for(; pos != end ; ++pos)
    if (fs::is_regular_file(pos->status()) )
      if (fs::extension(*pos) == ".pcd")
      {
        result.push_back (pos->path ().string ());
        std::cout << "added: " << result.back() << std::endl;
      }

  return result;  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SampledScopeTime : public StopWatch
{          
  enum { EACH = 33 };
  SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}
  ~SampledScopeTime()
  {
    static int i_ = 0;
    static boost::posix_time::ptime starttime_ = boost::posix_time::microsec_clock::local_time();
    time_ms_ += getTime ();
    if (i_ % EACH == 0 && i_)
    {
      boost::posix_time::ptime endtime_ = boost::posix_time::microsec_clock::local_time();
      std::cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )"
           << "( real: " << 1000.f * EACH / (endtime_-starttime_).total_milliseconds() << "fps )"  << std::endl;
      time_ms_ = 0;
      starttime_ = endtime_;
    }
    ++i_;
  }
private:    
  int& time_ms_;    
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
  for (std::size_t i = 0; i < colors.size (); ++i)
    (*merged_ptr)[i].rgba = colors[i].rgba;

  return merged_ptr;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::PolygonMesh::Ptr convertToMesh(const DeviceArray<PointXYZ>& triangles)
{ 
  if (triangles.empty())
          return pcl::PolygonMesh::Ptr();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width  = triangles.size();
  cloud.height = 1;
  triangles.download(cloud.points);

  PolygonMesh::Ptr mesh_ptr = pcl::make_shared<PolygonMesh> ();
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
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct CurrentFrameCloudView
{
  CurrentFrameCloudView() : cloud_device_ (480, 640), cloud_viewer_ ("Frame Cloud Viewer")
  {
    cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);

    cloud_viewer_.setBackgroundColor (0, 0, 0.15);
    cloud_viewer_.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    cloud_viewer_.addCoordinateSystem (1.0, "global");
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
  showScene (KinfuTracker& kinfu, const PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB>& rgb24, bool registration, Eigen::Affine3f* pose_ptr = nullptr)
  {
    if (pose_ptr)
    {
      raycaster_ptr_->run ( kinfu.volume (), *pose_ptr, kinfu.getCyclicalBufferStructure () ); //says in cmake it does not know it
      raycaster_ptr_->generateSceneView(view_device_);
    }
    else
    {
      kinfu.getImage (view_device_);
    }

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
    raycaster_ptr_->run(kinfu.volume(), pose, kinfu.getCyclicalBufferStructure ());
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

  visualization::ImageViewer viewerScene_; //view the raycasted model
  visualization::ImageViewer viewerDepth_; //view the current depth map
  //visualization::ImageViewer viewerColor_;

  KinfuTracker::View view_device_;
  KinfuTracker::View colors_device_;
  std::vector<pcl::gpu::kinfuLS::PixelRGB> view_host_;

  RayCaster::Ptr raycaster_ptr_;

  KinfuTracker::DepthMap generated_depth_;

#ifdef HAVE_OPENCV
  std::vector<cv::Mat> views_;
#endif
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// View the volume as 3D points
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
    cloud_viewer_.addCoordinateSystem (1.0, "global");
    cloud_viewer_.initCameraParameters ();
    cloud_viewer_.setPosition (0, 500);
    cloud_viewer_.setSize (640, 480);
    cloud_viewer_.setCameraClipDistances (0.01, 10.01);

    cloud_viewer_.addText ("H: print help", 2, 15, 20, 34, 135, 246);         
    cloud_viewer_.addText ("ICP State: ", 450, 55, 20, 0.0, 1.0, 0.0, "icp");
    cloud_viewer_.addText ("Press 'S' to save the current world", 450, 35, 10, 0.0, 1.0, 0.0, "icp_save");
    cloud_viewer_.addText ("Press 'R' to reset the system", 450, 15, 10, 0.0, 1.0, 0.0, "icp_reset");
  }
  
  inline void 
  drawCamera (Eigen::Affine3f& pose, const std::string& name, double r, double g, double b)
  {
    double focal = 575;
    double height = 480;
    double width = 640;
    
    // create a 5-point visual for each camera
    pcl::PointXYZ p1, p2, p3, p4, p5;
    p1.x=0; p1.y=0; p1.z=0;
    double dist = 0.75;
    double minX, minY, maxX, maxY;
    maxX = dist*tan (std::atan (width / (2.0*focal)));
    minX = -maxX;
    maxY = dist*tan (std::atan (height / (2.0*focal)));
    minY = -maxY;
    p2.x=minX; p2.y=minY; p2.z=dist;
    p3.x=maxX; p3.y=minY; p3.z=dist;
    p4.x=maxX; p4.y=maxY; p4.z=dist;
    p5.x=minX; p5.y=maxY; p5.z=dist;
    p1=pcl::transformPoint (p1, pose);
    p2=pcl::transformPoint (p2, pose);
    p3=pcl::transformPoint (p3, pose);
    p4=pcl::transformPoint (p4, pose);
    p5=pcl::transformPoint (p5, pose);
    std::stringstream ss;
    ss.str ("");
    ss << name << "_line1";
    cloud_viewer_.addLine (p1, p2, r, g, b, ss.str ());
    ss.str ("");
    ss << name << "_line2";
    cloud_viewer_.addLine (p1, p3, r, g, b, ss.str ());
    ss.str ("");
    ss << name << "_line3";
    cloud_viewer_.addLine (p1, p4, r, g, b, ss.str ());
    ss.str ("");
    ss << name << "_line4";
    cloud_viewer_.addLine (p1, p5, r, g, b, ss.str ());
    ss.str ("");
    ss << name << "_line5";
    cloud_viewer_.addLine (p2, p5, r, g, b, ss.str ());
    ss.str ("");
    ss << name << "_line6";
    cloud_viewer_.addLine (p5, p4, r, g, b, ss.str ());
    ss.str ("");
    ss << name << "_line7";
    cloud_viewer_.addLine (p4, p3, r, g, b, ss.str ());
    ss.str ("");
    ss << name << "_line8";
    cloud_viewer_.addLine (p3, p2, r, g, b, ss.str ());    
  }
  
  inline void 
  removeCamera (const std::string& name)
  {
    cloud_viewer_.removeShape (name);
    std::stringstream ss;
    ss.str ("");
    ss << name << "_line1";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name << "_line2";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name << "_line3";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name << "_line4";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name << "_line5";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name << "_line6";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name << "_line7";
    cloud_viewer_.removeShape (ss.str ());
    ss.str ("");
    ss << name << "_line8";
    cloud_viewer_.removeShape (ss.str ());
  }

  void
  displayICPState (KinfuTracker& kinfu, bool was_lost_)
  {
    std::string name = "last_good_track";
    std::string name_estimate = "last_good_estimate";
    if (was_lost_ && !kinfu.icpIsLost ()) //execute only when ICP just recovered (i.e. was_lost_ == true && icpIsLost == false)
    {
      removeCamera(name);
      removeCamera(name_estimate);
      clearClouds(false);
      cloud_viewer_.updateText ("ICP State: OK", 450, 55, 20, 0.0, 1.0, 0.0, "icp");
      cloud_viewer_.updateText ("Press 'S' to save the current world", 450, 35, 10, 0.0, 1.0, 0.0, "icp_save");
      cloud_viewer_.updateText ("Press 'R' to reset the system", 450, 15, 10, 0.0, 1.0, 0.0, "icp_reset");
    }
    else if (!was_lost_ && kinfu.icpIsLost()) //execute only when we just lost ourselves (i.e. was_lost_ = false && icpIsLost == true)
    { 
      // draw position of the last good track
      Eigen::Affine3f last_pose = kinfu.getCameraPose();
      drawCamera(last_pose, name, 0.0, 1.0, 0.0);
      
     
      
      cloud_viewer_.updateText ("ICP State: LOST", 450, 55, 20, 1.0, 0.0, 0.0, "icp");
      cloud_viewer_.updateText ("Press 'S' to save the current world", 450, 35, 10, 1.0, 0.0, 0.0, "icp_save");
      cloud_viewer_.updateText ("Press 'R' to reset the system", 450, 15, 10, 1.0, 0.0, 0.0, "icp_reset");
    }
    
    
    if( kinfu.icpIsLost() )
    {
      removeCamera(name_estimate);
       // draw current camera estimate
      Eigen::Affine3f last_pose_estimate = kinfu.getLastEstimatedPose();
      drawCamera(last_pose_estimate, name_estimate, 1.0, 0.0, 0.0);      
    }
      
      
      
//       std::cout << "current estimated pose: " << kinfu.getLastEstimatedPose().translation() << std::endl << kinfu.getLastEstimatedPose().linear() << std::endl;
//     
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
        pcl::gpu::kinfuLS::mergePointNormal (extracted, normals_device_, combined_device_);
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
      case 0: std::cout << "Cloud extraction mode: GPU, Connected-6" << std::endl; break;
      case 1: std::cout << "Cloud extraction mode: CPU, Connected-6    (requires a lot of memory)" << std::endl; break;
      case 2: std::cout << "Cloud extraction mode: CPU, Connected-26   (requires a lot of memory)" << std::endl; break;
    }         
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
    if (mesh_ptr_)
      cloud_viewer_.addPolygonMesh(*mesh_ptr_);	

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

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW

};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct KinFuLSApp
{
  enum { PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8 };

  KinFuLSApp(pcl::Grabber& source, float vsz, float shiftDistance, int snapshotRate) : exit_ (false), scan_ (false), scan_mesh_(false), scan_volume_ (false), independent_camera_ (false),
          registration_ (false), integrate_colors_ (false), pcd_source_ (false), focal_length_(-1.f), capture_ (source), was_lost_(false), time_ms_ (0)
  {    
    //Init Kinfu Tracker
    Eigen::Vector3f volume_size = Vector3f::Constant (vsz/*meters*/);    

    PCL_WARN ("--- CURRENT SETTINGS ---\n");
    PCL_INFO ("Volume size is set to %.2f meters\n", vsz);
    PCL_INFO ("Volume will shift when the camera target point is farther than %.2f meters from the volume center\n", shiftDistance);
    PCL_INFO ("The target point is located at [0, 0, %.2f] in camera coordinates\n", 0.6*vsz);
    PCL_WARN ("------------------------\n");

    // warning message if shifting distance is abnormally big compared to volume size
    if(shiftDistance > 2.5 * vsz)
      PCL_WARN ("WARNING Shifting distance (%.2f) is very large compared to the volume size (%.2f).\nYou can modify it using --shifting_distance.\n", shiftDistance, vsz);

    kinfu_ = new pcl::gpu::kinfuLS::KinfuTracker(volume_size, shiftDistance);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
    Eigen::Vector3f t = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

    Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);

    kinfu_->setInitialCameraPose (pose);
    kinfu_->volume().setTsdfTruncDist (0.030f/*meters*/);
    kinfu_->setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(20.f) ));
    //kinfu_->setDepthTruncationForICP(3.f/*meters*/);
    kinfu_->setCameraMovementThreshold(0.001f);

    //Init KinFuLSApp            
    tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
    image_view_.raycaster_ptr_ = RayCaster::Ptr( new RayCaster(kinfu_->rows (), kinfu_->cols ()) );

    scene_cloud_view_.cloud_viewer_.registerKeyboardCallback (keyboard_callback, (void*)this);
    image_view_.viewerScene_.registerKeyboardCallback (keyboard_callback, (void*)this);
    image_view_.viewerDepth_.registerKeyboardCallback (keyboard_callback, (void*)this);

    scene_cloud_view_.toggleCube(volume_size);
    frame_counter_ = 0;
    enable_texture_extraction_ = false;

    //~ float fx, fy, cx, cy;
    //~ openni_wrapper::OpenNIDevice::Ptr d = ((pcl::OpenNIGrabber)source).getDevice ();
    //~ kinfu_->getDepthIntrinsics (fx, fy, cx, cy);

    float height = 480.0f;
    float width = 640.0f;
    screenshot_manager_.setCameraIntrinsics (pcl::device::kinfuLS::FOCAL_LENGTH, height, width);
    snapshot_rate_ = snapshotRate;
    
    Eigen::Matrix3f Rid = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
    Eigen::Vector3f T = Vector3f (0, 0, -volume_size(0)*1.5f);
    delta_lost_pose_ = Eigen::Translation3f (T) * Eigen::AngleAxisf (Rid); 
    
  }

  ~KinFuLSApp()
  {
    if (evaluation_ptr_)
      evaluation_ptr_->saveAllPoses(*kinfu_);
  }

  void
  initCurrentFrameView ()
  {
    current_frame_cloud_view_ = pcl::make_shared<CurrentFrameCloudView> ();
    current_frame_cloud_view_->cloud_viewer_.registerKeyboardCallback (keyboard_callback, (void*)this);
    current_frame_cloud_view_->setViewerPose (kinfu_->getCameraPose ());
  }

  void
  initRegistration ()
  {
    registration_ =
    #if defined(HAVE_OPENNI) && !defined(HAVE_OPENNI2)
    capture_.providesCallback<pcl::ONIGrabber::sig_cb_openni_image_depth_image> ();
    #elif !defined(HAVE_OPENNI) && defined(HAVE_OPENNI2)
    capture_.providesCallback<pcl::io::OpenNI2Grabber::sig_cb_openni_image_depth_image> ();
    #elif defined(HAVE_OPENNI) && defined(HAVE_OPENNI2)
    capture_.providesCallback<pcl::ONIGrabber::sig_cb_openni_image_depth_image> () ||
    capture_.providesCallback<pcl::io::OpenNI2Grabber::sig_cb_openni_image_depth_image> ();
    #endif
    std::cout << "Registration mode: " << (registration_ ? "On" : "Off (not supported by source)") << std::endl;
  }

  void 
  toggleColorIntegration()
  {
    if(registration_)
    {
      const int max_color_integration_weight = 2;
      kinfu_->initColorIntegration(max_color_integration_weight);
      integrate_colors_ = true;      
    }    
    std::cout << "Color integration: " << (integrate_colors_ ? "On" : "Off ( requires registration mode )") << std::endl;
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

    kinfu_->setDepthIntrinsics (evaluation_ptr_->fx, evaluation_ptr_->fy, evaluation_ptr_->cx, evaluation_ptr_->cy);
    image_view_.raycaster_ptr_ = RayCaster::Ptr( new RayCaster(kinfu_->rows (), kinfu_->cols (), evaluation_ptr_->fx, evaluation_ptr_->fy, evaluation_ptr_->cx, evaluation_ptr_->cy) );
  }

  void execute(const PtrStepSz<const unsigned short>& depth, const PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB>& rgb24, bool has_data)
  {        
    bool has_image = false;
    frame_counter_++;

    was_lost_ = kinfu_->icpIsLost();
    
    if (has_data)
    {
      depth_device_.upload (depth.data, depth.step, depth.rows, depth.cols);
      if (integrate_colors_)
        image_view_.colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);

      {
        SampledScopeTime fps(time_ms_);

        //run kinfu algorithm
        if (integrate_colors_)
          has_image = (*kinfu_) (depth_device_, image_view_.colors_device_);
        else
          has_image = (*kinfu_) (depth_device_);
      }

      image_view_.showDepth (depth_);
      //image_view_.showGeneratedDepth(kinfu_, kinfu_->getCameraPose());
            
    }

    if (scan_ || (!was_lost_ && kinfu_->icpIsLost ()) ) //if scan mode is OR and ICP just lost itself => show current volume as point cloud
    {
      scan_ = false;
      //scene_cloud_view_.show (*kinfu_, integrate_colors_); // this triggers out of memory errors, so I comment it out for now (Raph)

      if (scan_volume_)
      {                
        std::cout << "Downloading TSDF volume from device ... " << std::flush;
        // kinfu_->volume().downloadTsdfAndWeighs (tsdf_volume_.volumeWriteable (), tsdf_volume_.weightsWriteable ());
        kinfu_->volume ().downloadTsdfAndWeightsLocal ();
        // tsdf_volume_.setHeader (Eigen::Vector3i (pcl::device::kinfuLS::VOLUME_X, pcl::device::kinfuLS::VOLUME_Y, pcl::device::kinfuLS::VOLUME_Z), kinfu_->volume().getSize ());
        kinfu_->volume ().setHeader (Eigen::Vector3i (pcl::device::kinfuLS::VOLUME_X, pcl::device::kinfuLS::VOLUME_Y, pcl::device::kinfuLS::VOLUME_Z), kinfu_->volume().getSize ());
        // std::cout << "done [" << tsdf_volume_.size () << " voxels]" << std::endl << std::endl;
        std::cout << "done [" << kinfu_->volume ().size () << " voxels]" << std::endl << std::endl;

        std::cout << "Converting volume to TSDF cloud ... " << std::flush;
        // tsdf_volume_.convertToTsdfCloud (tsdf_cloud_ptr_);
        kinfu_->volume ().convertToTsdfCloud (tsdf_cloud_ptr_);
        // std::cout << "done [" << tsdf_cloud_ptr_->size () << " points]" << std::endl << std::endl;        
        std::cout << "done [" << kinfu_->volume ().size () << " points]" << std::endl << std::endl;
      }
      else
        std::cout << "[!] tsdf volume download is disabled" << std::endl << std::endl;
    }
    

    if (scan_mesh_)
    {
      scan_mesh_ = false;
      scene_cloud_view_.showMesh(*kinfu_, integrate_colors_);
    }

    if (has_image)
    {
      Eigen::Affine3f viewer_pose = getViewerPose(scene_cloud_view_.cloud_viewer_);
      image_view_.showScene (*kinfu_, rgb24, registration_, independent_camera_ ? &viewer_pose : nullptr);
    }    

    if (current_frame_cloud_view_)
      current_frame_cloud_view_->show (*kinfu_);

    // if ICP is lost, we show the world from a farther view point
    if(kinfu_->icpIsLost())
    {
      setViewerPose (scene_cloud_view_.cloud_viewer_, kinfu_->getCameraPose () * delta_lost_pose_);
    }
    else
    if (!independent_camera_)
      setViewerPose (scene_cloud_view_.cloud_viewer_, kinfu_->getCameraPose ());

    if (enable_texture_extraction_ && !kinfu_->icpIsLost ()) {
      if ( (frame_counter_  % snapshot_rate_) == 0 )   // Should be defined as a parameter. Done.
      {
        screenshot_manager_.saveImage (kinfu_->getCameraPose (), rgb24);
      }
    }
    
    // display ICP state
    scene_cloud_view_.displayICPState (*kinfu_, was_lost_);
    
  }

  template<typename DepthImg>
  void source_cb1(const typename DepthImg::Ptr& depth_wrapper)
  {        
    {
      std::unique_lock<std::mutex> lock (data_ready_mutex_, std::try_to_lock);
      if (exit_ || !lock)
        return;

      depth_.cols = depth_wrapper->getWidth();
      depth_.rows = depth_wrapper->getHeight();
      depth_.step = depth_.cols * depth_.elemSize();

      source_depth_data_.resize(depth_.cols * depth_.rows);
      depth_wrapper->fillDepthImageRaw(depth_.cols, depth_.rows, &source_depth_data_[0]);
      depth_.data = &source_depth_data_[0];     
    }
    data_ready_cond_.notify_one();
  }

  template<typename Img, typename DepthImg>
  void source_cb2(const typename Img::Ptr& image_wrapper, const typename DepthImg::Ptr& depth_wrapper, float)
  {
    {
      std::unique_lock<std::mutex> lock (data_ready_mutex_, std::try_to_lock);

      if (exit_ || !lock)
      {
        return;
      }

      depth_.cols = depth_wrapper->getWidth();
      depth_.rows = depth_wrapper->getHeight();
      depth_.step = depth_.cols * depth_.elemSize();

      source_depth_data_.resize(depth_.cols * depth_.rows);
      depth_wrapper->fillDepthImageRaw(depth_.cols, depth_.rows, &source_depth_data_[0]);
      depth_.data = &source_depth_data_[0];      

      rgb24_.cols = image_wrapper->getWidth();
      rgb24_.rows = image_wrapper->getHeight();
      rgb24_.step = rgb24_.cols * rgb24_.elemSize(); 

      source_image_data_.resize(rgb24_.cols * rgb24_.rows);
      image_wrapper->fillRGB(rgb24_.cols, rgb24_.rows, (unsigned char*)&source_image_data_[0]);
      rgb24_.data = &source_image_data_[0];    

    }
    data_ready_cond_.notify_one();
  }

  void source_cb3(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & DC3)
  {
    {                             
      //std::cout << "Giving colors1\n";
      std::unique_lock<std::mutex> lock (data_ready_mutex_, std::try_to_lock);
      //std::cout << lock << std::endl; //causes compile errors 
      if (exit_ || !lock)
        return;
      //std::cout << "Giving colors2\n";
      int width  = DC3->width;
      int height = DC3->height;
      depth_.cols = width;
      depth_.rows = height;
      depth_.step = depth_.cols * depth_.elemSize();
      source_depth_data_.resize(depth_.cols * depth_.rows);   

      rgb24_.cols = width;
      rgb24_.rows = height;
      rgb24_.step = rgb24_.cols * rgb24_.elemSize(); 
      source_image_data_.resize(rgb24_.cols * rgb24_.rows);

      unsigned char *rgb    = (unsigned char *)  &source_image_data_[0];
      unsigned short *depth = (unsigned short *) &source_depth_data_[0];  

      //std::cout << "Giving colors3\n";
      for (int i=0; i<width*height; i++) {
        PointXYZRGBA pt = DC3->at(i);
        rgb[3*i +0] = pt.r;
        rgb[3*i +1] = pt.g;
        rgb[3*i +2] = pt.b;
        depth[i]    = pt.z/0.001;
      }
      //std::cout << "Giving colors4\n";
      rgb24_.data = &source_image_data_[0];   
      depth_.data = &source_depth_data_[0];      
    }	
    data_ready_cond_.notify_one();
  }

  void
  startMainLoop (bool triggered_capture, bool oni2_interface = false)
  {   
    boost::signals2::connection c;

    std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) >
        func3 = [this] (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
    {
      source_cb3 (cloud);
    };

    bool need_colors = integrate_colors_ || registration_ || enable_texture_extraction_;

    if(oni2_interface)
    {
      #ifdef HAVE_OPENNI2
      using namespace pcl::io;
      using DepthImagePtr = pcl::shared_ptr<DepthImage>;
      using ImagePtr = pcl::shared_ptr<Image>;

      std::function<void (const ImagePtr&, const DepthImagePtr&, float)> func1 =
          [this] (const ImagePtr& img, const DepthImagePtr& depth, float constant)
      {
        source_cb2<Image, DepthImage> (img, depth, constant);
      };
      std::function<void (const DepthImagePtr&)> func2 =
          [this] (const DepthImagePtr& depth) { source_cb1<DepthImage> (depth); };

      if(pcd_source_)
      {
        c = capture_.registerCallback (func3);
      }
      else if (need_colors)
      {
        c = capture_.registerCallback (func1);
      }
      else
      {
        c = capture_.registerCallback (func2);
      }
      #else
      PCL_ERROR ("OpenNI2 is disabled in this PCL. Please build PCL with OpenNI2 feature.\n");
      #endif
    }
    else
    {
      #ifdef HAVE_OPENNI
      using namespace openni_wrapper;
      using DepthImagePtr = pcl::shared_ptr<DepthImage>;
      using ImagePtr = pcl::shared_ptr<Image>;

      std::function<void (const ImagePtr&, const DepthImagePtr&, float)> func1 =
          [this] (const ImagePtr& img, const DepthImagePtr& depth, float constant)
      {
        source_cb2<Image, DepthImage> (img, depth, constant);
      };
      std::function<void (const DepthImagePtr&)> func2 =
          [this] (const DepthImagePtr& depth) { source_cb1<DepthImage> (depth); };

      if(pcd_source_)
      {
        c = capture_.registerCallback (func3);
      }
      else if (need_colors)
      {
        c = capture_.registerCallback (func1);
      }
      else
      {
        c = capture_.registerCallback (func2);
      }
      #else
      PCL_ERROR ("OpenNI is disabled in this PCL. Please build PCL with OpenNI feature.\n");
      #endif
    }

    {
      std::unique_lock<std::mutex> lock(data_ready_mutex_);

      if (!triggered_capture) 
        capture_.start ();

      while (!exit_ && !scene_cloud_view_.cloud_viewer_.wasStopped () && !image_view_.viewerScene_.wasStopped () && !this->kinfu_->isFinished ())
      { 
        if (triggered_capture)
          capture_.start(); // Triggers new frame

        bool has_data = (data_ready_cond_.wait_for(lock, 100ms) == std::cv_status::no_timeout);

        try { this->execute (depth_, rgb24_, has_data); }
        catch (const std::bad_alloc& /*e*/) { std::cout << "Bad alloc" << std::endl; break; }
        catch (const std::exception& e) { std::cout << "Exception: " << e.what() << std::endl; break; }

        scene_cloud_view_.cloud_viewer_.spinOnce (3);
        //~ std::cout << "In main loop" << std::endl;                  
      } 
      exit_ = true;
      std::this_thread::sleep_for(100ms);

      if (!triggered_capture)     
        capture_.stop (); // Stop stream
    }
    c.disconnect();       
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
  writeMesh(int format) const
  {
    if (scene_cloud_view_.mesh_ptr_) 
      writePolygonMeshFile(format, *scene_cloud_view_.mesh_ptr_);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    std::cout << "   L, l  : On the next shift, KinFu will extract the whole current cube, extract the world and stop" << std::endl;
    std::cout << "   S, s  : On the next shift, KinFu will extract the world and stop" << std::endl;
    std::cout << std::endl;
  }  

  bool exit_;
  bool scan_;
  bool scan_mesh_;
  bool scan_volume_;

  bool independent_camera_;
  int frame_counter_;
  bool enable_texture_extraction_;
  pcl::kinfuLS::ScreenshotManager screenshot_manager_;
  int snapshot_rate_;

  bool registration_;
  bool integrate_colors_;
  bool pcd_source_;
  float focal_length_;

  pcl::Grabber& capture_;
  KinfuTracker *kinfu_;

  SceneCloudView scene_cloud_view_;
  ImageView image_view_;
  pcl::shared_ptr<CurrentFrameCloudView> current_frame_cloud_view_;

  KinfuTracker::DepthMap depth_device_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;

  Evaluation::Ptr evaluation_ptr_;

  std::mutex data_ready_mutex_;
  std::condition_variable data_ready_cond_;

  std::vector<pcl::gpu::kinfuLS::PixelRGB> source_image_data_;
  std::vector<unsigned short> source_depth_data_;
  PtrStepSz<const unsigned short> depth_;
  PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB> rgb24_;  
  
  Eigen::Affine3f delta_lost_pose_;
  
  bool was_lost_;

  int time_ms_;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  static void
  keyboard_callback (const visualization::KeyboardEvent &e, void *cookie)
  {
    KinFuLSApp* app = reinterpret_cast<KinFuLSApp*> (cookie);

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
      case (int)'b': case (int)'B': app->scene_cloud_view_.toggleCube(app->kinfu_->volume().getSize()); break;
      case (int)'l': case (int)'L': app->kinfu_->performLastScan (); break;
      case (int)'s': case (int)'S': app->kinfu_->extractAndSaveWorld (); break;
      case (int)'r': case (int)'R': app->kinfu_->reset(); app->scene_cloud_view_.clearClouds(); break;
      case (int)'7': case (int)'8': app->writeMesh (key - (int)'0'); break;  
      case (int)'1': case (int)'2': case (int)'3': app->writeCloud (key - (int)'0'); break;      
      case '*': app->image_view_.toggleImagePaint (); break;
      
      case (int)'p': case (int)'P': app->kinfu_->setDisableICP(); break;

      case (int)'x': case (int)'X':
        app->scan_volume_ = !app->scan_volume_;
        std::cout << std::endl << "Volume scan: " << (app->scan_volume_ ? "enabled" : "disabled") << std::endl << std::endl;
        break;
      case (int)'v': case (int)'V':
        std::cout << "Saving TSDF volume to tsdf_volume.dat ... " << std::flush;
        // app->tsdf_volume_.save ("tsdf_volume.dat", true);
        app->kinfu_->volume ().save ("tsdf_volume.dat", true);
        // std::cout << "done [" << app->tsdf_volume_.size () << " voxels]" << std::endl;
        std::cout << "done [" << app->kinfu_->volume ().size () << " voxels]" << std::endl;
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
  if (format == KinFuLSApp::PCD_BIN)
  {
    std::cout << "Saving point cloud to 'cloud_bin.pcd' (binary)... " << std::flush;
    pcl::io::savePCDFile ("cloud_bin.pcd", *cloud_prt, true);
  }
  else if (format == KinFuLSApp::PCD_ASCII)
  {
    std::cout << "Saving point cloud to 'cloud.pcd' (ASCII)... " << std::flush;
    pcl::io::savePCDFile ("cloud.pcd", *cloud_prt, false);
  }
  else   /* if (format == KinFuLSApp::PLY) */
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
  if (format == KinFuLSApp::MESH_PLY)
  {
    std::cout << "Saving mesh to to 'mesh.ply'... " << std::flush;
    pcl::io::savePLYFile("mesh.ply", mesh);		
  }
  else /* if (format == KinFuLSApp::MESH_VTK) */
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
  std::cout << "\nKinFu parameters:" << std::endl;
  std::cout << "    --help, -h                          : print this message" << std::endl;  
  std::cout << "    --registration, -r                  : try to enable registration (source needs to support this)" << std::endl;
  std::cout << "    --current-cloud, -cc                : show current frame cloud" << std::endl;
  std::cout << "    --save-views, -sv                   : accumulate scene view and save in the end ( Requires OpenCV. Will cause 'bad_alloc' after some time )" << std::endl;  
  std::cout << "    --registration, -r                  : enable registration mode" << std::endl; 
  std::cout << "    --integrate-colors, -ic             : enable color integration mode (allows to get cloud with colors)" << std::endl;
  std::cout << "    --extract-textures, -et             : extract RGB PNG images to KinFuSnapshots folder." << std::endl;
  std::cout << "    --volume_size <in_meters>, -vs      : define integration volume size" << std::endl;
  std::cout << "    --shifting_distance <in_meters>, -sd : define shifting threshold (distance target-point / cube center)" << std::endl;
  std::cout << "    --snapshot_rate <X_frames>, -sr     : Extract RGB textures every <X_frames>. Default: 45  " << std::endl;
  std::cout << std::endl << "";
  std::cout << "Valid depth data sources:" << std::endl; 
  std::cout << "    -dev <device> (default), -oni <oni_file>, -pcd <pcd_file or directory>, -oni2 [file or device id]" << std::endl;
  std::cout << std::endl << "";
  std::cout << " For RGBD benchmark (Requires OpenCV):" << std::endl; 
  std::cout << "    -eval <eval_folder> [-match_file <associations_file_in_the_folder>]" << std::endl << std::endl;

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

  //  if (checkIfPreFermiGPU(device))
  //    return std::cout << std::endl << "Kinfu is supported only for Fermi and Kepler arhitectures. It is not even compiled for pre-Fermi by default. Exiting..." << std::endl, 1;

  pcl::shared_ptr<pcl::Grabber> capture;
  bool triggered_capture = false;
  bool pcd_input = false;
  bool oni2_interface = false;

  std::string eval_folder, match_file, openni_device, oni_file, pcd_dir, oni2_file{};
  try
  {
    #ifdef HAVE_OPENNI
    if (pc::parse_argument (argc, argv, "-dev", openni_device) > 0)
    {
      capture.reset (new pcl::OpenNIGrabber (openni_device));
    }
    else if (pc::parse_argument (argc, argv, "-oni", oni_file) > 0)
    {
      triggered_capture = true;
      bool repeat = false; // Only run ONI file once
      capture.reset (new pcl::ONIGrabber (oni_file, repeat, false));
    }
    #endif
    #if defined(HAVE_OPENNI) && defined(HAVE_OPENNI2)
    else if (pc::parse_argument (argc, argv, "-oni2", oni2_file) > 0)
    #elif defined(HAVE_OPENNI2)
    if (pc::parse_argument(argc, argv, "-oni2", oni2_file) > 0)
    #endif
    #ifdef HAVE_OPENNI2
    {
      capture.reset(new pcl::io::OpenNI2Grabber(oni2_file));
      oni2_interface = true;
    }
    #endif
    else if (pc::parse_argument (argc, argv, "-pcd", pcd_dir) > 0)
    {
      float fps_pcd = 15.0f;
      pc::parse_argument (argc, argv, "-pcd_fps", fps_pcd);

      std::vector<std::string> pcd_files = getPcdFilesInDir(pcd_dir);
      // Sort the read files by name
      sort (pcd_files.begin (), pcd_files.end ());
      capture.reset (new pcl::PCDGrabber<pcl::PointXYZRGBA> (pcd_files, fps_pcd, false));
      triggered_capture = true;
      pcd_input = true;
    }
    else if (pc::parse_argument (argc, argv, "-eval", eval_folder) > 0)
    {
      //init data source latter
      pc::parse_argument (argc, argv, "-match_file", match_file);
    }
    else
    {
      #ifdef HAVE_OPENNI
      capture.reset( new pcl::OpenNIGrabber() );
      #else
      capture.reset(new pcl::io::OpenNI2Grabber(oni2_file));
      oni2_interface = true;
      #endif

      //capture.reset( new pcl::ONIGrabber("d:/onis/20111013-224932.oni", true, true) );
      //capture.reset( new pcl::ONIGrabber("d:/onis/reg20111229-180846.oni, true, true) );    
      //capture.reset( new pcl::ONIGrabber("/media/Main/onis/20111013-224932.oni", true, true) );
      //capture.reset( new pcl::ONIGrabber("d:/onis/20111013-224551.oni", true, true) );
      //capture.reset( new pcl::ONIGrabber("d:/onis/20111013-224719.oni", true, true) );    
    }
  }
  catch (const pcl::PCLException& /*e*/) { return std::cout << "Can't open depth source" << std::endl, -1; }

  float volume_size = pcl::device::kinfuLS::VOLUME_SIZE;
  pc::parse_argument (argc, argv, "--volume_size", volume_size);
  pc::parse_argument (argc, argv, "-vs", volume_size);

  float shift_distance = pcl::device::kinfuLS::DISTANCE_THRESHOLD;
  pc::parse_argument (argc, argv, "--shifting_distance", shift_distance);
  pc::parse_argument (argc, argv, "-sd", shift_distance);

  int snapshot_rate = pcl::device::kinfuLS::SNAPSHOT_RATE; // defined in device.h
  pc::parse_argument (argc, argv, "--snapshot_rate", snapshot_rate);
  pc::parse_argument (argc, argv, "-sr", snapshot_rate);

  KinFuLSApp app (*capture, volume_size, shift_distance, snapshot_rate);

  if (pc::parse_argument (argc, argv, "-eval", eval_folder) > 0)
    app.toggleEvaluationMode(eval_folder, match_file);

  if (pc::find_switch (argc, argv, "--current-cloud") || pc::find_switch (argc, argv, "-cc"))
    app.initCurrentFrameView ();

  if (pc::find_switch (argc, argv, "--save-views") || pc::find_switch (argc, argv, "-sv"))
    app.image_view_.accumulate_views_ = true;  //will cause bad alloc after some time  

  if (pc::find_switch (argc, argv, "--registration") || pc::find_switch (argc, argv, "-r"))  {
    if (pcd_input) {
      app.pcd_source_   = true;
      app.registration_ = true; // since pcd provides registered rgbd
    } else {
      app.initRegistration();
    }
  }

  if (pc::find_switch (argc, argv, "--integrate-colors") || pc::find_switch (argc, argv, "-ic"))      
    app.toggleColorIntegration();

  if (pc::find_switch (argc, argv, "--extract-textures") || pc::find_switch (argc, argv, "-et"))      
    app.enable_texture_extraction_ = true;

  // executing
  if (triggered_capture) 
    std::cout << "Capture mode: triggered\n";
  else				     
    std::cout << "Capture mode: stream\n";

  // set verbosity level
  pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
  try { app.startMainLoop (triggered_capture, oni2_interface); }
  catch (const pcl::PCLException& e) { cout << "PCLException: " << e.what() << endl; }
  catch (const std::bad_alloc& /*e*/) { std::cout << "Bad alloc" << std::endl; }
  catch (const std::exception& e) { cout << "Exception: " << e.what() << endl; }

  //~ #ifdef HAVE_OPENCV
  //~ for (std::size_t t = 0; t < app.image_view_.views_.size (); ++t)
  //~ {
  //~ if (t == 0)
  //~ {
  //~ std::cout << "Saving depth map of first view." << std::endl;
  //~ cv::imwrite ("./depthmap_1stview.png", app.image_view_.views_[0]);
  //~ std::cout << "Saving sequence of (" << app.image_view_.views_.size () << ") views." << std::endl;
  //~ }
  //~ char buf[4096];
  //~ sprintf (buf, "./%06d.png", (int)t);
  //~ cv::imwrite (buf, app.image_view_.views_[t]);
  //~ printf ("writing: %s\n", buf);
  //~ }
  //~ #endif
  std::cout << "pcl_kinfu_largeScale exiting...\n";
  return 0;
}
