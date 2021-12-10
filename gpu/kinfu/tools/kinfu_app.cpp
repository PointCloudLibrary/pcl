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
#include <vector>

#include <pcl/console/parse.h>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp> // for microsec_clock::local_time

#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/gpu/kinfu/raycaster.h>
#include <pcl/gpu/kinfu/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/exceptions.h>

#include "openni_capture.h"
#include <pcl/visualization/point_cloud_color_handlers.h>
#include "evaluation.h"

#include <pcl/common/angles.h>
#include <pcl/memory.h>

#include "tsdf_volume.h"
#include "tsdf_volume.hpp"

#include "camera_pose.h"

#ifdef HAVE_OPENCV  
  #include <opencv2/highgui/highgui.hpp>
  #include <opencv2/imgproc/imgproc.hpp>
//#include "video_recorder.h"
#endif
using ScopeTimeT = pcl::ScopeTime;

#include "../src/internal.h"

using namespace pcl;
using namespace pcl::gpu;
using namespace Eigen;
using namespace std::chrono_literals;
namespace pc = pcl::console;

namespace pcl
{
  namespace gpu
  {
    void paint3DView (const KinfuTracker::View& rgb24, KinfuTracker::View& view, float colors_weight = 0.5f);
    void mergePointNormal (const DeviceArray<PointXYZ>& cloud, const DeviceArray<Normal>& normals, DeviceArray<PointNormal>& output);
  }

  namespace visualization
  {
    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief RGB handler class for colors. Uses the data present in the "rgb" or "rgba"
      * fields from an additional cloud as the color at each point.
      * \author Anatoly Baksheev
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudColorHandlerRGBCloud : public PointCloudColorHandler<PointT>
    {
      using PointCloudColorHandler<PointT>::capable_;
      using PointCloudColorHandler<PointT>::cloud_;

      using PointCloudConstPtr = typename PointCloudColorHandler<PointT>::PointCloud::ConstPtr;
      using RgbCloudConstPtr = pcl::PointCloud<RGB>::ConstPtr;

      public:
        using Ptr = pcl::shared_ptr<PointCloudColorHandlerRGBCloud<PointT> >;
        using ConstPtr = pcl::shared_ptr<const PointCloudColorHandlerRGBCloud<PointT> >;
        
        /** \brief Constructor. */
        PointCloudColorHandlerRGBCloud (const PointCloudConstPtr& cloud, const RgbCloudConstPtr& colors)
          : rgb_ (colors)
        {
          cloud_  = cloud;
          capable_ = true;
        }
              
        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \return Array of scalars if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), nullptr otherwise
          */
        vtkSmartPointer<vtkDataArray>
        getColor () const override
        {
          if (!capable_ || !cloud_)
            return nullptr;
         
          auto scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  
          scalars->SetNumberOfComponents (3);
            
          vtkIdType nr_points = vtkIdType (cloud_->size ());
          reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);
          unsigned char* colors = reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->GetPointer (0);
            
          // Color every point
          if (nr_points != static_cast<vtkIdType>(rgb_->size ()))
            std::fill (colors, colors + nr_points * 3, static_cast<unsigned char> (0xFF));
          else
            for (vtkIdType cp = 0; cp < nr_points; ++cp)
            {
              int idx = cp * 3;
              colors[idx + 0] = (*rgb_)[cp].r;
              colors[idx + 1] = (*rgb_)[cp].g;
              colors[idx + 2] = (*rgb_)[cp].b;
            }
          return scalars;
        }

      private:
        std::string 
        getFieldName () const override { return ("additional rgb"); }
        std::string 
        getName () const override { return ("PointCloudColorHandlerRGBCloud"); }
        
        RgbCloudConstPtr rgb_;
    };
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
      return pcl::PolygonMesh::Ptr ();

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
  using Ptr = pcl::shared_ptr<CurrentFrameCloudView>;
  using ConstPtr = pcl::shared_ptr<const CurrentFrameCloudView>;

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
  ImageView(int viz) : viz_(viz), paint_image_ (false), accumulate_views_ (false)
  {
    if (viz_)
    {
        viewerScene_ = pcl::visualization::ImageViewer::Ptr(new pcl::visualization::ImageViewer);
        viewerDepth_ = pcl::visualization::ImageViewer::Ptr(new pcl::visualization::ImageViewer);

        viewerScene_->setWindowTitle ("View3D from ray tracing");
        viewerScene_->setPosition (0, 0);
        viewerDepth_->setWindowTitle ("Kinect Depth stream");
        viewerDepth_->setPosition (640, 0);
        //viewerColor_.setWindowTitle ("Kinect RGB stream");
    }
  }

  void
  showScene (KinfuTracker& kinfu, const PtrStepSz<const KinfuTracker::PixelRGB>& rgb24, bool registration, Eigen::Affine3f* pose_ptr = nullptr)
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
    if (viz_)
        viewerScene_->showRGBImage (reinterpret_cast<unsigned char*> (&view_host_[0]), view_device_.cols (), view_device_.rows (), "rgb_image");    

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
  showDepth (const PtrStepSz<const unsigned short>& depth) const 
  { 
     if (viz_)
       viewerDepth_->showShortImage (depth.data, depth.cols, depth.rows, 0, 5000, true, "short_image"); 
  }
  
  void
  showGeneratedDepth (KinfuTracker& kinfu, const Eigen::Affine3f& pose)
  {            
    raycaster_ptr_->run(kinfu.volume(), pose);
    raycaster_ptr_->generateDepthImage(generated_depth_);    

    int c;
    std::vector<unsigned short> data;
    generated_depth_.download(data, c);

    if (viz_)
        viewerDepth_->showShortImage (&data[0], generated_depth_.cols(), generated_depth_.rows(), 0, 5000, true, "short_image");
  }

  void
  toggleImagePaint()
  {
    paint_image_ = !paint_image_;
    std::cout << "Paint image: " << (paint_image_ ? "On   (requires registration mode)" : "Off") << std::endl;
  }

  int viz_;
  bool paint_image_;
  bool accumulate_views_;

  visualization::ImageViewer::Ptr viewerScene_;
  visualization::ImageViewer::Ptr viewerDepth_;
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

  SceneCloudView(int viz) : viz_(viz), extraction_mode_ (GPU_Connected6), compute_normals_ (false), valid_combined_ (false), cube_added_(false)
  {
    cloud_ptr_ = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
    normals_ptr_ = PointCloud<Normal>::Ptr (new PointCloud<Normal>);
    combined_ptr_ = PointCloud<PointNormal>::Ptr (new PointCloud<PointNormal>);
    point_colors_ptr_ = PointCloud<RGB>::Ptr (new PointCloud<RGB>);

    if (viz_)
    {
        cloud_viewer_ = pcl::visualization::PCLVisualizer::Ptr( new pcl::visualization::PCLVisualizer("Scene Cloud Viewer") );

        cloud_viewer_->setBackgroundColor (0, 0, 0);
        cloud_viewer_->addCoordinateSystem (1.0, "global");
        cloud_viewer_->initCameraParameters ();
        cloud_viewer_->setPosition (0, 500);
        cloud_viewer_->setSize (640, 480);
        cloud_viewer_->setCameraClipDistances (0.01, 10.01);

        cloud_viewer_->addText ("H: print help", 2, 15, 20, 34, 135, 246);
    }
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

    if (viz_)
    {
        cloud_viewer_->removeAllPointClouds ();
        if (valid_combined_)
        {
          visualization::PointCloudColorHandlerRGBCloud<PointNormal> rgb(combined_ptr_, point_colors_ptr_);
          cloud_viewer_->addPointCloud<PointNormal> (combined_ptr_, rgb, "Cloud");
          cloud_viewer_->addPointCloudNormals<PointNormal>(combined_ptr_, 50);
        }
        else
        {
          visualization::PointCloudColorHandlerRGBCloud<PointXYZ> rgb(cloud_ptr_, point_colors_ptr_);
          cloud_viewer_->addPointCloud<PointXYZ> (cloud_ptr_, rgb);
        }
    }
  }

  void
  toggleCube(const Eigen::Vector3f& size)
  {
      if (!viz_)
          return;

      if (cube_added_)
          cloud_viewer_->removeShape("cube");
      else
        cloud_viewer_->addCube(size*0.5, Eigen::Quaternionf::Identity(), size(0), size(1), size(2));

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
  clearClouds (bool print_message = false) const
  {
    if (!viz_)
        return;

    cloud_viewer_->removeAllPointClouds ();
    cloud_ptr_->points.clear ();
    normals_ptr_->points.clear ();    
    if (print_message)
      std::cout << "Clouds/Meshes were cleared" << std::endl;
  }

  void
  showMesh(KinfuTracker& kinfu, bool /*integrate_colors*/)
  {
    if (!viz_)
       return;

    ScopeTimeT time ("Mesh Extraction");
    std::cout << "\nGetting mesh... " << std::flush;

    if (!marching_cubes_)
      marching_cubes_ = MarchingCubes::Ptr( new MarchingCubes() );

    DeviceArray<PointXYZ> triangles_device = marching_cubes_->run(kinfu.volume(), triangles_buffer_device_);    
    mesh_ptr_ = convertToMesh(triangles_device);
    
    cloud_viewer_->removeAllPointClouds ();
    if (mesh_ptr_)
      cloud_viewer_->addPolygonMesh(*mesh_ptr_);
    
    std::cout << "Done.  Triangles number: " << triangles_device.size() / MarchingCubes::POINTS_PER_TRIANGLE / 1000 << "K" << std::endl;
  }
    
  int viz_;
  int extraction_mode_;
  bool compute_normals_;
  bool valid_combined_;
  bool cube_added_;

  Eigen::Affine3f viewer_pose_;

  visualization::PCLVisualizer::Ptr cloud_viewer_;

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
  
  KinFuApp(pcl::Grabber& source, float vsz, int icp, int viz, CameraPoseProcessor::Ptr pose_processor=CameraPoseProcessor::Ptr () ) : exit_ (false), scan_ (false), scan_mesh_(false), scan_volume_ (false), independent_camera_ (false),
      registration_ (false), integrate_colors_ (false), pcd_source_ (false), focal_length_(-1.f), capture_ (source), scene_cloud_view_(viz), image_view_(viz), time_ms_(0), icp_(icp), viz_(viz), pose_processor_ (pose_processor)
  {    
    //Init Kinfu Tracker
    Eigen::Vector3f volume_size = Vector3f::Constant (vsz/*meters*/);    
    kinfu_.volume().setSize (volume_size);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
    Eigen::Vector3f t = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

    Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);

    kinfu_.setInitalCameraPose (pose);
    kinfu_.volume().setTsdfTruncDist (0.030f/*meters*/);    
    kinfu_.setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(20.f) ));
    //kinfu_.setDepthTruncationForICP(5.f/*meters*/);
    kinfu_.setCameraMovementThreshold(0.001f);

    if (!icp)
      kinfu_.disableIcp();

    //Init KinfuApp            
    tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
    image_view_.raycaster_ptr_ = RayCaster::Ptr( new RayCaster(kinfu_.rows (), kinfu_.cols ()) );

    if (viz_)
    {
        scene_cloud_view_.cloud_viewer_->registerKeyboardCallback (keyboard_callback, (void*)this);
        image_view_.viewerScene_->registerKeyboardCallback (keyboard_callback, (void*)this);
        image_view_.viewerDepth_->registerKeyboardCallback (keyboard_callback, (void*)this);

        scene_cloud_view_.toggleCube(volume_size);
    }
  }

  ~KinFuApp()
  {
    if (evaluation_ptr_)
      evaluation_ptr_->saveAllPoses(kinfu_);
  }

  void
  initCurrentFrameView ()
  {
    current_frame_cloud_view_ = pcl::make_shared<CurrentFrameCloudView> ();
    current_frame_cloud_view_->cloud_viewer_.registerKeyboardCallback (keyboard_callback, (void*)this);
    current_frame_cloud_view_->setViewerPose (kinfu_.getCameraPose ());
  }

  void
  initRegistration ()
  {        
    registration_ = capture_.providesCallback<pcl::ONIGrabber::sig_cb_openni_image_depth_image> ();
    std::cout << "Registration mode: " << (registration_ ? "On" : "Off (not supported by source)") << std::endl;
    if (registration_)
      kinfu_.setDepthIntrinsics(KINFU_DEFAULT_RGB_FOCAL_X, KINFU_DEFAULT_RGB_FOCAL_Y);
  }
  
  void
  setDepthIntrinsics(std::vector<float> depth_intrinsics)
  {
    float fx = depth_intrinsics[0];
    float fy = depth_intrinsics[1];
    
    if (depth_intrinsics.size() == 4)
    {
        float cx = depth_intrinsics[2];
        float cy = depth_intrinsics[3];
        kinfu_.setDepthIntrinsics(fx, fy, cx, cy);
        std::cout << "Depth intrinsics changed to fx="<< fx << " fy=" << fy << " cx=" << cx << " cy=" << cy << std::endl;
    }
    else {
        kinfu_.setDepthIntrinsics(fx, fy);
        std::cout << "Depth intrinsics changed to fx="<< fx << " fy=" << fy << std::endl;
    }
  }

  void 
  toggleColorIntegration()
  {
    if(registration_)
    {
      const int max_color_integration_weight = 2;
      kinfu_.initColorIntegration(max_color_integration_weight);
      integrate_colors_ = true;      
    }    
    std::cout << "Color integration: " << (integrate_colors_ ? "On" : "Off ( requires registration mode )") << std::endl;
  }
  
  void 
  enableTruncationScaling()
  {
    kinfu_.volume().setTsdfTruncDist (kinfu_.volume().getSize()(0) / 100.0f);
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
  
  void execute(const PtrStepSz<const unsigned short>& depth, const PtrStepSz<const KinfuTracker::PixelRGB>& rgb24, bool has_data)
  {        
    bool has_image = false;
      
    if (has_data)
    {
      depth_device_.upload (depth.data, depth.step, depth.rows, depth.cols);
      if (integrate_colors_)
          image_view_.colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
    
      {
        SampledScopeTime fps(time_ms_);
    
        //run kinfu algorithm
        if (integrate_colors_)
          has_image = kinfu_ (depth_device_, image_view_.colors_device_);
        else
          has_image = kinfu_ (depth_device_);                  
      }

      // process camera pose
      if (pose_processor_)
      {
        pose_processor_->processPose (kinfu_.getCameraPose ());
      }

      image_view_.showDepth (depth);
      //image_view_.showGeneratedDepth(kinfu_, kinfu_.getCameraPose());
    }

    if (scan_)
    {
      scan_ = false;
      scene_cloud_view_.show (kinfu_, integrate_colors_);
                    
      if (scan_volume_)
      {                
        std::cout << "Downloading TSDF volume from device ... " << std::flush;
        kinfu_.volume().downloadTsdfAndWeighs (tsdf_volume_.volumeWriteable (), tsdf_volume_.weightsWriteable ());
        tsdf_volume_.setHeader (Eigen::Vector3i (pcl::device::VOLUME_X, pcl::device::VOLUME_Y, pcl::device::VOLUME_Z), kinfu_.volume().getSize ());
        std::cout << "done [" << tsdf_volume_.size () << " voxels]" << std::endl << std::endl;
                
        std::cout << "Converting volume to TSDF cloud ... " << std::flush;
        tsdf_volume_.convertToTsdfCloud (tsdf_cloud_ptr_);
        std::cout << "done [" << tsdf_cloud_ptr_->size () << " points]" << std::endl << std::endl;        
      }
      else
        std::cout << "[!] tsdf volume download is disabled" << std::endl << std::endl;
    }

    if (scan_mesh_)
    {
        scan_mesh_ = false;
        scene_cloud_view_.showMesh(kinfu_, integrate_colors_);
    }
     
    if (viz_ && has_image)
    {
      Eigen::Affine3f viewer_pose = getViewerPose(*scene_cloud_view_.cloud_viewer_);
      image_view_.showScene (kinfu_, rgb24, registration_, independent_camera_ ? &viewer_pose : nullptr);
    }    

    if (current_frame_cloud_view_)
      current_frame_cloud_view_->show (kinfu_);    
      
    if (viz_ && !independent_camera_)
      setViewerPose (*scene_cloud_view_.cloud_viewer_, kinfu_.getCameraPose());
  }
  
  void source_cb1_device(const openni_wrapper::DepthImage::Ptr& depth_wrapper)
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

  void source_cb2_device(const openni_wrapper::Image::Ptr& image_wrapper, const openni_wrapper::DepthImage::Ptr& depth_wrapper, float)
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
      
      rgb24_.cols = image_wrapper->getWidth();
      rgb24_.rows = image_wrapper->getHeight();
      rgb24_.step = rgb24_.cols * rgb24_.elemSize(); 

      source_image_data_.resize(rgb24_.cols * rgb24_.rows);
      image_wrapper->fillRGB(rgb24_.cols, rgb24_.rows, (unsigned char*)&source_image_data_[0]);
      rgb24_.data = &source_image_data_[0];           
    }
    data_ready_cond_.notify_one();
  }


   void source_cb1_oni(const openni_wrapper::DepthImage::Ptr& depth_wrapper)
  {        
    {
      std::lock_guard<std::mutex> lock(data_ready_mutex_);
      if (exit_)
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

  void source_cb2_oni(const openni_wrapper::Image::Ptr& image_wrapper, const openni_wrapper::DepthImage::Ptr& depth_wrapper, float)
  {
    {
      std::lock_guard<std::mutex> lock(data_ready_mutex_);
      if (exit_)
          return;
                  
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

  void
  source_cb3 (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & DC3)
  {
    {
      std::unique_lock<std::mutex> lock (data_ready_mutex_, std::try_to_lock);
      if (exit_ || !lock)
        return;
      int width  = DC3->width;
      int height = DC3->height;
      depth_.cols = width;
      depth_.rows = height;
      depth_.step = depth_.cols * depth_.elemSize ();
      source_depth_data_.resize (depth_.cols * depth_.rows);

      rgb24_.cols = width;
      rgb24_.rows = height;
      rgb24_.step = rgb24_.cols * rgb24_.elemSize ();
      source_image_data_.resize (rgb24_.cols * rgb24_.rows);

      unsigned char *rgb    = (unsigned char *)  &source_image_data_[0];
      unsigned short *depth = (unsigned short *) &source_depth_data_[0];

      for (int i=0; i < width*height; i++) 
      {
        PointXYZRGBA pt = DC3->at (i);
        rgb[3*i +0] = pt.r;
        rgb[3*i +1] = pt.g;
        rgb[3*i +2] = pt.b;
        depth[i]    = pt.z/0.001;
      }
      rgb24_.data = &source_image_data_[0];
      depth_.data = &source_depth_data_[0];
    }
    data_ready_cond_.notify_one ();
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
  startMainLoop (bool triggered_capture)
  {   
    using namespace openni_wrapper;
    using DepthImagePtr = DepthImage::Ptr;
    using ImagePtr = Image::Ptr;

    std::function<void (const ImagePtr&, const DepthImagePtr&, float)> func1_dev = [this] (const ImagePtr& img, const DepthImagePtr& depth, float constant)
    {
      source_cb2_device (img, depth, constant);
    };
    std::function<void (const DepthImagePtr&)> func2_dev = [this] (const DepthImagePtr& depth) { source_cb1_device (depth); };

    std::function<void (const ImagePtr&, const DepthImagePtr&, float)> func1_oni = [this] (const ImagePtr& img, const DepthImagePtr& depth, float constant)
    {
      source_cb2_oni (img, depth, constant);
    };
    std::function<void (const DepthImagePtr&)> func2_oni = [this] (const DepthImagePtr& depth) { source_cb1_oni (depth); };

    bool is_oni = dynamic_cast<pcl::ONIGrabber*>(&capture_) != nullptr;
    std::function<void (const ImagePtr&, const DepthImagePtr&, float constant)> func1 = is_oni ? func1_oni : func1_dev;
    std::function<void (const DepthImagePtr&)> func2 = is_oni ? func2_oni : func2_dev;

    std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > func3 = [this] (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
    {
      source_cb3 (cloud);
    };

    bool need_colors = integrate_colors_ || registration_;
    if ( pcd_source_ && !capture_.providesCallback<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>() )
    {
      std::cout << "grabber doesn't provide pcl::PointCloud<pcl::PointXYZRGBA> callback !\n";
    }
    boost::signals2::connection c = pcd_source_? capture_.registerCallback (func3) : need_colors ? capture_.registerCallback (func1) : capture_.registerCallback (func2);

    {
      std::unique_lock<std::mutex> lock(data_ready_mutex_);

      if (!triggered_capture)
          capture_.start (); // Start stream

      bool scene_view_not_stopped= viz_ ? !scene_cloud_view_.cloud_viewer_->wasStopped () : true;
      bool image_view_not_stopped= viz_ ? !image_view_.viewerScene_->wasStopped () : true;
          
      while (!exit_ && scene_view_not_stopped && image_view_not_stopped)
      { 
        if (triggered_capture)
            capture_.start(); // Triggers new frame
        bool has_data = (data_ready_cond_.wait_for(lock, 100ms) == std::cv_status::no_timeout);
                       
        try { this->execute (depth_, rgb24_, has_data); }
        catch (const std::bad_alloc& /*e*/) { std::cout << "Bad alloc" << std::endl; break; }
        catch (const std::exception& /*e*/) { std::cout << "Exception" << std::endl; break; }
        
        if (viz_)
            scene_cloud_view_.cloud_viewer_->spinOnce (3);
      }
      
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

    // Points to export are either in cloud_ptr_ or combined_ptr_.
    // If none have points, we have nothing to export.
    if (view.cloud_ptr_->points.empty () && view.combined_ptr_->points.empty ())
    {
      std::cout << "Not writing cloud: Cloud is empty" << std::endl;
    }
    else
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
    std::cout << std::endl;
  }  

  bool exit_;
  bool scan_;
  bool scan_mesh_;
  bool scan_volume_;

  bool independent_camera_;

  bool registration_;
  bool integrate_colors_;
  bool pcd_source_;
  float focal_length_;
  
  pcl::Grabber& capture_;
  KinfuTracker kinfu_;

  SceneCloudView scene_cloud_view_;
  ImageView image_view_;
  CurrentFrameCloudView::Ptr current_frame_cloud_view_;

  KinfuTracker::DepthMap depth_device_;

  pcl::TSDFVolume<float, short> tsdf_volume_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;

  Evaluation::Ptr evaluation_ptr_;
  
  std::mutex data_ready_mutex_;
  std::condition_variable data_ready_cond_;
 
  std::vector<KinfuTracker::PixelRGB> source_image_data_;
  std::vector<unsigned short> source_depth_data_;
  PtrStepSz<const unsigned short> depth_;
  PtrStepSz<const KinfuTracker::PixelRGB> rgb24_;

  int time_ms_;
  int icp_, viz_;

  CameraPoseProcessor::Ptr pose_processor_;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
        app->tsdf_volume_.save ("tsdf_volume.dat", true);
        std::cout << "done [" << app->tsdf_volume_.size () << " voxels]" << std::endl;
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
  std::cout << "\nKinFu parameters:" << std::endl;
  std::cout << "    --help, -h                              : print this message" << std::endl;  
  std::cout << "    --registration, -r                      : try to enable registration (source needs to support this)" << std::endl;
  std::cout << "    --current-cloud, -cc                    : show current frame cloud" << std::endl;
  std::cout << "    --save-views, -sv                       : accumulate scene view and save in the end ( Requires OpenCV. Will cause 'bad_alloc' after some time )" << std::endl;  
  std::cout << "    --integrate-colors, -ic                 : enable color integration mode (allows to get cloud with colors)" << std::endl;   
  std::cout << "    --scale-truncation, -st                 : scale the truncation distance and raycaster based on the volume size" << std::endl;
  std::cout << "    -volume_size <size_in_meters>           : define integration volume size" << std::endl;
  std::cout << "    --depth-intrinsics <fx>,<fy>[,<cx>,<cy> : set the intrinsics of the depth camera" << std::endl;
  std::cout << "    -save_pose <pose_file.csv>              : write tracked camera positions to the specified file" << std::endl;
  std::cout << "Valid depth data sources:" << std::endl; 
  std::cout << "    -dev <device> (default), -oni <oni_file>, -pcd <pcd_file or directory>" << std::endl;
  std::cout << "";
  std::cout << " For RGBD benchmark (Requires OpenCV):" << std::endl; 
  std::cout << "    -eval <eval_folder> [-match_file <associations_file_in_the_folder>]" << std::endl;
    
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
  
  std::unique_ptr<pcl::Grabber> capture;
  
  bool triggered_capture = false;
  bool pcd_input = false;
  
  std::string eval_folder, match_file, openni_device, oni_file, pcd_dir;
  try
  {    
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
      capture.reset( new pcl::OpenNIGrabber() );
        
      //triggered_capture = true; capture.reset( new pcl::ONIGrabber("d:/onis/20111013-224932.oni", true, ! triggered_capture) );
      //triggered_capture = true; capture.reset( new pcl::ONIGrabber("d:/onis/reg20111229-180846.oni, true, ! triggered_capture) );    
      //triggered_capture = true; capture.reset( new pcl::ONIGrabber("/media/Main/onis/20111013-224932.oni", true, ! triggered_capture) );
      //triggered_capture = true; capture.reset( new pcl::ONIGrabber("d:/onis/20111013-224551.oni", true, ! triggered_capture) );
      //triggered_capture = true; capture.reset( new pcl::ONIGrabber("d:/onis/20111013-224719.oni", true, ! triggered_capture) );    
    }
  }
  catch (const pcl::PCLException& /*e*/) { return std::cout << "Can't open depth source" << std::endl, -1; }

  float volume_size = 3.f;
  pc::parse_argument (argc, argv, "-volume_size", volume_size);

  int icp = 1, visualization = 1;
  std::vector<float> depth_intrinsics;
  pc::parse_argument (argc, argv, "--icp", icp);
  pc::parse_argument (argc, argv, "--viz", visualization);
        
  std::string camera_pose_file;
  CameraPoseProcessor::Ptr pose_processor;
  if (pc::parse_argument (argc, argv, "-save_pose", camera_pose_file) && !camera_pose_file.empty ())
  {
    pose_processor.reset (new CameraPoseWriter (camera_pose_file));
  }

  KinFuApp app (*capture, volume_size, icp, visualization, pose_processor);

  if (pc::parse_argument (argc, argv, "-eval", eval_folder) > 0)
    app.toggleEvaluationMode(eval_folder, match_file);

  if (pc::find_switch (argc, argv, "--current-cloud") || pc::find_switch (argc, argv, "-cc"))
    app.initCurrentFrameView ();

  if (pc::find_switch (argc, argv, "--save-views") || pc::find_switch (argc, argv, "-sv"))
    app.image_view_.accumulate_views_ = true;  //will cause bad alloc after some time  
  
  if (pc::find_switch (argc, argv, "--registration") || pc::find_switch (argc, argv, "-r"))
  {
    if (pcd_input) 
    {
      app.pcd_source_   = true;
      app.registration_ = true; // since pcd provides registered rgbd
    } 
    else 
    {
      app.initRegistration ();
    }
  }
  if (pc::find_switch (argc, argv, "--integrate-colors") || pc::find_switch (argc, argv, "-ic"))
    app.toggleColorIntegration();

  if (pc::find_switch (argc, argv, "--scale-truncation") || pc::find_switch (argc, argv, "-st"))
    app.enableTruncationScaling();
  
  if (pc::parse_x_arguments (argc, argv, "--depth-intrinsics", depth_intrinsics) > 0)
  {
    if ((depth_intrinsics.size() == 4) || (depth_intrinsics.size() == 2))
    {
       app.setDepthIntrinsics(depth_intrinsics);
    }
    else
    {
        pc::print_error("Depth intrinsics must be given on the form fx,fy[,cx,cy].\n");
        return -1;
    }   
  }

  // executing
  try { app.startMainLoop (triggered_capture); }
  catch (const pcl::PCLException& /*e*/) { std::cout << "PCLException" << std::endl; }
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
