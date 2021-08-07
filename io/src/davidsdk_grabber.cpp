/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *  Author: Victor Lamoine (victor.lamoine@gmail.com)
 */

#include <pcl/pcl_config.h>
#include <pcl/io/davidsdk_grabber.h>
#include <pcl/exceptions.h>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>

// Possible improvements:
// TODO: Add presets for david::CodedLightPhaseShiftParams to enable easy scan quality changing
// TODO: Add texture support (call .Scan () instead of .Scan (false) and properly convert data
// TODO: Use mesh IDs rather than clearing all meshes every time
// TODO: In processGrabbing, start scanning again while transferring the mesh (not possible with SDK 1.5.2 because ExportMesh() is blocking)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::DavidSDKGrabber::DavidSDKGrabber () :
    client_connected_ (false),
    running_ (false),
    local_path_ ("C:/temp"),
    remote_path_ ("C:/temp"),
    file_format_ ("stl")
{
  point_cloud_signal_ = createSignal<sig_cb_davidsdk_point_cloud> ();
  mesh_signal_ = createSignal<sig_cb_davidsdk_mesh> ();
  image_signal_ = createSignal<sig_cb_davidsdk_image> ();
  point_cloud_image_signal_ = createSignal<sig_cb_davidsdk_point_cloud_image> ();
  mesh_image_signal_ = createSignal<sig_cb_davidsdk_mesh_image> ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::DavidSDKGrabber::~DavidSDKGrabber () noexcept
{
  try
  {
    stop ();

    disconnect_all_slots<sig_cb_davidsdk_point_cloud> ();
    disconnect_all_slots<sig_cb_davidsdk_mesh> ();
    disconnect_all_slots<sig_cb_davidsdk_image> ();
    disconnect_all_slots<sig_cb_davidsdk_point_cloud_image> ();
    disconnect_all_slots<sig_cb_davidsdk_mesh_image> ();
  }
  catch (...)
  {
    // destructor never throws
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
david::ServerInfo
pcl::DavidSDKGrabber::connect (const std::string &address,
                               std::uint16_t port)
{
  david::ServerInfo server_info;

  if (client_connected_)
    return (server_info);

  try
  {
    david_.Connect (address, port);
    client_connected_ = true;
  }
  catch (david::Exception& e)
  {
    e.PrintError ();
  }

  return (server_info);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DavidSDKGrabber::disconnect (const bool stop_server)
{
  if (!client_connected_)
    return;

  try
  {
    david_.Disconnect (stop_server);
  }
  catch (david::Exception& e)
  {
    e.PrintError ();
  }

  client_connected_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DavidSDKGrabber::start ()
{
  if (isRunning ())
    return;

  frequency_.reset ();
  running_ = true;
  grabber_thread_ = std::thread (&pcl::DavidSDKGrabber::processGrabbing, this);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DavidSDKGrabber::stop ()
{
  if (running_)
  {
    running_ = false;  // Stop processGrabbing () callback

    grabber_thread_.join ();  // join () waits for the thread to finish it's last iteration
    // See: http://www.boost.org/doc/libs/1_54_0/doc/html/thread/thread_management.html#thread.thread_management.thread.join
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::DavidSDKGrabber::isRunning () const
{
  return (running_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::DavidSDKGrabber::isConnected () const
{
  return (client_connected_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::DavidSDKGrabber::getName () const
{
  return ("DavidSDKGrabber");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::DavidSDKGrabber::getLocalPath ()
{
  return (local_path_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::DavidSDKGrabber::getRemotePath ()
{
  return (remote_path_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DavidSDKGrabber::setFileFormatToOBJ ()
{
  file_format_ = "obj";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DavidSDKGrabber::setFileFormatToPLY ()
{
  file_format_ = "ply";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DavidSDKGrabber::setFileFormatToSTL ()
{
  file_format_ = "stl";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::DavidSDKGrabber::getFileFormat ()
{
  return (file_format_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DavidSDKGrabber::setLocalPath (std::string path)
{
  local_path_ = path;

  if (path.empty ())
    local_path_ = "C:/temp";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DavidSDKGrabber::setRemotePath (std::string path)
{
  remote_path_ = path;

  if (path.empty ())
    remote_path_ = local_path_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DavidSDKGrabber::setLocalAndRemotePaths (std::string local_path,
                                              std::string remote_path)
{
  setLocalPath (local_path);
  setRemotePath (remote_path);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::DavidSDKGrabber::calibrate (double grid_size)
{
  if (!client_connected_ || running_)
    return (false);

  try
  {
    david_.sls ().Calibrate (grid_size);
  }
  catch (david::Exception& e)
  {
    e.PrintError ();
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::DavidSDKGrabber::grabSingleCloud (pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  if (!client_connected_ || running_)
    return (false);

  try
  {
    david_.sls ().Scan (false);
    david_.fusion ().DeleteAllMeshes ();
    david_.sls ().AddScanToShapeFusion ();
    david_.sls ().ExportMesh (remote_path_ + "scan." + file_format_);

    pcl::PolygonMesh mesh;
    if (file_format_ == "obj")
    {
      if (pcl::io::loadPolygonFileOBJ (local_path_ + "scan." + file_format_, mesh) == 0)
        return (false);
    }
    else if (file_format_ == "ply")
    {
      if (pcl::io::loadPolygonFilePLY (local_path_ + "scan." + file_format_, mesh) == 0)
        return (false);
    }
    else if (file_format_ == "stl")
    {
      if (pcl::io::loadPolygonFileSTL (local_path_ + "scan." + file_format_, mesh) == 0)
        return (false);
    }
    else
      return (false);

    pcl::fromPCLPointCloud2 (mesh.cloud, cloud);
  }
  catch (david::Exception& e)
  {
    e.PrintError ();
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::DavidSDKGrabber::grabSingleMesh (pcl::PolygonMesh &mesh)
{
  if (!client_connected_ || running_)
    return (false);

  try
  {
    david_.sls ().Scan (false);
    david_.fusion ().DeleteAllMeshes ();
    david_.sls ().AddScanToShapeFusion ();
    david_.sls ().ExportMesh (remote_path_ + "scan." + file_format_);

    if (file_format_ == "obj")
    {
      if (pcl::io::loadPolygonFileOBJ (local_path_ + "scan." + file_format_, mesh) == 0)
        return (false);
    }
    else if (file_format_ == "ply")
    {
      if (pcl::io::loadPolygonFilePLY (local_path_ + "scan." + file_format_, mesh) == 0)
        return (false);
    }
    else if (file_format_ == "stl")
    {
      if (pcl::io::loadPolygonFileSTL (local_path_ + "scan." + file_format_, mesh) == 0)
        return (false);
    }
    else
      return (false);

  }
  catch (david::Exception& e)
  {
    e.PrintError ();
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::DavidSDKGrabber::getFramesPerSecond () const
{
  std::lock_guard<std::mutex> lock (fps_mutex_);
  return (frequency_.getFrequency ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::DavidSDKGrabber::processGrabbing ()
{
  bool continue_grabbing = running_;
  while (continue_grabbing)
  {
    try
    {
      // Publish cloud / images
      if (num_slots<sig_cb_davidsdk_point_cloud> () > 0 || num_slots<sig_cb_davidsdk_mesh> () > 0 || num_slots<sig_cb_davidsdk_image> () > 0
          || num_slots<sig_cb_davidsdk_point_cloud_image> () > 0 || num_slots<sig_cb_davidsdk_mesh_image> () > 0)
      {
        pcl::PolygonMesh::Ptr mesh;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PCLImage::Ptr image;

        fps_mutex_.lock ();
        frequency_.event ();
        fps_mutex_.unlock ();

        // We need the image
        if (num_slots<sig_cb_davidsdk_image> () > 0 || num_slots<sig_cb_davidsdk_point_cloud_image> () > 0 || num_slots<sig_cb_davidsdk_mesh_image> () > 0)
        {
          image.reset (new pcl::PCLImage);
          int width, height;
          david_.sls ().GetLiveImage (image->data, width, height);
          image->width = (std::uint32_t) width;
          image->height = (std::uint32_t) height;
          image->encoding = "CV_8UC1";
        }

        // We need the cloud or mesh
        if (num_slots<sig_cb_davidsdk_point_cloud> () > 0 || num_slots<sig_cb_davidsdk_mesh> () > 0 || num_slots<sig_cb_davidsdk_point_cloud_image> () > 0
            || num_slots<sig_cb_davidsdk_mesh_image> () > 0)
        {
          mesh.reset (new pcl::PolygonMesh);
          david_.sls ().Scan (false);
          david_.fusion ().DeleteAllMeshes ();
          david_.sls ().AddScanToShapeFusion ();
          david_.sls ().ExportMesh (remote_path_ + "scan." + file_format_);

          if (file_format_ == "obj")
          {
            if (pcl::io::loadPolygonFileOBJ (local_path_ + "scan." + file_format_, *mesh) == 0)
              return;
          }
          else if (file_format_ == "ply")
          {
            if (pcl::io::loadPolygonFilePLY (local_path_ + "scan." + file_format_, *mesh) == 0)
              return;
          }
          else if (file_format_ == "stl")
          {
            if (pcl::io::loadPolygonFileSTL (local_path_ + "scan." + file_format_, *mesh) == 0)
              return;
          }
          else
            return;

          if (num_slots<sig_cb_davidsdk_point_cloud> () > 0 || num_slots<sig_cb_davidsdk_point_cloud_image> () > 0)
          {
            cloud.reset (new PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2 (mesh->cloud, *cloud);
          }
        }

        // Publish signals
        if (num_slots<sig_cb_davidsdk_point_cloud_image> () > 0)
          point_cloud_image_signal_->operator () (cloud, image);
        if (num_slots<sig_cb_davidsdk_mesh_image> () > 0)
          mesh_image_signal_->operator () (mesh, image);
        else if (num_slots<sig_cb_davidsdk_point_cloud> () > 0)
          point_cloud_signal_->operator () (cloud);
        else if (num_slots<sig_cb_davidsdk_mesh> () > 0)
          mesh_signal_->operator () (mesh);
        else if (num_slots<sig_cb_davidsdk_image> () > 0)
          image_signal_->operator () (image);
      }
      continue_grabbing = running_;
    }
    catch (david::Exception& e)
    {
      e.PrintError ();
    }
  }
}
