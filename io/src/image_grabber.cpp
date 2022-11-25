/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 */
// Looking for PCL_BUILT_WITH_VTK
#include <pcl/for_each_type.h>
#include <pcl/io/image_grabber.h>
#include <pcl/io/lzf_image_io.h>
#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp> // for exists, basename, is_directory, ...
#include <boost/algorithm/string/case_conv.hpp> // for to_upper_copy
#include <boost/date_time/posix_time/posix_time.hpp> // for posix_time

#ifdef PCL_BUILT_WITH_VTK
  #include <vtkImageReader2.h>
  #include <vtkImageReader2Factory.h>
  #include <vtkImageData.h>
  #include <vtkSmartPointer.h>
  #include <vtkTIFFReader.h>
  #include <vtkPNGReader.h>
  #include <vtkJPEGReader.h>
  #include <vtkPNMReader.h>
#endif

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// GrabberImplementation //////////////////////
struct pcl::ImageGrabberBase::ImageGrabberImpl
{
  //! Implementation of ImageGrabber
  ImageGrabberImpl (pcl::ImageGrabberBase& grabber,
                    const std::string& dir,
                    float frames_per_second,
                    bool repeat,
                    bool pclzf_mode=false);
  //! For now, split rgb / depth folders only makes sense for VTK images
  ImageGrabberImpl (pcl::ImageGrabberBase& grabber,
                    const std::string& depth_dir,
                    const std::string& rgb_dir,
                    float frames_per_second,
                    bool repeat);
  ImageGrabberImpl (pcl::ImageGrabberBase& grabber,
                    const std::vector<std::string>& depth_image_files,
                    float frames_per_second,
                    bool repeat);

  void
  trigger ();
  //! Read ahead -- figure out whether we are in VTK image or PCLZF mode
  void
  loadNextCloud ();

  //! Get cloud at a particular location
  bool
  getCloudAt (std::size_t idx, pcl::PCLPointCloud2 &blob, Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
              double &fx, double &fy, double &cx, double &cy) const;

  //! Get cloud at a particular location
  bool
  getCloudVTK (std::size_t idx, pcl::PCLPointCloud2 &blob, Eigen::Vector4f &origin, Eigen::Quaternionf &orientation) const;
  //! Get cloud at a particular location
  bool
  getCloudPCLZF (std::size_t idx, pcl::PCLPointCloud2 &blob, Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                 double &fx, double &fy, double &cx, double &cy) const;

  //! Scrapes a directory for image files which contain "rgb" or "depth" and
  //! updates our list accordingly
  void
  loadDepthAndRGBFiles (const std::string &dir);
  //! Scrapes a directory for image files which contain "rgb" or "depth" and
  //! updates our list accordingly
  void
  loadDepthAndRGBFiles (const std::string &depth_dir, const std::string &rgb_dir);
  //! Scrapes a directory for pclzf files which contain "rgb" or "depth and updates
  //  our list accordingly
  void
  loadPCLZFFiles (const std::string &dir);

  //! True if it is an image we know how to read
  bool
  isValidExtension (const std::string &extension) const;

  //! Convenience function to rewind to the last frame
  void
  rewindOnce ();

  //! Checks if a timestamp is given in the filename
  //! And returns if so
  bool
  getTimestampFromFilepath (const std::string &filepath, std::uint64_t &timestamp) const;

  std::size_t
  numFrames () const;


#ifdef PCL_BUILT_WITH_VTK
  //! Load an image file, return the vtkImageReader2, return false if it couldn't be opened
  bool
  getVtkImage (const std::string &filename, vtkSmartPointer<vtkImageData> &image) const;
#endif//PCL_BUILT_WITH_VTK

  pcl::ImageGrabberBase& grabber_;
  float frames_per_second_;
  bool repeat_;
  bool running_ = false;
  // VTK
  std::vector<std::string> depth_image_files_;
  std::vector<std::string> rgb_image_files_;
  // PCLZF
  std::vector<std::string> depth_pclzf_files_;
  std::vector<std::string> rgb_pclzf_files_;
  std::vector<std::string> xml_files_;

  std::size_t cur_frame_ = 0;

  TimeTrigger time_trigger_;

  pcl::PCLPointCloud2 next_cloud_;
  //! Two cases, for depth only and depth+color
  pcl::PointCloud<pcl::PointXYZ> next_cloud_depth_;
  pcl::PointCloud<pcl::PointXYZRGBA> next_cloud_color_;
  Eigen::Vector4f origin_;
  Eigen::Quaternionf orientation_;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  bool valid_ = false;
  //! Flag to say if a user set the focal length by hand
  //  (so we don't attempt to adjust for QVGA, QQVGA, etc).
  bool pclzf_mode_ = false;

  float depth_image_units_ = 1E-3f;

  bool manual_intrinsics_ = false;
  double focal_length_x_ = 525.;
  double focal_length_y_ = 525.;
  double principal_point_x_ = 319.5;
  double principal_point_y_ = 239.5;

  unsigned int num_threads_ = 1;
};

///////////////////////////////////////////////////////////////////////////////////////////
pcl::ImageGrabberBase::ImageGrabberImpl::ImageGrabberImpl (pcl::ImageGrabberBase& grabber,
                                                           const std::string& dir,
                                                           float frames_per_second,
                                                           bool repeat,
                                                           bool pclzf_mode)
  : grabber_ (grabber)
  , frames_per_second_ (frames_per_second)
  , repeat_ (repeat)
  , time_trigger_ (1.0 / static_cast<double> (std::max (frames_per_second, 0.001f)), [this] { trigger (); })
  , pclzf_mode_(pclzf_mode)
{
  if(pclzf_mode_)
  {
    loadPCLZFFiles(dir);
  }
  else
  {
    loadDepthAndRGBFiles (dir);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::ImageGrabberBase::ImageGrabberImpl::ImageGrabberImpl (pcl::ImageGrabberBase& grabber,
                                                           const std::string& depth_dir,
                                                           const std::string& rgb_dir,
                                                           float frames_per_second,
                                                           bool repeat)
  : grabber_ (grabber)
  , frames_per_second_ (frames_per_second)
  , repeat_ (repeat)
  , time_trigger_ (1.0 / static_cast<double> (std::max (frames_per_second, 0.001f)), [this] { trigger (); })
{
  loadDepthAndRGBFiles (depth_dir, rgb_dir);
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::ImageGrabberBase::ImageGrabberImpl::ImageGrabberImpl (pcl::ImageGrabberBase& grabber,
                                                           const std::vector<std::string>& depth_image_files,
                                                           float frames_per_second,
                                                           bool repeat)
  : grabber_ (grabber)
  , frames_per_second_ (frames_per_second)
  , repeat_ (repeat)
  , depth_image_files_ (depth_image_files)
  , time_trigger_ (1.0 / static_cast<double> (std::max (frames_per_second, 0.001f)), [this] { trigger (); })
{
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::ImageGrabberImpl::loadNextCloud ()
{
  if (cur_frame_ >= numFrames ())
  {
    if (repeat_)
      cur_frame_ = 0;
    else
    {
      valid_ = false;
      return;
    }
  }
  valid_ = getCloudAt (cur_frame_, next_cloud_, origin_, orientation_,
      focal_length_x_, focal_length_y_, principal_point_x_, principal_point_y_);
  cur_frame_++;
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::ImageGrabberImpl::trigger ()
{
  if (valid_)
  {
    grabber_.publish (next_cloud_,origin_,orientation_);
  }
  // Preload the next cloud
  loadNextCloud ();
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::ImageGrabberImpl::loadDepthAndRGBFiles (const std::string &dir)
{
  if (!boost::filesystem::exists (dir) || !boost::filesystem::is_directory (dir))
  {
    PCL_ERROR ("[pcl::ImageGrabber::loadDepthAndRGBFiles] Error: attempted to instantiate a pcl::ImageGrabber from a path which"
               " is not a directory: %s\n", dir.c_str ());
    return;
  }
  std::string pathname;
  std::string extension;
  std::string basename;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    extension = boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ()));
    pathname = itr->path ().string ();
    basename = boost::filesystem::basename (itr->path ());
    if (!boost::filesystem::is_directory (itr->status ())
        && isValidExtension (extension))
    {
      if (basename.find ("rgb") < std::string::npos)
      {
        rgb_image_files_.push_back (pathname);
      }
      else if (basename.find ("depth") < std::string::npos)
      {
        depth_image_files_.push_back (pathname);
      }
    }
  }
  sort (depth_image_files_.begin (), depth_image_files_.end ());
  if (!rgb_image_files_.empty ())
    sort (rgb_image_files_.begin (), rgb_image_files_.end ());
}

void
pcl::ImageGrabberBase::ImageGrabberImpl::loadDepthAndRGBFiles (const std::string &depth_dir, const std::string &rgb_dir)
{
  if (!boost::filesystem::exists (depth_dir) || !boost::filesystem::is_directory (depth_dir))
  {
    PCL_ERROR ("[pcl::ImageGrabber::loadDepthAndRGBFiles] Error: attempted to instantiate a pcl::ImageGrabber from a path which"
               " is not a directory: %s\n", depth_dir.c_str ());
    return;
  }
  if (!boost::filesystem::exists (rgb_dir) || !boost::filesystem::is_directory (rgb_dir))
  {
    PCL_ERROR ("[pcl::ImageGrabber::loadDepthAndRGBFiles] Error: attempted to instantiate a pcl::ImageGrabber from a path which"
               " is not a directory: %s\n", rgb_dir.c_str ());
    return;
  }
  std::string pathname;
  std::string extension;
  std::string basename;
  boost::filesystem::directory_iterator end_itr;
  // First iterate over depth images
  for (boost::filesystem::directory_iterator itr (depth_dir); itr != end_itr; ++itr)
  {
    extension = boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ()));
    pathname = itr->path ().string ();
    basename = boost::filesystem::basename (itr->path ());
    if (!boost::filesystem::is_directory (itr->status ())
        && isValidExtension (extension))
    {
      if (basename.find ("depth") < std::string::npos)
      {
        depth_image_files_.push_back (pathname);
      }
    }
  }
  // Then iterate over RGB images
  for (boost::filesystem::directory_iterator itr (rgb_dir); itr != end_itr; ++itr)
  {
    extension = boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ()));
    pathname = itr->path ().string ();
    basename = boost::filesystem::basename (itr->path ());
    if (!boost::filesystem::is_directory (itr->status ())
        && isValidExtension (extension))
    {
      if (basename.find ("rgb") < std::string::npos)
      {
        rgb_image_files_.push_back (pathname);
      }
    }
  }
  if (depth_image_files_.size () != rgb_image_files_.size () )
    PCL_WARN ("[pcl::ImageGrabberBase::ImageGrabberImpl::loadDepthAndRGBFiles] : Watch out not same amount of depth and rgb images\n");
  if (!depth_image_files_.empty ())
    sort (depth_image_files_.begin (), depth_image_files_.end ());
  else
    PCL_ERROR ("[pcl::ImageGrabberBase::ImageGrabberImpl::loadDepthAndRGBFiles] : no depth images added\n");
  if (!rgb_image_files_.empty ())
    sort (rgb_image_files_.begin (), rgb_image_files_.end ());
  else
    PCL_ERROR ("[pcl::ImageGrabberBase::ImageGrabberImpl::loadDepthAndRGBFiles] : no rgb images added\n");
}


///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::ImageGrabberImpl::loadPCLZFFiles (const std::string &dir)
{
  if (!boost::filesystem::exists (dir) || !boost::filesystem::is_directory (dir))
  {
    PCL_ERROR ("[pcl::ImageGrabber::loadPCLZFFiles] Error: attempted to instantiate a pcl::ImageGrabber from a path which"
               " is not a directory: %s\n", dir.c_str ());
    return;
  }
  std::string pathname;
  std::string extension;
  std::string basename;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    extension = boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ()));
    pathname = itr->path ().string ();
    basename = boost::filesystem::basename (itr->path ());
    if (!boost::filesystem::is_directory (itr->status ())
        && isValidExtension (extension))
    {
      if (basename.find ("rgb") < std::string::npos)
        rgb_pclzf_files_.push_back (pathname);
      else if (basename.find ("depth") < std::string::npos)
        depth_pclzf_files_.push_back (pathname);
      else
        xml_files_.push_back (pathname);

    }
  }
  sort (depth_pclzf_files_.begin (), depth_pclzf_files_.end ());
  if (!rgb_pclzf_files_.empty ())
    sort (rgb_pclzf_files_.begin (), rgb_pclzf_files_.end ());
  sort (xml_files_.begin(), xml_files_.end());
  if (depth_pclzf_files_.size() != xml_files_.size())
  {
    PCL_ERROR("[pcl::ImageGrabber::loadPCLZFFiles] # depth clouds != # xml files\n");
    return;
  }
  if (depth_pclzf_files_.size() != rgb_pclzf_files_.size() && !rgb_pclzf_files_.empty ())
  {
    PCL_ERROR("[pcl::ImageGrabber::loadPCLZFFiles] # depth clouds != # rgb clouds\n");
    return;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::ImageGrabberBase::ImageGrabberImpl::isValidExtension (const std::string &extension) const
{
  bool valid;
  if(pclzf_mode_)
  {
    valid = extension == ".PCLZF" || extension == ".XML";
  }
  else
  {
    valid = extension == ".TIFF" || extension == ".PNG"
         || extension == ".JPG" || extension == ".JPEG"
         || extension == ".PPM";
  }
  return (valid);
}

void
pcl::ImageGrabberBase::ImageGrabberImpl::rewindOnce ()
{
  if (cur_frame_ > 0)
    cur_frame_--;
}

//////////////////////////////////////////////////////////////////////////
bool
pcl::ImageGrabberBase::ImageGrabberImpl::getTimestampFromFilepath (
    const std::string &filepath,
    std::uint64_t &timestamp) const
{
  // For now, we assume the file is of the form frame_[22-char POSIX timestamp]_*
  char timestamp_str[256];
  int result = std::sscanf (boost::filesystem::basename (filepath).c_str (),
                            "frame_%22s_%*s",
                            timestamp_str);
  if (result > 0)
  {
    // Convert to std::uint64_t, microseconds since 1970-01-01
    boost::posix_time::ptime cur_date = boost::posix_time::from_iso_string (timestamp_str);
    boost::posix_time::ptime zero_date (
        boost::gregorian::date (1970,boost::gregorian::Jan,1));
    timestamp = (cur_date - zero_date).total_microseconds ();
    return (true);
  }
  return (false);
}

/////////////////////////////////////////////////////////////////////////////
bool
pcl::ImageGrabberBase::ImageGrabberImpl::getCloudAt (std::size_t idx,
                                                     pcl::PCLPointCloud2 &blob,
                                                     Eigen::Vector4f &origin,
                                                     Eigen::Quaternionf &orientation,
                                                     double &fx,
                                                     double &fy,
                                                     double &cx,
                                                     double &cy) const
{
  if (!depth_image_files_.empty ())
  {
    fx = focal_length_x_;
    fy = focal_length_y_;
    cx = principal_point_x_;
    cy = principal_point_y_;
    return (getCloudVTK (idx, blob, origin, orientation) );
  }
  if (!depth_pclzf_files_.empty ())
    return (getCloudPCLZF (idx, blob, origin, orientation, fx, fy, cx, cy) );
  PCL_ERROR ("[pcl::ImageGrabber::getCloudAt] Could not find VTK or PCLZF files.\n");
  return (false);
}

bool
pcl::ImageGrabberBase::ImageGrabberImpl::getCloudVTK (std::size_t idx,
                                                      pcl::PCLPointCloud2 &blob,
                                                      Eigen::Vector4f &origin,
                                                      Eigen::Quaternionf &orientation) const
{
#ifdef PCL_BUILT_WITH_VTK
  if (idx > depth_image_files_.size ())
  {
    return (false);
  }
  unsigned short* depth_pixel;
  unsigned char* color_pixel;
  vtkSmartPointer<vtkImageData> depth_image;
  vtkSmartPointer<vtkImageData> rgb_image;
  const std::string &depth_image_file = depth_image_files_[idx];
  // If there are RGB files, load an rgb image
  if (!rgb_image_files_.empty ())
  {
    const std::string &rgb_image_file = rgb_image_files_[idx];
    // If we were unable to pull a Vtk image, throw an error
    if (!getVtkImage (rgb_image_file, rgb_image) )
    {
      return (false);
    }
  }
  if (!getVtkImage (depth_image_file, depth_image) )
  {
    return (false);
  }
  int* dims = depth_image->GetDimensions ();

  // Fill in image data
  depth_pixel = static_cast<unsigned short*>(depth_image->GetScalarPointer ());

  // Set up intrinsics
  float scaleFactorX, scaleFactorY;
  float centerX, centerY;
  if (manual_intrinsics_)
  {
    scaleFactorX = 1.f / static_cast<float> (focal_length_x_);
    scaleFactorY = 1.f / static_cast<float> (focal_length_y_);
    centerX = static_cast<float> (principal_point_x_);
    centerY = static_cast<float> (principal_point_y_);
  }
  else
  {
    // The 525 factor default is only true for VGA. If not, we should scale
    scaleFactorX = scaleFactorY = 1/525.f * 640.f / static_cast<float> (dims[0]);
    centerX = ((float)dims[0] - 1.f)/2.f;
    centerY = ((float)dims[1] - 1.f)/2.f;
  }

  if(!rgb_image_files_.empty ())
  {
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_color;
    cloud_color.width = dims[0];
    cloud_color.height = dims[1];
    cloud_color.is_dense = false;
    cloud_color.resize (depth_image->GetNumberOfPoints ());

    for (int y = 0; y < dims[1]; ++y)
    {
      for (int x = 0; x < dims[0]; ++x, ++depth_pixel)
      {
        pcl::PointXYZRGBA &pt = cloud_color.at (x,y);
        float depth = static_cast<float> (*depth_pixel) * depth_image_units_;
        if (depth == 0.0f)
          pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
        else
        {
          pt.x = (static_cast<float> (x) - centerX) * scaleFactorX * depth;
          pt.y = (static_cast<float> (y) - centerY) * scaleFactorY * depth;
          pt.z = depth;
        }

        color_pixel = reinterpret_cast<unsigned char*> (rgb_image->GetScalarPointer (x, y, 0));
        pt.r = color_pixel[0];
        pt.g = color_pixel[1];
        pt.b = color_pixel[2];
      }
    }
    // Handle timestamps
    std::uint64_t timestamp;
    if (getTimestampFromFilepath (depth_image_file, timestamp))
    {
      cloud_color.header.stamp = timestamp;
    }

    pcl::toPCLPointCloud2 (cloud_color, blob);
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = dims[0];
    cloud.height = dims[1];
    cloud.is_dense = false;
    cloud.resize (depth_image->GetNumberOfPoints ());
    for (int y = 0; y < dims[1]; ++y)
    {
      for (int x = 0; x < dims[0]; ++x, ++depth_pixel)
      {
        pcl::PointXYZ &pt = cloud.at (x,y);
        float depth = static_cast<float> (*depth_pixel) * depth_image_units_;
        if (depth == 0.0f)
          pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
        else
        {
          pt.x = ((float)x - centerX) * scaleFactorX * depth;
          pt.y = ((float)y - centerY) * scaleFactorY * depth;
          pt.z = depth;
        }
      }
    }
    // Handle timestamps
    std::uint64_t timestamp;
    if (getTimestampFromFilepath (depth_image_file, timestamp))
    {
      cloud.header.stamp = timestamp;
    }

    pcl::toPCLPointCloud2 (cloud, blob);
  }
  // Origin 0, orientation is forward
  origin = Eigen::Vector4f::Zero ();
  orientation = Eigen::Quaternionf::Identity ();

  return (true);
#else
  pcl::utils::ignore(idx, blob, origin, orientation);
  PCL_ERROR ("[pcl::ImageGrabber::loadNextCloudVTK] Attempted to read image files, but PCL was not built with VTK [no -DPCL_BUILT_WITH_VTK]. \n");
  return false;
#endif //PCL_BUILT_WITH_VTK
}

bool
pcl::ImageGrabberBase::ImageGrabberImpl::getCloudPCLZF (std::size_t idx,
                                                        pcl::PCLPointCloud2 &blob,
                                                        Eigen::Vector4f &origin,
                                                        Eigen::Quaternionf &orientation,
                                                        double &fx,
                                                        double &fy,
                                                        double &cx,
                                                        double &cy) const
{
  if (idx > depth_pclzf_files_.size ())
  {
    return (false);
  }
  // Get the proper files
  const std::string &depth_pclzf_file = depth_pclzf_files_[idx];
  const std::string &xml_file = xml_files_[idx];
  if (!rgb_pclzf_files_.empty ())
  {
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_color;
    const std::string &rgb_pclzf_file = rgb_pclzf_files_[idx];
    pcl::io::LZFRGB24ImageReader rgb;
    pcl::io::LZFBayer8ImageReader bayer;
    pcl::io::LZFYUV422ImageReader yuv;
    pcl::io::LZFDepth16ImageReader depth;
    if (manual_intrinsics_)
    {
      pcl::io::CameraParameters manual_params;
      manual_params.focal_length_x = focal_length_x_;
      manual_params.focal_length_y = focal_length_y_;
      manual_params.principal_point_x = principal_point_x_;
      manual_params.principal_point_y = principal_point_y_;
      fx = focal_length_x_;
      fy = focal_length_y_;
      cx = principal_point_x_;
      cy = principal_point_y_;
      rgb.setParameters (manual_params);
      yuv.setParameters (manual_params);
      bayer.setParameters (manual_params);
      depth.setParameters (manual_params);
    }
    else
    {
      rgb.readParameters (xml_file);
      yuv.readParameters (xml_file);
      bayer.readParameters (xml_file);
      depth.readParameters (xml_file);
      // update intrinsics
      pcl::io::CameraParameters loaded_params = depth.getParameters ();
      // Set intrinsics so we can update our estimate, if necessary
      fx = loaded_params.focal_length_x;
      fy = loaded_params.focal_length_y;
      cx = loaded_params.principal_point_x;
      cy = loaded_params.principal_point_y;
    }
    cloud_color.is_dense = false;
    if (num_threads_ == 1)
    {
      if (!rgb.read (rgb_pclzf_file, cloud_color))
        if (!yuv.read (rgb_pclzf_file, cloud_color))
          bayer.read (rgb_pclzf_file, cloud_color);
      depth.read (depth_pclzf_file, cloud_color);
    }
    else
    {
      if (!rgb.read (rgb_pclzf_file, cloud_color))
        if (!yuv.readOMP (rgb_pclzf_file, cloud_color, num_threads_)) // Only YUV speeds up currently
          bayer.read (rgb_pclzf_file, cloud_color);
      depth.readOMP (depth_pclzf_file, cloud_color, num_threads_);
    }
    // handle timestamps
    std::uint64_t timestamp;
    if (getTimestampFromFilepath (depth_pclzf_file, timestamp))
    {
      cloud_color.header.stamp = timestamp;
    }
    pcl::toPCLPointCloud2 (cloud_color, blob);
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::LZFDepth16ImageReader depth;
    if (manual_intrinsics_)
    {
      pcl::io::CameraParameters manual_params;
      manual_params.focal_length_x = focal_length_x_;
      manual_params.focal_length_y = focal_length_y_;
      manual_params.principal_point_x = principal_point_x_;
      manual_params.principal_point_y = principal_point_y_;
      // Set intrinsics so we can update our estimate, if necessary
      fx = focal_length_x_;
      fy = focal_length_y_;
      cx = principal_point_x_;
      cy = principal_point_y_;
      depth.setParameters (manual_params);
    }
    else
    {
      depth.readParameters (xml_file);
      // update intrinsics
      pcl::io::CameraParameters loaded_params = depth.getParameters ();
      fx = loaded_params.focal_length_x;
      fy = loaded_params.focal_length_y;
      cx = loaded_params.principal_point_x;
      cy = loaded_params.principal_point_y;
    }
    cloud.is_dense = false;
    if (num_threads_ == 1)
      depth.read (depth_pclzf_file, cloud);
    else
      depth.readOMP (depth_pclzf_file, cloud, num_threads_);
    // handle timestamps
    std::uint64_t timestamp;
    if (getTimestampFromFilepath (depth_pclzf_file, timestamp))
    {
      cloud.header.stamp = timestamp;
    }
    pcl::toPCLPointCloud2 (cloud, blob);
  }

  // Origin 0, orientation is forward
  origin = Eigen::Vector4f::Zero ();
  orientation = Eigen::Quaternionf::Identity ();
  return (true);
}

////////////////////////////////////////////////////////////////////////
//
#ifdef PCL_BUILT_WITH_VTK
bool
pcl::ImageGrabberBase::ImageGrabberImpl::getVtkImage (
    const std::string &filename,
    vtkSmartPointer<vtkImageData> &image) const
{

  vtkSmartPointer<vtkImageReader2> reader;
  // Check extension to generate the proper reader
  int retval;
  std::string upper = boost::algorithm::to_upper_copy (filename);
  if (upper.find (".TIFF") < std::string::npos)
  {
    vtkSmartPointer<vtkTIFFReader> tiff_reader = vtkSmartPointer<vtkTIFFReader>::New ();
    retval = tiff_reader->CanReadFile (filename.c_str ());
    reader = tiff_reader;
  }
  else if (upper.find (".PNG") < std::string::npos)
  {
    vtkSmartPointer<vtkPNGReader> png_reader = vtkSmartPointer<vtkPNGReader>::New ();
    retval = png_reader->CanReadFile (filename.c_str ());
    reader = png_reader;
  }
  else if (upper.find (".JPG") < std::string::npos || upper.find (".JPEG") < std::string::npos)
  {
    vtkSmartPointer<vtkJPEGReader> jpg_reader = vtkSmartPointer<vtkJPEGReader>::New ();
    retval = jpg_reader->CanReadFile (filename.c_str ());
    reader = jpg_reader;
  }
  else if (upper.find (".PPM") < std::string::npos)
  {
    vtkSmartPointer<vtkPNMReader> ppm_reader = vtkSmartPointer<vtkPNMReader>::New ();
    retval = ppm_reader->CanReadFile (filename.c_str ());
    reader = ppm_reader;
  }
  else
  {
    PCL_ERROR ("[pcl::ImageGrabber::getVtkImage] Attempted to access an invalid filetype: %s\n", filename.c_str ());
    return (false);
  }
  if (retval == 0)
  {
    PCL_ERROR ("[pcl::ImageGrabber::getVtkImage] Image file can't be read: %s\n", filename.c_str ());
    return (false);
  }
  if (retval == 1)
  {
    PCL_ERROR ("[pcl::ImageGrabber::getVtkImage] Can't prove that I can read: %s\n", filename.c_str ());
    return (false);
  }
  reader->SetFileName (filename.c_str ());
  reader->Update ();
  image = reader->GetOutput ();
  return (true);
}
#endif //PCL_BUILT_WITH_VTK

///////////////////////////////////////////////////////////////////////////////////////////
size_t
pcl::ImageGrabberBase::ImageGrabberImpl::numFrames () const
{
  if (pclzf_mode_)
    return (depth_pclzf_files_.size ());
  return (depth_image_files_.size ());
}

//////////////////////// GrabberBase //////////////////////
pcl::ImageGrabberBase::ImageGrabberBase (const std::string& directory, float frames_per_second, bool repeat, bool pclzf_mode)
  : impl_ (new ImageGrabberImpl (*this, directory, frames_per_second, repeat, pclzf_mode))
{
}

//////////////////////////////////////////////////////////
pcl::ImageGrabberBase::ImageGrabberBase (const std::string& depth_directory, const std::string &rgb_directory, float frames_per_second, bool repeat)
  : impl_ (new ImageGrabberImpl (*this, depth_directory, rgb_directory, frames_per_second, repeat))
{
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::ImageGrabberBase::ImageGrabberBase (const std::vector<std::string>& depth_image_files, float frames_per_second, bool repeat)
  : impl_ (new ImageGrabberImpl (*this, depth_image_files, frames_per_second, repeat))
{
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::ImageGrabberBase::~ImageGrabberBase () noexcept
{
  stop ();
  delete impl_;
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::start ()
{
  if (impl_->frames_per_second_ > 0)
  {
    impl_->running_ = true;
    impl_->time_trigger_.start ();
  }
  else // manual trigger to preload the first cloud
    impl_->trigger ();
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::stop ()
{
  if (impl_->frames_per_second_ > 0)
  {
    impl_->time_trigger_.stop ();
    impl_->running_ = false;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::trigger ()
{
  if (impl_->frames_per_second_ > 0)
    return;
  impl_->trigger ();
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::ImageGrabberBase::isRunning () const
{
  return (impl_->running_);
}

///////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::ImageGrabberBase::getName () const
{
  return ("ImageGrabber");
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::rewind ()
{
  impl_->cur_frame_ = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////
float
pcl::ImageGrabberBase::getFramesPerSecond () const
{
  return (impl_->frames_per_second_);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::ImageGrabberBase::isRepeatOn () const
{
  return (impl_->repeat_);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::setRGBImageFiles (const std::vector<std::string>& rgb_image_files)
{
  impl_->rgb_image_files_ = rgb_image_files;
  impl_->cur_frame_ = 0;
}


///////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::setCameraIntrinsics (const double focal_length_x,
                                            const double focal_length_y,
                                            const double principal_point_x,
                                            const double principal_point_y)
{
  impl_->focal_length_x_ = focal_length_x;
  impl_->focal_length_y_ = focal_length_y;
  impl_->principal_point_x_ = principal_point_x;
  impl_->principal_point_y_ = principal_point_y;
  impl_->manual_intrinsics_ = true;
  // If we've already preloaded a valid cloud, we need to recompute it
  if (impl_->valid_)
  {
    impl_->rewindOnce ();
    impl_->loadNextCloud ();
  }
}

void
pcl::ImageGrabberBase::getCameraIntrinsics (double &focal_length_x,
                                            double &focal_length_y,
                                            double &principal_point_x,
                                            double &principal_point_y) const
{
  focal_length_x = impl_->focal_length_x_;
  focal_length_y = impl_->focal_length_y_;
  principal_point_x = impl_->principal_point_x_;
  principal_point_y = impl_->principal_point_y_;
}



///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::setDepthImageUnits (const float units)
{
  impl_->depth_image_units_ = units;
}
///////////////////////////////////////////////////////////////////////////////////////////
size_t
pcl::ImageGrabberBase::numFrames () const
{
  return (impl_->numFrames ());
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::ImageGrabberBase::getCloudAt (std::size_t idx,
                                   pcl::PCLPointCloud2 &blob,
                                   Eigen::Vector4f &origin,
                                   Eigen::Quaternionf &orientation) const
{
  double fx, fy, cx, cy;
  return (impl_->getCloudAt (idx, blob, origin, orientation, fx, fy, cx, cy));
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::ImageGrabberBase::atLastFrame () const
{
  return (impl_->cur_frame_ == numFrames () - 1);
}

//////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::ImageGrabberBase::getCurrentDepthFileName () const
{
  std::string pathname;
  if (impl_->pclzf_mode_)
    pathname = impl_->depth_pclzf_files_[impl_->cur_frame_];
  else
    pathname = impl_->depth_image_files_[impl_->cur_frame_];
  std::string basename = boost::filesystem::basename (pathname);
  return (basename);
}
//////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::ImageGrabberBase::getPrevDepthFileName () const
{
  std::string pathname;
  if (impl_->pclzf_mode_)
    pathname = impl_->depth_pclzf_files_[impl_->cur_frame_-1];
  else
    pathname = impl_->depth_image_files_[impl_->cur_frame_-1];
  std::string basename = boost::filesystem::basename (pathname);
  return (basename);
}

/////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::ImageGrabberBase::getDepthFileNameAtIndex (std::size_t idx) const
{
  std::string pathname;
  if (impl_->pclzf_mode_)
    pathname = impl_->depth_pclzf_files_[idx];
  else
    pathname = impl_->depth_image_files_[idx];
  std::string basename = boost::filesystem::basename (pathname);
  return (basename);
}

////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::ImageGrabberBase::getTimestampAtIndex (std::size_t idx, std::uint64_t &timestamp) const
{
  std::string filename;
  if (impl_->pclzf_mode_)
    filename = impl_->depth_pclzf_files_[idx];
  else
    filename = impl_->depth_image_files_[idx];
  return (impl_->getTimestampFromFilepath (filename, timestamp));
}

////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::setNumberOfThreads (unsigned int nr_threads)
{
  impl_->num_threads_ = nr_threads;
}
