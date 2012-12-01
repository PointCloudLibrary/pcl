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
#include <pcl/pcl_config.h>
#include <pcl/io/image_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/for_each_type.h>
#include <pcl/io/lzf_image_io.h>

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
  ImageGrabberImpl (pcl::ImageGrabberBase& grabber, const std::string& dir, 
      float frames_per_second, bool repeat, bool pclzf_mode=false);
  ImageGrabberImpl (pcl::ImageGrabberBase& grabber, const std::vector<std::string>& depth_image_files, float frames_per_second, bool repeat);
  
  void 
  trigger ();
  //! Read ahead -- figure out whether we are in VTK image or PCLZF mode
  void 
  loadNextCloud ();
  //! Read ahead, assuming depth_images and rgb_images are being set
  void
  loadNextCloudVTK ();
  //! Read ahead, assuming pclzf and xml files are set
  void
  loadNextCloudPCLZF ();

  //! Scrapes a directory for image files which contain "rgb" or "depth" and
  //! updates our list accordingly
  void
  loadDepthAndRGBFiles (const std::string &dir);
  //! Scrapes a directory for pclzf files which contain "rgb" or "depth and updates
  //  our list accordingly
  void
  loadPCLZFFiles (const std::string &dir);

  //! True if it is an image we know how to read
  bool
  isValidExtension (const std::string &extension);
  
  //! Load an image file, return the vtkImageReader2, return false if it couldn't be opened
  bool
  getVtkImage (const std::string &filename, vtkSmartPointer<vtkImageData> &image) const;
  
  pcl::ImageGrabberBase& grabber_;
  float frames_per_second_;
  bool repeat_;
  bool running_;
  // VTK
  std::vector<std::string> depth_image_files_;
  std::vector<std::string>::iterator depth_image_iterator_;
  std::vector<std::string> rgb_image_files_;
  std::vector<std::string>::iterator rgb_image_iterator_;
  // PCLZF
  std::vector<std::string> depth_pclzf_files_;
  std::vector<std::string>::iterator depth_pclzf_iterator_;
  std::vector<std::string> rgb_pclzf_files_;
  std::vector<std::string>::iterator rgb_pclzf_iterator_;
  std::vector<std::string> xml_files_;
  std::vector<std::string>::iterator xml_iterator_;

  TimeTrigger time_trigger_;

  sensor_msgs::PointCloud2 next_cloud_;
  //! Two cases, for depth only and depth+color
  pcl::PointCloud<pcl::PointXYZ> next_cloud_depth_;
  pcl::PointCloud<pcl::PointXYZRGBA> next_cloud_color_;
  Eigen::Vector4f origin_;
  Eigen::Quaternionf orientation_;
  bool valid_;
  //! Flag to say if a user set the focal length by hand
  //  (so we don't attempt to adjust for QVGA, QQVGA, etc).
  bool manual_focal_length_;
  bool pclzf_mode_;

  float depth_image_units_;
  float constant_;
};

///////////////////////////////////////////////////////////////////////////////////////////
pcl::ImageGrabberBase::ImageGrabberImpl::ImageGrabberImpl (pcl::ImageGrabberBase& grabber, const std::string& dir, float frames_per_second, bool repeat, bool pclzf_mode)
  : grabber_ (grabber)
  , frames_per_second_ (frames_per_second)
  , repeat_ (repeat)
  , running_ (false)
  , depth_image_files_ ()
  , depth_image_iterator_ ()
  , rgb_image_files_ ()
  , rgb_image_iterator_ ()
  , time_trigger_ (1.0 / static_cast<double> (std::max (frames_per_second, 0.001f)), boost::bind (&ImageGrabberImpl::trigger, this))
  , next_cloud_ ()
  , origin_ ()
  , orientation_ ()
  , valid_ (false)
  , manual_focal_length_ (false)
  , pclzf_mode_(pclzf_mode)
  , depth_image_units_ (1E-3)
  , constant_ (1.0f / 525.0f)
{
  if(pclzf_mode_)
  {
    loadPCLZFFiles(dir);
    depth_pclzf_iterator_ = depth_pclzf_files_.begin();
    if (rgb_pclzf_files_.size() > 0)
      rgb_pclzf_iterator_ = rgb_pclzf_files_.begin();
    xml_iterator_ = xml_files_.begin();
  }
  else
  {
    loadDepthAndRGBFiles (dir);
    depth_image_iterator_ = depth_image_files_.begin ();
    if (rgb_image_files_.size () > 0)
      rgb_image_iterator_ = rgb_image_files_.begin ();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::ImageGrabberBase::ImageGrabberImpl::ImageGrabberImpl (pcl::ImageGrabberBase& grabber, const std::vector<std::string>& depth_image_files, float frames_per_second, bool repeat)
  : grabber_ (grabber)
  , frames_per_second_ (frames_per_second)
  , repeat_ (repeat)
  , running_ (false)
  , depth_image_files_ ()
  , depth_image_iterator_ ()
  , rgb_image_files_ ()
  , rgb_image_iterator_ ()
  , time_trigger_ (1.0 / static_cast<double> (std::max (frames_per_second, 0.001f)), boost::bind (&ImageGrabberImpl::trigger, this))
  , next_cloud_ ()
  , origin_ ()
  , orientation_ ()
  , valid_ (false)
  , manual_focal_length_ (false)
  , pclzf_mode_(false)
  , depth_image_units_ (1E-3)
  , constant_ (1.0f / 525.0f)
{
  depth_image_files_ = depth_image_files;
  depth_image_iterator_ = depth_image_files_.begin ();
}

///////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::ImageGrabberBase::ImageGrabberImpl::loadNextCloud ()
{
  if(depth_image_files_.size() > 0)
    loadNextCloudVTK();
  else if(depth_pclzf_files_.size() > 0)
    loadNextCloudPCLZF();
  else
  {
    PCL_ERROR ("[pcl::ImageGrabber::loadNextCloud] Attempted to read ahead, but no VTK or PCLZF files were found. \n");
    return;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::ImageGrabberBase::ImageGrabberImpl::loadNextCloudPCLZF ()
{
  if(depth_pclzf_iterator_ == depth_pclzf_files_.end())
  {
    valid_ = false;
    return;
  }
  if(rgb_pclzf_files_.size() > 0)
  {
    pcl::io::LZFRGB24ImageReader rgb;
    pcl::io::LZFBayer8ImageReader bayer;
    pcl::io::LZFDepth16ImageReader depth;
    rgb.readParameters (*xml_iterator_);
    bayer.readParameters (*xml_iterator_);
    depth.readParameters (*xml_iterator_);
    next_cloud_color_.is_dense = false;
    if (!rgb.read (*rgb_pclzf_iterator_, next_cloud_color_))
      bayer.read(*rgb_pclzf_iterator_, next_cloud_color_);
    depth.read(*depth_pclzf_iterator_, next_cloud_color_);
    pcl::toROSMsg(next_cloud_color_, next_cloud_);
  }
  else
  {
    pcl::io::LZFDepth16ImageReader depth;
    depth.readParameters (*xml_iterator_);
    next_cloud_depth_.is_dense = false;
    depth.read(*depth_pclzf_iterator_, next_cloud_depth_);
    pcl::toROSMsg(next_cloud_depth_, next_cloud_);
  }
    
  valid_ = true;
  // Increment iterators
  if (++depth_pclzf_iterator_ == depth_pclzf_files_.end () && repeat_)
      depth_pclzf_iterator_ = depth_pclzf_files_.begin ();
  if (++xml_iterator_ == xml_files_.end() && repeat_)
    xml_iterator_ = xml_files_.begin();
    
  if (rgb_pclzf_files_.size () > 0)
  {
    if (++rgb_pclzf_iterator_ == rgb_pclzf_files_.end () && repeat_)
      rgb_pclzf_iterator_ = rgb_pclzf_files_.begin ();
  }

}
///////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::ImageGrabberBase::ImageGrabberImpl::loadNextCloudVTK ()
{
#ifdef PCL_BUILT_WITH_VTK
  if (depth_image_iterator_ == depth_image_files_.end())
  {
    valid_ = false;
    return;
  }
  unsigned short* depth_pixel;
  unsigned char* color_pixel;
  vtkSmartPointer<vtkImageData> depth_image;
  vtkSmartPointer<vtkImageData> rgb_image;
  // If there are RGB files, load an rgb image
  if (rgb_image_files_.size () != 0)
  {
    // If we've gone through the rgb iterator but not finished depth, throw error
    if (rgb_image_iterator_ == rgb_image_files_.end ())
    {
      PCL_ERROR ("[pcl::ImageGrabber::loadNextCloudVTK] Hit the end of all"
          "RGB files before Depth files\n");
      valid_ = false;
      return;
    }
    // If we were unable to pull a Vtk image, throw an error
    if (!getVtkImage (*rgb_image_iterator_, rgb_image) )
    {
      valid_ = false;
      if (++depth_image_iterator_ == depth_image_files_.end () && repeat_)
        depth_image_iterator_ = depth_image_files_.begin ();
      if (++rgb_image_iterator_ == rgb_image_files_.end () && repeat_)
        rgb_image_iterator_ = rgb_image_files_.begin ();
      return;
    }
  }
  if (!getVtkImage (*depth_image_iterator_, depth_image) )
  {
    valid_ = false;
    if (++depth_image_iterator_ == depth_image_files_.end () && repeat_)
      depth_image_iterator_ = depth_image_files_.begin ();
    return;
  }
  int* dims = depth_image->GetDimensions ();

  // Fill in image data
  depth_pixel = static_cast<unsigned short*>(depth_image->GetScalarPointer ());
  // Would like to not have two copies of the same logic floating around, 
  // but unsure of a better (readable) way to do this
  if(rgb_image_files_.size() > 0)
  {
    next_cloud_color_.width = dims[0];
    next_cloud_color_.height = dims[1];
    next_cloud_color_.is_dense = false;
    next_cloud_color_.points.resize(depth_image->GetNumberOfPoints());
    int centerX = (next_cloud_color_.width >> 1);
    int centerY = (next_cloud_color_.height >> 1);
    // The 525 factor default is only true for VGA. If not, we should scale
    float scaleFactor = manual_focal_length_ ? constant_ : constant_ * 640./dims[0];
    for (int y = 0; y < dims[1]; ++y)
    {
      for (int x = 0; x < dims[0]; ++x, ++depth_pixel)
      {
        pcl::PointXYZRGBA &pt = next_cloud_color_.at(x,y);
        float depth = static_cast<float> (*depth_pixel) * depth_image_units_;
        if (depth == 0.0f) 
          pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
        else
        {
          pt.x = (static_cast<float>(x - centerX)) * scaleFactor * depth;
          pt.y = (static_cast<float>(y - centerY)) * scaleFactor * depth; 
          pt.z = depth;
        }

        color_pixel = reinterpret_cast<unsigned char*> (rgb_image->GetScalarPointer (x, y, 0));
        pt.r = color_pixel[0];
        pt.g = color_pixel[1];
        pt.b = color_pixel[2];
      }
    }
    pcl::toROSMsg(next_cloud_color_, next_cloud_);
  }
  else
  {
    next_cloud_depth_.width = dims[0];
    next_cloud_depth_.height = dims[1];
    next_cloud_depth_.is_dense = false;
    next_cloud_depth_.points.resize(depth_image->GetNumberOfPoints());
    int centerX = (next_cloud_depth_.width >> 1);
    int centerY = (next_cloud_depth_.height >> 1);
    // The 525 factor default is only true for VGA. If not, we should scale
    float scaleFactor = manual_focal_length_ ? constant_ : constant_ * 640./dims[0];
    for (int y = 0; y < dims[1]; ++y)
    {
      for (int x = 0; x < dims[0]; ++x, ++depth_pixel)
      {
        pcl::PointXYZ &pt = next_cloud_depth_.at(x,y);
        float depth = static_cast<float> (*depth_pixel) * depth_image_units_;
        if (depth == 0.0f) 
          pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
        else
        {
          pt.x = (static_cast<float>(x - centerX)) * scaleFactor * depth;
          pt.y = (static_cast<float>(y - centerY)) * scaleFactor * depth; 
          pt.z = depth;
        }
      }
    }
    pcl::toROSMsg(next_cloud_depth_, next_cloud_);
  }

  valid_ = true;
  if (++depth_image_iterator_ == depth_image_files_.end () && repeat_)
    depth_image_iterator_ = depth_image_files_.begin ();
  
  if (rgb_image_files_.size () > 0)
  {
    if (++rgb_image_iterator_ == rgb_image_files_.end () && repeat_)
      rgb_image_iterator_ = rgb_image_files_.begin ();
  }

#else
    PCL_ERROR("[pcl::ImageGrabber::loadNextCloudVTK] Attempted to read image files, but PCL was not built with VTK [no -DPCL_BUILT_WITH_VTK]. \n");
    valid_ = false;
    return;
#endif
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
    PCL_ERROR ("Error: attempted to instantiate a pcl::ImageGrabber from a path which"
               " is not a directory: %s", dir.c_str ());
    return;
  }
  std::string pathname;
  std::string extension;
  std::string basename;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
#if BOOST_FILESYSTEM_VERSION == 3
    extension = boost::algorithm::to_upper_copy(boost::filesystem::extension (itr->path ()));
    pathname = itr->path ().string ();
    basename = boost::filesystem::basename (itr->path ());
#else
    extension = boost::algorithm::to_upper_copy(boost::filesystem::extension (itr->leaf ()));
    pathname = itr->path ().filename ();
    basename = boost::filesystem::basename (itr->leaf ());
#endif
    if (!boost::filesystem::is_directory (itr->status ()) 
        && isValidExtension (extension))
    {
      if (basename.find ("rgb") < basename.npos)
      {
        rgb_image_files_.push_back (pathname);
      }
      else if (basename.find ("depth") < basename.npos)
      {
        depth_image_files_.push_back (pathname);
      }
    }
  }
  sort (depth_image_files_.begin (), depth_image_files_.end ());
  if (rgb_image_files_.size () > 0)
    sort (rgb_image_files_.begin (), rgb_image_files_.end ());
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::ImageGrabberImpl::loadPCLZFFiles (const std::string &dir)
{
  if (!boost::filesystem::exists (dir) || !boost::filesystem::is_directory (dir))
  {
    PCL_ERROR ("Error: attempted to instantiate a pcl::ImageGrabber from a path which"
               " is not a directory: %s", dir.c_str ());
    return;
  }
  std::string pathname;
  std::string extension;
  std::string basename;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
#if BOOST_FILESYSTEM_VERSION == 3
    extension = boost::algorithm::to_upper_copy(boost::filesystem::extension (itr->path ()));
    pathname = itr->path ().string ();
    basename = boost::filesystem::basename (itr->path ());
#else
    extension = boost::algorithm::to_upper_copy(boost::filesystem::extension (itr->leaf ()));
    pathname = itr->path ().filename ();
    basename = boost::filesystem::basename (itr->leaf ());
#endif
    if (!boost::filesystem::is_directory (itr->status ()) 
        && isValidExtension (extension))
    {
      if (basename.find ("rgb") < basename.npos)
        rgb_pclzf_files_.push_back (pathname);
      else if (basename.find ("depth") < basename.npos)
        depth_pclzf_files_.push_back (pathname);
      else
        xml_files_.push_back(pathname);

    }
  }
  sort (depth_pclzf_files_.begin (), depth_pclzf_files_.end ());
  if (rgb_pclzf_files_.size () > 0)
    sort (rgb_pclzf_files_.begin (), rgb_pclzf_files_.end ());
  sort (xml_files_.begin(), xml_files_.end());
  if(depth_pclzf_files_.size() != xml_files_.size())
  {
    PCL_ERROR("[pcl::ImageGrabber::loadPCLZFFiles] # depth clouds != # xml files\n");
    return;
  }
  if(depth_pclzf_files_.size() != rgb_pclzf_files_.size() && rgb_pclzf_files_.size() > 0)
  {
    PCL_ERROR("[pcl::ImageGrabber::loadPCLZFFiles] # depth clouds != # rgb clouds\n");
    return;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::ImageGrabberBase::ImageGrabberImpl::isValidExtension (const std::string &extension)
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

////////////////////////////////////////////////////////////////////////
//
bool
pcl::ImageGrabberBase::ImageGrabberImpl::getVtkImage (
    const std::string &filename, 
    vtkSmartPointer<vtkImageData> &image) const
{

  vtkSmartPointer<vtkImageReader2> reader;
  // Check extension to generate the proper reader
  int retval;
  std::string upper = boost::algorithm::to_upper_copy (filename);
  if (upper.find (".TIFF") < upper.npos)
  {
    vtkSmartPointer<vtkTIFFReader> tiff_reader = vtkSmartPointer<vtkTIFFReader>::New();
    retval = tiff_reader->CanReadFile (filename.c_str ());
    reader = tiff_reader;
  }
  else if (upper.find (".PNG") < upper.npos)
  {
    vtkSmartPointer<vtkPNGReader> png_reader = vtkSmartPointer<vtkPNGReader>::New();
    retval = png_reader->CanReadFile (filename.c_str ());
    reader = png_reader;
  }
  else if (upper.find (".JPG") < upper.npos || upper.find (".JPEG") < upper.npos)
  {
    vtkSmartPointer<vtkJPEGReader> jpg_reader = vtkSmartPointer<vtkJPEGReader>::New();
    retval = jpg_reader->CanReadFile (filename.c_str ());
    reader = jpg_reader;
  }
  else if (upper.find (".PPM") < upper.npos)
  {
    vtkSmartPointer<vtkPNMReader> ppmg_reader = vtkSmartPointer<vtkPNMReader>::New();
    retval = ppmg_reader->CanReadFile (filename.c_str ());
    reader = ppmg_reader;
  }
  else
  {
    PCL_ERROR ("Attempted to access an invalid filetype: %s\n", filename.c_str ());
    return (false);
  }
  if (retval == 0)
  {
    PCL_ERROR ("Image file can't be read: %s\n", filename.c_str ());
    return (false);
  }
  else if (retval == 1)
  {
    PCL_ERROR ("Can't prove that I can read: %s\n", filename.c_str ());
    return (false);
  }
  reader->SetFileName (filename.c_str ());
  reader->Update ();
  image = reader->GetOutput ();
  return (true);
}

//////////////////////// GrabberBase //////////////////////
pcl::ImageGrabberBase::ImageGrabberBase (const std::string& directory, float frames_per_second, bool repeat, bool pclzf_mode)
  : impl_ (new ImageGrabberImpl (*this, directory, frames_per_second, repeat, pclzf_mode))
{
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::ImageGrabberBase::ImageGrabberBase (const std::vector<std::string>& depth_image_files, float frames_per_second, bool repeat)
  : impl_ (new ImageGrabberImpl (*this, depth_image_files, frames_per_second, repeat))
{
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::ImageGrabberBase::~ImageGrabberBase () throw ()
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
  else // manual trigger
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
  impl_->depth_image_iterator_ = impl_->depth_image_files_.begin ();
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
  impl_->rgb_image_iterator_ = impl_->rgb_image_files_.begin ();
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::setFocalLength (const float focal_length)
{
  impl_->constant_ = 1./focal_length;
  impl_->manual_focal_length_ = true;
}


///////////////////////////////////////////////////////////////////////////////////////////
float
pcl::ImageGrabberBase::getFocalLength() const
{
  return (1. / impl_->constant_);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::setDepthImageUnits (const float units)
{
  impl_->depth_image_units_ = units;
}

