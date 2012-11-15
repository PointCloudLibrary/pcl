/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#include <pcl/pcl_config.h>
#include <pcl/io/image_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/for_each_type.h>

#include <vtkImageReader2.h>
#include <vtkImageReader2Factory.h>
#include <vtkImageData.h>

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// GrabberImplementation //////////////////////
struct pcl::ImageGrabberBase::ImageGrabberImpl
{
  //! Implementation of ImageGrabber
  ImageGrabberImpl (pcl::ImageGrabberBase& grabber, const std::string& depth_image_file, float frames_per_second, bool repeat);
  ImageGrabberImpl (pcl::ImageGrabberBase& grabber, const std::vector<std::string>& depth_image_files, float frames_per_second, bool repeat);
  
  void 
  trigger ();

  void 
  readAhead ();
  //! Scrapes a directory for image files which contain "rgb" or "depth" and
  //! updates our list accordingly
  void
  loadDepthAndRGBFiles(const std::string &dir);
  //! True iff it is an image we know how to read
  bool
  isValidExtension(const std::string &extension);
  
  
  pcl::ImageGrabberBase& grabber_;
  float frames_per_second_;
  bool repeat_;
  bool running_;
  std::vector<std::string> depth_image_files_;
  std::vector<std::string>::iterator depth_image_iterator_;
  std::vector<std::string> rgb_image_files_;
  std::vector<std::string>::iterator rgb_image_iterator_;
  TimeTrigger time_trigger_;

  sensor_msgs::PointCloud2 next_cloud_;
  Eigen::Vector4f origin_;
  Eigen::Quaternionf orientation_;
  bool valid_;

  float depth_image_units_;
  float constant_;
};

///////////////////////////////////////////////////////////////////////////////////////////
pcl::ImageGrabberBase::ImageGrabberImpl::ImageGrabberImpl (pcl::ImageGrabberBase& grabber, const std::string& dir, float frames_per_second, bool repeat)
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
  , depth_image_units_ (1E-3)
  , constant_ (1.0f / 525.0f)
{
  //depth_image_files_.push_back (depth_image_file);
  loadDepthAndRGBFiles(dir);
  depth_image_iterator_ = depth_image_files_.begin ();
  if(rgb_image_files_.size() > 0)
    rgb_image_iterator_ = rgb_image_files_.begin ();
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
  , depth_image_units_ (1E-3)
  , constant_ (1.0f / 525.0f)
{
  depth_image_files_ = depth_image_files;
  depth_image_iterator_ = depth_image_files_.begin ();
}

///////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::ImageGrabberBase::ImageGrabberImpl::readAhead ()
{
  if (depth_image_iterator_ != depth_image_files_.end ())
  {
    unsigned short* depth_pixel;
    unsigned char* color_pixel;
    vtkImageData* depth_image;
    vtkImageData* rgb_image;
    vtkImageReader2Factory* reader_factory = vtkImageReader2Factory::New ();
    if (rgb_image_files_.size () != 0)
    {
      if (rgb_image_iterator_ == rgb_image_files_.end ())
      {
	PCL_ERROR ("[pcl::ImageGrabber::setRGBImageFiles] Number of depth images %d != number of rgb images %d \n", 
		   depth_image_files_.size (), rgb_image_files_.size ());
	valid_ = false;
	return;
      }
      else
      {
	vtkImageReader2* rgb_reader = reader_factory->CreateImageReader2 ((*rgb_image_iterator_).c_str ());
	rgb_reader->SetFileName ((*rgb_image_iterator_).c_str ());
	rgb_reader->Update ();
	rgb_image = rgb_reader->GetOutput ();
      }
    }
    vtkImageReader2* depth_reader = reader_factory->CreateImageReader2 ((*depth_image_iterator_).c_str ());
    depth_reader->SetFileName ((*depth_image_iterator_).c_str ());
    depth_reader->Update ();
    depth_image = depth_reader->GetOutput ();
    int* dims = depth_image->GetDimensions ();

    // Set up next_cloud_ meta data:
    size_t point_size;
    next_cloud_.fields.clear();
    if (rgb_image_files_.size () == 0)
    {
      point_size = sizeof(pcl::PointXYZ);
      for_each_type<traits::fieldList<pcl::PointXYZ>::type> 
	(detail::FieldAdder<pcl::PointXYZ>(next_cloud_.fields));
    }
    else 
    {
      point_size = sizeof(pcl::PointXYZRGBA);
      for_each_type<traits::fieldList<pcl::PointXYZRGBA>::type> 
	(detail::FieldAdder<pcl::PointXYZRGBA>(next_cloud_.fields));
    }
    next_cloud_.data.clear();
    next_cloud_.data.resize (point_size * depth_image->GetNumberOfPoints ());
    next_cloud_.width = dims[0];
    next_cloud_.height = dims[1];
    next_cloud_.is_dense = false;
    next_cloud_.point_step = point_size;
    next_cloud_.row_step = next_cloud_.width * next_cloud_.point_step;

    // Fill in image data
    int centerX = (next_cloud_.width >> 1);
    int centerY = (next_cloud_.height >> 1);
    depth_pixel = static_cast<unsigned short*>(depth_image->GetScalarPointer ());
    for (int y=0; y<dims[1]; ++y)
    {
      for (int x=0; x<dims[0]; ++x, ++depth_pixel)
      {
	uint8_t* p_i = &(next_cloud_.data[y * next_cloud_.row_step + x * next_cloud_.point_step]);
	float data[point_size / sizeof(float)];
	float depth = (float)(*depth_pixel) * depth_image_units_;
	if (depth == 0.0f) 
	  data[0] = data[1] = data[2] = std::numeric_limits<float>::quiet_NaN ();
	else
	{
	  data[0] = ((float)(x - centerX)) * constant_ * depth;
	  data[1] = ((float)(y - centerY)) * constant_ * depth; 
	  data[2] = depth;
	}
	data[3] = 1.0f;

	if (rgb_image_files_.size () != 0)
	{
	  color_pixel = static_cast<unsigned char*> (rgb_image->GetScalarPointer (x,y,0));
	  uint32_t rgb = (uint32_t)color_pixel[0] << 16 | (uint32_t)color_pixel[1] << 8 | (uint32_t)color_pixel[2];
	  data[4] = *reinterpret_cast<float*> (&rgb);
	}
	memcpy(p_i, &data, sizeof(data));
      }
    }

    valid_ = true;
    if (++depth_image_iterator_ == depth_image_files_.end () && repeat_)
      depth_image_iterator_ = depth_image_files_.begin ();
    if (rgb_image_files_.size () != 0)
    {
      if (++rgb_image_iterator_ == rgb_image_files_.end () && repeat_)
	rgb_image_iterator_ = rgb_image_files_.begin ();
    }
  }
  else
  {
    valid_ = false;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::ImageGrabberBase::ImageGrabberImpl::trigger ()
{
  if (valid_)
    grabber_.publish (next_cloud_,origin_,orientation_);
  // use remaining time, if there is time left!
  readAhead ();
}

///////////////////////////////////////////////////////////////////////////////////////////

void
pcl::ImageGrabberBase::ImageGrabberImpl::loadDepthAndRGBFiles(const std::string &dir)
{
  if(!boost::filesystem::exists( dir ) || !boost::filesystem::is_directory( dir ) )
  {
    PCL_ERROR("Error: attempted to instantiate a pcl::ImageGrabber from a path which"
        " is not a directory: %s", dir.c_str());
    return;
  }
  std::string pathname;
  std::string extension;
  std::string basename;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
#if BOOST_FILESYSTEM_VERSION == 3
      extension = boost::algorithm::to_upper_copy(boost::filesystem::extension (itr->path()));
      pathname = itr->path().string();
      basename = boost::filesystem::basename(itr->path());
#else
      extension = boost::algorithm::to_upper_copy(boost::filesystem::extension (itr->path()));
      pathname = itr->path();
      basename = boost::filesystem::basename(itr->leaf());
#endif
    if (!boost::filesystem::is_directory (itr->status ()) 
        && isValidExtension(extension))
    {
      if(basename.find("rgb") < basename.npos)
      {
        rgb_image_files_.push_back (pathname);
      }
      else if(basename.find("depth") < basename.npos)
      {
        depth_image_files_.push_back (pathname);
      }
    }
  }
  sort(depth_image_files_.begin(), depth_image_files_.end());
  if(rgb_image_files_.size() > 0)
    sort(rgb_image_files_.begin(), rgb_image_files_.end());
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::ImageGrabberBase::ImageGrabberImpl::isValidExtension(const std::string &extension)
{
  bool valid = 
    extension == ".TIFF" || extension == ".PNG" 
    || extension == ".JPG" || extension == ".PPM";
  return (valid);
}

//////////////////////// GrabberBase //////////////////////
pcl::ImageGrabberBase::ImageGrabberBase (const std::string& depth_image_file, float frames_per_second, bool repeat)
  : impl_ (new ImageGrabberImpl (*this, depth_image_file, frames_per_second, repeat))
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
}


///////////////////////////////////////////////////////////////////////////////////////////
float
pcl::ImageGrabberBase::getFocalLength() const
{
  return 1./impl_->constant_;
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::ImageGrabberBase::setDepthImageUnits (const float units)
{
  impl_->depth_image_units_ = units;
}
