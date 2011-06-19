#include <pcl/pcl_config.h>
#ifdef HAVE_OPENNI

#include <pcl/io/openni_camera/openni_image_rgb24.h>
#include <openni/XnCppWrapper.h>

namespace openni_wrapper
{
ImageRGB24::ImageRGB24 (boost::shared_ptr<xn::ImageMetaData> image_meta_data) throw ()
: Image (image_meta_data)
{
}

ImageRGB24::~ImageRGB24 () throw ()
{
}

void ImageRGB24::fillGrayscale (unsigned width, unsigned height, unsigned char* gray_buffer, unsigned gray_line_step) const throw (OpenNIException)
{
  return;
}

void ImageRGB24::fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer, unsigned rgb_line_step) const throw (OpenNIException)
{
  if (width > image_md_->XRes () || height > image_md_->YRes ())
    THROW_OPENNI_EXCEPTION ("Upsampling only possible for multiple of 2 in both dimensions. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);

  if (width == image_md_->XRes () && height == image_md_->YRes ())
    memcpy (rgb_buffer, image_md_->Data(), image_md_->DataSize());
}

bool ImageRGB24::isResizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const
{
  return ImageRGB24::resizingSupported (input_width, input_height, output_width, output_height);
}
}

#endif //HAVE_OPENNI