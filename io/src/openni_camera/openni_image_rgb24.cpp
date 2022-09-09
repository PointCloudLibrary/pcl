#include <pcl/pcl_config.h>
#include <pcl/memory.h>
#ifdef HAVE_OPENNI

#include <pcl/io/openni_camera/openni_image_rgb24.h>

namespace openni_wrapper
{
ImageRGB24::ImageRGB24 (pcl::shared_ptr<xn::ImageMetaData> image_meta_data) noexcept
: Image (std::move(image_meta_data))
{
}

ImageRGB24::~ImageRGB24 () noexcept = default;

void ImageRGB24::fillGrayscale (unsigned width, unsigned height, unsigned char* gray_buffer, unsigned gray_line_step) const
{
  if (width > image_md_->XRes () || height > image_md_->YRes ())
    THROW_OPENNI_EXCEPTION ("Up-sampling not supported. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);

  if (image_md_->XRes () % width == 0 && image_md_->YRes () % height == 0)
  {
    unsigned src_step = image_md_->XRes () / width;
    unsigned src_skip = (image_md_->YRes () / height - 1) * image_md_->XRes ();

    if (gray_line_step == 0)
      gray_line_step = width;

    unsigned dst_skip = gray_line_step - width; // skip of padding values in bytes

    unsigned char* dst_line = gray_buffer;
    const XnRGB24Pixel* src_line = image_md_->RGB24Data();

    for (unsigned yIdx = 0; yIdx < height; ++yIdx, src_line += src_skip, dst_line += dst_skip)
    {
      for (unsigned xIdx = 0; xIdx < width; ++xIdx, src_line += src_step, dst_line ++)
      {
        *dst_line = static_cast<unsigned char>((static_cast<int> (src_line->nRed)   * 299 + 
                                                static_cast<int> (src_line->nGreen) * 587 +
                                                static_cast<int> (src_line->nBlue)  * 114) * 0.001);
      }
    }
  }
  else
  {
    THROW_OPENNI_EXCEPTION ("Down-sampling only possible for integer scale. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);
  }
}

void ImageRGB24::fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer, unsigned rgb_line_step) const
{
  if (width > image_md_->XRes () || height > image_md_->YRes ())
    THROW_OPENNI_EXCEPTION ("Up-sampling not supported. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);

  if (width == image_md_->XRes () && height == image_md_->YRes ())
  {
    unsigned line_size = width * 3;
    if (rgb_line_step == 0 || rgb_line_step == line_size)
    {
      memcpy (rgb_buffer, image_md_->Data(), image_md_->DataSize());
    }
    else // line by line
    {
      unsigned char* rgb_line = rgb_buffer;
      const auto* src_line = static_cast<const unsigned char*> (image_md_->Data());
      for (unsigned yIdx = 0; yIdx < height; ++yIdx, rgb_line += rgb_line_step, src_line += line_size)
      {
        std::copy(src_line, src_line + line_size, rgb_line);
      }
    }
  }
  else if (image_md_->XRes () % width == 0 && image_md_->YRes () % height == 0) // downsamplig
  {
    unsigned src_step = image_md_->XRes () / width;
    unsigned src_skip = (image_md_->YRes () / height - 1) * image_md_->XRes ();

    if (rgb_line_step == 0)
      rgb_line_step = width * 3;

    unsigned dst_skip = rgb_line_step - width * 3; // skip of padding values in bytes

    auto* dst_line = reinterpret_cast<XnRGB24Pixel*> (rgb_buffer);
    const XnRGB24Pixel* src_line = image_md_->RGB24Data();

    for (unsigned yIdx = 0; yIdx < height; ++yIdx, src_line += src_skip)
    {
      for (unsigned xIdx = 0; xIdx < width; ++xIdx, src_line += src_step, dst_line ++)
      {
        *dst_line = *src_line;
      }

      if (dst_skip != 0)
      {
        // use bytes to skip rather than XnRGB24Pixel's, since line_step does not need to be multiple of 3
        auto* temp = reinterpret_cast <unsigned char*> (dst_line);
        dst_line = reinterpret_cast <XnRGB24Pixel*> (temp + dst_skip);
      }
    }
  }
  else
  {
    THROW_OPENNI_EXCEPTION ("Down-sampling only possible for integer scale. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);
  }
}

bool ImageRGB24::isResizingSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const
{
  return ImageRGB24::resizingSupported (input_width, input_height, output_width, output_height);
}
}

#endif //HAVE_OPENNI

