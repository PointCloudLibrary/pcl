#include <cstring>

#include "rgbd_image.h"

RGBDImage::RGBDImage()
  : width(0), height(0)
{ }

RGBDImage::RGBDImage(boost::uint32_t width, boost::uint32_t height)
  : width(width), height(height), pixels(width * height)
{ }

void RGBDImage::parse(const char * data)
{
  boost::uint32_t width, height;

  std::memcpy (&width, data, sizeof width);
  std::memcpy (&height, data + sizeof width, sizeof height);

  resize(width, height);
  std::memcpy (&pixels.front(), data + sizeof width + sizeof height,
               pixels.size() * sizeof (RGBD));
}

void RGBDImage::readColors(boost::int32_t * colors) const
{
  for (std::size_t i = 0; i < pixels.size(); ++i)
    colors[i] = (0xFF << 24) | (pixels[i].r << 16) | (pixels[i].g << 8) | (pixels[i].b << 0);
}

void RGBDImage::resize(boost::uint32_t width, boost::uint32_t height)
{
  if (this->height == height && this->width == width)
    return;

  this->height = height;
  this->width = width;
  pixels.resize(width * height);
}
