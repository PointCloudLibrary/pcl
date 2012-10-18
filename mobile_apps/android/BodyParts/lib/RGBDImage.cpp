#include <cstring>

#include "RGBDImage.h"

RGBDImage::RGBDImage(boost::uint32_t width, boost::uint32_t height)
  : width(width), height(height), pixels(width * height)
{ }

RGBDImage * RGBDImage::parse(const char * data)
{
  boost::uint32_t width, height;

  std::memcpy (&width, data, sizeof width);
  std::memcpy (&height, data + sizeof width, sizeof height);

  RGBDImage * new_image = new RGBDImage(width, height);
  std::memcpy (&new_image->pixels.front(), data + sizeof width + sizeof height,
               new_image->pixels.size() * sizeof (RGBD));

  return new_image;
}

void RGBDImage::readColors(boost::int32_t * colors)
{
  for (std::size_t i = 0; i < pixels.size(); ++i)
    colors[i] = (0xFF << 24) | (pixels[i].r << 16) | (pixels[i].g << 8) | (pixels[i].b << 0);
}
