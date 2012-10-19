#ifndef RGBD_IMAGE_H_
#define RGBD_IMAGE_H_

#include <vector>

#include <boost/cstdint.hpp>

typedef boost::int16_t Depth;

struct RGBD
{
  boost::uint8_t r, g, b;
  boost::uint8_t dummy; // ensure alignment
  Depth d;
};

struct RGBDImage
{
  boost::uint32_t width, height;
  std::vector<RGBD> pixels;

  RGBDImage(boost::uint32_t width, boost::uint32_t height);
  static RGBDImage * parse(const char * data);
  void readColors(boost::int32_t * colors);
};

#endif // RGBD_IMAGE_H_
