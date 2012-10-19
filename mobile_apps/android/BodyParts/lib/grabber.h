#ifndef GRABBER_H_
#define GRABBER_H_

#include <string>

#include "rgbd_image.h"

struct Grabber
{
  static Grabber * createOpenNIGrabber();
  static Grabber * createFileGrabber(const std::string & directory);

  virtual ~Grabber() {}

  virtual void getFrame(RGBDImage * frame) = 0;

  virtual void start() = 0;
  virtual void stop() = 0;
};

#endif
