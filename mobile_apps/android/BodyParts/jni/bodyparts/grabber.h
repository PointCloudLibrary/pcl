#ifndef GRABBER_H_
#define GRABBER_H_

#include <string>

#include "cloud.h"

struct Grabber
{
  static Grabber * createOpenNIGrabber();
  static Grabber * createFileGrabber(const std::string & directory);

  virtual ~Grabber() {}

  virtual bool isConnected() const = 0;

  virtual void getFrame(Cloud & frame) = 0;

  virtual void start() = 0;
  virtual void stop() = 0;
};

#endif
