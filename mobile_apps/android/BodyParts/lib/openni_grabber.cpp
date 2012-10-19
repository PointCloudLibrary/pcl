#include <android/log.h>
#include <boost/noncopyable.hpp>

#include <XnCppWrapper.h>

#include "grabber.h"

class OpenNIGrabber : public Grabber, boost::noncopyable
{
  xn::Context context;
  xn::DepthGenerator depth;
  xn::ImageGenerator image;

public:
  OpenNIGrabber();

  virtual void getFrame(RGBDImage * frame);

  virtual void start();
  virtual void stop();
};

#define CHECK_XN(statement) do { XnStatus status = statement; \
  if (status != XN_STATUS_OK) \
  __android_log_print(ANDROID_LOG_DEBUG, "OpenNI", "OpenNI error at line %d: %s", __LINE__, xnGetStatusString(status)); \
  } while (0)

OpenNIGrabber::OpenNIGrabber()
{
  CHECK_XN(context.Init());
  CHECK_XN(depth.Create(context));
  CHECK_XN(image.Create(context));
}

void OpenNIGrabber::start()
{
  CHECK_XN(context.StartGeneratingAll());
}

void OpenNIGrabber::stop()
{
  CHECK_XN(context.StopGeneratingAll());
}

void OpenNIGrabber::getFrame(RGBDImage * frame)
{
  context.WaitOneUpdateAll(depth);

  xn::DepthMetaData depthMD;
  depth.GetMetaData(depthMD);
  __android_log_print(ANDROID_LOG_DEBUG, "OpenNIGrabber", "Depth MD: width %d height %d.", depthMD.XRes(), depthMD.YRes());

  xn::ImageMetaData imageMD;
  image.GetMetaData(imageMD);
  __android_log_print(ANDROID_LOG_DEBUG, "OpenNIGrabber", "Image MD: width %d height %d.", imageMD.XRes(), imageMD.YRes());

  frame->resize(depthMD.XRes(), depthMD.YRes());

  for (unsigned i = 0; i < frame->height * frame->width; ++i)
  {
    frame->pixels[i].d = depthMD.Data()[i];
    XnRGB24Pixel color = imageMD.RGB24Data()[i];
    frame->pixels[i].r = color.nRed;
    frame->pixels[i].g = color.nGreen;
    frame->pixels[i].b = color.nBlue;
  }
}

Grabber * Grabber::createOpenNIGrabber()
{
  return new OpenNIGrabber();
}
