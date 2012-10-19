#ifndef GLES_HELPER_H_
#define GLES_HELPER_H_

#include <EGL/egl.h>
#include <GLES2/gl2.h>

#include <boost/utility.hpp>

class GlesHelper : boost::noncopyable
{
  EGLDisplay dpy;
  EGLSurface dummy_surface;
  EGLContext ctx;

  GLuint prog, fb;

  int next_tex_unit;

  static void logObjectLog(GLuint object);

public:
  GlesHelper(const std::vector<const char *> & fs_sources);
  ~GlesHelper();

  int addTexture();
  int addTexture(unsigned width, unsigned height, const void * data);
  void bindTextureToUniform(int tex_id, const char * uniform);
  void bindTextureToOutput(int tex_id);
  void run(unsigned width, unsigned height, void * result);
  void setTextureData(int tex_id, unsigned width, unsigned height, const void * data);
  void setUniform(const char * uniform, int value);
};

#endif // GLES_HELPER_H_
