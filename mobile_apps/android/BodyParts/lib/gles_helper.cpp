#include <vector>

#include <android/log.h>

#include "gles_helper.h"
#include "sources.h"

#define CHECK_GL(statement) do { statement; \
  { GLenum error = glGetError (); \
    if (error != GL_NO_ERROR) \
  __android_log_print(ANDROID_LOG_DEBUG, "GlesHelper", "GLES error at line %d: 0x%x", __LINE__, error); \
  } } while (0)

void
GlesHelper::logObjectLog(GLuint object)
{
  const char * object_type_str;
  void (* get_info_log)(GLuint object, GLsizei bufsize, GLsizei * length, GLchar * infolog);
  GLint log_len;

  if (glIsProgram(object))
  {
    object_type_str = "Program";
    get_info_log = glGetProgramInfoLog;
    glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_len);
  }
  else if (glIsShader(object))
  {
    int shader_type;
    glGetShaderiv(object, GL_SHADER_TYPE, &shader_type);
    if (shader_type == GL_VERTEX_SHADER) object_type_str = "Vertex shader";
    else if (shader_type == GL_FRAGMENT_SHADER) object_type_str = "Fragment shader";
    else object_type_str = "Unknown shader";

    get_info_log = glGetShaderInfoLog;
    glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_len);
  }
  else
  {
    return;
  }

  if (log_len > 0)
  {
    std::vector<GLchar> log(log_len);
    get_info_log(object, log_len, NULL, &log.front());
    __android_log_print(ANDROID_LOG_ERROR, "GlesHelper", "%s log: %s", object_type_str, &log.front());
  }
  else
  {
    __android_log_print(ANDROID_LOG_DEBUG, "GlesHelper", "%s: No log.", object_type_str);
  }
}

GlesHelper::GlesHelper(const std::vector<const char *> & fs_sources)
  : next_tex_unit(0)
{
  dpy = eglGetDisplay (EGL_DEFAULT_DISPLAY);
  eglBindAPI (EGL_OPENGL_API);

  EGLConfig config;
  EGLint num_configs;
  EGLint config_attribs[] = {
    EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
    EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
    EGL_NONE
  };
  eglChooseConfig (dpy, config_attribs, &config, 1, &num_configs);

  EGLint ctx_attribs[] = {
    EGL_CONTEXT_CLIENT_VERSION, 2,
    EGL_NONE
  };
  ctx = eglCreateContext (dpy, config, EGL_NO_CONTEXT, ctx_attribs);
  dummy_surface = eglCreatePbufferSurface (dpy, config, NULL);
  eglMakeCurrent (dpy, dummy_surface, dummy_surface, ctx);

  const char * vs_source = reinterpret_cast<const char *> (source_generic_vsh);
  GLuint vs = glCreateShader (GL_VERTEX_SHADER);
  CHECK_GL(glShaderSource (vs, 1, &vs_source, NULL));
  CHECK_GL(glCompileShader (vs));
  logObjectLog(vs);

  GLuint fs = glCreateShader (GL_FRAGMENT_SHADER);
  CHECK_GL(glShaderSource (fs, fs_sources.size(), const_cast<const GLchar **> (&fs_sources.front()), NULL));
  CHECK_GL(glCompileShader (fs));
  logObjectLog(fs);

  prog = glCreateProgram ();
  CHECK_GL(glAttachShader (prog, vs));
  CHECK_GL(glAttachShader (prog, fs));
  CHECK_GL(glLinkProgram (prog));
  logObjectLog(prog);

  glDeleteShader(vs);
  glDeleteShader(fs);

  glUseProgram(prog);

  glGenFramebuffers(1, &fb);
  CHECK_GL(glBindFramebuffer(GL_FRAMEBUFFER, fb));

  GLuint pos_buffer;
  glGenBuffers(1, &pos_buffer);
  CHECK_GL(glBindBuffer(GL_ARRAY_BUFFER, pos_buffer));
  float pos_array[] = { -1., -1., -1, 1., 1., -1., 1., 1. };
  CHECK_GL(glBufferData(GL_ARRAY_BUFFER, sizeof pos_array, pos_array, GL_STATIC_DRAW));

  int position_location = glGetAttribLocation(prog, "position");
  CHECK_GL(glVertexAttribPointer(position_location, 2, GL_FLOAT, GL_FALSE, 0, NULL));
  CHECK_GL(glEnableVertexAttribArray(position_location));
}

GlesHelper::~GlesHelper()
{
  eglBindAPI (EGL_OPENGL_API);
  eglMakeCurrent (dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

  eglDestroySurface (dpy, dummy_surface);
  eglDestroyContext (dpy, ctx);
}

int
GlesHelper::addTexture()
{
  eglMakeCurrent (dpy, dummy_surface, dummy_surface, ctx);

  CHECK_GL(glActiveTexture(GL_TEXTURE0 + next_tex_unit));
  GLuint tex;
  CHECK_GL(glGenTextures(1, &tex));
  CHECK_GL(glBindTexture(GL_TEXTURE_2D, tex));
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  return next_tex_unit++;
}

int
GlesHelper::addTexture(unsigned width, unsigned height, const void * data)
{
  GLuint tex_id = addTexture();
  setTextureData(tex_id, width, height, data);
  return tex_id;
}

void
GlesHelper::bindTextureToUniform(int tex_id, const char * uniform)
{
  eglMakeCurrent (dpy, dummy_surface, dummy_surface, ctx);
  CHECK_GL(glUniform1i(glGetUniformLocation(prog, uniform), tex_id));
}

void
GlesHelper::bindTextureToOutput(int tex_id)
{
  eglMakeCurrent (dpy, dummy_surface, dummy_surface, ctx);
  glActiveTexture(GL_TEXTURE0 + tex_id);

  GLint tex;
  glGetIntegerv(GL_TEXTURE_BINDING_2D, &tex);

  CHECK_GL(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0));
}

void GlesHelper::run(unsigned width, unsigned height, void * result)
{
  eglMakeCurrent (dpy, dummy_surface, dummy_surface, ctx);
  glViewport(0, 0, width, height);

  CHECK_GL(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
  //glFinish();

  CHECK_GL(glReadPixels(0, 0, width, height,
                        GL_RGBA, GL_UNSIGNED_BYTE, result));
}

void GlesHelper::setTextureData(int tex_id, unsigned width, unsigned height, const void * data)
{
  eglMakeCurrent (dpy, dummy_surface, dummy_surface, ctx);
  glActiveTexture(GL_TEXTURE0 + tex_id);
  CHECK_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data));
}

void GlesHelper::setUniform(const char * uniform, int value)
{
  eglMakeCurrent (dpy, dummy_surface, dummy_surface, ctx);
  CHECK_GL(glUniform1i(glGetUniformLocation(prog, uniform), value));
}
