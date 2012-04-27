/*
 * glsl_shader.cpp
 *
 *  Created on: Nov 27, 2011
 *      Author: hordurj
 */

#include <pcl/simulation/glsl_shader.h>
#include <iostream>
#include <fstream>

using namespace pcl::simulation::gllib;

char*
readTextFile (const char* filename)
{
  using namespace std;
  char* buf = NULL;
  ifstream file;
  file.open (filename, ios::in|ios::binary|ios::ate);
  if (file.is_open ())
  {
    ifstream::pos_type size;
    size = file.tellg();
    buf = new char[size + static_cast<ifstream::pos_type> (1)];
    file.seekg (0, ios::beg);
    file.read (buf, size);
    file.close ();
    buf[size] = 0;
  }
  return buf;
}

pcl::simulation::gllib::Program::Program ()
{
  program_id_ = glCreateProgram ();
}

pcl::simulation::gllib::Program::~Program ()
{
}

int
pcl::simulation::gllib::Program::getUniformLocation (const std::string& name)
{
  return glGetUniformLocation (program_id_, name.c_str ());
}

void
pcl::simulation::gllib::Program::setUniform (const std::string& name, const Eigen::Vector2f& v)
{
  GLuint loc = getUniformLocation (name.c_str ());
  glUniform2f(loc, v (0), v (1));
}

void
pcl::simulation::gllib::Program::setUniform (const std::string& name, const Eigen::Vector3f& v)
{
  GLuint loc = getUniformLocation (name.c_str ());
  glUniform3f(loc, v (0), v (1), v (2));
}

void
pcl::simulation::gllib::Program::setUniform(const std::string& name, const Eigen::Vector4f& v)
{
  GLuint loc = getUniformLocation (name.c_str ());
  glUniform4f (loc, v (0), v (1), v (2), v (4));
}

void
pcl::simulation::gllib::Program::setUniform (const std::string& name, const Eigen::Matrix3f& v)
{
  GLuint loc = getUniformLocation (name.c_str ());
  glUniformMatrix3fv (loc, 1, false, v.data ());
}

void
pcl::simulation::gllib::Program::setUniform (const std::string& name, const Eigen::Matrix4f& v)
{
  GLuint loc = getUniformLocation (name.c_str ());
  glUniformMatrix4fv (loc, 1, false, v.data ());
}

void
pcl::simulation::gllib::Program::setUniform (const std::string& name, float v)
{
  GLuint loc = getUniformLocation (name.c_str ());
  glUniform1f (loc, v);
}

void
pcl::simulation::gllib::Program::setUniform (const std::string& name, int v)
{
  GLuint loc = getUniformLocation (name.c_str ());
  glUniform1i (loc, v);
}

void
pcl::simulation::gllib::Program::setUniform (const std::string& name, bool v)
{
  GLuint loc = getUniformLocation (name.c_str ());
  glUniform1i(loc, (v?1:0));
}

bool
pcl::simulation::gllib::Program::addShaderText (const std::string& text, ShaderType shader_type)
{
  GLuint id;
  GLint compile_status;
  id = glCreateShader (shader_type);
  const char* source_list = text.c_str ();

  glShaderSource (id, 1, &source_list, NULL);

  glCompileShader (id);
  printShaderInfoLog (id);
  glGetShaderiv (id, GL_COMPILE_STATUS, &compile_status);
  if (compile_status == GL_FALSE) return false;

  if (getGLError () != GL_NO_ERROR) return false;

  glAttachShader (program_id_, id);
  return true;
}

bool
pcl::simulation::gllib::Program::addShaderFile (const std::string& filename, ShaderType shader_type)
{
  char* text = readTextFile (filename.c_str ());
  if(text == NULL)  return (false);

  std::string source(text);

  bool rval = addShaderText (text, shader_type);
  delete [] text;
  return rval;
}

bool
pcl::simulation::gllib::Program::link ()
{
  glLinkProgram (program_id_);
  printProgramInfoLog (program_id_);

  if (getGLError () != GL_NO_ERROR) return false;
  return true;
}

void pcl::simulation::gllib::Program::use ()
{
  glUseProgram (program_id_);
}

GLenum pcl::simulation::gllib::getGLError ()
{
  GLenum last_error = GL_NO_ERROR;
  GLenum error = glGetError ();
  while (error != GL_NO_ERROR)
  {
    last_error = error;
    std::cout << "Error: OpenGL: " << gluErrorString(error) << std::endl;
    error = glGetError ();
  }
  return last_error;
}

void
pcl::simulation::gllib::printShaderInfoLog (GLuint shader)
{
  GLsizei max_length;
  GLsizei length;
  GLchar* info_log;

  glGetShaderiv (shader, GL_INFO_LOG_LENGTH, &length);
  max_length = length;
  info_log = new GLchar[length+1];

  glGetShaderInfoLog (shader, max_length, &length, info_log);

  info_log[max_length] = 0;

  std::cout << "Shader info log: " << std::endl << info_log << std::endl;

  delete [] info_log;
}

void
pcl::simulation::gllib::printProgramInfoLog (GLuint program)
{
  GLsizei max_length;
  GLsizei length;
  GLchar* info_log;

  glGetProgramiv (program, GL_INFO_LOG_LENGTH, &length);
  max_length = length;
  info_log = new GLchar[length+1];

  glGetProgramInfoLog (program, max_length, &length, info_log);

  info_log[max_length] = 0;

  std::cout << "Program info log: " << std::endl << info_log << std::endl;

  delete [] info_log;
}

Program::Ptr
pcl::simulation::gllib::Program::loadProgramFromText (const std::string& vertex_shader_text, const std::string& fragment_shader_text)
{
  // Load shader
  Program::Ptr program = gllib::Program::Ptr (new gllib::Program ());
  if (!program->addShaderText (vertex_shader_text, gllib::VERTEX))
  {
    std::cerr << "Failed loading vertex shader" << std::endl;
  }

  // TODO: to remove file dependency include the shader source in the binary
  if (!program->addShaderFile (fragment_shader_text, gllib::FRAGMENT))
  {
    std::cerr << "Failed loading fragment shader" << std::endl;
  }

  program->link ();

  return program;
}

Program::Ptr
pcl::simulation::gllib::Program::loadProgramFromFile (const std::string& vertex_shader_file, const std::string& fragment_shader_file)
{
  // Load shader
  Program::Ptr program = gllib::Program::Ptr (new gllib::Program ());
  if (!program->addShaderFile (vertex_shader_file, gllib::VERTEX))
  {
    std::cerr << "Failed loading vertex shader" << std::endl;
  }

  // TODO: to remove file dependency include the shader source in the binary
  if (!program->addShaderFile (fragment_shader_file, gllib::FRAGMENT))
  {
    std::cerr << "Failed loading fragment shader" << std::endl;
  }

  program->link ();

  return program;
}
