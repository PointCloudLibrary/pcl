#include <pcl/simulation/sum_reduce.h>

using namespace pcl::simulation;

pcl::simulation::SumReduce::SumReduce (int width, int height, int levels) : levels_ (levels),
                                                                            width_ (width),
                                                                            height_ (height)
{
  std::cout << "SumReduce: levels: " << levels_ << std::endl;

  // Load shader
  sum_program_ = gllib::Program::Ptr (new gllib::Program ());
  // TODO: to remove file dependency include the shader source in the binary
  if (!sum_program_->addShaderFile ("sum_score.vert", gllib::VERTEX))
  {
    std::cout << "Failed loading vertex shader" << std::endl;
    exit (-1);
  }

  // TODO: to remove file dependency include the shader source in the binary
  if (!sum_program_->addShaderFile ("sum_score.frag", gllib::FRAGMENT))
  {
    std::cout << "Failed loading fragment shader" << std::endl;
    exit (-1);
  }

  sum_program_->link ();

  // Setup the framebuffer object for rendering
  glGenFramebuffers (1, &fbo_);
  arrays_ = new GLuint[levels_];

  glGenTextures (levels_, arrays_);

  int level_width = width_;
  int level_height = height_;

  for (int i = 0; i < levels_; ++i)
  {
    level_width = level_width / 2;
    level_height = level_height / 2;

    glBindTexture (GL_TEXTURE_2D, arrays_[i]);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glTexImage2D (GL_TEXTURE_2D, 0, GL_R32F, level_width, level_height, 0, GL_RED, GL_FLOAT, NULL);
    glBindTexture (GL_TEXTURE_2D, 0);
  }
}

pcl::simulation::SumReduce::~SumReduce ()
{
  glDeleteTextures (levels_, arrays_);
  glDeleteFramebuffers (1, &fbo_);
}

void
pcl::simulation::SumReduce::sum (GLuint input_array, float* output_array)
{
  if (gllib::getGLError () != GL_NO_ERROR)
  {
    std::cout << "SumReduce::sum enter" << std::endl;
  }

  glDisable (GL_DEPTH_TEST);

  glBindFramebuffer (GL_FRAMEBUFFER, fbo_);
  int width = width_;
  int height = height_;

  glActiveTexture (GL_TEXTURE0);
  glBindTexture (GL_TEXTURE_2D, input_array);

  // use program
  sum_program_->use ();
  glUniform1i (sum_program_->getUniformLocation ("ArraySampler"), 0);

  if (gllib::getGLError () != GL_NO_ERROR)
  {
    std::cout << "SumReduce::sum  set sampler" << std::endl;
  }

  for (int i=0; i < levels_; ++i)
  {
    glFramebufferTexture2D (GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, arrays_[i], 0);
    glDrawBuffer (GL_COLOR_ATTACHMENT0);

    glViewport (0, 0, width/2, height/2);

    float step_x = 1.0f / float (width);
    float step_y = 1.0f / float (height);
    sum_program_->setUniform ("step_x", step_x);
    sum_program_->setUniform ("step_y", step_y);
    //float step_x = 1.0f / static_cast<float> (width);
    //float step_y = 1.0f / static_cast<float> (height);

    quad_.render ();

    if (gllib::getGLError () != GL_NO_ERROR)
    {
      std::cout << "SumReduce::sum  render" << std::endl;
    }

    width = width / 2;
    height = height / 2;

    glActiveTexture (GL_TEXTURE0);
    glBindTexture (GL_TEXTURE_2D, arrays_[i]);
  }

  glBindFramebuffer (GL_FRAMEBUFFER, 0);

  glActiveTexture (GL_TEXTURE0);
  glBindTexture (GL_TEXTURE_2D, 0);

  glUseProgram (0);

  // Final results is in arrays_[levels_-1]
  glActiveTexture (GL_TEXTURE0);
  glBindTexture (GL_TEXTURE_2D, arrays_[levels_-1]);
  glGetTexImage (GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, output_array);
  glBindTexture (GL_TEXTURE_2D, 0);

  if (gllib::getGLError () != GL_NO_ERROR)
  {
    std::cout << "Error: SumReduce exit" << std::endl;
  }
}

