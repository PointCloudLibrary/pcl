/*
 * glsl_shader.h
 *
 *  Created on: Nov 27, 2011
 *      Author: Hordur Johannsson
 */

#ifndef PCL_SIMULATION_GLSL_SHADER
#define PCL_SIMULATION_GLSL_SHADER

#include <GL/glew.h>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

namespace pcl
{
namespace simulation
{
namespace gllib
{
  enum ShaderType { VERTEX = GL_VERTEX_SHADER,
                    FRAGMENT = GL_FRAGMENT_SHADER,
                    GEOMETRY = GL_GEOMETRY_SHADER,
                    TESS_CONTROL = GL_TESS_CONTROL_SHADER,
                    TESS_EVALUATION = GL_TESS_EVALUATION_SHADER };

  /**
   * A GLSL shader program.
   */
  class Program
  {
    public:
      typedef boost::shared_ptr<Program> Ptr;
      typedef boost::shared_ptr<const Program> ConstPtr;

      /**
       * Construct an empty shader program.
       */
      Program();

      /**
       * Destruct the shader program.
       */
      ~Program();

      /**
       * Add a new shader object to the program.
       */
      bool add_shader_text(const std::string& text, ShaderType shader_type);

      /**
       * Add a new shader object to the program.
       */
      bool add_shader_file(const std::string& text, ShaderType shader_type);

      /**
       * Link the program.
       */
      bool link();

      /**
       * Return true if the program is linked.
       */
      bool is_linked();

      /**
       * Use the program.
       */
      void use();

      // Set uniforms
      void set_uniform(const std::string& name, const Eigen::Vector2f& v);
      void set_uniform(const std::string& name, const Eigen::Vector3f& v);
      void set_uniform(const std::string& name, const Eigen::Vector4f& v);
      void set_uniform(const std::string& name, const Eigen::Matrix2f& v);
      void set_uniform(const std::string& name, const Eigen::Matrix3f& v);
      void set_uniform(const std::string& name, const Eigen::Matrix4f& v);
      void set_uniform(const std::string& name, float v);
      void set_uniform(const std::string& name, int v);
      void set_uniform(const std::string& name, bool v);

      int get_uniform_location(const std::string& name);

      void print_active_uniforms();
      void print_active_attribs();

      GLuint program_id() { return program_id_; }

      static Ptr load_program_from_file(const std::string& vertex_shader_file, const std::string& fragment_shader_file);
      static Ptr load_program_from_text(const std::string& vertex_shader_text, const std::string& fragment_shader_text);

    private:
      GLuint program_id_;
  };

  GLenum get_gl_error();
  void print_shader_info_log(GLuint shader);
  void print_program_info_log(GLuint program);

//  static const char*
} // namespace - gllib
} // namespace - simulation
} // namespace - pcl

#endif /* PCL_SIMULATION_GLSL_SHADER_HPP_ */
