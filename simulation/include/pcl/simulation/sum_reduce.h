/*
 * sum_reduce.hpp
 *
 *      Author: Hordur Johannsson
 *
 */

#ifndef PCL_SIMULATION_SUM_REDUCE
#define PCL_SIMULATION_SUM_REDUCE

#include <GL/glew.h>

#include <pcl/pcl_config.h>
#ifdef OPENGL_IS_A_FRAMEWORK
# include <OpenGL/gl.h>
#else
# include <GL/gl.h>
#endif

#include <pcl/simulation/glsl_shader.h>
#include <pcl/simulation/model.h>

namespace pcl
{
  namespace simulation
  {
    /** \brief Implements a parallel summation of float arrays using GLSL.
     * The input array is provided as a float texture and the summation
     * is performed over set number of levels, where each level halfs each
     * dimension.
     *
     * \author Hordur Johannsson
     * \ingroup simulation
     */
    class PCL_EXPORTS SumReduce
    {
      public:
        /** \brief Construct a new summation object for an array of given size.
         * \param width[in] the width of the input array.
         * \param width[in] the height of the input array.
         * \param levels[in] the number of levels to carry out the reduction.
         */
        SumReduce (int width, int height, int levels);

        /** \brief Release any allocated resources. */
        ~SumReduce ();

        /** \brief Reduce the array with summation over set number of levels.
         *  \param[in] input_array name of the input texture.
         *  \param[out] a pointer to an array that can store the summation result.
         */
        void sum (GLuint input_array, float* output_array);

      private:
        GLuint fbo_;
        GLuint* arrays_;
        Quad quad_;
        int levels_;
        int width_;
        int height_;
        gllib::Program::Ptr sum_program_;
    };
  } // namespace - simulation
} // namespace - pcl

#endif /* PCL_SIMULATION_SUM_REDUCE */
