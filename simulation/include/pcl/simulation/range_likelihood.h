#ifndef PCL_RANGE_LIKELIHOOD
#define PCL_RANGE_LIKELIHOOD

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

//#include <math.h>
#include <Eigen/StdVector>

#include <pcl/pcl_macros.h>
//#include <pcl/win32_macros.h>
#include <pcl/range_image/range_image_planar.h>
#include "pcl/common/transforms.h"
#include "pcl/simulation/camera.h"
#include "pcl/simulation/scene.h"
#include "pcl/simulation/glsl_shader.h"
#include "pcl/simulation/sum_reduce.h"

namespace pcl
{
  namespace simulation
  {
    class PCL_EXPORTS RangeLikelihood
    {
      public:
        typedef boost::shared_ptr<RangeLikelihood> Ptr;
        typedef boost::shared_ptr<const RangeLikelihood> ConstPtr;

      public:
        /**
         * Create a new object to compute range image likelihoods.
         *
         * OpenGL is used to render the images. It is assumed that
         * render buffers have already been setup. The area used is
         * from 0,0 to cols*col_width,rows*row_height.
         *
         * @param rows - number of rows to use in the render buffer.
         * @param cols - number of columns to use in the render buffer.
         * @param row_height - height of the image for a single particle.
         * @param col_width  - width of the image for a single particle.
         * @param scene - a pointer to the scene that should be rendered when
         *                computing likelihoods.
         *
         */
        RangeLikelihood (int rows,
                         int cols,
                         int row_height,
                         int col_width,
                         Scene::Ptr scene,
                         int x_offset = 0);

        /**
         * Destroy the RangeLikelihood object and release any memory allocated.
         */
        ~RangeLikelihood ();

        /**
         * Computes the likelihood of reference for each of the provided poses.
         *
         * @param reference is a depth image.
         * @param poses is a vector of the poses to test.
         * @param scores is an output argument. The resulting log likelihoods will be written in score.
         *
         */
        void
        computeLikelihoods (float* reference,
                            std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses,
                            std::vector<float> & scores,
                            float *depth_field,
                            bool do_depth_field);

        /**
         * Set the basic camera intrinsic parameters
         */
        void
        setCameraIntrinsicsParameters(int camera_width_in,
                                      int camera_height_in,
                                      float camera_fx_in,
                                      float camera_fy_in,
                                      float camera_cx_in,
                                      float camera_cy_in)
        {
          camera_width_ = camera_width_in;
          camera_height_ = camera_height_in;
          camera_fx_ = camera_fx_in;
          camera_fy_ = camera_fy_in;
          camera_cx_ = camera_cx_in;
          camera_cy_ = camera_cy_in;
        }

        int getRows () {return rows_;}
        int getCols () {return cols_;}
        int getRowHeight () {return row_height_;}
        int getColWidth () {return col_width_;}
        int getWidth () {return width_;}
        int getHeight () {return height_;}
        const float* getDepthBuffer () const {return depth_buffer_;}
        const uint8_t* getColorBuffer () const {return color_buffer_;}

        // Convenience function to return simulated RGB-D PointCloud
        // Two modes:
        // global=false - PointCloud is as would be captured by an RGB-D camera [default]
        // global=true  - PointCloud is transformed into the model/world frame using the camera pose
        void getPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
              bool make_global, const Eigen::Isometry3d & pose);
        // Convenience function to return RangeImagePlanar containing
        // simulated RGB-D:
        void getRangeImagePlanar (pcl::RangeImagePlanar &rip);

        // Add various types of noise to simulated RGB-D data
        void addNoise ();
        double sampleNormal (double sigma = 1.0);
        void computeScores (int cols, int rows,
            int col_width, int row_height, float* reference, float* depth_buffer,
            std::vector<float> & scores, float* depth_field, bool do_depth_field);

      protected:
        void draw_particles (std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses);

      // private:
        void apply_camera_transform (const Eigen::Isometry3d & pose);
        void apply_camera_transform (const Camera & camera);
        void setup_projection_matrix ();

        Scene::Ptr scene_;
        int rows_;
        int cols_;
        int row_height_;
        int col_width_;
        int width_;
        int height_;
        float* depth_buffer_;
        uint8_t* color_buffer_;

        // Offset for visualization, for operation will be zero
        int x_offset_;

        // Camera Intrinsic Parameters
        int camera_width_;
        int camera_height_;
        float camera_fx_;
        float camera_fy_;
        float camera_cx_;
        float camera_cy_;

        // min and max range of the rgbd sensor
        // everything outside this doesnt appear in depth images
        float z_near_;
        float z_far_;
    };


    class PCL_EXPORTS RangeLikelihoodGLSL : public RangeLikelihood
    {
      public:
        typedef boost::shared_ptr<RangeLikelihoodGLSL> Ptr;
        typedef boost::shared_ptr<const RangeLikelihoodGLSL> ConstPtr;

        /**
         * Create a new object to compute range image likelihoods.
         *
         * OpenGL is used to render the images. It is assumed that
         * render buffers have already been setup. The area used is
         * from 0,0 to cols*col_width,rows*row_height.
         *
         * @param rows - number of rows to use in the render buffer.
         * @param cols - number of columns to use in the render buffer.
         * @param row_height - height of the image for a single particle.
         * @param col_width  - width of the image for a single particle.
         * @param scene - a pointer to the scene that should be rendered when
         *                computing likelihoods.
         *
         */
        RangeLikelihoodGLSL (int rows,
                             int cols,
                             int row_height,
                             int col_width,
                             Scene::Ptr scene,
                             int x_offset = 0);

        /**
         * Destroy the RangeLikelihood object and release any memory allocated.
         */
        ~RangeLikelihoodGLSL ();

        /**
         * Computes the likelihood of reference for each of the provided poses.
         *
         * @param reference is a depth image.
         * @param poses is a vector of the poses to test.
         * @param scores is an output argument. The resulting log likelihoods will be written in score.
         *
         */
        void
        computeLikelihoods (float* reference,
                            std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses,
                            std::vector<float> & scores,
                            float *depth_field,
                            bool do_depth_field );

        void
        computeScoresShader(int cols,
                            int rows,
                            int col_width,
                            int row_height,
                            float* reference,
                            float* depth_buffer,
                            std::vector<float> & scores,
                            float *depth_field,
                            bool do_depth_field );

        const float* getScoreBuffer() const {return score_buffer_;}

        void
        setComputeOnCPU(bool compute_on_cpu) { compute_likelihood_on_cpu_ = compute_on_cpu; }

        void
        setSumOnCPU(bool sum_on_cpu) { aggregate_on_cpu_ = sum_on_cpu; }

      private:
        void
        render (const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > & poses);

      private:
        GLuint fbo_;
        GLuint score_fbo_;

        GLuint depth_render_buffer_;
        GLuint color_render_buffer_;
        GLuint color_texture_;
        GLuint depth_texture_;
        GLuint score_texture_;
        GLuint score_summarized_texture_;
        GLuint sensor_texture_;
        GLuint likelihood_texture_;

        bool compute_likelihood_on_cpu_;
        bool aggregate_on_cpu_;
        bool use_instancing_;
        bool generate_color_image_;
        bool use_color_;

        gllib::Program::Ptr likelihood_program_;
        GLuint quad_vbo_;
        std::vector<Eigen::Vector3f> vertices_;
        float* score_buffer_;
        Quad quad_;
        SumReduce sum_reduce_;
    };

    template<class T> T
    rad_to_deg (T rad) { return rad/M_PI*180.0; }

    template<class T> T
    deg_to_rad (T deg) { return deg/180.0*M_PI; }
    
    template<class T> T
    sqr(T val) { return val*val; }
    
  } // namespace - simulation

} // namespace - pcl

#endif
