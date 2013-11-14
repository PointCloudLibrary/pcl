#ifndef PCL_RANGE_LIKELIHOOD
#define PCL_RANGE_LIKELIHOOD

#include <GL/glew.h>

#include <pcl/pcl_config.h>
#ifdef OPENGL_IS_A_FRAMEWORK
# include <OpenGL/gl.h>
# include <OpenGL/glu.h>
#else
# include <GL/gl.h>
# include <GL/glu.h>
#endif

#include <boost/random/linear_congruential.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution

//#include <math.h>
#include <Eigen/StdVector>

#include <pcl/pcl_macros.h>
//#include <pcl/win32_macros.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/common/transforms.h>
#include <pcl/simulation/camera.h>
#include <pcl/simulation/scene.h>
#include <pcl/simulation/glsl_shader.h>
#include <pcl/simulation/sum_reduce.h>

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
                         Scene::Ptr scene);

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
                            std::vector<float> & scores);

        /**
         * Set the basic camera intrinsic parameters
         */
        void
        setCameraIntrinsicsParameters (int camera_width_in,
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
        
        /**
         * Set the cost function to be used - one of 4 hard coded currently
         */
        void setCostFunction (int which_cost_function_in){  which_cost_function_ = which_cost_function_in;}
        void setSigma (double sigma_in){  sigma_ = sigma_in;	}
        void setFloorProportion (double floor_proportion_in){  floor_proportion_ = floor_proportion_in;}

        int getRows () {return rows_;}
        int getCols () {return cols_;}
        int getRowHeight () {return row_height_;}
        int getColWidth () {return col_width_;}
        int getWidth () {return width_;}
        int getHeight () {return height_;}

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
	
        void
        setComputeOnCPU(bool compute_on_cpu) { compute_likelihood_on_cpu_ = compute_on_cpu; }

        void
        setSumOnCPU(bool sum_on_cpu) { aggregate_on_cpu_ = sum_on_cpu; }

        void
        setUseColor(bool use_color) { use_color_ = use_color; }

        const uint8_t*
        getColorBuffer ();

        const float*
        getDepthBuffer ();

        const float*
        getScoreBuffer ();

      private:
        /**
         * Evaluate the likelihood/score for a set of particles
         *
         * @param[in] reference - input Measurement depth image (raw data)
         * @param[out] scores - output score
         */
        void
        computeScores (float* reference,
                       std::vector<float> & scores);

        void
        computeScoresShader (float* reference);

        void
        render (const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > & poses);

        void
        drawParticles (std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses);

        void
        applyCameraTransform (const Eigen::Isometry3d & pose);

        void
        applyCameraTransform (const Camera & camera);

        void
        setupProjectionMatrix ();

        Scene::Ptr scene_;
        int rows_;
        int cols_;
        int row_height_;
        int col_width_;
        int width_;
        int height_;
        float* depth_buffer_;
        uint8_t* color_buffer_;

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

        bool depth_buffer_dirty_;
        bool color_buffer_dirty_;
        bool score_buffer_dirty_;
	
	int which_cost_function_;
	double floor_proportion_;
	double sigma_;

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
    sqr(T val) { return val*val; }
    
  } // namespace - simulation

} // namespace - pcl

#endif
