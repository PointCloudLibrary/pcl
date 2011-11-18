#ifndef PCL_RANGE_LIKELIHOOD
#define PCL_RANGE_LIKELIHOOD

#include "camera.hpp"
#include "scene.hpp"

#include <math.h>
#include <Eigen/StdVector>

#include <pcl/win32_macros.h>

namespace pcl
{

class PCL_EXPORTS RangeLikelihood
{
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
  RangeLikelihood(int rows, int cols, int row_height, int col_width, Scene::Ptr scene, int x_offset = 0);

  /**
   * Destroy the RangeLikelihood object and release any memory allocated.
   */
  ~RangeLikelihood();

  /**
   * Computes the likelihood of reference for each of the provided poses.
   *
   * @param reference is a depth image.
   * @param poses is a vector of the poses to test.
   * @param scores is an output argument. The resulting log likelihoods will be written in score.
   *
   */
  void compute_likelihoods(float* reference, 
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses, std::vector<float> & scores,
  float *depth_field,bool do_depth_field );

  int rows() {return rows_;}
  int cols() {return cols_;}
  int row_height() {return row_height_;}
  int col_width() {return col_width_;}
  int width() {return width_;}
  int height() {return height_;}
  const float* depth_buffer() const {return depth_buffer_;}
  const uint8_t* color_buffer() const {return color_buffer_;}

  typedef boost::shared_ptr<RangeLikelihood> Ptr;
  typedef boost::shared_ptr<const RangeLikelihood> ConstPtr;
  
  void wait()
  {
	std::cout << "Press enter to continue";
	getchar();
	std::cout << "\n\n";
  }

private:
  void apply_camera_transform(const Eigen::Isometry3d & pose);
  void apply_camera_transform(const Camera & camera);
  void draw_particles(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses);
  void compute_scores(int cols, int rows, 
      int col_width, int row_height, float* reference, float* depth_buffer, 
      std::vector<float> & scores, float* depth_field, bool do_depth_field);
  void setup_projection_matrix();

  Scene::Ptr scene_;
  int rows_;
  int cols_;
  int row_height_;
  int col_width_;
  int width_;
  int height_;
  float* depth_buffer_;
  uint8_t* color_buffer_;
  int x_offset_;
};

template<class T>
T rad_to_deg(T rad) { return rad/M_PI*180.0; }

template<class T>
T deg_to_rad(T deg) { return deg/180.0*M_PI; }

template<class T>
T sqr(T val) { return val*val; }

}

#endif
