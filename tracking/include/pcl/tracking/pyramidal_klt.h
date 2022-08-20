/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <pcl/common/intensity.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/tracking/tracker.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

namespace pcl {
namespace tracking {
/** Pyramidal Kanade Lucas Tomasi tracker.
 * This is an implementation of the Pyramidal Kanade Lucas Tomasi tracker that
 * operates on organized 3D keypoints with color/intensity information (this is
 * the default behaviour but you can alterate it by providing another operator
 * as second template argument). It is an affine tracker that iteratively
 * computes the optical flow to find the best guess for a point p at t given its
 * location at t-1. User is advised to respect the Tomasi condition: the
 * response computed is the maximum eigenvalue of the second moment matrix but
 * no restrictin are applied to points to track so you can use a detector of
 * your choice to indicate points to track.
 *
 * \author Nizar Sallem
 */
template <typename PointInT,
          typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT>>
class PyramidalKLTTracker : public Tracker<PointInT, Eigen::Affine3f> {
public:
  using TrackerBase = pcl::tracking::Tracker<PointInT, Eigen::Affine3f>;
  using PointCloudIn = typename TrackerBase::PointCloudIn;
  using PointCloudInPtr = typename PointCloudIn::Ptr;
  using PointCloudInConstPtr = typename PointCloudIn::ConstPtr;
  using FloatImage = pcl::PointCloud<float>;
  using FloatImagePtr = FloatImage::Ptr;
  using FloatImageConstPtr = FloatImage::ConstPtr;
  using Ptr = shared_ptr<PyramidalKLTTracker<PointInT, IntensityT>>;
  using ConstPtr = shared_ptr<const PyramidalKLTTracker<PointInT, IntensityT>>;

  using TrackerBase::indices_;
  using TrackerBase::input_;
  using TrackerBase::tracker_name_;

  /** Constructor */
  PyramidalKLTTracker(int nb_levels = 5,
                      int tracking_window_width = 7,
                      int tracking_window_height = 7)
  : ref_()
  , nb_levels_(nb_levels)
  , track_width_(tracking_window_width)
  , track_height_(tracking_window_height)
  , threads_(0)
  , initialized_(false)
  {
    tracker_name_ = "PyramidalKLTTracker";
    accuracy_ = 0.1;
    epsilon_ = 1e-3;
    max_iterations_ = 10;
    keypoints_nbr_ = 100;
    min_eigenvalue_threshold_ = 1e-4;
    kernel_ << 1.f / 16, 1.f / 4, 3.f / 8, 1.f / 4, 1.f / 16;
    kernel_size_2_ = kernel_.size() / 2;
    kernel_last_ = kernel_.size() - 1;
  }

  /** Destructor */
  ~PyramidalKLTTracker() override = default;

  /** \brief Set the number of pyramid levels
   * \param levels desired number of pyramid levels
   */
  inline void
  setNumberOfPyramidLevels(int levels)
  {
    nb_levels_ = levels;
  }

  /** \return the number of pyramid levels */
  inline int
  getNumberOfPyramidLevels() const
  {
    return (nb_levels_);
  }

  /** Set accuracy
   * \param[in] accuracy desired accuracy.
   */
  inline void
  setAccuracy(float accuracy)
  {
    accuracy_ = accuracy;
  }

  /** \return the accuracy */
  inline float
  getAccuracy() const
  {
    return (accuracy_);
  }

  /** Set epsilon
   * \param[in] epsilon desired epsilon.
   */
  inline void
  setEpsilon(float epsilon)
  {
    epsilon_ = epsilon;
  }

  /** \return the epsilon */
  inline float
  getEpsilon() const
  {
    return (epsilon_);
  }

  /** \brief Set the maximum number of points to track after sorting detected keypoints
   * according to their response measure.
   * \param[in] number the desired number of points to detect.
   */
  inline void
  setNumberOfKeypoints(std::size_t number)
  {
    keypoints_nbr_ = number;
  }

  /** \return the maximum number of keypoints to keep */
  inline std::size_t
  getNumberOfKeypoints()
  {
    return (keypoints_nbr_);
  }

  /** \brief set the tracking window size
   * \param[in] width the tracking window width
   * \param[in] height the tracking window height
   */
  inline void
  setTrackingWindowSize(int width, int height);

  /** \brief Set tracking window width */
  inline void
  setTrackingWindowWidth(int width)
  {
    track_width_ = width;
  };

  /** \return the tracking window size */
  inline int
  getTrackingWindowWidth()
  {
    return (track_width_);
  }

  /** \brief Set tracking window height */
  inline void
  setTrackingWindowHeight(int height)
  {
    track_height_ = height;
  };

  /** \return the tracking window size */
  inline int
  getTrackingWindowHeight()
  {
    return (track_height_);
  }

  /** \brief Initialize the scheduler and set the number of threads to use.
   * \param nr_threads the number of hardware threads to use (0 sets the value
   * back to automatic).
   */
  inline void
  setNumberOfThreads(unsigned int nr_threads = 0)
  {
    threads_ = nr_threads;
  }

  /** \brief Get a pointer of the cloud at t-1. */
  inline PointCloudInConstPtr
  getReferenceCloud() const
  {
    return (ref_);
  }

  /** \brief Set the maximum number of iterations in the Lucas Kanade loop.
   * \param[in] max the desired maximum number of iterations
   */
  inline void
  setMaxIterationsNumber(unsigned int max)
  {
    max_iterations_ = max;
  }

  /** \return the maximum iterations number */
  inline unsigned int
  getMaxIterationsNumber() const
  {
    return (max_iterations_);
  }

  /** \brief Provide a pointer to points to track.
   * \param points the const boost shared pointer to a PointIndices message
   */
  inline void
  setPointsToTrack(const pcl::PointIndicesConstPtr& points);

  /** \brief Provide a pointer to points to track.
   * \param points the const boost shared pointer to a PointIndices message
   */
  inline void
  setPointsToTrack(const pcl::PointCloud<pcl::PointUV>::ConstPtr& points);

  /** \return a pointer to the points successfully tracked. */
  inline pcl::PointCloud<pcl::PointUV>::ConstPtr
  getTrackedPoints() const
  {
    return (keypoints_);
  };

  /** \return the status of points to track.
   * Status == 0  --> points successfully tracked;
   * Status < 0   --> point is lost;
   * Status == -1 --> point is out of bond;
   * Status == -2 --> optical flow can not be computed for this point.
   */
  PCL_DEPRECATED(1, 15, "use getStatusOfPointsToTrack instead")
  inline pcl::PointIndicesConstPtr
  getPointsToTrackStatus() const
  {
    pcl::PointIndicesPtr res(new pcl::PointIndices);
    res->indices.insert(
        res->indices.end(), keypoints_status_->begin(), keypoints_status_->end());
    return (res);
  }

  /** \return the status of points to track.
   * Status == 0  --> points successfully tracked;
   * Status < 0   --> point is lost;
   * Status == -1 --> point is out of bond;
   * Status == -2 --> optical flow can not be computed for this point.
   */
  inline pcl::shared_ptr<const std::vector<int>>
  getStatusOfPointsToTrack() const
  {
    return (keypoints_status_);
  }

  /** \brief Return the computed transformation from tracked points. */
  Eigen::Affine3f
  getResult() const override
  {
    return (motion_);
  }

  /** \return initialization state */
  bool
  getInitialized() const
  {
    return (initialized_);
  }

protected:
  bool
  initCompute() override;

  /** \brief compute Scharr derivatives of a source cloud.
   * \param[in]  src the image for which gradients are to be computed
   * \param[out] grad_x image gradient along X direction
   * \param[out] grad_y image gradient along Y direction
   */
  void
  derivatives(const FloatImage& src, FloatImage& grad_x, FloatImage& grad_y) const;

  /** \brief downsample input
   * \param[in]  input the image to downsample
   * \param[out] output the downsampled image
   */
  void
  downsample(const FloatImageConstPtr& input, FloatImageConstPtr& output) const;

  /** \brief downsample input and compute output gradients.
   * \param[in]  input the image to downsample
   * \param[out] output the downsampled image
   * \param[out] output_grad_x downsampled image gradient along X direction
   * \param[out] output_grad_y downsampled image gradient along Y direction
   */
  void
  downsample(const FloatImageConstPtr& input,
             FloatImageConstPtr& output,
             FloatImageConstPtr& output_grad_x,
             FloatImageConstPtr& output_grad_y) const;

  /** \brief Separately convolve image with decomposable convolution kernel.
   * \param[in]  input input the image to convolve
   * \param[out] output output the convolved image
   */
  void
  convolve(const FloatImageConstPtr& input, FloatImage& output) const;

  /** \brief Convolve image columns.
   * \param[in]  input input the image to convolve
   * \param[out] output output the convolved image
   */
  void
  convolveCols(const FloatImageConstPtr& input, FloatImage& output) const;

  /** \brief Convolve image rows.
   * \param[in]  input input the image to convolve
   * \param[out] output output the convolved image
   */
  void
  convolveRows(const FloatImageConstPtr& input, FloatImage& output) const;

  /** \brief extract the patch from the previous image, previous image gradients
   * surrounding pixel alocation while interpolating image and gradients data
   * and compute covariation matrix of derivatives.
   * \param[in] img original image
   * \param[in] grad_x original image gradient along X direction
   * \param[in] grad_y original image gradient along Y direction
   * \param[in] location pixel at the center of the patch
   * \param[in] weights bilinear interpolation weights at this location computed from
   * subpixel location
   * \param[out] win patch with interpolated intensity values
   * \param[out] grad_x_win patch with interpolated gradient along X values
   * \param[out] grad_y_win patch with interpolated gradient along Y values
   * \param[out] covariance covariance matrix coefficients
   */
  virtual void
  spatialGradient(const FloatImage& img,
                  const FloatImage& grad_x,
                  const FloatImage& grad_y,
                  const Eigen::Array2i& location,
                  const Eigen::Array4f& weights,
                  Eigen::ArrayXXf& win,
                  Eigen::ArrayXXf& grad_x_win,
                  Eigen::ArrayXXf& grad_y_win,
                  Eigen::Array3f& covariance) const;
  void
  mismatchVector(const Eigen::ArrayXXf& prev,
                 const Eigen::ArrayXXf& prev_grad_x,
                 const Eigen::ArrayXXf& prev_grad_y,
                 const FloatImage& next,
                 const Eigen::Array2i& location,
                 const Eigen::Array4f& weights,
                 Eigen::Array2f& b) const;

  /** \brief Compute the pyramidal representation of an image.
   * \param[in] input the input cloud
   * \param[out] pyramid computed pyramid levels along with their respective
   * gradients
   * \param[in]  border_type
   */
  virtual void
  computePyramids(const PointCloudInConstPtr& input,
                  std::vector<FloatImageConstPtr>& pyramid,
                  pcl::InterpolationType border_type) const;

  virtual void
  track(const PointCloudInConstPtr& previous_input,
        const PointCloudInConstPtr& current_input,
        const std::vector<FloatImageConstPtr>& previous_pyramid,
        const std::vector<FloatImageConstPtr>& current_pyramid,
        const pcl::PointCloud<pcl::PointUV>::ConstPtr& previous_keypoints,
        pcl::PointCloud<pcl::PointUV>::Ptr& current_keypoints,
        std::vector<int>& status,
        Eigen::Affine3f& motion) const;

  void
  computeTracking() override;

  /** \brief input pyranid at t-1 */
  std::vector<FloatImageConstPtr> ref_pyramid_;
  /** \brief point cloud at t-1 */
  PointCloudInConstPtr ref_;
  /** \brief number of pyramid levels */
  int nb_levels_;
  /** \brief detected keypoints 2D coordinates */
  pcl::PointCloud<pcl::PointUV>::ConstPtr keypoints_;
  /** \brief status of keypoints of t-1 at t */
  pcl::shared_ptr<std::vector<int>> keypoints_status_;
  /** \brief number of points to detect */
  std::size_t keypoints_nbr_;
  /** \brief tracking width */
  int track_width_;
  /** \brief half of tracking window width */
  int track_width_2_;
  /** \brief tracking height */
  int track_height_;
  /** \brief half of tracking window height */
  int track_height_2_;
  /** \brief maximum number of iterations */
  unsigned int max_iterations_;
  /** \brief accuracy criterion to stop iterating */
  float accuracy_;
  float min_eigenvalue_threshold_;
  /** \brief epsilon for subpixel computation */
  float epsilon_;
  float max_residue_;
  /** \brief number of hardware threads */
  unsigned int threads_;
  /** \brief intensity accessor */
  IntensityT intensity_;
  /** \brief is the tracker initialized ? */
  bool initialized_;
  /** \brief compute transformation from successfully tracked points */
  pcl::TransformationFromCorrespondences transformation_computer_;
  /** \brief computed transformation between tracked points */
  Eigen::Affine3f motion_;
  /** \brief smoothing kernel */
  Eigen::Array<float, 5, 1> kernel_;
  /** \brief smoothing kernel half size */
  int kernel_size_2_;
  /** \brief index of last element in kernel */
  int kernel_last_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace tracking
} // namespace pcl

#include <pcl/tracking/impl/pyramidal_klt.hpp>
