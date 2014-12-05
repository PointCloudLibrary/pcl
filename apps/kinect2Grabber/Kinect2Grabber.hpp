/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *  This is preliminary software and/or hardware and APIs are preliminary and subject to change.
 *
 * Author: Giacomo Dabisias (g.dabisias@gmail.com, g.dabisias@sssup.it)
 */

#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "GL/glew.h"
#include <libfreenect2/opengl.h>
#include <signal.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/rgb_packet_stream_parser.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

bool shut_down = false;

void
sigint_handler (int s)
{
  shut_down = true;
}

namespace pcl
{

  namespace Kinect2Grabber
  {

    template <typename PointT>
    class Kinect2Grabber
    {
      public:

        Kinect2Grabber (const std::string rgb_image_folder_path,
                        const std::string depth_image_folder_path,
                        const int image_number,
                        const cv::Size board_size,
                        const double square_size) :
            cloud_ (new pcl::PointCloud<PointT> ()),
            init_ (true)
        {
          glfwInit ();
          dev_ = freenect2_.openDefaultDevice ();
          listener_ = new libfreenect2::SyncMultiFrameListener (libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
          if (dev_ == 0)
          {
            PCL_ERROR ("no device connected or failure opening the default one!\n");
            exit (1);
          }
          signal (SIGINT, sigint_handler);
          shut_down = false;
          dev_->setColorFrameListener (listener_);
          dev_->setIrAndDepthFrameListener (listener_);
          dev_->start ();
          PCL_INFO ("starting calibration\n");
          calibrateCamera (rgb_image_folder_path, depth_image_folder_path, image_number, board_size, square_size);
          PCL_INFO ("finished calibration\n");
          PCL_INFO ("device initialized\n");
        }

        Kinect2Grabber (const std::string rgb_calibration_file,
                        const std::string depth_calibration_file,
                        const std::string pose_calibration_file) :
            cloud_ (new pcl::PointCloud<PointT> ()),
            init_ (true)
        {

          glfwInit ();
          dev_ = freenect2_.openDefaultDevice ();
          listener_ = new libfreenect2::SyncMultiFrameListener (libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
          if (dev_ == 0)
          {
            PCL_ERROR ("no device connected or failure opening the default one!\n");
            exit (1);
          }
          signal (SIGINT, sigint_handler);
          shut_down = false;
          dev_->setColorFrameListener (listener_);
          dev_->setIrAndDepthFrameListener (listener_);
          dev_->start ();

          PCL_INFO ("device initialized\n");
          loadCalibration (rgb_calibration_file, depth_calibration_file, pose_calibration_file);
        }

        ~Kinect2Grabber ()
        {
          this->shutDown ();
        }

        void
        loadCalibration (const std::string rgb_calibration_file,
                         const std::string depth_calibration_file,
                         const std::string pose_calibration_file)
        {

          cv::FileStorage fs;
          fs.open (rgb_calibration_file, cv::FileStorage::READ);
          int x, y;
          if (fs.isOpened ())
          {
            fs["image_width"] >> x;
            fs["image_height"] >> y;
            size_rgb_ = cv::Size (x, y);
            fs["camera_matrix"] >> rgb_camera_matrix_;
            fs["distortion_coefficients"] >> rgb_distortion_;
            fs.release ();
          }
          else
          {
            PCL_ERROR ("could not find rgb calibration file\n");
            exit (-1);
          }

          fs.open (depth_calibration_file, cv::FileStorage::READ);

          if (fs.isOpened ())
          {
            fs["image_width"] >> x;
            fs["image_height"] >> y;
            size_depth_ = cv::Size (x, y);
            fs["camera_matrix"] >> depth_camera_matrix_;
            fs["distortion_coefficients"] >> depth_distortion_;
            fs.release ();
          }
          else
          {
            PCL_ERROR ("could not find depth calibration file\n");
            exit (-1);
          }

          fs.open (pose_calibration_file, cv::FileStorage::READ);

          if (fs.isOpened ())
          {
            fs["rotation matrix"] >> rotation_;
            fs["translation matrix"] >> translation_;
            fs["fundamental matrix"] >> fundamental_;
            fs["essential matrix"] >> essential_;
            fs.release ();
          }
          else
          {
            PCL_ERROR ("could not find pose calibration file\n");
            exit (-1);
          }

        }

        void
        calcBoardCornerPositions (const cv::Size board_size,
                                  const float square_size_,
                                  std::vector<cv::Point3f>& corners)
        {
          corners.clear ();
          for (int i = 0; i < board_size.height; ++i)
            for (int j = 0; j < board_size.width; ++j)
              corners.push_back (cv::Point3f (float (j * square_size_), float (i * square_size_), 0));
        }

        void
        saveCameraParams (const std::string& filename,
                          const cv::Size image_size,
                          const cv::Size board_size,
                          const float square_size_,
                          const float aspect_ratio,
                          const int flags,
                          const cv::Mat& camera_matrix,
                          const cv::Mat& distortion,
                          const double total_error) const
        {

          cv::FileStorage fs (filename, cv::FileStorage::WRITE);

          time_t t;
          time (&t);
          struct tm *t2 = localtime (&t);
          char buf[1024];
          strftime (buf, sizeof (buf) - 1, "%c", t2);

          fs << "calibration_time" << buf;
          fs << "image_width" << image_size.width;
          fs << "image_height" << image_size.height;
          fs << "board_width" << board_size.width;
          fs << "board_height" << board_size.height;
          fs << "square_size_" << square_size_;

          if (flags & cv::CALIB_FIX_ASPECT_RATIO)
            fs << "aspectRatio" << aspect_ratio;

          if (flags != 0)
          {
            sprintf (buf, "flags: %s%s%s%s", flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                     flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "", flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                     flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
          }

          fs << "flags" << flags;
          fs << "camera_matrix" << camera_matrix;
          fs << "distortion_coefficients" << distortion;
          fs << "avg_reprojection_error" << total_error;
        }

        void
        savePoseParams (const std::string& filename,
                        const cv::Mat & rotation,
                        const cv::Mat & translation,
                        const cv::Mat & essential,
                        const cv::Mat & fundamental,
                        const double total_error) const
        {

          cv::FileStorage fs (filename, cv::FileStorage::WRITE);

          time_t t;
          time (&t);
          struct tm *t2 = localtime (&t);
          char buf[1024];
          strftime (buf, sizeof (buf) - 1, "%c", t2);

          fs << "calibration_time" << buf;
          fs << "rotation matrix" << rotation;
          fs << "translation matrix" << translation;
          fs << "essential matrix" << essential;
          fs << "fundamental matrix" << fundamental;
          fs << "avg_reprojection_error" << total_error;
        }

        void
        calibrateCamera (std::string rgb_image_folder_path,
                         std::string depth_image_folder_path,
                         int image_number,
                         cv::Size board_size,
                         double square_size)
        {

          const cv::TermCriteria term_criteria (cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON);
          std::vector < std::vector < cv::Point2f >> rgbImagePoints;
          std::vector < std::vector < cv::Point2f >> irImagePoints;
          std::vector<cv::Mat> rvecs, tvecs;

          if (rgb_image_folder_path.back () != '/')
            rgb_image_folder_path += std::string ("/");

          if (depth_image_folder_path.back () != '/')
            depth_image_folder_path += std::string ("/");

          int count = 0;
          for (int i = 0; i < image_number; ++i, ++count)
          {

            std::string rgb_name = rgb_image_folder_path + std::string ("rgb_image_") + std::to_string (count) + std::string (".jpg");
            std::string ir_name = depth_image_folder_path + std::string ("ir_image_") + std::to_string (count) + std::string (".jpg");

            cv::Mat rgb_gray = cv::imread (rgb_name, 0);
            cv::Mat ir_gray = cv::imread (ir_name, 0);
            size_depth_ = ir_gray.size ();
            size_rgb_ = rgb_gray.size ();

            std::vector < cv::Point2f > camera1ImagePoints;
            bool found1 = cv::findChessboardCorners (rgb_gray, board_size, camera1ImagePoints, cv::CALIB_CB_FAST_CHECK);

            std::vector < cv::Point2f > camera2ImagePoints;
            bool found2 = cv::findChessboardCorners (ir_gray, board_size, camera2ImagePoints, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

            if (found1)
            {
              cv::cornerSubPix (rgb_gray, camera1ImagePoints, cv::Size (11, 11), cv::Size (-1, -1), term_criteria);
              rgbImagePoints.push_back (camera1ImagePoints);
            }
            if (found2)
            {
              cv::cornerSubPix (ir_gray, camera2ImagePoints, cv::Size (11, 11), cv::Size (-1, -1), term_criteria);
              irImagePoints.push_back (camera2ImagePoints);
            }
          }

          std::vector < std::vector<cv::Point3f> > pointsBoard (1);
          calcBoardCornerPositions (board_size, square_size, pointsBoard[0]);
          pointsBoard.resize (image_number, pointsBoard[0]);

          double error_1 = calibrateCamera (pointsBoard, rgbImagePoints, size_rgb_, rgb_camera_matrix_, rgb_distortion_, rvecs, tvecs);
          saveCameraParams ("rgb_calibration.yaml", size_rgb_, board_size, square_size, 0, 0, rgb_camera_matrix_, rgb_distortion_, error_1);

          double error_2 = calibrateCamera (pointsBoard, irImagePoints, size_depth_, depth_camera_matrix_, depth_distortion_, rvecs, tvecs);
          saveCameraParams ("depth_calibration.yaml", size_depth_, board_size, square_size, 0, 0, depth_camera_matrix_, depth_distortion_, error_2);

          double rms = cv::stereoCalibrate (pointsBoard, rgbImagePoints, irImagePoints, rgb_camera_matrix_, rgb_distortion_, depth_camera_matrix_,
                                            depth_distortion_, size_rgb_, rotation_, translation_, essential_, fundamental_, cv::CALIB_FIX_INTRINSIC,
                                            term_criteria);


          PCL_INFO ("\n");
          PCL_INFO ("rgb error:%f\n", error_1);
          PCL_INFO ("depth error:%f\n", error_2);
          PCL_INFO ("stereo error:%f\n", rms);

          savePoseParams ("pose_calibration.yaml", rotation_, translation_, essential_, fundamental_, rms);
        }

        libfreenect2::Frame *
        getRgbFrame ()
        {
          listener_->waitForNewFrame (frames_);
          return frames_[libfreenect2::Frame::Color];
        }

        libfreenect2::Frame *
        getIrFrame ()
        {
          listener_->waitForNewFrame (frames_);
          return frames_[libfreenect2::Frame::Ir];
        }

        libfreenect2::Frame *
        getDepthFrame ()
        {
          listener_->waitForNewFrame (frames_);
          return frames_[libfreenect2::Frame::Depth];
        }

        libfreenect2::FrameMap *
        getRawFrames ()
        {
          listener_->waitForNewFrame (frames_);
          return &frames_;
        }

        void
        shutDown ()
        {
          dev_->stop ();
          dev_->close ();
        }

        cv::Mat
        getCameraMatrixColor () const
        {
          return rgb_camera_matrix_;
        }

        cv::Mat
        getCameraMatrixDepth () const
        {
          return depth_camera_matrix_;
        }

        cv::Mat
        getRgbDistortion () const
        {
          return rgb_distortion_;
        }

        cv::Mat
        getDepthDistortion () const
        {
          return depth_distortion_;
        }

        void
        freeFrames ()
        {
          listener_->release (frames_);
        }

        void
        createCloud (const cv::Mat &depth,
                     const cv::Mat &color,
                     typename pcl::PointCloud<PointT>::Ptr &cloud) const
        {
          const float badPoint = std::numeric_limits<float>::quiet_NaN ();

#pragma omp parallel for
          for (int y = 0; y < depth.rows; ++y)
          {
            PointT *itP = &cloud->points[y * depth.cols];
            const uint16_t *itD = depth.ptr < uint16_t > (y);
            const cv::Vec3b *itC = color.ptr < cv::Vec3b > (y);

            for (size_t x = 0; x < (size_t) depth.cols; ++x, ++itP, ++itD, ++itC)
            {
              const float depth_value = *itD / 1000.0f;
              // Check for invalid measurements
              if (isnan (depth_value) || depth_value <= 0.001)
              {
                // invalid point
                itP->x = itP->y = itP->z = badPoint;
                itP->rgba = 0;
                continue;
              }

              //to depth world
              float final_x = (x - ir_cx_) / (ir_fx_) * depth_value;
              float final_y = (y - ir_cy_) / (ir_fy_) * depth_value;
              Eigen::Vector3d ir_world (final_x, final_y, depth_value);

              //depth world to rgb world
              Eigen::Map < Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > mappedMat ((double*) (rotation_.data));
              Eigen::Matrix3d rotation = mappedMat;
              Eigen::Map < Eigen::Vector3d > mappedMat2 ((double*) (translation_.data));
              Eigen::Vector3d translation = mappedMat2;

              Eigen::Vector3d rgb_world = rotation * ir_world + translation;

              //rgb world to rgb image
              int rgb_image_x = ( (rgb_world.x () * (rgb_fx_) / depth_value)) + rgb_cx_;
              int rgb_image_y = ( (rgb_world.y () * (rgb_fy_) / depth_value)) + rgb_cy_;

              if (rgb_image_x > 0 && rgb_image_x < color.cols && rgb_image_y > 0 && rgb_image_y < color.rows)
              {
                itP->z = depth_value;
                itP->x = final_x;
                itP->y = final_y;
                const cv::Vec3b tmp = color.at < cv::Vec3b > (rgb_image_y, rgb_image_x);
                itP->b = tmp.val[0];
                itP->g = tmp.val[1];
                itP->r = tmp.val[2];
              }
            }
          }
        }

        typename pcl::PointCloud<PointT>::Ptr
        getCloud (int size_x = 512,
                  int size_y = 424)
        {

          frames_ = *getRawFrames ();
          rgb_ = frames_[libfreenect2::Frame::Color];
          depth_ = frames_[libfreenect2::Frame::Depth];
          tmp_depth_ = cv::Mat (depth_->height, depth_->width, CV_32FC1, depth_->data);
          tmp_rgb_ = cv::Mat (rgb_->height, rgb_->width, CV_8UC3, rgb_->data);
          cv::flip (tmp_depth_, tmp_depth_, 1);
          cv::flip (tmp_rgb_, tmp_rgb_, 1);
          tmp_depth_.convertTo (tmp_depth_, CV_16U);

          if (init_)
          {

            const float sx_depth = ((float) size_x / (float) size_depth_.width);
            const float sy_depth = ((float) size_y / (float) size_depth_.height);
            const float sx_rgb = ((float) size_x / (float) size_rgb_.width);
            const float sy_rgb = ((float) size_y / (float) size_rgb_.height);

            rgb_fx_ = rgb_camera_matrix_.at<double> (0, 0) * sx_rgb;
            rgb_fy_ = rgb_camera_matrix_.at<double> (1, 1) * sy_rgb;
            rgb_cx_ = rgb_camera_matrix_.at<double> (0, 2) * sx_rgb;
            rgb_cy_ = rgb_camera_matrix_.at<double> (1, 2) * sy_rgb;
            ir_fx_ = depth_camera_matrix_.at<double> (0, 0) * sx_depth;
            ir_fy_ = depth_camera_matrix_.at<double> (1, 1) * sy_depth;
            ir_cx_ = depth_camera_matrix_.at<double> (0, 2) * sx_depth;
            ir_cy_ = depth_camera_matrix_.at<double> (1, 2) * sy_depth;

            cloud_->height = size_y;
            cloud_->width = size_x;
            cloud_->is_dense = false;
            cloud_->points.resize (cloud_->height * cloud_->width);

            init_ = false;
          }

          cv::resize (tmp_rgb_, rgb_scaled_, cv::Size (size_x, size_y), cv::INTER_CUBIC);
          createCloud (tmp_depth_, rgb_scaled_, cloud_);

          return cloud_;
        }

      private:

        libfreenect2::Freenect2 freenect2_;
        libfreenect2::Freenect2Device * dev_ = 0;
        libfreenect2::SyncMultiFrameListener * listener_ = 0;
        libfreenect2::FrameMap frames_;
        cv::Mat scaled_, registered_, map_x_, map_y_, rgb_scaled_;
        cv::Mat rotation_, translation_, essential_, fundamental_;
        libfreenect2::Frame *depth_, *rgb_;
        cv::Size size_registered_, size_depth_, size_rgb_;
        cv::Mat lookup_y_, lookup_x_, tmp_depth_, tmp_rgb_;
        typename pcl::PointCloud<PointT>::Ptr cloud_;
        cv::Mat rgb_camera_matrix_, depth_distortion_, depth_camera_matrix_, rgb_distortion_;
        double rgb_fx_, rgb_fy_, rgb_cx_, rgb_cy_;
        double ir_fx_, ir_fy_, ir_cx_, ir_cy_;
        bool init_;
    };

  }

}
