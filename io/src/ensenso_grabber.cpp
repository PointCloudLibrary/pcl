/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 *  Author: Victor Lamoine (victor.lamoine@gmail.com)
 */

#include <pcl/pcl_config.h>
#include <pcl/io/ensenso_grabber.h>
#include <pcl/exceptions.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <boost/format.hpp>
#include <boost/algorithm/string/predicate.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Handle Ensenso SDK exceptions
// This function is called whenever an exception is raised to provide details about the error
void
ensensoExceptionHandling (const NxLibException &ex,
                          std::string func_nam)
{
  PCL_ERROR ("%s: NxLib error %s (%d) occurred while accessing item %s.\n", func_nam.c_str (), ex.getErrorText ().c_str (), ex.getErrorCode (),
            ex.getItemPath ().c_str ());
  if (ex.getErrorCode () == NxLibExecutionFailed)
  {
    NxLibCommand cmd ("");
    PCL_WARN ("\n%s\n", cmd.result ().asJson (true, 4, false).c_str ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::EnsensoGrabber::EnsensoGrabber () :
    device_open_ (false),
    tcp_open_ (false),
    running_ (false)
{
  point_cloud_signal_ = createSignal<sig_cb_ensenso_point_cloud> ();
  images_signal_ = createSignal<sig_cb_ensenso_images> ();
  point_cloud_images_signal_ = createSignal<sig_cb_ensenso_point_cloud_images> ();
  PCL_INFO ("Initialising nxLib\n");

  try
  {
    nxLibInitialize ();
    root_.reset (new NxLibItem);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "EnsensoGrabber");
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not initialise NxLib.");  // If constructor fails; throw exception
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::EnsensoGrabber::~EnsensoGrabber () throw ()
{
  try
  {
    stop ();
    root_.reset ();

    disconnect_all_slots<sig_cb_ensenso_point_cloud> ();
    disconnect_all_slots<sig_cb_ensenso_images> ();
    disconnect_all_slots<sig_cb_ensenso_point_cloud_images> ();

    if (tcp_open_)
      closeTcpPort ();
    nxLibFinalize ();
  }
  catch (...)
  {
    // destructor never throws
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::EnsensoGrabber::enumDevices () const
{
  int camera_count = 0;

  try
  {
    NxLibItem cams = NxLibItem ("/Cameras/BySerialNo");
    camera_count = cams.count ();

    // Print information for all cameras in the tree
    PCL_INFO ("Number of connected cameras: %d\n", camera_count);
    PCL_INFO ("Serial No    Model   Status\n");

    for (int n = 0; n < cams.count (); ++n)
    {
      PCL_INFO ("%s   %s   %s\n", cams[n][itmSerialNumber].asString ().c_str (),
                                  cams[n][itmModelName].asString ().c_str (),
                                  cams[n][itmStatus].asString ().c_str ());
    }
    PCL_INFO ("\n");
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "enumDevices");
  }

  return (camera_count);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::openDevice (const int device)
{
  if (device_open_)
    PCL_THROW_EXCEPTION (pcl::IOException, "Cannot open multiple devices!");

  PCL_INFO ("Opening Ensenso stereo camera id = %d\n", device);

  try
  {
    // Create a pointer referencing the camera's tree item, for easier access:
    camera_ = (*root_)[itmCameras][itmBySerialNo][device];

    if (!camera_.exists () || camera_[itmType] != valStereo)
    {
      PCL_THROW_EXCEPTION (pcl::IOException, "Please connect a single stereo camera to your computer!");
    }

    NxLibCommand open (cmdOpen);
    open.parameters ()[itmCameras] = camera_[itmSerialNumber].asString ();
    open.execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "openDevice");
    return (false);
  }

  device_open_ = true;
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::closeDevice ()
{
  if (!device_open_)
    return (false);

  stop ();
  PCL_INFO ("Closing Ensenso stereo camera\n");

  try
  {
    NxLibCommand (cmdClose).execute ();
    device_open_ = false;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "closeDevice");
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::EnsensoGrabber::start ()
{
  if (isRunning ())
    return;

  if (!device_open_)
    openDevice (0);

  frequency_.reset ();
  running_ = true;
  grabber_thread_ = boost::thread (&pcl::EnsensoGrabber::processGrabbing, this);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::EnsensoGrabber::stop ()
{
  if (running_)
  {
    running_ = false;  // Stop processGrabbing () callback

    grabber_thread_.join ();  // join () waits for the thread to finish it's last iteration
    // See: http://www.boost.org/doc/libs/1_54_0/doc/html/thread/thread_management.html#thread.thread_management.thread.join
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::isRunning () const
{
  return (running_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::isTcpPortOpen () const
{
  return (tcp_open_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::EnsensoGrabber::getName () const
{
  return ("EnsensoGrabber");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::configureCapture (const bool auto_exposure,
                                       const bool auto_gain,
                                       const int bining,
                                       const float exposure,
                                       const bool front_light,
                                       const int gain,
                                       const bool gain_boost,
                                       const bool hardware_gamma,
                                       const bool hdr,
                                       const int pixel_clock,
                                       const bool projector,
                                       const int target_brightness,
                                       const std::string trigger_mode,
                                       const bool use_disparity_map_area_of_interest) const
{
  if (!device_open_)
    return (false);

  try
  {
    NxLibItem captureParams = camera_[itmParameters][itmCapture];
    captureParams[itmAutoExposure].set (auto_exposure);
    captureParams[itmAutoGain].set (auto_gain);
    captureParams[itmBinning].set (bining);
    captureParams[itmExposure].set (exposure);
    captureParams[itmFrontLight].set (front_light);
    captureParams[itmGain].set (gain);
    captureParams[itmGainBoost].set (gain_boost);
    captureParams[itmHardwareGamma].set (hardware_gamma);
    captureParams[itmHdr].set (hdr);
    captureParams[itmPixelClock].set (pixel_clock);
    captureParams[itmProjector].set (projector);
    captureParams[itmTargetBrightness].set (target_brightness);
    captureParams[itmTriggerMode].set (trigger_mode);
    captureParams[itmUseDisparityMapAreaOfInterest].set (use_disparity_map_area_of_interest);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "configureCapture");
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::grabSingleCloud (pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {
    NxLibCommand (cmdCapture).execute ();

    // Stereo matching task
    NxLibCommand (cmdComputeDisparityMap).execute ();

    // Convert disparity map into XYZ data for each pixel
    NxLibCommand (cmdComputePointMap).execute ();

    // Get info about the computed point map and copy it into a std::vector
    double timestamp;
    std::vector<float> pointMap;
    int width, height;
    camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);  // Get raw image timestamp
    camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
    camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);

    // Copy point cloud and convert in meters
    cloud.header.stamp = getPCLStamp (timestamp);
    cloud.resize (height * width);
    cloud.width = width;
    cloud.height = height;
    cloud.is_dense = false;

    // Copy data in point cloud (and convert milimeters in meters)
    for (size_t i = 0; i < pointMap.size (); i += 3)
    {
      cloud.points[i / 3].x = pointMap[i] / 1000.0;
      cloud.points[i / 3].y = pointMap[i + 1] / 1000.0;
      cloud.points[i / 3].z = pointMap[i + 2] / 1000.0;
    }

    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "grabSingleCloud");
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::initExtrinsicCalibration (const int grid_spacing) const
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {
    if (!clearCalibrationPatternBuffer ())
      return (false);
    (*root_)[itmParameters][itmPattern][itmGridSpacing].set (grid_spacing);  // GridSize can't be changed, it's protected in the tree
    // With the speckle projector on it is nearly impossible to recognize the pattern
    // (the 3D calibration is based on stereo images, not on 3D depth map)

    // Most important parameters are: projector=off, front_light=on
    configureCapture (true, true, 1, 0.32, true, 1, false, false, false, 10, false, 80, "Software", false);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "initExtrinsicCalibration");
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::clearCalibrationPatternBuffer () const
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {
    NxLibCommand (cmdDiscardPatterns).execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "clearCalibrationPatternBuffer");
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::EnsensoGrabber::captureCalibrationPattern () const
{
  if (!device_open_)
    return (-1);

  if (running_)
    return (-1);

  try
  {
    NxLibCommand (cmdCapture).execute ();
    NxLibCommand (cmdCollectPattern).execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "captureCalibrationPattern");
    return (-1);
  }

  return ( (*root_)[itmParameters][itmPatternCount].asInt ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::estimateCalibrationPatternPose (Eigen::Affine3d &pattern_pose) const
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {
    NxLibCommand estimate_pattern_pose (cmdEstimatePatternPose);
    estimate_pattern_pose.execute ();
    NxLibItem tf = estimate_pattern_pose.result ()[itmPatternPose];
    // Convert tf into a matrix
    if (!jsonTransformationToMatrix (tf.asJson (), pattern_pose))
      return (false);
    pattern_pose.translation () /= 1000.0;  // Convert translation in meters (Ensenso API returns milimeters)
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "estimateCalibrationPatternPoses");
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::computeCalibrationMatrix (const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &robot_poses,
                                               std::string &json,
                                               const std::string setup,
                                               const std::string target,
                                               const Eigen::Affine3d &guess_tf,
                                               const bool pretty_format) const
{
  if ( (*root_)[itmVersion][itmMajor] < 2 && (*root_)[itmVersion][itmMinor] < 3)
    PCL_WARN ("EnsensoSDK 1.3.x fixes bugs into extrinsic calibration matrix optimization, please update your SDK!\n"
              "http://www.ensenso.de/support/sdk-download/\n");

  try
  {
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > robot_poses_mm (robot_poses);
    std::vector<std::string> robot_poses_json;
    robot_poses_json.resize (robot_poses.size ());
    for (uint i = 0; i < robot_poses_json.size (); ++i)
    {
      robot_poses_mm[i].translation () *= 1000.0; // Convert meters in millimeters
      if (!matrixTransformationToJson (robot_poses_mm[i], robot_poses_json[i]))
        return (false);
    }

    NxLibCommand calibrate (cmdCalibrateHandEye);

    // Set Hand-Eye calibration parameters
    if (boost::iequals (setup, "Fixed"))
      calibrate.parameters ()[itmSetup].set (valFixed);
    else
      calibrate.parameters ()[itmSetup].set (valMoving);

    calibrate.parameters ()[itmTarget].set (target);

    // Set guess matrix
    if (guess_tf.matrix () != Eigen::Matrix4d::Identity ())
    {
      // Matrix > JSON > Angle axis
      NxLibItem tf ("/tmpTF");
      if (!matrixTransformationToJson (guess_tf, json))
        return (false);
      tf.setJson (json);

      // Rotation
      double theta = tf[itmRotation][itmAngle].asDouble ();  // Angle of rotation
      double x = tf[itmRotation][itmAxis][0].asDouble ();   // X component of Euler vector
      double y = tf[itmRotation][itmAxis][1].asDouble ();   // Y component of Euler vector
      double z = tf[itmRotation][itmAxis][2].asDouble ();   // Z component of Euler vector
      tf.erase(); // Delete tmpTF node

      calibrate.parameters ()[itmLink][itmRotation][itmAngle].set (theta);
      calibrate.parameters ()[itmLink][itmRotation][itmAxis][0].set (x);
      calibrate.parameters ()[itmLink][itmRotation][itmAxis][1].set (y);
      calibrate.parameters ()[itmLink][itmRotation][itmAxis][2].set (z);

      // Translation
      calibrate.parameters ()[itmLink][itmTranslation][0].set (guess_tf.translation ()[0] * 1000.0);
      calibrate.parameters ()[itmLink][itmTranslation][1].set (guess_tf.translation ()[1] * 1000.0);
      calibrate.parameters ()[itmLink][itmTranslation][2].set (guess_tf.translation ()[2] * 1000.0);
    }

    // Feed all robot poses into the calibration command
    for (uint i = 0; i < robot_poses_json.size (); ++i)
    {
      // Very weird behavior here:
      // If you modify this loop, check that all the transformations are still here in the [itmExecute][itmParameters] node
      // because for an unknown reason sometimes the old transformations are erased in the tree ("null" in the tree)
      // Ensenso SDK 2.3.348: If not moved after guess calibration matrix, the vector is empty.
      calibrate.parameters ()[itmTransformations][i].setJson (robot_poses_json[i], false);
    }

    calibrate.execute ();  // Might take up to 120 sec (maximum allowed by Ensenso API)

    if (calibrate.successful ())
    {
      json = calibrate.result ().asJson (pretty_format);
      return (true);
    }
    else
    {
      json.clear ();
      return (false);
    }
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "computeCalibrationMatrix");
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::storeEEPROMExtrinsicCalibration () const
{
  try
  {
    NxLibCommand store (cmdStoreCalibration);
    store.execute ();
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "storeEEPROMExtrinsicCalibration");
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::clearEEPROMExtrinsicCalibration ()
{
  try
  {
    setExtrinsicCalibration("");
    NxLibCommand store (cmdStoreCalibration);
    store.execute ();
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "clearEEPROMExtrinsicCalibration");
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::setExtrinsicCalibration (const double euler_angle,
                                              Eigen::Vector3d &rotation_axis,
                                              const Eigen::Vector3d &translation,
                                              const std::string target)
{
  if (!device_open_)
    return (false);

  if (rotation_axis != Eigen::Vector3d (0, 0, 0))  // Otherwise the vector becomes NaN
    rotation_axis.normalize ();

  try
  {
    NxLibItem calibParams = camera_[itmLink];
    calibParams[itmTarget].set (target);
    calibParams[itmRotation][itmAngle].set (euler_angle);

    calibParams[itmRotation][itmAxis][0].set (rotation_axis[0]);
    calibParams[itmRotation][itmAxis][1].set (rotation_axis[1]);
    calibParams[itmRotation][itmAxis][2].set (rotation_axis[2]);

    calibParams[itmTranslation][0].set (translation[0] * 1000.0);  // Convert in millimeters
    calibParams[itmTranslation][1].set (translation[1] * 1000.0);
    calibParams[itmTranslation][2].set (translation[2] * 1000.0);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setExtrinsicCalibration");
    return (false);
  }
  return (true);
}

bool
pcl::EnsensoGrabber::setExtrinsicCalibration (const std::string target)
{
  if (!device_open_)
    return (false);

  Eigen::Vector3d rotation (Eigen::Vector3d::Zero ());
  Eigen::Vector3d translation (Eigen::Vector3d::Zero ());
  return (setExtrinsicCalibration (0.0, rotation, translation, target));
}

bool
pcl::EnsensoGrabber::setExtrinsicCalibration (const Eigen::Affine3d &transformation,
                                              const std::string target)
{
  std::string json;
  if (!matrixTransformationToJson (transformation, json))
    return (false);

  double euler_angle;
  Eigen::Vector3d rotation_axis;
  Eigen::Vector3d translation;

  if (!jsonTransformationToAngleAxis (json, euler_angle, rotation_axis, translation))
    return (false);

  return (setExtrinsicCalibration (euler_angle, rotation_axis, translation, target));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::EnsensoGrabber::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  return (frequency_.getFrequency ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::openTcpPort (const int port)
{
  try
  {
    nxLibOpenTcpPort (port);
    tcp_open_ = true;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "openTcpPort");
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::closeTcpPort ()
{
  try
  {
    nxLibCloseTcpPort ();
    tcp_open_ = false;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "closeTcpPort");
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::EnsensoGrabber::getTreeAsJson (const bool pretty_format) const
{
  try
  {
    return (root_->asJson (pretty_format));
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getTreeAsJson");
    return ("");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::EnsensoGrabber::getResultAsJson (const bool pretty_format) const
{
  try
  {
    NxLibCommand cmd ("");
    return (cmd.result ().asJson (pretty_format));
  }

  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getResultAsJson");
    return ("");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::jsonTransformationToEulerAngles (const std::string &json,
                                                      double &x,
                                                      double &y,
                                                      double &z,
                                                      double &w,
                                                      double &p,
                                                      double &r) const
{
  try
  {
    NxLibCommand convert (cmdConvertTransformation);
    convert.parameters ()[itmTransformation].setJson (json, false);
    convert.parameters ()[itmSplitRotation].set (valXYZ);

    convert.execute ();

    NxLibItem tf = convert.result ()[itmTransformations];
    x = tf[0][itmTranslation][0].asDouble ();
    y = tf[0][itmTranslation][1].asDouble ();
    z = tf[0][itmTranslation][2].asDouble ();
    r = tf[0][itmRotation][itmAngle].asDouble ();  // Roll
    p = tf[1][itmRotation][itmAngle].asDouble ();  // Pitch
    w = tf[2][itmRotation][itmAngle].asDouble ();  // yaW
    return (true);
  }

  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "jsonTransformationToEulerAngles");
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::jsonTransformationToAngleAxis (const std::string json,
                                                    double &alpha,
                                                    Eigen::Vector3d &axis,
                                                    Eigen::Vector3d &translation) const
{
  try
  {
    NxLibItem tf ("/tmpTF");
    tf.setJson(json);
    translation[0] = tf[itmTranslation][0].asDouble ();
    translation[1] = tf[itmTranslation][1].asDouble ();
    translation[2] = tf[itmTranslation][2].asDouble ();

    alpha = tf[itmRotation][itmAngle].asDouble ();  // Angle of rotation
    axis[0] = tf[itmRotation][itmAxis][0].asDouble ();  // X component of Euler vector
    axis[1] = tf[itmRotation][itmAxis][1].asDouble ();  // Y component of Euler vector
    axis[2] = tf[itmRotation][itmAxis][2].asDouble ();  // Z component of Euler vector
    tf.erase(); // Delete tmpTF node
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "jsonTransformationToAngleAxis");
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::jsonTransformationToMatrix (const std::string transformation,
                                                 Eigen::Affine3d &matrix) const
{
  try
  {
    NxLibCommand convert_transformation (cmdConvertTransformation);
    convert_transformation.parameters ()[itmTransformation].setJson (transformation);
    convert_transformation.execute ();
    Eigen::Affine3d tmp (Eigen::Affine3d::Identity ());

    // Rotation
    tmp.linear ().col (0) = Eigen::Vector3d (convert_transformation.result ()[itmTransformation][0][0].asDouble (),
                                             convert_transformation.result ()[itmTransformation][0][1].asDouble (),
                                             convert_transformation.result ()[itmTransformation][0][2].asDouble ());

    tmp.linear ().col (1) = Eigen::Vector3d (convert_transformation.result ()[itmTransformation][1][0].asDouble (),
                                             convert_transformation.result ()[itmTransformation][1][1].asDouble (),
                                             convert_transformation.result ()[itmTransformation][1][2].asDouble ());

    tmp.linear ().col (2) = Eigen::Vector3d (convert_transformation.result ()[itmTransformation][2][0].asDouble (),
                                             convert_transformation.result ()[itmTransformation][2][1].asDouble (),
                                             convert_transformation.result ()[itmTransformation][2][2].asDouble ());

    // Translation
    tmp.translation () = Eigen::Vector3d (convert_transformation.result ()[itmTransformation][3][0].asDouble (),
                                          convert_transformation.result ()[itmTransformation][3][1].asDouble (),
                                          convert_transformation.result ()[itmTransformation][3][2].asDouble ());
    matrix = tmp;
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "jsonTransformationToMatrix");
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::eulerAnglesTransformationToJson (const double x,
                                                      const double y,
                                                      const double z,
                                                      const double w,
                                                      const double p,
                                                      const double r,
                                                      std::string &json,
                                                      const bool pretty_format) const
{
  try
  {
    NxLibCommand chain (cmdChainTransformations);
    NxLibItem tf = chain.parameters ()[itmTransformations];

    if (!angleAxisTransformationToJson (x, y, z, 0, 0, 1, r, json))
      return (false);
    tf[0].setJson (json, false);  // Roll

    if (!angleAxisTransformationToJson (0, 0, 0, 0, 1, 0, p, json))
       return (false);
    tf[1].setJson (json, false);  // Pitch

    if (!angleAxisTransformationToJson (0, 0, 0, 1, 0, 0, w, json))
       return (false);
    tf[2].setJson (json, false);  // yaW

    chain.execute ();
    json = chain.result ()[itmTransformation].asJson (pretty_format);
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "eulerAnglesTransformationToJson");
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::angleAxisTransformationToJson (const double x,
                                                    const double y,
                                                    const double z,
                                                    const double rx,
                                                    const double ry,
                                                    const double rz,
                                                    const double alpha,
                                                    std::string &json,
                                                    const bool pretty_format) const
{
  try
  {
    NxLibItem tf ("/tmpTF");
    tf[itmTranslation][0].set (x);
    tf[itmTranslation][1].set (y);
    tf[itmTranslation][2].set (z);

    tf[itmRotation][itmAngle].set (alpha);  // Angle of rotation
    tf[itmRotation][itmAxis][0].set (rx);  // X component of Euler vector
    tf[itmRotation][itmAxis][1].set (ry);  // Y component of Euler vector
    tf[itmRotation][itmAxis][2].set (rz);  // Z component of Euler vector

    json = tf.asJson (pretty_format);
    tf.erase ();
    return (true);
  }

  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "angleAxisTransformationToJson");
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EnsensoGrabber::matrixTransformationToJson (const Eigen::Affine3d &matrix,
                                                 std::string &json,
                                                 const bool pretty_format) const
{
  try
  {
    NxLibCommand convert_transformation (cmdConvertTransformation);
    // Rotation
    convert_transformation.parameters ()[itmTransformation][0][0].set (matrix.linear ().col (0)[0]);
    convert_transformation.parameters ()[itmTransformation][0][1].set (matrix.linear ().col (0)[1]);
    convert_transformation.parameters ()[itmTransformation][0][2].set (matrix.linear ().col (0)[2]);
    convert_transformation.parameters ()[itmTransformation][0][3].set (0.0);

    convert_transformation.parameters ()[itmTransformation][1][0].set (matrix.linear ().col (1)[0]);
    convert_transformation.parameters ()[itmTransformation][1][1].set (matrix.linear ().col (1)[1]);
    convert_transformation.parameters ()[itmTransformation][1][2].set (matrix.linear ().col (1)[2]);
    convert_transformation.parameters ()[itmTransformation][1][3].set (0.0);

    convert_transformation.parameters ()[itmTransformation][2][0].set (matrix.linear ().col (2)[0]);
    convert_transformation.parameters ()[itmTransformation][2][1].set (matrix.linear ().col (2)[1]);
    convert_transformation.parameters ()[itmTransformation][2][2].set (matrix.linear ().col (2)[2]);
    convert_transformation.parameters ()[itmTransformation][2][3].set (0.0);

    // Translation
    convert_transformation.parameters ()[itmTransformation][3][0].set (matrix.translation ()[0]);
    convert_transformation.parameters ()[itmTransformation][3][1].set (matrix.translation ()[1]);
    convert_transformation.parameters ()[itmTransformation][3][2].set (matrix.translation ()[2]);
    convert_transformation.parameters ()[itmTransformation][3][3].set (1.0);

    convert_transformation.execute ();
    json = convert_transformation.result ()[itmTransformation].asJson (pretty_format);
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "matrixTransformationToJson");
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::uint64_t
pcl::EnsensoGrabber::getPCLStamp (const double ensenso_stamp)
{
#if defined _WIN32 || defined _WIN64
  return (ensenso_stamp * 1000000.0);
#else
  return ( (ensenso_stamp - 11644473600.0) * 1000000.0);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::EnsensoGrabber::getOpenCVType (const int channels,
                                    const int bpe,
                                    const bool isFlt)
{
  int bits = bpe * 8;
  char type = isFlt ? 'F' : (bpe > 3 ? 'S' : 'U');
  return (boost::str (boost::format ("CV_%i%cC%i") % bits % type % channels));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::EnsensoGrabber::processGrabbing ()
{
  bool continue_grabbing = running_;
  while (continue_grabbing)
  {
    try
    {
      // Publish cloud / images
      if (num_slots<sig_cb_ensenso_point_cloud> () > 0 || num_slots<sig_cb_ensenso_images> () > 0 || num_slots<sig_cb_ensenso_point_cloud_images> () > 0)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        boost::shared_ptr<PairOfImages> images (new PairOfImages);

        fps_mutex_.lock ();
        frequency_.event ();
        fps_mutex_.unlock ();

        NxLibCommand (cmdCapture).execute ();
        double timestamp;
        camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);

        // Gather images
        if (num_slots<sig_cb_ensenso_images> () > 0 || num_slots<sig_cb_ensenso_point_cloud_images> () > 0)
        {
          // Rectify images
          NxLibCommand (cmdRectifyImages).execute ();
          int width, height, channels, bpe;
          bool isFlt, collected_pattern = false;

          try  // Try to collect calibration pattern, if not possible, publish RAW images instead
          {
            NxLibCommand collect_pattern (cmdCollectPattern);
            collect_pattern.parameters ()[itmBuffer].set (false);  // Do NOT store the pattern into the buffer!
            collect_pattern.execute ();
            collected_pattern = true;
          }
          catch (const NxLibException &ex)
          {
            // We failed to collect the pattern but the RAW images are available!
          }

          if (collected_pattern)
          {
            camera_[itmImages][itmWithOverlay][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            images->first.header.stamp = images->second.header.stamp = getPCLStamp (timestamp);
            images->first.width = images->second.width = width;
            images->first.height = images->second.height = height;
            images->first.data.resize (width * height * sizeof(float));
            images->second.data.resize (width * height * sizeof(float));
            images->first.encoding = images->second.encoding = getOpenCVType (channels, bpe, isFlt);

            camera_[itmImages][itmWithOverlay][itmLeft].getBinaryData (images->first.data.data (), images->first.data.size (), 0, 0);
            camera_[itmImages][itmWithOverlay][itmRight].getBinaryData (images->second.data.data (), images->second.data.size (), 0, 0);
          }
          else
          {
            camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (&width, &height, &channels, &bpe, &isFlt, 0);
            images->first.header.stamp = images->second.header.stamp = getPCLStamp (timestamp);
            images->first.width = images->second.width = width;
            images->first.height = images->second.height = height;
            images->first.data.resize (width * height * sizeof(float));
            images->second.data.resize (width * height * sizeof(float));
            images->first.encoding = images->second.encoding = getOpenCVType (channels, bpe, isFlt);

            camera_[itmImages][itmRaw][itmLeft].getBinaryData (images->first.data.data (), images->first.data.size (), 0, 0);
            camera_[itmImages][itmRaw][itmRight].getBinaryData (images->second.data.data (), images->second.data.size (), 0, 0);
          }
        }

        // Gather point cloud
        if (num_slots<sig_cb_ensenso_point_cloud> () > 0 || num_slots<sig_cb_ensenso_point_cloud_images> () > 0)
        {
          // Stereo matching task
          NxLibCommand (cmdComputeDisparityMap).execute ();

          // Convert disparity map into XYZ data for each pixel
          NxLibCommand (cmdComputePointMap).execute ();

          // Get info about the computed point map and copy it into a std::vector
          std::vector<float> pointMap;
          int width, height;
          camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
          camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);

          // Copy point cloud and convert in meters
          cloud->header.stamp = getPCLStamp (timestamp);
          cloud->points.resize (height * width);
          cloud->width = width;
          cloud->height = height;
          cloud->is_dense = false;

          // Copy data in point cloud (and convert milimeters in meters)
          for (size_t i = 0; i < pointMap.size (); i += 3)
          {
            cloud->points[i / 3].x = pointMap[i] / 1000.0;
            cloud->points[i / 3].y = pointMap[i + 1] / 1000.0;
            cloud->points[i / 3].z = pointMap[i + 2] / 1000.0;
          }
        }

        // Publish signals
        if (num_slots<sig_cb_ensenso_point_cloud_images> () > 0)
          point_cloud_images_signal_->operator () (cloud, images);
        else if (num_slots<sig_cb_ensenso_point_cloud> () > 0)
          point_cloud_signal_->operator () (cloud);
        else if (num_slots<sig_cb_ensenso_images> () > 0)
          images_signal_->operator () (images);
      }
      continue_grabbing = running_;
    }
    catch (NxLibException &ex)
    {
      ensensoExceptionHandling (ex, "processGrabbing");
    }
  }
}

