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

#ifndef __PCL_IO_ENSENSO_GRABBER__
#define __PCL_IO_ENSENSO_GRABBER__

#include <pcl/common/time.h>
#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <boost/thread.hpp>

#include <pcl/io/grabber.h>
#include <pcl/common/synchronizer.h>

#include <nxLib.h> // Ensenso SDK

namespace pcl
{
  struct PointXYZ;
  template <typename T> class PointCloud;

  /** @brief Grabber for IDS-Imaging Enenso's devices
   * The <a href="http://www.ensenso.de/manual/">Ensenso SDK</a> allow to use multiple Ensenso devices to produce a single cloud.\n
   * This feature is not implemented here, it is up to the user to configure multiple Ensenso cameras.\n
   * @author Victor Lamoine (victor.lamoine@gmail.com)\n
   * @ingroup io
   */
  class PCL_EXPORTS EnsensoGrabber : public Grabber
  {
    public:
      typedef boost::shared_ptr<EnsensoGrabber> Ptr;
      typedef boost::shared_ptr<const EnsensoGrabber> ConstPtr;

      // Define callback signature typedefs
      typedef void
      (sig_cb_ensenso_point_cloud) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);

      /** @brief Constructor */
      EnsensoGrabber ();

      /** @brief Destructor inherited from the Grabber interface. It never throws. */
      virtual
      ~EnsensoGrabber () throw ();

      /** @brief Searches for available devices
       * @returns the number of Ensenso devices connected */
      int
      enumDevices ();

      /** @brief Opens an Ensenso device
       * @param[in] device the device ID to open
       * @return true if successful, false otherwise */
      bool
      openDevice (const int device = 0);

      /** @brief Closes the Ensenso device */
      bool
      closeDevice ();

      /** @brief Start the data acquisition 
        * @note Opens device "0" if no device is open */
      void
      start ();

      /** @brief Stop the data acquisition */
      void
      stop ();

      /** @brief Check if the data acquisition is still running */
      bool
      isRunning () const;

      /** @brief Get class name
       * @returns a string containing the class name
       */
      std::string
      getName () const;

      /** @brief Configure Ensenso capture settings
       * @param[in] auto_exposure If set to yes, the exposure parameter will be ignored
       * @param[in] auto_gain if set yo yes, the gain parameter will be ignored
       * @param[in] bining Pixel bining: 1, 2 or 4
       * @param[in] exposure In milliseconds, from 0.01 to 20 ms
       * @param[in] front_light Infrared front light (usefull for calibration)
       * @param[in] gain Float between 1 and 4
       * @param[in] gain_boost
       * @param[in] hardware_gamma
       * @param[in] hdr High Dynamic Range (check compatibility with other options in Ensenso manual)
       * @param[in] pixel_clock in MegaHertz, from 5 to 85
       * @param[in] projector Use the central infrared projector or not
       * @param[in] target_brightness Between 40 and 210
       * @param[in] trigger_mode
       * @param[in] use_disparity_map_area_of_interest
       * @return True if successful, false otherwise
       * @note See <a href="http://www.ensenso.de/manual/index.html?capture.htm">Capture tree item</a> for more
       * details about the parameters. */
      bool
      configureCapture (const bool auto_exposure = true,
                        const bool auto_gain = true,
                        const int bining = 1,
                        const float exposure = 0.32,
                        const bool front_light = false,
                        const int gain = 1,
                        const bool gain_boost = false,
                        const bool hardware_gamma = false,
                        const bool hdr = false,
                        const int pixel_clock = 10,
                        const bool projector = true,
                        const int target_brightness = 80,
                        const std::string trigger_mode = "Software",
                        const bool use_disparity_map_area_of_interest = false) const;

      /** @brief Update Link node in NxLib tree
       * @param[in] target "Hand" or "Workspace" for example
       * @param[in] euler_angle
       * @param[in] rotation_axis
       * @param[in] translation
       * @return True if successful, false otherwise
       * @warning Translation are in millimetres, rotation angles in radians!
       * @note If a calibration has been stored in the EEPROM, it is copied in the Link node at nxLib tree start.
       * This method overwrites the Link node but does not write to the EEPROM.
       *
       * More information on the parameters can be found in <a href="http://www.ensenso.de/manual/index.html?cameralink.htm">Link node</a> section of the Ensenso manual.
       *
       * The point cloud you get from the Ensenso is already transformed using this calibration matrix.
       * Make sure it is the identity transformation if you want the original point cloud!*/
      bool
      setExtrinsicCalibration (const std::string target = "Hand",
                              const float euler_angle = 0.0,
                              const Eigen::Vector3f rotation_axis = Eigen::Vector3f (0.0, 0.0, 0.0),
                              const Eigen::Vector3f translation = Eigen::Vector3f (0.0, 0.0, 0.0));

      /** @brief Obtain the number of frames per second (FPS) */
      float
      getFramesPerSecond () const;

      /** @brief Open TCP port to enable access via the <a href="http://www.ensenso.de/manual/software_components.htm">nxTreeEdit</a> program.
       * @param[in] port The port number
       * @return True if successful, false otherwise */
      bool
      openTcpPort (const int port = 24000) const;

      /** @brief Close TCP port program
       * @return True if successful, false otherwise
       * @warning If you do not close the TCP port the program might exit with the port still open, if it is the case
       * use @code ps -ef@endcode and @code kill PID @endcode to kill the application and effectively close the port. */
      bool
      closeTcpPort (void) const;

      /** @brief Returns the full NxLib tree as a JSON string
       * @param[in] pretty_format JSON formatting style
       * @return A string containing the NxLib tree in JSON format
       */
      std::string
      getTreeAsJson (const bool pretty_format = true) const;

      /** @brief Returns the Result node (of the last command) as a JSON string
       * @param[in] pretty_format JSON formatting style
       * @return A string containing the Result node in JSON format
       */
      std::string
      getResultAsJson (const bool pretty_format = true) const;

      /** @brief Get the Euler angles corresponding to a JSON string (an angle axis transformation)
       * @param[in] json A string containing the angle axis transformation in JSON format
       * @param[out] x The X translation
       * @param[out] y The Y translation
       * @param[out] z The Z translation
       * @param[out] w The yaW angle
       * @param[out] p The Pitch angle
       * @param[out] r The Roll angle
       * @return True if successful, false otherwise
       * @warning The units are meters and radians!
       * @note See: <a href="http://www.ensenso.de/manual/transformation.htm">transformation page</a> in the EnsensoSDK documentation
       */
      bool
      transformationJsonToEulerAngles (const std::string &json,
                                       double &x,
                                       double &y,
                                       double &z,
                                       double &w,
                                       double &p,
                                       double &r);

      /** @brief Get the JSON string corresponding to an angle axis transformation
       * @param[in] x The X angle
       * @param[in] y The Y angle
       * @param[in] z The Z angle
       * @param[in] rx The X component of the Euler axis
       * @param[in] ry The Y component of the Euler axis
       * @param[in] rz The Z componenet of the Euler axis
       * @param[in] alpha The Euler rotation angle
       * @param[in] pretty_format JSON formatting style
       * @return A string containing the angle axis transformation in JSON format
       * @warning The units are meters and radians! (the Euler axis doesn't need to be normalized)
       * @note See: <a href="http://www.ensenso.de/manual/transformation.htm">transformation page</a> in the EnsensoSDK documentation
       */
      std::string
      angleAxisToTransformationJson (const double x,
                                     const double y,
                                     const double z,
                                     const double rx,
                                     const double ry,
                                     const double rz,
                                     const double alpha,
                                     const bool pretty_format = true);

      /** @brief Get the JSON string corresponding to the Euler angles transformation
       * @param[in] x The X translation
       * @param[in] y The Y translation
       * @param[in] z The Z translation
       * @param[in] w The yaW angle
       * @param[in] p The Pitch angle
       * @param[in] r The Roll angle
       * @param[in] pretty_format JSON formatting style
       * @return A string containing the Euler angles transformation in JSON format
       * @warning The units are meters and radians!
       * @note See: <a href="http://www.ensenso.de/manual/transformation.htm">transformation page</a> in the EnsensoSDK documentation
       */
      std::string
      eulerAnglesToTransformationJson (const double x,
                                       const double y,
                                       const double z,
                                       const double w,
                                       const double p,
                                       const double r,
                                       const bool pretty_format = true);

      /** @brief Reference to the NxLib tree root
       * @warning You must handle NxLib exceptions manually when playing with root!
       * See void ensensoExceptionHandling in ensenso_grabber.cpp */
      boost::shared_ptr<const NxLibItem> root_;

    protected:
      /** @brief Grabber thread */
      boost::thread grabber_thread_;

      /** @brief Reference to the camera tree */
      NxLibItem camera_;

      /** @brief Boost point cloud signal */
      boost::signals2::signal<sig_cb_ensenso_point_cloud>* point_cloud_signal_;

      /** @brief Whether an Ensenso device is opened or not */
      bool device_open_;

      /** @brief Whether an Ensenso device is running or not */
      bool running_;

      /** @brief Point cloud capture/processing frequency */
      pcl::EventFrequency frequency_;

      /** @brief Mutual exclusion for FPS computation */
      mutable boost::mutex fps_mutex_;

      /** @brief Continously asks for data from the device and publishes it if available. */
      void
      processGrabbing ();

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace pcl

#endif // __PCL_IO_ENSENSO_GRABBER__

