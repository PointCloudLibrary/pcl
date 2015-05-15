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

#ifndef __PCL_IO_DAVIDSDK_GRABBER__
#define __PCL_IO_DAVIDSDK_GRABBER__

#include <pcl/common/time.h>
#include <pcl/common/io.h>
#include <boost/thread.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/io/grabber.h>

#include <david.h>

namespace pcl
{
  struct PointXYZ;
  template <typename T> class PointCloud;

  /** @brief Grabber for davidSDK structured light compliant devices.\n
   * The [davidSDK SDK](http://www.david-3d.com/en/products/david-sdk) allows to use a structured light scanner to
   * fetch clouds/meshes.\n
   * The purpose of this grabber is NOT to provide all davidSDK functionalities but rather provide a PCL-unified interface to the sensor for
   * basic operations.\n
   * Please consult the [David-3d wiki](http://wiki.david-3d.com/david-wiki) for more information.
   * @author Victor Lamoine (victor.lamoine@gmail.com)\n
   * @ingroup io
   */
  class PCL_EXPORTS DavidSDKGrabber : public Grabber
  {
    public:
      /** @cond */
      typedef boost::shared_ptr<DavidSDKGrabber> Ptr;
      typedef boost::shared_ptr<const DavidSDKGrabber> ConstPtr;

      // Define callback signature typedefs
      typedef void
      (sig_cb_davidsdk_point_cloud) (const pcl::PointCloud<pcl::PointXYZ>::Ptr &);

      typedef void
      (sig_cb_davidsdk_mesh) (const pcl::PolygonMesh::Ptr &);

      typedef void
      (sig_cb_davidsdk_image) (const boost::shared_ptr<pcl::PCLImage> &);

      typedef void
      (sig_cb_davidsdk_point_cloud_image) (const pcl::PointCloud<pcl::PointXYZ>::Ptr &,
                                           const boost::shared_ptr<pcl::PCLImage> &);

      typedef void
      (sig_cb_davidsdk_mesh_image) (const pcl::PolygonMesh::Ptr &,
                                    const boost::shared_ptr<pcl::PCLImage> &);
      /** @endcond */

      /** @brief Constructor */
      DavidSDKGrabber ();

      /** @brief Destructor inherited from the Grabber interface. It never throws. */
      virtual
      ~DavidSDKGrabber () throw ();

      /** @brief [Connect](http://docs.david-3d.com/sdk/en/classdavid_1_1_client_json_rpc.html#a4b948e57a2e5e7f9cdcf1171c500aa24) client
       * @param[in] address
       * @param[in] port
       * @return Server info*/
      david::ServerInfo
      connect (const std::string & address = "127.0.0.1",
               uint16_t port = david::DAVID_SDK_DefaultPort);

      /** @brief [Disconnect](http://docs.david-3d.com/sdk/en/classdavid_1_1_client_json_rpc.html#a2770728a6de2c708df767bedf8be0814) client
       * @param[in] stop_server */
      void
      disconnect (const bool stop_server);

      /** @brief Start the point cloud and or image acquisition */
      void
      start ();

      /** @brief Stop the data acquisition */
      void
      stop ();

      /** @brief Check if the data acquisition is still running
       * @return True if running, false otherwise */
      bool
      isRunning () const;

      /** @brief Check if the client is connected
       * @return True if connected, false otherwise */
      bool
      isConnected () const;

      /** @brief Get class name
       * @returns A string containing the class name */
      std::string
      getName () const;

      /** @brief Get @ref local_path_ path directory
       * @returns the path */
      std::string
      getLocalPath ();

      /** @brief Get @ref remote_path_ path directory
       * @returns the path */
      std::string
      getRemotePath ();

      /** @brief Set @ref file_format_ to "obj" */
      void
      setFileFormatToOBJ ();

      /** @brief Set @ref file_format_ to "ply"
       * @warning This format is NOT available with trial versions of the davidSDK server! */
      void
      setFileFormatToPLY ();

      /** @brief Set @ref file_format_ to "stl" */
      void
      setFileFormatToSTL ();

      /** @brief Get @ref file_format_
       * @returns the file format */
      std::string
      getFileFormat ();

      /** @brief Set @ref local_path_ path directory for scanning files
       * @param[in] path The directory path
       *
       * If the path is empty, using default value ("C:/temp") */
      void
      setLocalPath (std::string path);

      /** @brief Set @ref remote_path_ path directory for scanning files
       * @param[in] path The directory path
       *
       * If the string is empty, @ref remote_path_ = @ref local_path_ */
      void
      setRemotePath (std::string path);

      /** @brief Set @ref local_path_ and @ref remote_path_ directory paths
       * @param[in] local_path
       * @param[in] remote_path
       *
       * If the path is empty, using default value ("C:/temp") */
      void
      setLocalAndRemotePaths (std::string local_path,
                              std::string remote_path);

      /** @brief Calibrate the scanner
       * @param[in] grid_size Size of the calibration grid in millimeters
       * @return True if successful, false otherwise
       *
       * More information [here](http://wiki.david-3d.com/david3_user_manual/structured_light).\n
       * Also see [ImportCalibration](http://docs.david-3d.com/sdk/en/classdavid_1_1_i_structured_light_scanner.html#a68e888636883d90aac7891d2ef9e6b27).\n
       * and [ExportCalibration](http://docs.david-3d.com/sdk/en/classdavid_1_1_i_structured_light_scanner.html#a66817b07227f9a8852663d9141ae48db).
       *
       * @warning You MUST perform calibration each time you modify the camera/projector focus or move the camera relatively to the projector.\n
       */
      bool
      calibrate (double grid_size);

      /** @brief Capture a single point cloud and store it
       * @param[out] cloud The cloud to be filled
       * @return True if successful, false otherwise
       * @warning Calls [DeleteAllMeshes()](http://docs.david-3d.com/sdk/en/classdavid_1_1_i_shape_fusion.html#aed22e458b51f1361803360c02c2d1403) */
      bool
      grabSingleCloud (pcl::PointCloud<pcl::PointXYZ> &cloud);

      /** @brief Capture a single mesh and store it
       * @param[out] mesh The mesh to be filled
       * @return True if successful, false otherwise
       * @warning Calls [DeleteAllMeshes()](http://docs.david-3d.com/sdk/en/classdavid_1_1_i_shape_fusion.html#aed22e458b51f1361803360c02c2d1403) */
      bool
      grabSingleMesh (pcl::PolygonMesh &mesh);

      /** @brief Obtain the number of frames per second (FPS) */
      float
      getFramesPerSecond () const;

      /** @brief davidSDK client */
      david::Client david_;

    protected:
      /** @brief Grabber thread */
      boost::thread grabber_thread_;

      /** @brief Boost point cloud signal */
      boost::signals2::signal<sig_cb_davidsdk_point_cloud>* point_cloud_signal_;

      /** @brief Boost mesh signal */
      boost::signals2::signal<sig_cb_davidsdk_mesh>* mesh_signal_;

      /** @brief Boost image signal */
      boost::signals2::signal<sig_cb_davidsdk_image>* image_signal_;

      /** @brief Boost image + point cloud signal */
      boost::signals2::signal<sig_cb_davidsdk_point_cloud_image>* point_cloud_image_signal_;

      /** @brief Boost mesh + image signal */
      boost::signals2::signal<sig_cb_davidsdk_mesh_image>* mesh_image_signal_;

      /** @brief Whether the client is connected */
      bool client_connected_;

      /** @brief Whether an davidSDK device is running or not */
      bool running_;

      /** @brief Local path of directory where the scanning file will be located.
       * @note Default value is @c C:/temp */
      std::string local_path_;

      /** @brief Remote path of directory where the scanning file will be located.
       * @note If this is empty, the @ref local_path_ will be used instead
       * Default value is @c C:/temp */
      std::string remote_path_;

      /** @brief Export file extension, available formats are STL, OBJ, PLY */
      std::string file_format_;

      /** @brief processGrabbing capture/processing frequency */
      pcl::EventFrequency frequency_;

      /** @brief Mutual exclusion for FPS computation */
      mutable boost::mutex fps_mutex_;

      /** @brief Continuously asks for images and or point clouds/meshes data from the device and publishes them if available. */
      void
      processGrabbing ();
  };
}  // namespace pcl

#endif // __PCL_IO_DAVIDSDK_GRABBER__
