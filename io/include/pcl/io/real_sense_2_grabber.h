/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2018-, Open Perception, Inc.
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
 */

#pragma once

#include <thread>
#include <mutex>

#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <librealsense2/rs.hpp>

namespace pcl
{

  /** \brief Grabber for Intel Realsense 2 SDK devices (D400 series)
    * \note Device width/height defaults to 424/240, the lowest resolutions for D400 devices.
    * \note Testing on the in_hand_scanner example we found the lower default resolution allowed the app to perform adequately.
    * \note Developers should use this resolution as a starting point and gradually increase to get the best results
    * \author Patrick Abadi <patrickabadi@gmail.com>, Daniel Packard <pack3754@gmail.com>
    * \ingroup io
  */
  class PCL_EXPORTS RealSense2Grabber : public pcl::Grabber
  {
  public:
    /** \brief Constructor
    * \param[in] file_name_or_serial_number used for either loading bag file or specific device by serial number
    * \param[in] repeat_playback whether to repeat playback when reading from file
    */
    RealSense2Grabber ( const std::string& file_name_or_serial_number = "", const bool repeat_playback = true );

    /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
    ~RealSense2Grabber () override;

    /** \brief Set the device options
    * \param[in] width resolution
    * \param[in] height resolution
    * \param[in] fps target frames per second for the device
    */
    inline void
    setDeviceOptions ( std::uint32_t width, std::uint32_t height, std::uint32_t fps = 30 )
    {
      device_width_ = width;
      device_height_ = height;
      target_fps_ = fps;

      reInitialize ();
    }

    /** \brief Start the data acquisition. */
    void
    start () override;

    /** \brief Stop the data acquisition. */
    void
    stop () override;

    /** \brief Check if the data acquisition is still running. */
    bool
    isRunning () const override;

    /** \brief Obtain the number of frames per second (FPS). */
    float
    getFramesPerSecond () const override;

    /** \brief defined grabber name*/
    std::string
    getName () const override { return std::string ( "RealSense2Grabber" ); }

    //define callback signature typedefs
    using signal_librealsense_PointXYZ = void( const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& );
    using signal_librealsense_PointXYZI = void( const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& );
    using signal_librealsense_PointXYZRGB = void( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& );
    using signal_librealsense_PointXYZRGBA = void( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& );

  protected:

    boost::signals2::signal<signal_librealsense_PointXYZ>* signal_PointXYZ;
    boost::signals2::signal<signal_librealsense_PointXYZI>* signal_PointXYZI;
    boost::signals2::signal<signal_librealsense_PointXYZRGB>* signal_PointXYZRGB;
    boost::signals2::signal<signal_librealsense_PointXYZRGBA>* signal_PointXYZRGBA;

    /** \brief Handle when a signal callback has been changed
    */
    void
    signalsChanged () override;

    /** \brief the thread function
    */
    void
    threadFunction ();

    /** \brief Dynamic reinitialization.
    */
    void
    reInitialize ();

    /** \brief Convert a Depth image to a pcl::PointCloud<pcl::PointXYZ>
    * \param[in] points the depth points
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    convertDepthToPointXYZ ( const rs2::points& points );

    /** \brief Convert an Infrared Depth image to a pcl::PointCloud<pcl::PointXYZI>
    * \param[in] points the depth points
    * \param[in] ir Infrared video frame
    */
    pcl::PointCloud<pcl::PointXYZI>::Ptr
    convertIntensityDepthToPointXYZRGBI ( const rs2::points& points, const rs2::video_frame& ir );

    /** \brief Convert an rgb Depth image to a pcl::PointCloud<pcl::PointXYZRGB>
    * \param[in] points the depth points
    * \param[in] rgb rgb video frame
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    convertRGBDepthToPointXYZRGB ( const rs2::points& points, const rs2::video_frame& rgb );

    /** \brief Convert an rgb Depth image to a pcl::PointCloud<pcl::PointXYZRGBA>
    * \param[in] points the depth points
    * \param[in] rgb rgb video frame
    */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    convertRGBADepthToPointXYZRGBA ( const rs2::points& points, const rs2::video_frame& rgb );

    /** \brief template function to convert realsense point cloud to PCL point cloud
    * \param[in] points - realsense point cloud array
    * \param[in] mapColorFunc dynamic function to convert individual point color or intensity values
    */
    template <typename PointT, typename Functor>
    typename pcl::PointCloud<PointT>::Ptr
    convertRealsensePointsToPointCloud ( const rs2::points& points, Functor mapColorFunc );

    /** \brief Retrieve pixel index for UV texture coordinate
    * \param[in] texture the texture
    * \param[in] u 2D coordinate
    * \param[in] v 2D coordinate
    */
    static size_t
    getTextureIdx (const rs2::video_frame & texture, float u, float v);

    /** \brief Retrieve RGB color from texture video frame
    * \param[in] texture the texture
    * \param[in] u 2D coordinate
    * \param[in] v 2D coordinate
    */
    static pcl::RGB
    getTextureColor ( const rs2::video_frame& texture, float u, float v );

    /** \brief Retrieve color intensity from texture video frame
    * \param[in] texture the texture
    * \param[in] u 2D coordinate
    * \param[in] v 2D coordinate
    */
    static std::uint8_t
    getTextureIntensity ( const rs2::video_frame& texture, float u, float v );


    /** \brief handle to the thread */
    std::thread thread_;
    /** \brief Defines either a file path to a bag file or a realsense device serial number. */
    std::string file_name_or_serial_number_;
    /** \brief Repeat playback when reading from file */
    bool repeat_playback_;
    /** \brief controlling the state of the thread. */
    bool quit_;
    /** \brief Is the grabber running. */
    bool running_;
    /** \brief Calculated FPS for the grabber. */
    float fps_;
    /** \brief Width for the depth and color sensor. Default 424*/
    std::uint32_t device_width_;
    /** \brief Height for the depth and color sensor. Default 240 */
    std::uint32_t device_height_;
    /** \brief Target FPS for the device. Default 30. */
    std::uint32_t target_fps_;
    /** \brief Declare pointcloud object, for calculating pointclouds and texture mappings */
    rs2::pointcloud pc_;
    /** \brief Declare RealSense pipeline, encapsulating the actual device and sensors */
    rs2::pipeline pipe_;
  };

}
