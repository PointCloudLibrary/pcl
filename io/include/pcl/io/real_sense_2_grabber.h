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

#ifndef PCL_IO_REAL_SENSE_2_GRABBER_H
#define PCL_IO_REAL_SENSE_2_GRABBER_H

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <librealsense2/rs.hpp>

namespace pcl
{

  /** \brief Grabber for Intel Realsense 2 SDK devices (D400 series)
  * \author Patrick Abadi <patrickabadi@gmail.com>, Daniel Packard <pack3754@gmail.com>
  * \ingroup io
  */
  class PCL_EXPORTS RealSense2Grabber : public pcl::Grabber
  {
  public:
    /** \brief Constructor
    */
    RealSense2Grabber ();

    /** \brief Create with Bag file
    * \param[in] file_name path to bag file
    */
    static RealSense2Grabber& createFromBagFile (const std::string& file_name) { return RealSense2Grabber (file_name, ""); } 

    /** \brief Create with specific device serial number
    * \param[in] serial_number
    */
    static RealSense2Grabber& createFromSerialNumber (const std::string& serial_number) { return RealSense2Grabber ("", serial_number); } 
   

    /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
    virtual ~RealSense2Grabber () throw ();

    /** \brief Set the device options
    * \param[in] width resolution
    * \param[in] height resolution
    * \param[in] fps target frames per second for the device
    */
    inline void 
    setDeviceOptions (uint32_t width, uint32_t height, uint32_t fps = 30)
    {
      device_width_ = width;
      device_height_ = height;
      target_fps_ = fps;

      if (isRunning ())
      {
        stop ();
        start ();
      }
    }

    /** \brief Start the data acquisition. */
    virtual void 
    start ();

    /** \brief Stop the data acquisition. */
    virtual void
    stop ();

    /** \brief Check if the data acquisition is still running. */
    virtual bool 
    isRunning () const;

    /** \brief Obtain the number of frames per second (FPS). */
    virtual float 
    getFramesPerSecond () const;

    /** \brief defined grabber name*/
    virtual std::string 
    getName () const { return std::string ("RealSense2Grabber"); }

    //define callback signature typedefs
    typedef void (signal_librealsense_PointXYZ) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>&);
    typedef void (signal_librealsense_PointXYZI) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>&);
    typedef void (signal_librealsense_PointXYZRGB) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>&);
    typedef void (signal_librealsense_PointXYZRGBA) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>>&);

  protected:
    /** \brief Constructor
    * \param[in] file_name used for loading bag file
    * \param[in] serial_number to load a specific device
    */
    RealSense2Grabber (const std::string& file_name, const std::string& serial_number);

    boost::signals2::signal<signal_librealsense_PointXYZ>* signal_PointXYZ;
    boost::signals2::signal<signal_librealsense_PointXYZI>* signal_PointXYZI;
    boost::signals2::signal<signal_librealsense_PointXYZRGB>* signal_PointXYZRGB;
    boost::signals2::signal<signal_librealsense_PointXYZRGBA>* signal_PointXYZRGBA;

    /** \brief Convert a Depth image to a pcl::PointCloud<pcl::PointXYZ>
    * \param[in] points the depth points
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr 
    convertDepthToPointXYZ (const rs2::points& points);

    /** \brief Convert an Infrared Depth image to a pcl::PointCloud<pcl::PointXYZI>
    * \param[in] points the depth points
    * \param[in] ir Infrared video frame
    */
    pcl::PointCloud<pcl::PointXYZI>::Ptr 
    convertInfraredDepthToPointXYZI (const rs2::points & points, rs2::video_frame & ir);

    /** \brief Convert an rgb Depth image to a pcl::PointCloud<pcl::PointXYZRGB>
    * \param[in] points the depth points
    * \param[in] rgb rgb video frame
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    convertRGBDepthToPointXYZRGB (const rs2::points& points, rs2::video_frame &rgb);

    /** \brief Convert an rgb Depth image to a pcl::PointCloud<pcl::PointXYZRGBA>
    * \param[in] points the depth points
    * \param[in] rgb rgb video frame
    */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 
    convertRGBADepthToPointXYZRGBA (const rs2::points& points, rs2::video_frame &rgb);

    /** \brief Retrieve RGB color from texture video frame
    * \param[in] texture the texture
    * \param[in] u 2D coordinate
    * \param[in] v 2D coordinate
    */
    std::tuple<uint8_t, uint8_t, uint8_t> 
    getTextureColor (rs2::video_frame & texture, float u, float v);

    /** \brief Retrieve color intensity from texture video frame
    * \param[in] texture the texture
    * \param[in] u 2D coordinate
    * \param[in] v 2D coordinate
    */
    uint8_t
    getTextureIntensity (rs2::video_frame &texture, float u, float v);

    /** \brief the thread function
    */
    void 
    threadFunction ();

    std::thread thread_;
    mutable std::mutex mutex_;
    std::string serial_number_;
    std::string file_name_;
    bool quit_;
    bool running_;
    float fps_;
    uint32_t device_width_;
    uint32_t device_height_;
    uint32_t target_fps_;
    rs2_format ir_format_;

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc_;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe_;
  };

}

#endif /* PCL_IO_REAL_SENSE_2_GRABBER_H */
