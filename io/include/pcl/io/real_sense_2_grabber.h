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
    */
    RealSense2Grabber (const std::string& file_name_or_serial_number = "");   

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
    
    boost::signals2::signal<signal_librealsense_PointXYZ>* signal_PointXYZ;
    boost::signals2::signal<signal_librealsense_PointXYZI>* signal_PointXYZI;
    boost::signals2::signal<signal_librealsense_PointXYZRGB>* signal_PointXYZRGB;
    boost::signals2::signal<signal_librealsense_PointXYZRGBA>* signal_PointXYZRGBA;    

    /** \brief the thread function
    */
    void 
    threadFunction ();

    /** \brief handle to the thread */
    std::thread thread_;
    /** \brief mutex to lock data between thread and UI */
    mutable std::mutex mutex_;
    /** \brief Defines either a file path to a bag file or a realsense device serial number. */
    std::string file_name_or_serial_number_;
    /** \brief controlling the state of the thread. */
    bool quit_;
    /** \brief Is the grabber running. */
    bool running_;
    /** \brief Calculated FPS for the grabber. */
    float fps_;
    /** \brief Width for the depth and color sensor. Default 424*/
    uint32_t device_width_;
    /** \brief Height for the depth and color sensor. Default 240 */
    uint32_t device_height_;
    /** \brief Target FPS for the device. Default 30. */
    uint32_t target_fps_;
    /** \brief format for the IR sensor. */
    rs2_format ir_format_;
    /** \brief Declare pointcloud object, for calculating pointclouds and texture mappings */
    rs2::pointcloud pc_;
    /** \brief Declare RealSense pipeline, encapsulating the actual device and sensors */
    rs2::pipeline pipe_;
  };

}

#endif /* PCL_IO_REAL_SENSE_2_GRABBER_H */
