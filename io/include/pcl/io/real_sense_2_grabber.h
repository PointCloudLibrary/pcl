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

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>

#include <librealsense2/rs.hpp>

namespace pcl
{
	struct PointXYZ;
	struct PointXYZRGB;
	struct PointXYZRGBA;
	struct PointXYZI;
	template <typename T> class PointCloud;

	class PCL_EXPORTS RealSense2Grabber : public pcl::Grabber
	{
	public:
		RealSense2Grabber(const std::string& serial_number = "");

		virtual ~RealSense2Grabber() throw ();
		void setDeviceOptions(int width, int height, int fps = 30);
		virtual void start();
		virtual void stop();
		virtual bool isRunning() const;
		virtual float getFramesPerSecond() const;
		virtual std::string getName() const { return std::string("RealSense2Grabber"); }


		typedef void (signal_librealsense_PointXYZ)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>&);
		typedef void (signal_librealsense_PointXYZI)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>&);
		typedef void (signal_librealsense_PointXYZRGB)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>&);
		typedef void (signal_librealsense_PointXYZRGBA)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>>&);

	protected:
		boost::signals2::signal<signal_librealsense_PointXYZ>* signal_PointXYZ;
		boost::signals2::signal<signal_librealsense_PointXYZI>* signal_PointXYZI;
		boost::signals2::signal<signal_librealsense_PointXYZRGB>* signal_PointXYZRGB;
		boost::signals2::signal<signal_librealsense_PointXYZRGBA>* signal_PointXYZRGBA;

		pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ(const rs2::points& points);
		pcl::PointCloud<pcl::PointXYZI>::Ptr convertInfraredDepthToPointXYZI(const rs2::points & points, rs2::video_frame & ir);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB(const rs2::points& points, rs2::video_frame &rgb);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA(const rs2::points& points, rs2::video_frame &rgb);

		std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame & texture, float u, float v);

		std::thread thread;
		mutable std::mutex mutex;

		void threadFunction();

		std::string serial_number;
		bool quit;
		bool running;
		float fps;
		int deviceWidth;
		int deviceHeight;
		int targetFps;


		// Declare pointcloud object, for calculating pointclouds and texture mappings
		rs2::pointcloud pc;
		// Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::pipeline pipe;
	};

}

