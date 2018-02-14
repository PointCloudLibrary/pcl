#include <pcl/io/eigen.h>
#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <librealsense2/rs.hpp>
#include <pcl/io/real_sense_2_grabber.h>
#include <pcl/common/time.h>

#ifdef _OPENMP
#define PARALLEL_FOR #pragma omp parallel for 
#else
#define PARALLEL_FOR 
#endif

namespace pcl
{

RealSense2Grabber::RealSense2Grabber(const std::string& serial_number)
: running(false)
, quit(false)
, serial_number(serial_number)
, signal_PointXYZ(nullptr)
, signal_PointXYZI(nullptr)
, signal_PointXYZRGB(nullptr)
, signal_PointXYZRGBA(nullptr)
, fps(0)
, deviceWidth(424)
, deviceHeight(240)
, targetFps(30)
{
    
    // for now we only create the xyzrgb signal
    signal_PointXYZ = createSignal<signal_librealsense_PointXYZ>();
    signal_PointXYZI = createSignal<signal_librealsense_PointXYZI>();
    signal_PointXYZRGB = createSignal<signal_librealsense_PointXYZRGB>();
    signal_PointXYZRGBA = createSignal<signal_librealsense_PointXYZRGBA>();
}


RealSense2Grabber::~RealSense2Grabber()
{
    stop();

    disconnect_all_slots<signal_librealsense_PointXYZ>();
    disconnect_all_slots<signal_librealsense_PointXYZI>();
    disconnect_all_slots<signal_librealsense_PointXYZRGB>();
    disconnect_all_slots<signal_librealsense_PointXYZRGBA>();

    thread.join();

    pipe.stop();    
}

void RealSense2Grabber::setDeviceOptions(int width, int height, int fps)
{
	deviceWidth = width;
	deviceHeight = height;
	targetFps = fps;
}

void RealSense2Grabber::start()
{
    running = true;

	rs2::config cfg;

	if (!serial_number.empty())
		cfg.enable_device(serial_number);

	cfg.enable_stream(RS2_STREAM_COLOR, deviceWidth, deviceHeight, RS2_FORMAT_RGB8, targetFps);
	cfg.enable_stream(RS2_STREAM_DEPTH, deviceWidth, deviceHeight, RS2_FORMAT_ANY, targetFps);
	cfg.enable_stream(RS2_STREAM_INFRARED, deviceWidth, deviceHeight, RS2_FORMAT_ANY, targetFps);

	pipe.start(cfg);

    thread = std::thread(&RealSense2Grabber::threadFunction, this);
}

void RealSense2Grabber::stop()
{
	std::lock_guard<std::mutex> guard(mutex);

    quit = true;
    running = false;
}

bool RealSense2Grabber::isRunning() const
{
	std::lock_guard<std::mutex> guard(mutex);

    return running;
}

float RealSense2Grabber::getFramesPerSecond() const
{
    return fps;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RealSense2Grabber::convertDepthToPointXYZ(const rs2::points & points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto vertices_ptr = points.get_vertices();

#ifdef _OPENMP
#pragma omp parallel for 
#endif
	for (int index = 0; index < cloud->points.size(); ++index)
	{
		auto ptr = vertices_ptr + index;
		auto p = cloud->points[index];

		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
	}

	return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RealSense2Grabber::convertInfraredDepthToPointXYZI(const rs2::points & points, rs2::video_frame & ir)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto vertices_ptr = points.get_vertices();
	auto texture_ptr = points.get_texture_coordinates();

	uint8_t r, g, b;

#ifdef _OPENMP
#pragma omp parallel for 
#endif
	for (int index = 0; index < cloud->points.size(); ++index)
	{
		auto ptr = vertices_ptr + index;
		auto uvptr = texture_ptr + index;
		auto p = cloud->points[index];

		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;

		std::tie(r, g, b) = get_texcolor(ir, uvptr->u, uvptr->v);

		p.intensity = 0.299f * static_cast <float> (r) + 0.587f * static_cast <float> (g) + 0.114f * static_cast <float> (b);
		
	}

	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RealSense2Grabber::convertRGBDepthToPointXYZRGB(const rs2::points & points, rs2::video_frame & rgb)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto vertices_ptr = points.get_vertices();
    auto texture_ptr = points.get_texture_coordinates();

#ifdef _OPENMP
#pragma omp parallel for 
#endif
	for (int index = 0; index < cloud->points.size(); ++index)
	{
		auto ptr = vertices_ptr + index;
		auto uvptr = texture_ptr + index;
		auto p = cloud->points[index];

		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;

		std::tie(p.r, p.g, p.b) = get_texcolor(rgb, uvptr->u, uvptr->v);

		p.a = 255;
	}

    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RealSense2Grabber::convertRGBADepthToPointXYZRGBA(const rs2::points & points, rs2::video_frame & rgb)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());

	auto vertices_ptr = points.get_vertices();
	auto texture_ptr = points.get_texture_coordinates();

#ifdef _OPENMP
#pragma omp parallel for 
#endif
	for (int index = 0; index < cloud->points.size(); ++index)
	{
		auto ptr = vertices_ptr + index;
		auto uvptr = texture_ptr + index;
		auto p = cloud->points[index];

		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;

		std::tie(p.r, p.g, p.b) = get_texcolor(rgb, uvptr->u, uvptr->v);

		p.a = 255;
	}

    return cloud;
}

std::tuple<uint8_t, uint8_t, uint8_t> RealSense2Grabber::get_texcolor(rs2::video_frame &texture, float u, float v)
{
    auto ptr = dynamic_cast<rs2::video_frame*>(&texture);

    const int w = ptr->get_width(), h = ptr->get_height();
    int x = std::min(std::max(int(u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(v*h + .5f), 0), h - 1);
    int idx = x * ptr->get_bits_per_pixel() / 8 + y * ptr->get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(ptr->get_data());
    return std::make_tuple(texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}

void RealSense2Grabber::threadFunction()
{
	pcl::StopWatch sw;

	rs2::frameset frames;
	rs2::depth_frame depth = NULL;	
	rs2::video_frame rgb = NULL;
	rs2::video_frame ir = NULL;
	rs2::points points;

    while (!quit)
    {
		sw.reset();

		{
			std::lock_guard<std::mutex> guard(mutex);

			// Wait for the next set of frames from the camera
			frames = pipe.wait_for_frames();

			depth = frames.get_depth_frame();

			// Generate the pointcloud and texture mappings
			points = pc.calculate(depth);

			rgb = frames.get_color_frame();

			// Tell pointcloud object to map to this color frame
			pc.map_to(rgb);

			ir = frames.get_infrared_frame();
		}

        if (signal_PointXYZ->num_slots() > 0) 
        {
            signal_PointXYZ->operator()(convertDepthToPointXYZ(points));
        }

        if (signal_PointXYZI->num_slots() > 0) 
        {
            signal_PointXYZI->operator()(convertInfraredDepthToPointXYZI(points, ir));
        }

        if (signal_PointXYZRGB->num_slots() > 0) 
        {
            signal_PointXYZRGB->operator()(convertRGBDepthToPointXYZRGB(points, rgb));
        }

        if (signal_PointXYZRGBA->num_slots() > 0) 
        {
            signal_PointXYZRGBA->operator()(convertRGBADepthToPointXYZRGBA(points, rgb));
        }

		fps = 1.0f / static_cast <float> (sw.getTimeSeconds());

    }
}

}
