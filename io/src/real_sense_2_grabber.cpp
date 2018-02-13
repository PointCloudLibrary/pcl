#include <pcl/io/eigen.h>
#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <librealsense2/rs.hpp>
#include <pcl/io/real_sense_2_grabber.h>

namespace pcl
{

RealSense2Grabber::RealSense2Grabber()
: running(false)
, quit(false)
, signal_PointXYZ(nullptr)
, signal_PointXYZI(nullptr)
, signal_PointXYZRGB(nullptr)
, signal_PointXYZRGBA(nullptr)
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

void RealSense2Grabber::start()
{
    running = true;

	rs2::config cfg;

	cfg.enable_stream(RS2_STREAM_COLOR, 424, 240, RS2_FORMAT_ANY, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_ANY, 30);
	cfg.enable_stream(RS2_STREAM_INFRARED, 424, 240, RS2_FORMAT_ANY, 30);

	pipe.start(cfg);

    thread = boost::thread(&RealSense2Grabber::threadFunction, this);
}

void RealSense2Grabber::stop()
{
    boost::unique_lock<boost::mutex> lock(mutex);

    quit = true;
    running = false;

    lock.unlock();
}

bool RealSense2Grabber::isRunning() const
{
    boost::unique_lock<boost::mutex> lock(mutex);

    return running;

    lock.unlock();
}

float RealSense2Grabber::getFramesPerSecond() const
{
    return 30.0f;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RealSense2Grabber::convertDepthToPointXYZ(const rs2::points & points)
{
    return pcl::PointCloud<pcl::PointXYZ>::Ptr();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RealSense2Grabber::convertInfraredDepthToPointXYZI(const rs2::points & points)
{
    return pcl::PointCloud<pcl::PointXYZI>::Ptr();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RealSense2Grabber::convertRGBDepthToPointXYZRGB(const rs2::points & points, rs2::video_frame & rgb)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto ptr = points.get_vertices();
    auto uvptr = points.get_texture_coordinates();

    uint8_t r, g, b;
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = -1 * ptr->y;
        p.z = ptr->z;

        std::tie(r, g, b) = get_texcolor(rgb, uvptr->u, uvptr->v);

        p.r = r;
        p.g = g;
        p.b = b;
        p.a = 255;

        ptr++;
        uvptr++;
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

    auto ptr = points.get_vertices();
    auto uvptr = points.get_texture_coordinates();

    uint8_t r, g, b;
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = -1 * ptr->y;
        p.z = ptr->z;

        std::tie(r, g, b) = get_texcolor(rgb, uvptr->u, uvptr->v);

        p.r = r;
        p.g = g;
        p.b = b;
        p.a = 255;

        ptr++;
        uvptr++;
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
    while (!quit)
    {
        boost::unique_lock<boost::mutex> lock(mutex);

        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        auto points = pc.calculate(depth);

        // Tell pointcloud object to map to this color frame
        auto rgb = frames.get_color_frame();

        // Tell pointcloud object to map to this color frame
        pc.map_to(rgb);

        lock.unlock();

        if (signal_PointXYZ->num_slots() > 0) 
        {
            signal_PointXYZ->operator()(convertDepthToPointXYZ(points));
        }

        if (signal_PointXYZI->num_slots() > 0) 
        {
            signal_PointXYZI->operator()(convertInfraredDepthToPointXYZI(points));
        }

        if (signal_PointXYZRGB->num_slots() > 0) 
        {
            signal_PointXYZRGB->operator()(convertRGBDepthToPointXYZRGB(points, rgb));
        }

        if (signal_PointXYZRGBA->num_slots() > 0) 
        {
            signal_PointXYZRGBA->operator()(convertRGBADepthToPointXYZRGBA(points, rgb));
        }

    }
}

}
