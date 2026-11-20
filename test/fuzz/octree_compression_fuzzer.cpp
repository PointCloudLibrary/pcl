// Fuzz target for the octree point-cloud compression decoder.
//
// Threat model:
// - Untrusted input: a compressed point-cloud stream (the .pcc artefacts
//   OctreePointCloudCompression::encodePointCloud produces and consumers
//   decode). The whole stream is attacker-controlled: the
//   "<PCL-OCT-COMPRESSED>" frame magic, frame header (frame id, point
//   count), binary octree structure, entropy-coded (static range coder)
//   octree/point/color data.
// - Entry point: pcl::io::OctreePointCloudCompression::decodePointCloud(
//   std::istream&, PointCloudPtr&) for the three explicit instantiations
//   compiled into libpcl_io (PointXYZ, PointXYZRGB, PointXYZRGBA).
// - Expected bug classes: out-of-bounds reads on forged octree binary data
//   and point-count fields, assertion/abort on inconsistent octree state,
//   arithmetic-coder UB, OOM from inflated counts.
//
// The first input byte selects the point-type instantiation; the remainder
// is the compressed stream fed through std::istringstream.

#include <sstream>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_types.h>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    if (size < 1)
        return 0;
    const uint8_t sel = data[0];
    std::string body(reinterpret_cast<const char *>(data) + 1, size - 1);
    std::istringstream stream(body);
    switch (sel % 3) {
    case 0: {
        pcl::io::OctreePointCloudCompression<pcl::PointXYZ> codec;
        pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
        codec.decodePointCloud(stream, out);
        break;
    }
    case 1: {
        pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> codec;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>());
        codec.decodePointCloud(stream, out);
        break;
    }
    default: {
        pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> codec;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGBA>());
        codec.decodePointCloud(stream, out);
        break;
    }
    }
    return 0;
}
