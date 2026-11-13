// Structured seed generator for octree_compression_fuzzer.
//
// Produces valid compressed point-cloud streams by running real PCL clouds
// through OctreePointCloudCompression::encodePointCloud with several
// profiles (online/offline, with/without color, manual configuration with
// voxel-grid downsampling). The outputs cover all three point-type
// instantiations exercised by the fuzzer (PointXYZ, PointXYZRGB,
// PointXYZRGBA) and give the decoder structurally valid frames to start
// from. Run from test/fuzz/build.sh after the PCL build; outputs are
// zipped into the fuzzer's seed corpus.

#include <fstream>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

template <typename PointT>
bool
encode (const char* in_pcd, const char* out_file,
        pcl::io::compression_Profiles_e profile,
        bool do_color, bool do_voxel, unsigned char selector_byte)
{
  typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile (std::string (in_pcd), *cloud) < 0 || cloud->empty ()) {
    fprintf (stderr, "could not load %s\n", in_pcd);
    return false;
  }
  pcl::io::OctreePointCloudCompression<PointT> encoder (
      profile, false, 0.001, 0.01, do_voxel, 30, do_color, 6);
  std::ofstream f (out_file, std::ios::binary);
  // The fuzz target consumes one selector byte before the stream; the seed
  // must be packaged in exactly that format or the decoder never even finds
  // the frame magic.
  f.put (static_cast<char> (selector_byte));
  encoder.encodePointCloud (cloud, f);
  return f.good ();
}

int
main (int argc, char** argv)
{
  if (argc < 4) {
    fprintf (stderr, "usage: gen_octree_seeds <outdir> <xyz.pcd> <rgb.pcd>\n");
    return 1;
  }
  const std::string dir = argv[1];
  encode<pcl::PointXYZ> (argv[2], (dir + "/xyz_low_offline").c_str (),
                         pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR, false, false, 0);
  encode<pcl::PointXYZ> (argv[2], (dir + "/xyz_med_online").c_str (),
                         pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false, false, 0);
  encode<pcl::PointXYZRGB> (argv[3], (dir + "/rgb_med_online").c_str (),
                            pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR, true, false, 1);
  encode<pcl::PointXYZRGB> (argv[3], (dir + "/rgb_high_offline").c_str (),
                            pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR, true, false, 1);
  encode<pcl::PointXYZRGBA> (argv[3], (dir + "/rgba_manual_voxel").c_str (),
                             pcl::io::MANUAL_CONFIGURATION, true, true, 2);
  return 0;
}
