#include <cstdlib>
#include <fstream>
#include <iostream>

#include <boost/cstdlib.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

struct RGBDHeader
{
  boost::uint32_t width;
  boost::uint32_t height;
};

struct RGB
{
  boost::uint8_t r, g, b;
  boost::uint8_t dummy; // ensure alignment
};

int
main(int argc, char * argv[])
{
  if (argc != 3)
  {
    std::cerr << "Usage " << argv[0] << " cloud.pcd image.rgbd\n";
    return (EXIT_FAILURE);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile (argv[1], *cloud) == -1)
  {
    std::cerr << "Failed to open " << argv[1] << ".\n";
    return (EXIT_FAILURE);
  }

  if (!cloud->isOrganized ())
  {
    std::cerr << "Expected an organized cloud.\n";
    return (EXIT_FAILURE);
  }

  std::ofstream rgbd_file (argv[2], std::ios::out | std::ios::binary);

  if (!rgbd_file)
  {
    std::cerr << "Couldn't open " << argv[2] << ".\n";
    return (EXIT_FAILURE);
  }

  RGBDHeader header = { cloud->width, cloud->height };
  rgbd_file.write (reinterpret_cast<char *> (&header), sizeof header);

  for (unsigned i = 0; i < cloud->size(); ++i)
  {
    RGB pixel = {
      cloud->at(i).r, cloud->at(i).g, cloud->at(i).b, 0,
    };
    rgbd_file.write(reinterpret_cast<char *> (&pixel), sizeof pixel);
  }

  for (unsigned i = 0; i < cloud->size(); ++i)
  {
    boost::int16_t pixel(cloud->at(i).z * 1000);
    rgbd_file.write(reinterpret_cast<char *> (&pixel), sizeof pixel);
  }  

  if (!rgbd_file)
  {
    std::cerr << "Write error.\n";
    return (EXIT_FAILURE);
  }

  return (EXIT_SUCCESS);
}
