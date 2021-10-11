#include <pcl/io/pcd_io.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <fstream>

namespace pcl {
namespace rec_3d_framework {
namespace PersistenceUtils {

inline bool
writeCentroidToFile(const std::string& file, Eigen::Vector3f& centroid)
{
  std::ofstream out(file.c_str());
  if (!out) {
    std::cout << "Cannot open file.\n";
    return false;
  }

  out << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
  out.close();

  return true;
}

inline bool
getCentroidFromFile(const std::string& file, Eigen::Vector3f& centroid)
{
  std::ifstream in;
  in.open(file.c_str(), std::ifstream::in);

  char linebuf[256];
  in.getline(linebuf, 256);
  std::string line(linebuf);
  std::vector<std::string> strs;
  boost::split(strs, line, boost::is_any_of(" "));
  centroid[0] = static_cast<float>(atof(strs[0].c_str()));
  centroid[1] = static_cast<float>(atof(strs[1].c_str()));
  centroid[2] = static_cast<float>(atof(strs[2].c_str()));

  return true;
}

inline bool
writeMatrixToFile(const std::string& file, Eigen::Matrix4f& matrix)
{
  std::ofstream out(file.c_str());
  if (!out) {
    std::cout << "Cannot open file.\n";
    return false;
  }

  for (std::size_t i = 0; i < 4; i++) {
    for (std::size_t j = 0; j < 4; j++) {
      out << matrix(i, j);
      if (!(i == 3 && j == 3))
        out << " ";
    }
  }
  out.close();

  return true;
}

inline bool
writeFloatToFile(const std::string& file, float value)
{
  std::ofstream out(file.c_str());
  if (!out) {
    std::cout << "Cannot open file.\n";
    return false;
  }

  out << value;
  out.close();

  return true;
}

inline std::string
getViewId(std::string id)
{
  std::vector<std::string> strs;
  boost::split(strs, id, boost::is_any_of("_"));

  return strs[strs.size() - 2];
}

inline bool
readFloatFromFile(const std::string& file, float& value)
{

  std::ifstream in;
  in.open(file.c_str(), std::ifstream::in);

  char linebuf[1024];
  in.getline(linebuf, 1024);
  value = static_cast<float>(atof(linebuf));

  return true;
}

inline bool
readMatrixFromFile(const std::string& file, Eigen::Matrix4f& matrix)
{

  std::ifstream in;
  in.open(file.c_str(), std::ifstream::in);

  char linebuf[1024];
  in.getline(linebuf, 1024);
  std::string line(linebuf);
  std::vector<std::string> strs_2;
  boost::split(strs_2, line, boost::is_any_of(" "));

  for (int i = 0; i < 16; i++) {
    matrix(i % 4, i / 4) = static_cast<float>(atof(strs_2[i].c_str()));
  }

  return true;
}
} // namespace PersistenceUtils
} // namespace rec_3d_framework
} // namespace pcl
