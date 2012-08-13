#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/io/pcd_io.h>

namespace pcl
{
  namespace rec_3d_framework
  {
    namespace PersistenceUtils
    {

      inline bool
      writeCentroidToFile (std::string file, Eigen::Vector3f & centroid)
      {
        std::ofstream out (file.c_str ());
        if (!out)
        {
          std::cout << "Cannot open file.\n";
          return false;
        }

        out << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
        out.close ();

        return true;
      }

      inline bool
      getCentroidFromFile (std::string file, Eigen::Vector3f & centroid)
      {
        std::ifstream in;
        in.open (file.c_str (), std::ifstream::in);

        char linebuf[256];
        in.getline (linebuf, 256);
        std::string line (linebuf);
        std::vector < std::string > strs;
        boost::split (strs, line, boost::is_any_of (" "));
        centroid[0] = static_cast<float> (atof (strs[0].c_str ()));
        centroid[1] = static_cast<float> (atof (strs[1].c_str ()));
        centroid[2] = static_cast<float> (atof (strs[2].c_str ()));

        return true;
      }

      inline bool
      writeMatrixToFile (std::string file, Eigen::Matrix4f & matrix)
      {
        std::ofstream out (file.c_str ());
        if (!out)
        {
          std::cout << "Cannot open file.\n";
          return false;
        }

        for (size_t i = 0; i < 4; i++)
        {
          for (size_t j = 0; j < 4; j++)
          {
            out << matrix (i, j);
            if (!(i == 3 && j == 3))
              out << " ";
          }
        }
        out.close ();

        return true;
      }

      inline bool
      writeFloatToFile (std::string file, float value)
      {
        std::ofstream out (file.c_str ());
        if (!out)
        {
          std::cout << "Cannot open file.\n";
          return false;
        }

        out << value;
        out.close ();

        return true;
      }

      inline std::string
      getViewId (std::string id)
      {
        //descriptor_xxx_xx.pcd
        //and we want xxx

        std::vector < std::string > strs;
        boost::split (strs, id, boost::is_any_of ("_"));

        //std::cout << "id:" << id << std::endl;
        //std::cout << "returned:" << strs[strs.size() - 2] << std::endl;

        return strs[strs.size () - 2];
      }

      inline bool
      readFloatFromFile (std::string dir, std::string file, float& value)
      {

        std::vector < std::string > strs;
        boost::split (strs, file, boost::is_any_of ("/"));

        std::string str;
        for (size_t i = 0; i < (strs.size () - 1); i++)
        {
          str += strs[i] + "/";
        }

        std::stringstream matrix_file;
        matrix_file << dir << "/" << str << "entropy_" << getViewId (file) << ".txt";

        std::ifstream in;
        in.open (matrix_file.str ().c_str (), std::ifstream::in);

        char linebuf[1024];
        in.getline (linebuf, 1024);
        value = static_cast<float> (atof (linebuf));

        return true;
      }

      inline bool
      readFloatFromFile (std::string file, float& value)
      {

        std::ifstream in;
        in.open (file.c_str (), std::ifstream::in);

        char linebuf[1024];
        in.getline (linebuf, 1024);
        value = static_cast<float> (atof (linebuf));

        return true;
      }

      inline bool
      readMatrixFromFile (std::string dir, std::string file, Eigen::Matrix4f & matrix)
      {

        //get the descriptor name from dir
        std::vector < std::string > path;
        boost::split (path, dir, boost::is_any_of ("/"));

        std::string dname = path[path.size () - 1];
        std::string file_replaced;
        for (size_t i = 0; i < (path.size () - 1); i++)
        {
          file_replaced += path[i] + "/";
        }

        boost::split (path, file, boost::is_any_of ("/"));
        std::string id;

        for (size_t i = 0; i < (path.size () - 1); i++)
        {
          id += path[i];
          if (i < (path.size () - 1))
          {
            id += "/";
          }
        }

        boost::split (path, file, boost::is_any_of ("/"));
        std::string filename = path[path.size () - 1];

        std::stringstream matrix_file;
        matrix_file << file_replaced << id << "/" << dname << "/pose_" << getViewId (file) << ".txt";
        //std::cout << matrix_file.str() << std::endl;

        std::ifstream in;
        in.open (matrix_file.str ().c_str (), std::ifstream::in);

        char linebuf[1024];
        in.getline (linebuf, 1024);
        std::string line (linebuf);
        std::vector < std::string > strs_2;
        boost::split (strs_2, line, boost::is_any_of (" "));

        for (int i = 0; i < 16; i++)
        {
          matrix (i % 4, i / 4) = static_cast<float> (atof (strs_2[i].c_str ()));
        }

        return true;
      }

      inline bool
      readMatrixFromFile (std::string file, Eigen::Matrix4f & matrix)
      {

        std::ifstream in;
        in.open (file.c_str (), std::ifstream::in);

        char linebuf[1024];
        in.getline (linebuf, 1024);
        std::string line (linebuf);
        std::vector < std::string > strs_2;
        boost::split (strs_2, line, boost::is_any_of (" "));

        for (int i = 0; i < 16; i++)
        {
          matrix (i % 4, i / 4) = static_cast<float> (atof (strs_2[i].c_str ()));
        }

        return true;
      }

      inline bool
      readMatrixFromFile2 (std::string file, Eigen::Matrix4f & matrix)
      {

        std::ifstream in;
        in.open (file.c_str (), std::ifstream::in);

        char linebuf[1024];
        in.getline (linebuf, 1024);
        std::string line (linebuf);
        std::vector < std::string > strs_2;
        boost::split (strs_2, line, boost::is_any_of (" "));

        for (int i = 0; i < 16; i++)
        {
          matrix (i / 4, i % 4) = static_cast<float> (atof (strs_2[i].c_str ()));
        }

        return true;
      }

      template<typename PointInT>
        inline
        void
        getPointCloudFromFile (std::string dir, std::string file, typename pcl::PointCloud<PointInT>::Ptr & cloud)
        {

          //get the descriptor name from dir
          std::vector < std::string > path;
          boost::split (path, dir, boost::is_any_of ("/"));

          std::string dname = path[path.size () - 1];
          std::string file_replaced;
          for (size_t i = 0; i < (path.size () - 1); i++)
          {
            file_replaced += path[i] + "/";
          }

          boost::split (path, file, boost::is_any_of ("/"));
          std::string id;

          for (size_t i = 0; i < (path.size () - 1); i++)
          {
            id += path[i];
            if (i < (path.size () - 1))
            {
              id += "/";
            }
          }

          std::stringstream view_file;
          view_file << file_replaced << id << "/" << dname << "/view_" << getViewId (file) << ".pcd";

          pcl::io::loadPCDFile (view_file.str (), *cloud);
        }
    }
  }
}

