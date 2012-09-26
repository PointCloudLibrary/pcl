/*
 * face_detection_apps_utils.h
 *
 *  Created on: 22 Sep 2012
 *      Author: ari
 */

#ifndef FACE_DETECTION_APPS_UTILS_H_
#define FACE_DETECTION_APPS_UTILS_H_

namespace face_detection_apps_utils
{
  inline bool readMatrixFromFile(std::string file, Eigen::Matrix4f & matrix)
  {

    std::ifstream in;
    in.open (file.c_str (), std::ifstream::in);
    if (!in.is_open ())
    {
      return false;
    }

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

  inline bool sortFiles(const std::string & file1, const std::string & file2)
  {
    std::vector < std::string > strs1;
    boost::split (strs1, file1, boost::is_any_of ("/"));

    std::vector < std::string > strs2;
    boost::split (strs2, file2, boost::is_any_of ("/"));

    std::string id_1 = strs1[strs1.size () - 1];
    std::string id_2 = strs2[strs2.size () - 1];

    {
      std::vector < std::string > strs1;
      boost::split (strs1, id_1, boost::is_any_of ("_"));

      std::vector < std::string > strs2;
      boost::split (strs2, id_2, boost::is_any_of ("_"));

      std::string id_1 = strs1[strs1.size () - 1];
      std::string id_2 = strs2[strs2.size () - 1];

      size_t pos1 = id_1.find (".pcd");
      size_t pos2 = id_2.find (".pcd");

      id_1 = id_1.substr (0, pos1);
      id_2 = id_2.substr (0, pos2);

      return atoi (id_1.c_str ()) < atoi (id_2.c_str ());
    }
  }

  inline
  void getFilesInDirectory(bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext)
  {
    bf::directory_iterator end_itr;
    for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
    {
      //check if its a directory, then get models in it
      if (bf::is_directory (*itr))
      {
#if BOOST_FILESYSTEM_VERSION == 3
        std::string so_far = rel_path_so_far + (itr->path().filename()).string() + "/";
#else
        std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

        bf::path curr_path = itr->path ();
        getFilesInDirectory (curr_path, so_far, relative_paths, ext);
      } else
      {
        //check that it is a ply file and then add, otherwise ignore..
        std::vector < std::string > strs;
#if BOOST_FILESYSTEM_VERSION == 3
        std::string file = (itr->path().filename()).string();
#else
        std::string file = (itr->path ()).filename ();
#endif

        boost::split (strs, file, boost::is_any_of ("."));
        std::string extension = strs[strs.size () - 1];

        if (extension.compare (ext) == 0)
        {
#if BOOST_FILESYSTEM_VERSION == 3
          std::string path = rel_path_so_far + (itr->path().filename()).string();
#else
          std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif

          relative_paths.push_back (path);
        }
      }
    }
  }

  void displayHeads(std::vector<Eigen::VectorXf> & heads, pcl::visualization::PCLVisualizer & vis)
  {
    for (size_t i = 0; i < heads.size (); i++)
    {
      std::stringstream name;
      name << "sphere" << i;
      pcl::PointXYZ center_point;
      center_point.x = heads[i][0];
      center_point.y = heads[i][1];
      center_point.z = heads[i][2];
      vis.addSphere (center_point, 0.02, 0, 255, 0, name.str ());

      pcl::ModelCoefficients cylinder_coeff;
      cylinder_coeff.values.resize (7); // We need 7 values
      cylinder_coeff.values[0] = center_point.x;
      cylinder_coeff.values[1] = center_point.y;
      cylinder_coeff.values[2] = center_point.z;

      Eigen::Vector3f vec = Eigen::Vector3f::UnitZ () * -1.f;
      Eigen::Matrix3f matrixxx;

      matrixxx = Eigen::AngleAxisf (heads[i][3], Eigen::Vector3f::UnitX ()) * Eigen::AngleAxisf (heads[i][4], Eigen::Vector3f::UnitY ())
          * Eigen::AngleAxisf (heads[i][5], Eigen::Vector3f::UnitZ ());

      vec = matrixxx * vec;

      cylinder_coeff.values[3] = vec[0];
      cylinder_coeff.values[4] = vec[1];
      cylinder_coeff.values[5] = vec[2];

      cylinder_coeff.values[6] = 0.01f;
      name << "cylinder";
      vis.addCylinder (cylinder_coeff, name.str ());
    }
  }
}

#endif /* FACE_DETECTION_APPS_UTILS_H_ */
