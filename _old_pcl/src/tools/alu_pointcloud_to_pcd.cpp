/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 */

/**

\author Bastian Steder

@b convert_alu_pc_to_ros_pc loads a binary point cloud file as used in software by the University of Freiburg and saves it as a pcd file.

**/

#include <iostream>
#include <fstream>
using namespace std;

#include "pcl/common/file_io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"


void
printUsage (const char *progName)
{
  cout << "\n\nUsage: " << progName <<
    " [options] <point_cloud_file_1.pointCloud> [point_cloud_file_2.pointCloud] ... [point_cloud_file_n.pointCloud]\n\n"
    << "Options:\n" << "-------------------------------------------\n" <<
    "-f         Store far ranges in an extra file\n" << "-h         this help\n" << "\n\n";
}

bool
checkFileHeader (std::istream & file, const std::string headerKeyWord)
{
  unsigned int posInFile = file.tellg ();
  unsigned int headerKeyWordLength = headerKeyWord.length ();

  char *headerKeyWordFromFileC = new char[headerKeyWordLength + 1];
  headerKeyWordFromFileC[headerKeyWordLength] = 0;
  file.read (headerKeyWordFromFileC, headerKeyWordLength);
  string headerKeyWordFromFile = headerKeyWordFromFileC;

  file.seekg (posInFile);       // Go back to former pos in file

  if (headerKeyWord == headerKeyWordFromFile)
  {
    //cout << "Found correct header \""<<headerKeyWordFromFile<<"\".\n";
    delete[]headerKeyWordFromFileC;
    return (true);
  }

  //cout << "Did not find correct header: \""<<headerKeyWordFromFile<<"\" instead of \""<<headerKeyWord<<"\".\n";
  delete[]headerKeyWordFromFileC;
  return (false);
}

struct GeneralPoint
{
  float x, y, z, distance, vp_x, vp_y, vp_z;
  union
  {
    uint32_t color_rgba;
    struct
    {
      unsigned char color_r, color_g, color_b, color_a;
    };
  };
};

void
getAverageViewpoint (const std::vector < GeneralPoint > &points, float &vp_mean_x, float &vp_mean_y, float &vp_mean_z)
{
  vp_mean_x = vp_mean_y = vp_mean_z = 0.0f;
  for (unsigned int i = 0; i < points.size (); ++i)
  {
    vp_mean_x += points[i].vp_x;
    vp_mean_y += points[i].vp_y;
    vp_mean_z += points[i].vp_z;
  }
  vp_mean_x /= points.size ();
  vp_mean_y /= points.size ();
  vp_mean_z /= points.size ();
}

/* ---[ */
int
main (int argc, char **argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  bool store_far_ranges_in_extra_file = false;

  // Read command line arguments.
#ifndef _WIN32
  for (char c; (c = getopt (argc, argv, "fh")) != -1;)
  {
    switch (c)
    {
      case 'f':
      {
        store_far_ranges_in_extra_file = true;
        cout << "Will store far ranges in an extra file.\n";
        break;
      }
      case 'h':
      {
        printUsage (argv[0]);
        exit (0);
      }
    }
  }
  if (optind >= argc)
  {
    std::cerr << "No file name(s) given as command line argument(s).\n\n";
    printUsage (argv[0]);
    exit (1);
  }
#endif

  // --------------------------------------

#ifndef _WIN32
  for (int argument_index = optind; argument_index < argc; ++argument_index)
#else
  for (int argument_index = 0; argument_index < argc; ++argument_index)
#endif
  {
    string fileName = argv[argument_index];
    cout << "\nTrying to open \"" << fileName << "\".\n";
    std::ifstream file (fileName.c_str ());
    if (!file)
    {
      std::cerr << "Was not able to open file.\n";
      continue;
    }
    string output_file_name = pcl::getFilenameWithoutExtension (fileName) + ".pcd",
      output_file_name_far_ranges = pcl::getFilenameWithoutExtension (fileName) + "_far_ranges.pcd";

    std::vector < GeneralPoint > points, far_ranges;

    std::string headerLine;
    std::string headerKeyWord = "PointCloudColoredWithSensorPosesT<float>";
    if (checkFileHeader (file, headerKeyWord))
    {
      std::cout << "File is of type \"" << headerKeyWord.c_str () << "\".\n";
      getline (file, headerLine);

      uint32_t num_points;
      file.read ((char *) &num_points, sizeof (num_points));
      std::cout << "Point cloud has " << num_points << " points.\n";

      for (unsigned int i = 0; i < num_points; ++i)
      {
        GeneralPoint point;

        uint32_t vectorSize;
        file.read ((char *) &vectorSize, sizeof (vectorSize));
        if (vectorSize != 3)
        {
          ROS_ERROR ("Vector size is not 3!");
          continue;
        }

        float freiburg_x, freiburg_y, freiburg_z;
        // Read current point and convert to different coordinate system
        file.read ((char *) &freiburg_x, sizeof (freiburg_x));
        file.read ((char *) &freiburg_y, sizeof (freiburg_y));
        file.read ((char *) &freiburg_z, sizeof (freiburg_z));
        point.x = -freiburg_y;
        point.y = -freiburg_z;
        point.z = freiburg_x;

        file.read ((char *) &vectorSize, sizeof (vectorSize));
        if (vectorSize != 3)
        {
          ROS_ERROR ("Vector size is not 3!");
          continue;
        }

        // Read current sensor pos and convert to different coordinate system
        file.read ((char *) &freiburg_x, sizeof (freiburg_x));
        file.read ((char *) &freiburg_y, sizeof (freiburg_y));
        file.read ((char *) &freiburg_z, sizeof (freiburg_z));
        point.vp_x = -freiburg_y;
        point.vp_y = -freiburg_z;
        point.vp_z = freiburg_x;

        point.distance =
          sqrtf (powf (point.x - point.vp_x, 2) + powf (point.y - point.vp_y, 2) + powf (point.y - point.vp_y, 2));

        bool has_color;
        file.read ((char *) &has_color, sizeof (has_color));
        point.color_r = point.color_g = point.color_b = point.color_a = 0;
        if (has_color)
        {
          file.read ((char *) &point.color_r, sizeof (point.color_r));
          file.read ((char *) &point.color_g, sizeof (point.color_g));
          file.read ((char *) &point.color_b, sizeof (point.color_b));
          point.color_a = 255;
        }

        points.push_back (point);
      }

      float vp_mean_x, vp_mean_y, vp_mean_z;
      getAverageViewpoint (points, vp_mean_x, vp_mean_y, vp_mean_z);

      std::ofstream file_out (output_file_name.c_str ());
      file_out << "# .PCD v.7 - Point Cloud Data file format\n"
        << "FIELDS x y z distance vp_x vp_y vp_z rgba\n"
        << "SIZE 4 4 4 4 4 4 4 4\n"
        << "TYPE F F F F F F F U\n"
        << "COUNT 1 1 1 1 1 1 1 1\n"
        << "WIDTH " << points.size () << "\n"
        << "HEIGHT 1\n"
        << "VIEWPOINT " << vp_mean_x << " " << vp_mean_y << " " << vp_mean_z << " 1 0 0 0\n"
        << "POINTS " << points.size () << "\n" << "DATA ascii\n";
      for (unsigned int i = 0; i < points.size (); ++i)
      {
        const GeneralPoint & point = points[i];
        file_out << point.x << " " << point.y << " " << point.z << " " << point.distance << " "
          << point.vp_x << " " << point.vp_y << " " << point.vp_z << " " << point.color_rgba << "\n";
      }
      std::cout << "Done.\n\n";
      file.close ();
      file_out.close ();
      continue;
    }

    headerKeyWord = "PointCloudWithSensorPosesT<float>";
    if (checkFileHeader (file, headerKeyWord))
    {
      std::cout << "File is of type \"" << headerKeyWord.c_str () << "\".\n";
      getline (file, headerLine);

      uint32_t num_points;
      file.read ((char *) &num_points, sizeof (num_points));
      std::cout << "Point cloud has " << num_points << " normal points.\n";

      for (unsigned int i = 0; i < num_points; ++i)
      {
        GeneralPoint point;

        float freiburg_x, freiburg_y, freiburg_z;
        // Read current point and convert to different coordinate system
        file.read ((char *) &freiburg_x, sizeof (freiburg_x));
        file.read ((char *) &freiburg_y, sizeof (freiburg_y));
        file.read ((char *) &freiburg_z, sizeof (freiburg_z));
        point.x = -freiburg_y;
        point.y = -freiburg_z;
        point.z = freiburg_x;

        // Read current sensor pos and convert to different coordinate system
        file.read ((char *) &freiburg_x, sizeof (freiburg_x));
        file.read ((char *) &freiburg_y, sizeof (freiburg_y));
        file.read ((char *) &freiburg_z, sizeof (freiburg_z));
        point.vp_x = -freiburg_y;
        point.vp_y = -freiburg_z;
        point.vp_z = freiburg_x;

        point.distance =
          sqrtf (powf (point.x - point.vp_x, 2) + powf (point.y - point.vp_y, 2) + powf (point.y - point.vp_y, 2));

        points.push_back (point);
      }

      uint32_t num_far_ranges;
      file.read ((char *) &num_far_ranges, sizeof (num_far_ranges));
      std::cout << "Point cloud has " << num_far_ranges << " far range points.\n";

      for (unsigned int i = 0; i < num_far_ranges; ++i)
      {
        GeneralPoint point;

        float freiburg_x, freiburg_y, freiburg_z;
        // Read current point and convert to different coordinate system
        file.read ((char *) &freiburg_x, sizeof (freiburg_x));
        file.read ((char *) &freiburg_y, sizeof (freiburg_y));
        file.read ((char *) &freiburg_z, sizeof (freiburg_z));
        point.x = -freiburg_y;
        point.y = -freiburg_z;
        point.z = freiburg_x;

        // Read current sensor pos and convert to different coordinate system
        file.read ((char *) &freiburg_x, sizeof (freiburg_x));
        file.read ((char *) &freiburg_y, sizeof (freiburg_y));
        file.read ((char *) &freiburg_z, sizeof (freiburg_z));
        point.vp_x = -freiburg_y;
        point.vp_y = -freiburg_z;
        point.vp_z = freiburg_x;

        if (store_far_ranges_in_extra_file)
        {
          far_ranges.push_back (point);
        }
        else
        {
          // Nasty convention to encode far ranges:
          point.distance = point.x;
          point.x = std::numeric_limits<float>::quiet_NaN ();
          points.push_back (point);
        }
      }
      file.close ();

      float vp_mean_x, vp_mean_y, vp_mean_z;
      getAverageViewpoint (points, vp_mean_x, vp_mean_y, vp_mean_z);

      std::ofstream file_out (output_file_name.c_str ());
      file_out << "# .PCD v.7 - Point Cloud Data file format\n"
        << "FIELDS x y z distance vp_x vp_y vp_z\n"
        << "SIZE 4 4 4 4 4 4 4\n"
        << "TYPE F F F F F F F\n"
        << "COUNT 1 1 1 1 1 1 1\n"
        << "WIDTH " << points.size () << "\n"
        << "HEIGHT 1\n"
        << "VIEWPOINT " << vp_mean_x << " " << vp_mean_y << " " << vp_mean_z << " 1 0 0 0\n"
        << "POINTS " << points.size () << "\n" << "DATA ascii\n";
      for (unsigned int i = 0; i < points.size (); ++i)
      {
        const GeneralPoint & point = points[i];
        file_out << point.x << " " << point.y << " " << point.z << " " << point.distance << " "
          << point.vp_x << " " << point.vp_y << " " << point.vp_z << "\n";
      }
      file_out.close ();

      if (!far_ranges.empty ())
      {
        file_out.open (output_file_name_far_ranges.c_str ());
        file_out << "# .PCD v.7 - Point Cloud Data file format\n"
          << "FIELDS x y z vp_x vp_y vp_z\n"
          << "SIZE 4 4 4 4 4 4\n"
          << "TYPE F F F F F F\n"
          << "COUNT 1 1 1 1 1 1\n"
          << "WIDTH " << far_ranges.size () << "\n"
          << "HEIGHT 1\n"
          << "VIEWPOINT " << vp_mean_x << " " << vp_mean_y << " " << vp_mean_z << " 1 0 0 0\n"
          << "POINTS " << far_ranges.size () << "\n" << "DATA ascii\n";
        for (unsigned int i = 0; i < far_ranges.size (); ++i)
        {
          const GeneralPoint & point = far_ranges[i];
          file_out << point.x << " " << point.y << " " << point.z << " " << " "
            << point.vp_x << " " << point.vp_y << " " << point.vp_z << "\n";
        }
        file_out.close ();
      }

      std::cout << "Done.\n";
      continue;
    }

    headerKeyWord = "PointCloudT<float>";
    if (checkFileHeader (file, headerKeyWord))
    {
      ROS_INFO ("File is of type \"%s\".", headerKeyWord.c_str ());
      ROS_ERROR ("Sorry, not implemented yet.");

      continue;
    }

    ROS_ERROR ("Could not determine file type. Doing nothing.\n");

    continue;
  }
}

/* ]--- */
